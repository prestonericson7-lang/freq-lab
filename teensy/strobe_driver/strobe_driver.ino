/*
 * strobe_driver.ino -- Teensy 4.1 flashing-light driver with randomized trials
 *
 * READ THIS FIRST
 *   Flashing light between roughly 3 and 60 Hz can set off a seizure in people
 *   with photosensitive epilepsy, including people who have never had one.
 *   Red flicker is the most provoking kind. Nobody with a seizure history near
 *   it, start dim, keep runs short, and the moment anyone feels odd: look away
 *   and send `stop`. The sketch lights nothing until you type `arm yes`, and it
 *   starts at 20 % brightness.
 *
 * WHAT IT DOES
 *   The light stimulator the old experiments used, plus the parts they could
 *   not check:
 *   - flash rate from a 32-bit phase accumulator: 0.1 to 100 Hz in steps of
 *     5 micro-hertz, as accurate as the crystal, edges on a 50 us grid
 *   - trials whose order is drawn by the processor's hardware random generator,
 *     each one starting and ending on an exact tick
 *   - a marker pin for every flash and for every trial, so a recording made
 *     anywhere else can be lined up with what the light did
 *   - `check`: a light sensor measures the flashes that actually came out
 *
 * WHERE THE NUMBERS COME FROM
 *   CIA-RDP96-00787R000100220001-8 and CIA-RDP96-00788R001300350001-0 (SRI):
 *     one person was flashed at 6 and 16 Hz, mixed with no-flash trials in
 *     random order, while a second person's EEG was recorded in another room.
 *     `sri` runs that layout: 36 trials of 10 s, twelve each of 0, 6 and 16 Hz.
 *     The 5 s gap between trials is a placeholder, not SRI's number.
 *   CIA-RDP96-00788R001700210004-8 (Monroe Institute paper): a 10 Hz strobe
 *     pulls brain rhythm toward 10 Hz, red light works best, and you start near
 *     the rate the person is already at and slide from there. Those are claims
 *     in the paper, not measurements. `glide` does the slide.
 *
 * WIRING (Teensy 4.1)
 *   pin 5  -> channel A. A small LED straight off the pin:
 *               pin 5 -> 470 ohm -> LED -> GND
 *             Anything brighter: pin 5 -> 100 ohm -> gate of a logic-level
 *             N-MOSFET, 10 k from gate to GND, LED and its resistor between
 *             the supply and the drain.
 *   pin 6  -> channel B, same circuit (a second colour, or the other eye)
 *   pin 3  -> flash marker: high while channel A is lit
 *   pin 4  -> trial marker: high from the first tick of a trial to the last
 *   pin 7  <- trigger in for `ext on` (3.3 V logic; hemisync_gen's pin 2 works)
 *   pin 8  <- ABORT button to GND (internal pull-up): all light off within 50 us
 *   pin 14 (A0) <- light sensor for `check`, optional: phototransistor with
 *             collector to 3.3 V and emitter to A0, 10 k and 100 nF from A0 to GND
 *   SD card in the Teensy's slot: one file per block of trials.
 *
 *   Brightness below 100 % is done by chopping the LED at 40 kHz, locked to the
 *   flash timing so every flash carries the same amount of light.
 *
 * COMMANDS (USB serial, newline-terminated)
 *   arm                      prints the warning; `arm yes` enables the outputs
 *   rate <Hz>                0.1 .. 100
 *   duty <percent>           lit part of each period, 1 .. 99 (default 50)
 *   width <ms>               fixed flash length instead, 0.05 .. 500; 0 = back to duty
 *   level <percent>          brightness 1 .. 100 (default 20)
 *   chan <a|b|both>          which outputs light (default a)
 *   phase <degrees>          channel B relative to A; 180 = the two alternate
 *   on | off                 flash continuously at the set rate
 *   glide <Hz0> <Hz1> <s>    start at Hz0 and slide to Hz1 over s seconds, then hold
 *   run <Hz> <s>             one timed trial, logged
 *   sri [trials] [s] [gap_s] SRI's layout, defaults 36 trials, 10 s, 5 s gap
 *   trials <n> <s> <gap_s> <Hz> [<Hz> ...]   any set of up to 8 rates (0 = dark),
 *                            equal numbers of each, order drawn at random
 *   blind <on|off>           on: the order goes to the SD card only, not the screen
 *   ext <on|off>             one flash per rising edge at pin 7 (`width` sets its length)
 *   check <s>                measure the real light with the sensor on A0
 *   stop                     everything off
 *   stat
 *
 * SAFETY GATE (list item #150, strobe_safety.h, PC-tested 16/16)
 *   ramp-in       light that starts after 10 s of darkness climbs from zero to the set
 *                 level over 3 s (trial rows log `ramped` = 1 when it happened)
 *   cap <pct> [yes]  brightness cap, default 50 %; above 50 % only with `yes`
 *   screen        a 60 s sweep 2 -> 55 Hz at 5 %, the person holding the abort button;
 *   screen ok     ...then this, only if nothing odd happened (kept in EEPROM);
 *   screen reset  until accepted, anything touching 2-55 Hz runs at 5 % at most
 *   Esc key / pin-8 button   abort: everything dark at once, the block is logged as aborted
 *
 * FILES ON THE SD CARD
 *   STRBnnn.CSV   one line per trial: number, start and stop time in seconds from
 *                 the start of the block, rate, flashes delivered
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62). Timing, trials, glide, check
 *   and the arm lock run tick by tick on a PC; the safety gate's logic is PC-tested
 *   (test/pc_test_safety.cpp). Not yet run on hardware. The pre-gate version is
 *   kept in backup/strobe_driver.ino.v1.
 */

#include <Arduino.h>
#include <SD.h>
#include <Entropy.h>
#include <EEPROM.h>
#include "strobe_safety.h"

const int    PIN_LED_A = 5, PIN_LED_B = 6, PIN_SYNC = 3, PIN_TRIAL = 4, PIN_EXT = 7, PIN_SENSE = 14;
const int    PIN_ABORT = 8;                    // button to GND: lights off from inside the interrupt
const int    EE_SCREEN_ADDR = 16;              // EEPROM byte: 0xA5 = screen accepted
const double TICK_HZ   = 20000.0;
const float  PWM_HZ    = 40000.0f;      // two brightness cycles per tick, off the same crystal
const int    PWM_FULL  = 4096;          // 12-bit
const float  RATE_MIN  = 0.1f, RATE_MAX = 100.0f;
const int    MAX_TRIALS = 240, MAX_RATES = 8;
const uint32_t NEVER   = 0xFFFFFFFFu;
const int    CHK_MAX   = 80000, EDGE_MAX = 2048;

// ------------------------------------------------- shared with the 20 kHz interrupt
IntervalTimer ticker;
volatile uint32_t tickNow = 0;
volatile uint32_t phase = 0, inc = 0;            // channel A is lit while phase < dutyThr
volatile uint32_t dutyThr = 0x80000000u, phaseB = 0, widthTicks = 0;
volatile uint32_t sinceA = 0, sinceB = 0;        // ticks since each channel's period began
volatile bool     lightOn = false, enA = true, enB = false, levelDirty = false, extMode = false;
volatile int      levelCode = PWM_FULL / 5;
volatile bool     outA = false, outB = false;
volatile uint32_t flashCount = 0, extLeft = 0;
volatile double   incF = 0, incStep = 0;         // glide
volatile uint32_t glideLeft = 0, glideEndInc = 0;
volatile bool     trialArmed = false, trialRunning = false, trialEnded = false;
volatile uint32_t trialStartTick = 0, trialStopTick = 0, trialInc = 0, trialFlashes = 0;
// `check` capture
DMAMEM uint16_t   chkBuf[CHK_MAX];
DMAMEM uint32_t   chkEdge[EDGE_MAX];             // tick of every flash start while capturing
volatile bool     chkOn = false, chkDone = false;
volatile uint32_t chkN = 0, chkWant = 0, chkDiv = 1, chkPhase = 0, chkEdges = 0, chkTick0 = 0;
// safety gate (#150)
strobe_safety::Ramp ramp;
volatile bool     abortHit = false, trialRamped = false;
volatile uint32_t rampRefresh = 0;
bool   screened = false, screenRunning = false;
float  capPct = strobe_safety::DEFAULT_CAP_PCT, levelBeforeScreen = 20.0f;
elapsedMillis screenClock;

// ------------------------------------------------- main-loop state
bool   armed = false, blindMode = false, sdOk = false;
float  rateSet = 10.0f, dutyPct = 50.0f, widthMs = 0.0f, levelPct = 20.0f, phaseDeg = 0.0f;
bool   blockOn = false, blockBlind = false;
int    nTrials = 0, trialIdx = 0, nRates = 0;
float  rateList[MAX_RATES];
uint8_t order[MAX_TRIALS];
uint32_t trialTicks = 0, restTicks = 0, blockTick0 = 0;
char   blockFile[16] = "";

// =================================================================== interrupt
FASTRUN void tickISR() {
  uint32_t t = tickNow;
  // abort button: everything dark at once, the main loop tidies up and reports
  if (digitalReadFast(PIN_ABORT) == LOW && (lightOn || extMode || trialArmed)) {
    lightOn = false; trialArmed = false; trialRunning = false; extLeft = 0; glideLeft = 0;
    analogWrite(PIN_LED_A, 0); analogWrite(PIN_LED_B, 0);
    digitalWriteFast(PIN_SYNC, LOW); digitalWriteFast(PIN_TRIAL, LOW);
    outA = outB = false;
    abortHit = true;
  }
  if (trialArmed) {
    if (!trialRunning && (int32_t)(t - trialStartTick) >= 0) {
      ramp.start();
      trialRamped = ramp.ramping();
      trialRunning = true;
      phase = 0;
      sinceA = 0;
      sinceB = phaseB ? NEVER : 0;
      inc = trialInc;
      lightOn = (trialInc != 0);
      flashCount = 0;
      digitalWriteFast(PIN_TRIAL, HIGH);
    }
    if (trialRunning && (int32_t)(t - trialStopTick) >= 0) {
      trialRunning = false;
      trialArmed = false;
      lightOn = false;
      trialFlashes = flashCount;
      trialEnded = true;
      digitalWriteFast(PIN_TRIAL, LOW);
    }
  }
  bool a = false, b = false;
  if (extMode) {
    if (extLeft) { extLeft--; a = b = true; }
  } else if (lightOn && inc) {
    uint32_t ph = phase, pb = ph + phaseB;
    if (widthTicks) { a = sinceA < widthTicks; b = sinceB < widthTicks; }
    else            { a = ph < dutyThr;        b = pb < dutyThr; }
    if (glideLeft) {
      incF += incStep;
      inc = (uint32_t)incF;
      if (--glideLeft == 0) inc = glideEndInc;
    }
    uint32_t nph = ph + inc;
    if (nph < ph) sinceA = 0; else if (sinceA != NEVER) sinceA++;
    if ((uint32_t)(nph + phaseB) < pb) sinceB = 0; else if (sinceB != NEVER) sinceB++;
    phase = nph;
  }
  if (a && !outA) {
    flashCount++;
    if (chkOn && chkEdges < (uint32_t)EDGE_MAX) chkEdge[chkEdges++] = t;
  }
  // ramp-in: the drive level climbs over 3 s after a long dark spell; refresh the
  // PWM every 5 ms while it climbs so a long flash brightens smoothly too
  ramp.tick((lightOn && inc) || extMode);
  if (ramp.ramping() && ++rampRefresh >= 100) { rampRefresh = 0; levelDirty = true; }
  int lv = ramp.level(levelCode);
  if (a != outA || levelDirty) {
    analogWrite(PIN_LED_A, (a && enA) ? lv : 0);
    digitalWriteFast(PIN_SYNC, a);
  }
  if (b != outB || levelDirty) analogWrite(PIN_LED_B, (b && enB) ? lv : 0);
  outA = a;
  outB = b;
  levelDirty = false;
  if (chkOn && ++chkPhase >= chkDiv) {
    chkPhase = 0;
    if (chkN == 0) chkTick0 = t;
    chkBuf[chkN] = (uint16_t)analogRead(PIN_SENSE);
    if (++chkN >= chkWant) { chkOn = false; chkDone = true; }
  }
  tickNow = t + 1;
}

void extISR() { extLeft = widthTicks ? widthTicks : 200; }     // 10 ms unless `width` says otherwise

// =================================================================== helpers
uint32_t incFor(double hz) { return (uint32_t)llround(hz / TICK_HZ * 4294967296.0); }
double   hzOf(uint32_t i)  { return (double)i * TICK_HZ / 4294967296.0; }

bool needArm() {
  if (armed) return false;
  Serial.println("# not armed: type `arm` and read it first");
  return true;
}

// the #150 gate: true (and a message) if [lo, hi] Hz at the current level is not allowed
bool gateRefuses(float lo, float hi) {
  if (screenRunning) return false;               // the screen itself runs at the screen level
  const char *why = strobe_safety::check(lo, hi, levelPct, capPct, screened);
  if (!why) return false;
  Serial.printf("# refused: %s\n", why);
  return true;
}

void lightStart(double hz) {
  noInterrupts();
  ramp.start();
  glideLeft = 0;
  phase = 0;
  sinceA = 0;
  sinceB = phaseB ? NEVER : 0;
  inc = incFor(hz);
  flashCount = 0;
  lightOn = true;
  interrupts();
}

void allOff() {
  noInterrupts();
  lightOn = false;
  glideLeft = 0;
  trialArmed = false;
  trialRunning = false;
  trialEnded = false;
  extLeft = 0;
  interrupts();
  digitalWriteFast(PIN_TRIAL, LOW);
}

void logLine(const char *s) {
  if (!sdOk || !blockFile[0]) return;
  File f = SD.open(blockFile, FILE_WRITE);
  if (f) { f.println(s); f.close(); }
}

void armTrial(uint32_t startTick, float hz) {
  noInterrupts();
  trialInc = incFor(hz);
  trialStartTick = startTick;
  trialStopTick = startTick + trialTicks;
  trialEnded = false;
  trialRunning = false;
  trialArmed = true;
  interrupts();
}

// =================================================================== trial blocks
void startBlock(int n, float trialS, float gapS, const float *rates, int nr, bool blind) {
  if (needArm()) return;
  if (blockOn) { Serial.println("# a block is already running: `stop` first"); return; }
  if (nr < 1 || nr > MAX_RATES) { Serial.println("# give 1 to 8 rates"); return; }
  int per = (n + nr - 1) / nr;
  if (per < 1) per = 1;
  n = per * nr;                                    // equal numbers of each rate
  if (n > MAX_TRIALS) { Serial.printf("# too many trials (limit %d)\n", MAX_TRIALS); return; }
  if (trialS < 0.5f) trialS = 0.5f;
  if (gapS < 0.5f) gapS = 0.5f;
  for (int i = 0; i < nr; i++) {
    float r = rates[i];
    rateList[i] = (r <= 0) ? 0 : constrain(r, RATE_MIN, RATE_MAX);
  }
  for (int i = 0; i < nr; i++)
    if (rateList[i] > 0 && gateRefuses(rateList[i], rateList[i])) return;
  nRates = nr;
  for (int i = 0; i < n; i++) order[i] = (uint8_t)(i % nr);
  for (int i = n - 1; i > 0; i--) {                // Fisher-Yates with the hardware generator
    int j = (int)Entropy.random((uint32_t)(i + 1));
    uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
  }
  allOff();
  if (extMode) { detachInterrupt(digitalPinToInterrupt(PIN_EXT)); extMode = false; }
  nTrials = n;
  trialIdx = 0;
  trialTicks = (uint32_t)llround((double)trialS * TICK_HZ);
  restTicks = (uint32_t)llround((double)gapS * TICK_HZ);
  blockBlind = blind;
  blockFile[0] = 0;
  if (sdOk) {
    for (int i = 0; i < 1000; i++) {
      snprintf(blockFile, sizeof(blockFile), "STRB%03d.CSV", i);
      if (!SD.exists(blockFile)) break;
    }
    char h[200];
    if (widthMs > 0) snprintf(h, sizeof(h), "# %d trials of %.3f s, gap %.3f s, flash %.2f ms, level %.0f %%, phase B %.1f deg",
                              n, trialTicks / TICK_HZ, restTicks / TICK_HZ, widthMs, levelPct, phaseDeg);
    else             snprintf(h, sizeof(h), "# %d trials of %.3f s, gap %.3f s, duty %.1f %%, level %.0f %%, phase B %.1f deg",
                              n, trialTicks / TICK_HZ, restTicks / TICK_HZ, dutyPct, levelPct, phaseDeg);
    logLine(h);
    logLine("trial,start_s,stop_s,rate_Hz,flashes,ramped");
  }
  Serial.printf("# block: %d trials of %.1f s, gap %.1f s, rates", n, trialTicks / TICK_HZ, restTicks / TICK_HZ);
  for (int i = 0; i < nr; i++) Serial.printf(" %.3f", rateList[i]);
  Serial.printf(" Hz, order %s\n", blind ? "hidden" : "shown as it runs");
  if (sdOk) Serial.printf("# log%s -> %s\n", blind ? " and key" : "", blockFile);
  else Serial.println(blind ? "# NO SD CARD: a blind block with no card leaves no key. `stop` and insert one."
                            : "# no SD card: this block is not logged");
  if (!blind) Serial.println("trial,start_s,stop_s,rate_Hz,flashes,ramped");
  blockTick0 = tickNow + (uint32_t)TICK_HZ;        // first trial starts in exactly one second
  blockOn = true;
  armTrial(blockTick0, rateList[order[0]]);
}

void serviceBlock() {
  if (!blockOn || !trialEnded) return;
  noInterrupts();
  trialEnded = false;
  uint32_t fl = trialFlashes, st = trialStartTick, sp = trialStopTick;
  bool rmp = trialRamped;
  interrupts();
  char b[96];
  // ramped = 1: this trial began after 10 s of darkness, so its first 3 s were dimmer
  snprintf(b, sizeof(b), "%d,%.5f,%.5f,%.4f,%lu,%d", trialIdx + 1, (double)(st - blockTick0) / TICK_HZ,
           (double)(sp - blockTick0) / TICK_HZ, rateList[order[trialIdx]], (unsigned long)fl, rmp ? 1 : 0);
  logLine(b);
  if (blockBlind) Serial.printf("# trial %d of %d done\n", trialIdx + 1, nTrials);
  else Serial.println(b);
  trialIdx++;
  if (trialIdx >= nTrials) {
    blockOn = false;
    Serial.printf("# block finished%s%s\n", sdOk ? ", saved in " : "", sdOk ? blockFile : "");
  } else {
    armTrial(sp + restTicks, rateList[order[trialIdx]]);
  }
}

// =================================================================== check
// Measures what the light did: rate, lit fraction, period spread, delay behind the pin.
void analyzeCheck() {
  static double rise[EDGE_MAX], fall[EDGE_MAX];
  int n = (int)chkN, ne = (int)chkEdges;
  double dt = (double)chkDiv / TICK_HZ;
  int lo = 4095, hi = 0;
  for (int i = 0; i < n; i++) {
    int v = chkBuf[i];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  int span = hi - lo;
  Serial.printf("# check: %.2f s, one sample every %.0f us, sensor codes %d..%d\n", n * dt, dt * 1e6, lo, hi);
  if (ne >= 2)
    Serial.printf("# pin    %.4f Hz (%d flashes started)\n",
                  (double)(ne - 1) * TICK_HZ / (double)(chkEdge[ne - 1] - chkEdge[0]), ne);
  else
    Serial.printf("# pin    %d flashes started: too few to time\n", ne);
  if (span < 40) {
    Serial.printf("# light  no flashing seen at the sensor (it moved by only %d codes)\n", span);
    return;
  }
  double mid = lo + 0.5 * span, h1 = lo + 0.35 * span, h2 = lo + 0.65 * span;
  int nr = 0, nf = 0;
  bool above = chkBuf[0] >= mid;
  for (int i = 1; i < n; i++) {
    int v = chkBuf[i];
    if (!above && v >= h2) {
      int j = i;
      while (j > 1 && chkBuf[j - 1] >= mid) j--;
      double v0 = chkBuf[j - 1], v1 = chkBuf[j];
      if (nr < EDGE_MAX) rise[nr++] = (j - 1) + ((v1 > v0) ? (mid - v0) / (v1 - v0) : 0.5);
      above = true;
    } else if (above && v <= h1) {
      int j = i;
      while (j > 1 && chkBuf[j - 1] <= mid) j--;
      double v0 = chkBuf[j - 1], v1 = chkBuf[j];
      if (nf < EDGE_MAX) fall[nf++] = (j - 1) + ((v0 > v1) ? (v0 - mid) / (v0 - v1) : 0.5);
      above = false;
    }
  }
  if (nr < 3) {
    Serial.printf("# light  only %d flashes seen: make the check longer\n", nr);
    return;
  }
  double total = rise[nr - 1] - rise[0], meanP = total / (nr - 1), var = 0, lit = 0;
  for (int k = 1; k < nr; k++) {
    double d = (rise[k] - rise[k - 1]) - meanP;
    var += d * d;
  }
  double spreadUs = sqrt(var / (nr - 1)) * dt * 1e6;
  for (int k = 0, m = 0; k < nr - 1; k++) {         // lit time of every complete period
    while (m < nf && fall[m] <= rise[k]) m++;
    if (m < nf && fall[m] < rise[k + 1]) lit += fall[m] - rise[k];
  }
  double lag = 0;
  int nLag = 0;
  for (int k = 0, m = 0; k < nr; k++) {             // each light edge against the pin edge before it
    double tk = (double)chkTick0 + rise[k] * chkDiv;
    while (m + 1 < ne && (double)chkEdge[m + 1] <= tk) m++;
    if (m < ne && (double)chkEdge[m] <= tk && tk - (double)chkEdge[m] < 0.5 * meanP * chkDiv) {
      lag += tk - (double)chkEdge[m];
      nLag++;
    }
  }
  Serial.printf("# light  %.4f Hz, lit %.1f %% of each period, period spread %.0f us rms", 1.0 / (meanP * dt),
                100.0 * lit / total, spreadUs);
  if (nLag) Serial.printf(", %.2f ms behind the pin", lag / nLag / TICK_HZ * 1e3);
  Serial.println();
}

void cmdCheck(float seconds) {
  if (chkOn) { Serial.println("# a check is already running"); return; }
  seconds = constrain(seconds, 0.5f, 120.0f);
  uint32_t total = (uint32_t)(seconds * TICK_HZ);
  uint32_t div = (total + CHK_MAX - 1) / CHK_MAX;
  if (div < 1) div = 1;
  noInterrupts();
  chkDiv = div;
  chkWant = total / div;
  chkN = 0;
  chkPhase = 0;
  chkEdges = 0;
  chkDone = false;
  chkOn = true;
  interrupts();
  Serial.printf("# measuring the light for %.1f s\n", seconds);
}

// =================================================================== commands
void printHelp() {
  Serial.println(F(
    "# strobe_driver commands:\n"
    "#  arm | rate <Hz> | duty <%> | width <ms> | level <%> | chan <a|b|both> | phase <deg>\n"
    "#  on | off | glide <Hz0> <Hz1> <s> | run <Hz> <s>\n"
    "#  sri [trials] [s] [gap_s] | trials <n> <s> <gap_s> <Hz> [<Hz> ...] | blind <on|off>\n"
    "#  ext <on|off> | check <s> | stop | stat\n"
    "#  safety: cap <pct> [yes] | screen | screen ok | screen reset   (Esc key or pin-8 button = abort)"));
}

void abortAll(const char *why) {
  bool was = blockOn;
  allOff();
  if (extMode) { detachInterrupt(digitalPinToInterrupt(PIN_EXT)); extMode = false; }
  if (screenRunning) { screenRunning = false; levelPct = levelBeforeScreen; levelCode = (int)lroundf(levelPct / 100.0f * PWM_FULL); levelDirty = true; }
  analogWrite(PIN_LED_A, 0);
  analogWrite(PIN_LED_B, 0);
  if (was) {
    char b[64];
    snprintf(b, sizeof(b), "# ABORTED (%s) during trial %d", why, trialIdx + 1);
    logLine(b);
    blockOn = false;
  }
  Serial.printf("# ABORTED by %s: lights off\n", why);
}

void setLevelNow(float pct) {
  levelPct = pct;
  noInterrupts();
  levelCode = (int)lroundf(levelPct / 100.0f * PWM_FULL);
  levelDirty = true;
  interrupts();
}

void serviceScreen() {
  if (!screenRunning || screenClock < 62000) return;
  allOff();
  screenRunning = false;
  setLevelNow(levelBeforeScreen);
  Serial.println(F(
    "# screen finished. If the person felt nothing unusual -- no jerks, no blank spells, no\n"
    "# nausea, no odd sensations beyond seeing the flicker -- type `screen ok`. If anything\n"
    "# at all was odd: stop here, no flicker work for that person, and see a doctor."));
}

void printStat() {
  Serial.printf("# %s, %s, set rate %.4f Hz (actual %.6f)", armed ? "ARMED" : "not armed",
                blockOn ? "BLOCK RUNNING" : extMode ? "following pin 7" : lightOn ? "FLASHING" : "dark",
                rateSet, hzOf(incFor(rateSet)));
  if (widthMs > 0) Serial.printf(", flash %.2f ms", widthMs);
  else Serial.printf(", duty %.1f %%", dutyPct);
  Serial.printf(", level %.0f %%, channel %s, B at %.1f deg, blind %s, SD %s\n", levelPct,
                (enA && enB) ? "both" : enB ? "b" : "a", phaseDeg, blindMode ? "on" : "off", sdOk ? "present" : "none");
  Serial.printf("# safety: cap %.0f %%, screen %s%s, ramp %s\n", capPct, screened ? "accepted" : "not done (2-55 Hz <= 5 %)",
                screenRunning ? " (SCREEN RUNNING)" : "", ramp.ramping() ? "climbing" : "idle");
  if (blockOn) Serial.printf("# trial %d of %d %s\n", trialIdx + 1, nTrials, trialRunning ? "running" : "waiting");
  else if (lightOn || extMode) Serial.printf("# %lu flashes since it started\n", (unsigned long)flashCount);
}

void handleLine(char *line) {
  char *tok[14];
  int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 14; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    printHelp();
  } else if (!strcmp(c, "arm")) {
    if (n >= 2 && !strcmp(tok[1], "yes")) {
      armed = true;
      Serial.printf("# armed. Brightness is %.0f %%.\n", levelPct);
    } else {
      Serial.println(F(
        "# Flashing light between about 3 and 60 Hz can trigger a seizure in people with\n"
        "# photosensitive epilepsy, including people who have never had one. Nobody with a\n"
        "# seizure history near it. Start dim. If anyone feels odd: look away, send `stop`.\n"
        "# Type `arm yes` to enable the outputs."));
    }
  } else if (!strcmp(c, "stop")) {
    bool was = blockOn;
    allOff();
    if (extMode) { detachInterrupt(digitalPinToInterrupt(PIN_EXT)); extMode = false; }
    if (was) {
      char b[64];
      snprintf(b, sizeof(b), "# stopped by hand during trial %d", trialIdx + 1);
      logLine(b);
      blockOn = false;
    }
    Serial.println("# stopped, lights off");
  } else if (!strcmp(c, "off")) {
    if (blockOn) { Serial.println("# a block is running: use `stop` to abandon it"); return; }
    allOff();
    Serial.println("# off");
  } else if (!strcmp(c, "rate") && n >= 2) {
    float want = constrain((float)atof(tok[1]), RATE_MIN, RATE_MAX);
    if (lightOn && !blockOn && gateRefuses(want, want)) return;
    rateSet = want;
    if (lightOn && !blockOn) {
      noInterrupts();
      glideLeft = 0;
      inc = incFor(rateSet);
      interrupts();
    }
    Serial.printf("# rate %.4f Hz (actual %.6f)\n", rateSet, hzOf(incFor(rateSet)));
  } else if (!strcmp(c, "duty") && n >= 2) {
    dutyPct = constrain((float)atof(tok[1]), 1.0f, 99.0f);
    widthMs = 0;
    noInterrupts();
    dutyThr = (uint32_t)((double)dutyPct / 100.0 * 4294967296.0);
    widthTicks = 0;
    interrupts();
    Serial.printf("# duty %.1f %%\n", dutyPct);
  } else if (!strcmp(c, "width") && n >= 2) {
    float w = atof(tok[1]);
    if (w <= 0) {
      widthMs = 0;
      widthTicks = 0;
      Serial.printf("# back to duty %.1f %%\n", dutyPct);
    } else {
      w = constrain(w, 0.05f, 500.0f);
      uint32_t wt = (uint32_t)lroundf(w * (float)TICK_HZ / 1000.0f);
      if (wt < 1) wt = 1;
      widthMs = wt * 1000.0f / (float)TICK_HZ;
      widthTicks = wt;
      Serial.printf("# every flash %.2f ms long\n", widthMs);
    }
  } else if (!strcmp(c, "level") && n >= 2) {
    float want = constrain((float)atof(tok[1]), 1.0f, 100.0f);
    if (want > capPct) { Serial.printf("# refused: %.0f %% is above the %.0f %% brightness cap (`cap <pct> yes`)\n", want, capPct); return; }
    if ((lightOn || extMode || blockOn) && !screened) {
      // the light is already running: apply the same rule as at start
      float lo = blockOn ? 0.1f : rateSet, hi = blockOn ? 100.0f : rateSet;
      if (extMode) { lo = 0.1f; hi = 100.0f; }
      const char *why = strobe_safety::check(lo, hi, want, capPct, screened);
      if (why) { Serial.printf("# refused: %s\n", why); return; }
    }
    setLevelNow(want);
    Serial.printf("# level %.0f %%\n", levelPct);
  } else if (!strcmp(c, "cap") && n >= 2) {
    float want = constrain((float)atof(tok[1]), 1.0f, 100.0f);
    if (want > strobe_safety::DEFAULT_CAP_PCT && !(n >= 3 && !strcmp(tok[2], "yes"))) {
      Serial.printf("# raising the cap above %.0f %% needs `cap %.0f yes`\n", strobe_safety::DEFAULT_CAP_PCT, want);
      return;
    }
    capPct = want;
    if (levelPct > capPct) setLevelNow(capPct);
    Serial.printf("# brightness cap %.0f %% (level now %.0f %%)\n", capPct, levelPct);
  } else if (!strcmp(c, "screen")) {
    if (n >= 2 && !strcmp(tok[1], "ok")) {
      screened = true;
      EEPROM.write(EE_SCREEN_ADDR, 0xA5);
      Serial.println("# screen accepted and stored: 2-55 Hz work may now go above 5 % (up to the cap)");
    } else if (n >= 2 && !strcmp(tok[1], "reset")) {
      screened = false;
      EEPROM.write(EE_SCREEN_ADDR, 0x00);
      Serial.println("# screen cleared: 2-55 Hz limited to 5 % again");
    } else {
      if (needArm()) return;
      if (blockOn || extMode || lightOn) { Serial.println("# busy: `stop` first"); return; }
      Serial.println(F(
        "# SCREEN: a 60 s sweep from 2 to 55 Hz at 5 % brightness, starting dim (3 s ramp).\n"
        "# The person sits in normal room light, looks at the light from 1 m, and holds the\n"
        "# abort button (pin 8). Anyone with a seizure history, or a family history of\n"
        "# photosensitive epilepsy, does not do this. Someone else watches them."));
      levelBeforeScreen = levelPct;
      setLevelNow(strobe_safety::SCREEN_LEVEL_PCT);
      uint32_t i0 = incFor(2.0), i1 = incFor(55.0), ticks = (uint32_t)llround(60.0 * TICK_HZ);
      noInterrupts();
      ramp.start();
      phase = 0; sinceA = 0; sinceB = phaseB ? NEVER : 0; flashCount = 0;
      inc = i0; incF = (double)i0; incStep = ((double)i1 - (double)i0) / (double)ticks;
      glideEndInc = i1; glideLeft = ticks; lightOn = true;
      interrupts();
      screenRunning = true;
      screenClock = 0;
    }
  } else if (!strcmp(c, "chan") && n >= 2) {
    bool a = !strcmp(tok[1], "a") || !strcmp(tok[1], "both"), b = !strcmp(tok[1], "b") || !strcmp(tok[1], "both");
    if (!a && !b) { Serial.println("# chan a | b | both"); return; }
    noInterrupts();
    enA = a;
    enB = b;
    levelDirty = true;
    interrupts();
    Serial.printf("# channel %s\n", (a && b) ? "both" : b ? "b" : "a");
  } else if (!strcmp(c, "phase") && n >= 2) {
    phaseDeg = fmodf((float)atof(tok[1]), 360.0f);
    if (phaseDeg < 0) phaseDeg += 360.0f;
    phaseB = (uint32_t)((double)phaseDeg / 360.0 * 4294967296.0);
    Serial.printf("# channel B %.1f degrees after A (takes effect at the next start)\n", phaseDeg);
  } else if (!strcmp(c, "on")) {
    if (needArm()) return;
    if (blockOn) { Serial.println("# a block is running"); return; }
    if (extMode) { Serial.println("# following pin 7: `ext off` first"); return; }
    if (gateRefuses(rateSet, rateSet)) return;
    lightStart(rateSet);
    Serial.printf("# flashing at %.4f Hz, level %.0f %%\n", rateSet, levelPct);
  } else if (!strcmp(c, "glide") && n >= 4) {
    if (needArm()) return;
    if (blockOn || extMode) { Serial.println("# busy: `stop` first"); return; }
    float f0 = constrain((float)atof(tok[1]), RATE_MIN, RATE_MAX), f1 = constrain((float)atof(tok[2]), RATE_MIN, RATE_MAX);
    float s = constrain((float)atof(tok[3]), 1.0f, 7200.0f);
    if (gateRefuses(f0, f1)) return;
    uint32_t i0 = incFor(f0), i1 = incFor(f1), ticks = (uint32_t)llround((double)s * TICK_HZ);
    noInterrupts();
    ramp.start();
    phase = 0;
    sinceA = 0;
    sinceB = phaseB ? NEVER : 0;
    flashCount = 0;
    inc = i0;
    incF = (double)i0;
    incStep = ((double)i1 - (double)i0) / (double)ticks;
    glideEndInc = i1;
    glideLeft = ticks;
    lightOn = true;
    interrupts();
    rateSet = f1;
    Serial.printf("# sliding %.3f -> %.3f Hz over %.0f s, then holding\n", f0, f1, s);
  } else if (!strcmp(c, "run") && n >= 3) {
    float r = atof(tok[1]);
    startBlock(1, atof(tok[2]), 0.5f, &r, 1, false);
  } else if (!strcmp(c, "sri")) {
    const float r[3] = {0.0f, 6.0f, 16.0f};
    startBlock(n >= 2 ? atoi(tok[1]) : 36, n >= 3 ? atof(tok[2]) : 10.0f, n >= 4 ? atof(tok[3]) : 5.0f, r, 3, blindMode);
  } else if (!strcmp(c, "trials") && n >= 5) {
    float r[MAX_RATES];
    int nr = 0;
    for (int i = 4; i < n && nr < MAX_RATES; i++) r[nr++] = atof(tok[i]);
    startBlock(atoi(tok[1]), atof(tok[2]), atof(tok[3]), r, nr, blindMode);
  } else if (!strcmp(c, "blind") && n >= 2) {
    blindMode = !strcmp(tok[1], "on");
    Serial.printf("# blind %s\n", blindMode ? "on: the next block's order goes to the SD card only" : "off");
  } else if (!strcmp(c, "ext") && n >= 2) {
    if (!strcmp(tok[1], "on")) {
      if (needArm()) return;
      if (blockOn) { Serial.println("# a block is running"); return; }
      if (gateRefuses(0.1f, 100.0f)) return;        // the trigger can come at any rate
      allOff();
      flashCount = 0;
      noInterrupts();
      ramp.start();
      interrupts();
      extMode = true;
      attachInterrupt(digitalPinToInterrupt(PIN_EXT), extISR, RISING);
      Serial.printf("# one %.2f ms flash per rising edge at pin %d\n", widthTicks ? widthMs : 10.0f, PIN_EXT);
    } else {
      detachInterrupt(digitalPinToInterrupt(PIN_EXT));
      extMode = false;
      extLeft = 0;
      Serial.println("# ext off");
    }
  } else if (!strcmp(c, "check")) {
    cmdCheck(n >= 2 ? (float)atof(tok[1]) : 4.0f);
  } else if (!strcmp(c, "stat")) {
    printStat();
  } else {
    Serial.println("# unknown command, type help");
  }
}

// =================================================================== setup/loop
void setup() {
  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWriteFast(PIN_LED_A, LOW);
  digitalWriteFast(PIN_LED_B, LOW);
  pinMode(PIN_SYNC, OUTPUT);
  pinMode(PIN_TRIAL, OUTPUT);
  digitalWriteFast(PIN_SYNC, LOW);
  digitalWriteFast(PIN_TRIAL, LOW);
  pinMode(PIN_EXT, INPUT_PULLDOWN);
  pinMode(PIN_ABORT, INPUT_PULLUP);
  screened = (EEPROM.read(EE_SCREEN_ADDR) == 0xA5);
  analogWriteResolution(12);
  analogWriteFrequency(PIN_LED_A, PWM_HZ);
  analogWriteFrequency(PIN_LED_B, PWM_HZ);
  analogWrite(PIN_LED_A, 0);
  analogWrite(PIN_LED_B, 0);
  analogReadResolution(12);
  analogReadAveraging(1);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Entropy.Initialize();
  sdOk = SD.begin(BUILTIN_SDCARD);
  ticker.priority(32);
  ticker.begin(tickISR, 1000000.0 / TICK_HZ);
  Serial.println("# strobe_driver -- flashing-light driver. Outputs are locked until you type `arm`.");
  Serial.printf("# safety: brightness cap %.0f %%, screen %s, abort = Esc or the pin-8 button\n",
                capPct, screened ? "accepted earlier" : "NOT done (2-55 Hz limited to 5 %)");
  printHelp();
}

void loop() {
  static char line[160];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 27) {                               // Esc: abort at once, no Enter needed
      len = 0;
      abortAll("Esc key");
      continue;
    }
    if (ch == '\n' || ch == '\r') {
      if (len > 0) {
        line[len] = 0;
        len = 0;
        handleLine(line);
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
  if (abortHit) {
    abortHit = false;
    abortAll("the abort button");
  }
  serviceScreen();
  serviceBlock();
  if (chkDone) {
    chkDone = false;
    analyzeCheck();
  }
}
