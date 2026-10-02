/*
 * sleep_cue.ino -- Teensy 4.1 closed-loop slow-oscillation cueing for sleep
 *                  (closed-loop auditory stimulation and targeted memory
 *                  reactivation), with a sealed word-pair split and blinded
 *                  stim/sham nights. Item #87 of the list.
 *
 * WHAT IT DOES
 *   Reads one frontal EEG channel (and a chin/temple EMG channel if present),
 *   decides every 30 s whether the sleeper is in deep (slow-wave) sleep, finds
 *   each slow-oscillation trough as it happens, and plays a cue 0.5 s after the
 *   trough -- on the up-state, where Ngo et al. (Neuron 2013) found closed-loop
 *   pink-noise clicks enhanced slow oscillations and next-morning recall, and
 *   where targeted-memory-reactivation studies (Rudoy 2009; Hu et al. 2020
 *   meta-analysis, g = 0.29) play the sounds of half of the items learned
 *   before sleep.
 *     mode click : a 50 ms pink-noise click per cue (Ngo 2013)
 *     mode tmr   : the cue sound of the next CUED pair, WAVs from the SD card
 *     mode sham  : everything runs and is logged, nothing is played
 *   `night` picks stim or sham at random for the night and hides it behind a
 *   SHA-256 commitment. `pairs` splits the learned pairs into cued/uncued with
 *   the hardware RNG, writes the split to the SD card, and prints only its
 *   commitment, so the morning test is scored blind by tools/sleep_cue_score.py.
 *
 *   Signal processing (sleep_analysis.h, PC-tested by test/pc_test.cpp, 22/22):
 *   band trackers, a 30 s-epoch deep-sleep gate (delta fraction >= 0.45, alpha
 *   <= 0.15, quiet EMG, two epochs in a row), a slow-oscillation trough detector
 *   (raw-equivalent trough <= -75 uV, filter delay of 72 ms compensated, so cues
 *   land within 10 ms of trough + 0.5 s), a cue scheduler (2.5 s refractory,
 *   at most 6 cues/min) and an arousal stop (alpha > 8 uV or EMG > 3x its
 *   quietest epoch -> no cues for 5 min).
 *
 * HARDWARE
 *   EEG: an analog biopotential front end, e.g. BioAmp EXG Pill (EEG config) or
 *   any instrumentation amp + 0.3-45 Hz band-pass with its output centred on
 *   1.65 V and within 0..3.3 V. Electrodes: Fpz (or Fp1) active, mastoid (M1)
 *   reference, other mastoid/forehead driven-right-leg or ground.
 *     front-end out -> 1 k -> pin 14 (A0), 100 nF to GND
 *   EMG (optional): a second front end, chin or temple, -> pin 15 (A1). If not
 *   connected, `emg off` uses the EEG's 20-100 Hz band for the muscle check.
 *   Audio (MQS, no audio board): pin 12 -> 470 ohm -> 10 uF -> sleep earbuds L
 *                               pin 10 -> 470 ohm -> 10 uF -> R; GND -> sleeve
 *   SD card in the Teensy's slot: CUE001.WAV..CUE200.WAV (16-bit 44.1 kHz mono,
 *   the cue sound of pair n), logs, the sealed key.
 *   pin 3: cue marker (high 50 ms at each cue) for a scope or a second logger.
 *   BATTERY POWER ONLY while electrodes are on a person (a USB power bank or
 *   a LiPo; laptop disconnected from mains, or a USB isolator).
 *
 * COMMANDS (USB serial 115200)
 *   help | stat
 *   gain <x>               front-end gain (uV at the electrodes = ADC volts / gain); default 1000
 *   emg <on|off>           use the A1 channel for EMG (default off -> EEG high band)
 *   mode <click|tmr|sham>
 *   vol <0..1>             cue volume (default 0.08: start low, raise until barely audible awake)
 *   test                   play one cue now (set the volume while awake)
 *   pairs <n>              split n pairs (2..200): writes SEALED.KEY, prints the commitment
 *   night [minutes]        start a blinded night: stim or sham drawn at random, commitment printed;
 *                          optional maximum duration (default 480)
 *   start [minutes]        unblinded start in the current mode
 *   stop
 *   reveal                 after the morning test: prints the night's condition and the pair split
 *   thresh <uV>            slow-oscillation trough threshold (default 75)
 *   live <on|off>          print each epoch's band fractions and the gate decision
 *
 * FILES
 *   NIGHTnnn.CSV  E,<t_s>,<delta>,<theta>,<alpha>,<sigma>,<emg_uV>,<deep>    per epoch
 *                 C,<t_s>,<trough_uV>,<pair or -1>                         per cue (sham: logged, not played)
 *                 A,<t_s>,<alpha_uV>,<emg_uV>                              per arousal
 *   SEALED.KEY    mask (1 = cued) : salt   -- do not open before scoring
 *   NIGHT.KEY     condition : salt         -- idem
 *   COMMITS.TXT   every commitment printed, with time
 *
 * STATUS: written for Teensyduino 1.62. Signal processing PC-tested; see README.
 *   Not yet run on hardware. Not a medical device: no claims about sleep or memory.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <Audio.h>
#include <SD.h>
#include <Entropy.h>
#include "sleep_analysis.h"

using namespace sleep;

const int PIN_EEG = 14, PIN_EMG = 15, PIN_MARK = 3;
const double FS_ADC = 1000.0;              // per channel; decimated by 4 to 250 Hz
const int DECIM = 4;

// ------------------------------------------------------------------ audio graph
AudioSynthNoisePink  pink;
AudioEffectEnvelope  clickEnv;
AudioPlaySdWav       wav;
AudioMixer4          mixL, mixR;
AudioOutputMQS       mqs;
AudioConnection      c1(pink, clickEnv);
AudioConnection      c2(clickEnv, 0, mixL, 0);
AudioConnection      c3(clickEnv, 0, mixR, 0);
AudioConnection      c4(wav, 0, mixL, 1);
AudioConnection      c5(wav, 0, mixR, 1);
AudioConnection      c6(mixL, 0, mqs, 0);
AudioConnection      c7(mixR, 0, mqs, 1);

// ------------------------------------------------------------------ sampling
IntervalTimer adcTimer;
volatile float fifoE[512], fifoM[512];
volatile uint32_t fHead = 0, fTail = 0, fDrops = 0;
Biquad aaE[2], aaM[2];                     // anti-alias low-pass at 100 Hz before decimation
volatile int decPhase = 0;
volatile int adcMinE = 4095, adcMaxE = 0;

void adcISR() {
  int e = analogRead(PIN_EEG);
  int m = analogRead(PIN_EMG);
  if (e < adcMinE) adcMinE = e;
  if (e > adcMaxE) adcMaxE = e;
  double xe = aaE[1].step(aaE[0].step((double)e));
  double xm = aaM[1].step(aaM[0].step((double)m));
  if (++decPhase >= DECIM) {
    decPhase = 0;
    uint32_t nx = (fHead + 1) % 512;
    if (nx == fTail) fDrops++;
    else { fifoE[fHead] = (float)xe; fifoM[fHead] = (float)xm; fHead = nx; }
  }
}

// ------------------------------------------------------------------ state
enum Mode { M_CLICK = 0, M_TMR = 1, M_SHAM = 2 };
const char *MODE_NAME[3] = {"click", "tmr", "sham"};
Mode mode = M_CLICK;
double gainFront = 1000.0;
bool useEmg = false, running = false, liveOn = false, sdOk = false, blindNight = false;
float vol = 0.08f;
uint32_t maxMinutes = 480;
elapsedMillis runClock;

StageGate gate;
SoDetector so;
CueScheduler sched;
BandTracker alphaT, emgT;
double dcE = 2048, dcM = 2048;             // slow DC trackers (front-end offset)
long sampleIdx = 0;

// pairs
int nPairs = 0;
uint8_t pairMask[200];
int cuedList[200], nCued = 0, cueRot = 0;
bool pairsLoaded = false;

File nightFile;
char nightName[16] = "";
int markOffAtMs = -1;
elapsedMillis markClock;

// ------------------------------------------------------------------ helpers
uint32_t hwRandom() { return Entropy.random(); }

void randomSalt(char *out, int nbytes) {
  for (int i = 0; i < nbytes; i++) { uint8_t b = hwRandom() & 0xFF; out[2 * i] = "0123456789abcdef"[b >> 4]; out[2 * i + 1] = "0123456789abcdef"[b & 15]; }
  out[2 * nbytes] = 0;
}

void logCommit(const char *what, const char *hex) {
  Serial.printf("# %s commitment (SHA-256): %s\n", what, hex);
  Serial.println("#   write it down or send it to someone now; it proves the key was fixed before the night.");
  if (sdOk) { File f = SD.open("COMMITS.TXT", FILE_WRITE); if (f) { f.printf("%lu %s %s\n", (unsigned long)(millis() / 1000), what, hex); f.close(); } }
}

double uvPerCount() { return 3.3 / 4096.0 / gainFront * 1e6; }

// The WAV player reads the SD card from the audio interrupt; writing the night log
// from loop() at the same moment can corrupt both. Every log write goes through
// here with the audio interrupt held off for the few hundred microseconds it takes.
void nightLog(const char *fmt, ...) {
  if (!nightFile) return;
  char buf[160];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  AudioNoInterrupts();
  nightFile.print(buf);
  nightFile.flush();
  AudioInterrupts();
}

// ------------------------------------------------------------------ pairs and night keys
void makePairs(int n) {
  if (n < 2 || n > 200) { Serial.println("# pairs: 2..200"); return; }
  nPairs = n;
  splitPairs(n, pairMask, hwRandom);
  char salt[33]; randomSalt(salt, 16);
  char hex[65]; commitment(pairMask, n, salt, hex);
  if (sdOk) {
    SD.remove("SEALED.KEY");
    File f = SD.open("SEALED.KEY", FILE_WRITE);
    if (f) { for (int i = 0; i < n; i++) f.print(pairMask[i] ? '1' : '0'); f.printf(":%s\n", salt); f.close(); }
  } else Serial.println("# WARNING: no SD card -- the split exists only until power-off");
  nCued = 0;
  for (int i = 0; i < n; i++) if (pairMask[i]) cuedList[nCued++] = i;
  pairsLoaded = true; cueRot = 0;
  Serial.printf("# %d pairs split: %d will be cued, %d not. Which is which is sealed.\n", n, nCued, n - nCued);
  logCommit("pair split", hex);
}

bool loadPairs() {
  if (!sdOk || !SD.exists("SEALED.KEY")) return false;
  File f = SD.open("SEALED.KEY");
  if (!f) return false;
  nPairs = 0;
  while (f.available() && nPairs < 200) { char c = f.read(); if (c == ':') break; if (c == '0' || c == '1') pairMask[nPairs++] = (c == '1'); }
  f.close();
  nCued = 0;
  for (int i = 0; i < nPairs; i++) if (pairMask[i]) cuedList[nCued++] = i;
  pairsLoaded = nPairs > 0; cueRot = 0;
  return pairsLoaded;
}

// ------------------------------------------------------------------ cues
void playCue(int pair) {
  digitalWriteFast(PIN_MARK, HIGH);
  markOffAtMs = 50; markClock = 0;
  if (mode == M_SHAM) return;
  if (mode == M_TMR && pair >= 0 && pair < 200) {
    char name[24]; snprintf(name, sizeof(name), "CUE%03d.WAV", pair + 1);
    if (sdOk && SD.exists(name)) { wav.play(name); return; }
  }
  clickEnv.noteOn();                      // 5 ms attack, 40 ms hold, 5 ms release = 50 ms click
}

// ------------------------------------------------------------------ night control
void openNightFile() {
  if (!sdOk) return;
  for (int i = 0; i < 1000; i++) { snprintf(nightName, sizeof(nightName), "NIGHT%03d.CSV", i); if (!SD.exists(nightName)) break; }
  nightFile = SD.open(nightName, FILE_WRITE);
  if (nightFile) {
    nightLog("# sleep_cue night, mode %s, blind %d, gain %.0f, thresh %.0f uV, pairs %d (%d cued)\n",
             blindNight ? "hidden" : MODE_NAME[mode], blindNight ? 1 : 0, gainFront, so.thresh_uV, nPairs, nCued);
    Serial.printf("# logging to %s\n", nightName);
  }
}

void startNight(uint32_t minutes) {
  gate = StageGate(); gate.setup(250.0, useEmg);
  double th = so.thresh_uV; so = SoDetector(); so.thresh_uV = th; so.setup(250.0);
  sched = CueScheduler(); sched.setup(250.0);
  alphaT.setup(8, 12, 250.0, 2.0); emgT.setup(20, 100, 250.0, 2.0);
  sampleIdx = 0; maxMinutes = minutes; runClock = 0;
  if (mode == M_TMR && !pairsLoaded && !loadPairs()) {
    Serial.println("# tmr mode needs a pair split: run `pairs <n>` first (or put SEALED.KEY on the card)");
    return;
  }
  openNightFile();
  running = true;
  Serial.printf("# night started (%s), max %lu min. Deep-sleep cueing begins after two qualifying epochs and any arousal pause.\n",
                blindNight ? "condition hidden" : MODE_NAME[mode], (unsigned long)minutes);
}

void stopNight() {
  if (!running) return;
  running = false;
  if (nightFile) {
    nightLog("# end: %ld cues, %ld arousals (%ld within 10 s of a cue), skipped gate %ld refractory %ld rate %ld pause %ld\n",
             sched.cues, sched.arousals, sched.arousals_after_cue, sched.skipped_gate, sched.skipped_refractory, sched.skipped_rate, sched.skipped_pause);
    AudioNoInterrupts(); nightFile.close(); AudioInterrupts();
  }
  Serial.printf("# night stopped: %ld cue events, %ld arousals (%ld within 10 s after a cue)\n", sched.cues, sched.arousals, sched.arousals_after_cue);
}

void blindNightStart(uint32_t minutes) {
  bool stim = (hwRandom() & 1) != 0;
  char salt[33]; randomSalt(salt, 16);
  char text[64]; snprintf(text, sizeof(text), "%s:%s", stim ? "stim" : "sham", salt);
  Sha256 s; s.update(text); char hex[65]; s.finish(hex);
  if (sdOk) { SD.remove("NIGHT.KEY"); File f = SD.open("NIGHT.KEY", FILE_WRITE); if (f) { f.println(text); f.close(); } }
  logCommit("night condition", hex);
  Mode want = (mode == M_SHAM) ? M_CLICK : mode;   // the active condition is the current non-sham mode
  mode = stim ? want : M_SHAM;
  blindNight = true;
  startNight(minutes);
}

void reveal() {
  if (running) { Serial.println("# stop the night first"); return; }
  if (!sdOk) { Serial.println("# no SD card"); return; }
  if (SD.exists("NIGHT.KEY")) { File f = SD.open("NIGHT.KEY"); Serial.print("# night key (condition:salt): "); while (f.available()) Serial.write(f.read()); f.close(); }
  if (SD.exists("SEALED.KEY")) { File f = SD.open("SEALED.KEY"); Serial.print("# pair key (mask:salt): "); while (f.available()) Serial.write(f.read()); f.close(); }
  Serial.println("# score with: python tools/sleep_cue_score.py recall.csv --key <pair key> --commit <commitment>");
}

// ------------------------------------------------------------------ processing
void processSample(double rawE, double rawM) {
  // remove the front end's DC slowly (tau ~ 10 s), convert to microvolts
  dcE += (rawE - dcE) * 0.0004; dcM += (rawM - dcM) * 0.0004;
  double eeg = (rawE - dcE) * uvPerCount();
  double emg = useEmg ? (rawM - dcM) * uvPerCount() : eeg;
  alphaT.step(eeg); emgT.step(emg);
  double t_s = sampleIdx / 250.0;
  if (gate.step(eeg, emg)) {
    if (gate.emg_quiet > 0) sched.emg_floor_uV = 3.0 * gate.emg_quiet;
    nightLog("E,%.1f,%.3f,%.3f,%.3f,%.3f,%.2f,%d\n", t_s, gate.ep_delta, gate.ep_theta, gate.ep_alpha, gate.ep_sigma, gate.ep_emg, gate.is_deep ? 1 : 0);
    if (liveOn) Serial.printf("# epoch %d t=%.0fs delta %.2f theta %.2f alpha %.2f sigma %.2f emg %.1f uV -> %s\n",
                              gate.epochs, t_s, gate.ep_delta, gate.ep_theta, gate.ep_alpha, gate.ep_sigma, gate.ep_emg, gate.is_deep ? "DEEP" : "-");
  }
  if (so.step(eeg)) sched.schedule(so.trough_idx);
  long arousalsBefore = sched.arousals;
  if (sched.step(gate.is_deep, alphaT.rms(), emgT.rms())) {
    int pair = -1;
    if (mode == M_TMR && nCued > 0) { pair = cuedList[cueRot % nCued]; cueRot++; }
    playCue(pair);
    nightLog("C,%.3f,%.1f,%d\n", t_s, so.trough_uV, pair);
  }
  if (sched.arousals != arousalsBefore) nightLog("A,%.1f,%.2f,%.2f\n", t_s, alphaT.rms(), emgT.rms());
  sampleIdx++;
}

// ------------------------------------------------------------------ commands
void printHelp() {
  Serial.println(F("# sleep_cue: gain <x> | emg on|off | mode click|tmr|sham | vol <0..1> | test | pairs <n> |\n"
                   "#   night [min] | start [min] | stop | reveal | thresh <uV> | live on|off | stat"));
}

void handleLine(char *line) {
  char *tok[4]; int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "gain") && n >= 2) { double g = atof(tok[1]); if (g > 0) gainFront = g; Serial.printf("# gain %.0f (%.3f uV per count)\n", gainFront, uvPerCount()); }
  else if (!strcmp(c, "emg") && n >= 2) { useEmg = !strcmp(tok[1], "on"); Serial.printf("# EMG channel %s\n", useEmg ? "A1" : "off (EEG high band)"); }
  else if (!strcmp(c, "mode") && n >= 2) {
    if (running) { Serial.println("# stop first"); return; }
    mode = !strcmp(tok[1], "tmr") ? M_TMR : !strcmp(tok[1], "sham") ? M_SHAM : M_CLICK; blindNight = false;
    Serial.printf("# mode %s\n", MODE_NAME[mode]);
  }
  else if (!strcmp(c, "vol") && n >= 2) { vol = constrain(atof(tok[1]), 0.0, 1.0); mixL.gain(0, vol); mixR.gain(0, vol); mixL.gain(1, vol); mixR.gain(1, vol); Serial.printf("# volume %.3f\n", vol); }
  else if (!strcmp(c, "test")) { Mode m = mode; if (m == M_SHAM) mode = M_CLICK; playCue(nCued ? cuedList[0] : -1); mode = m; Serial.println("# cue played"); }
  else if (!strcmp(c, "pairs") && n >= 2) makePairs(atoi(tok[1]));
  else if (!strcmp(c, "night")) { if (running) { Serial.println("# already running"); return; } blindNightStart(n >= 2 ? atoi(tok[1]) : 480); }
  else if (!strcmp(c, "start")) { if (running) { Serial.println("# already running"); return; } blindNight = false; startNight(n >= 2 ? atoi(tok[1]) : 480); }
  else if (!strcmp(c, "stop")) stopNight();
  else if (!strcmp(c, "reveal")) reveal();
  else if (!strcmp(c, "thresh") && n >= 2) { so.thresh_uV = atof(tok[1]); Serial.printf("# trough threshold %.0f uV\n", so.thresh_uV); }
  else if (!strcmp(c, "live") && n >= 2) liveOn = !strcmp(tok[1], "on");
  else if (!strcmp(c, "stat")) {
    int lo = adcMinE, hi = adcMaxE; adcMinE = 4095; adcMaxE = 0;
    Serial.printf("# %s, mode %s%s, %.1f min, epochs %d, deep %d, SO %ld, cues %ld, arousals %ld (%ld after cues), pause %s, drops %lu,\n"
                  "#   EEG codes %d..%d (%.0f uV p-p at the electrodes), SD %s, pairs %d (%d cued), filter delay %ld samples\n",
                  running ? "RUNNING" : "idle", blindNight ? "hidden" : MODE_NAME[mode], "", runClock / 60000.0, gate.epochs, gate.is_deep,
                  so.count, sched.cues, sched.arousals, sched.arousals_after_cue, sched.idx < sched.pause_until ? "ON" : "off",
                  (unsigned long)fDrops, lo, hi, (hi - lo) * uvPerCount(), sdOk ? "present" : "none", nPairs, nCued, so.delay_samples);
  } else Serial.println("# unknown command, type help");
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_MARK, OUTPUT);
  analogReadResolution(12);
  analogReadAveraging(4);
  Entropy.Initialize();
  AudioMemory(24);
  pink.amplitude(0.8f);
  clickEnv.delay(0); clickEnv.attack(5); clickEnv.hold(40); clickEnv.decay(0); clickEnv.sustain(0); clickEnv.release(5);
  for (int i = 0; i < 4; i++) { mixL.gain(i, 0); mixR.gain(i, 0); }
  mixL.gain(0, vol); mixR.gain(0, vol); mixL.gain(1, vol); mixR.gain(1, vol);
  static const double Q[2] = {0.54119610, 1.30656296};
  for (int i = 0; i < 2; i++) { aaE[i].lowpass(100.0, Q[i], FS_ADC); aaM[i].lowpass(100.0, Q[i], FS_ADC); }
  so.setup(250.0);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  if (sdOk) loadPairs();
  adcTimer.priority(64);
  adcTimer.begin(adcISR, 1000000.0 / FS_ADC);
  Serial.println("# sleep_cue -- closed-loop slow-oscillation cueing. Battery power only with electrodes on.");
  printHelp();
}

void loop() {
  static char line[64]; static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len > 0) { line[len] = 0; len = 0; handleLine(line); } }
    else if (len < (int)sizeof(line) - 1) line[len++] = ch;
  }
  while (fTail != fHead) {
    double e = fifoE[fTail], m = fifoM[fTail];
    fTail = (fTail + 1) % 512;
    if (running) processSample(e, m);
  }
  if (markOffAtMs >= 0 && markClock >= (uint32_t)markOffAtMs) { digitalWriteFast(PIN_MARK, LOW); markOffAtMs = -1; }
  if (running && runClock >= maxMinutes * 60000UL) stopNight();
}
