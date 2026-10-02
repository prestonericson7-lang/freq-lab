/*
 * rf_resonance.ino -- RF resonance probe & passive-cavity demodulator
 *
 * WHAT IT DOES
 *   Two SX1262 boards form a transmit/receive pair for finding radio-frequency
 *   resonances in objects. The TX board outputs a clean CW carrier; the RX
 *   board measures signal strength at that frequency.  Sweeping with an object
 *   between the two antennas reveals where it absorbs or re-radiates energy.
 *
 *   CAVITY mode streams RSSI at audio rate while the TX board illuminates a
 *   passive resonant structure (a tuned loop, dipole, or any conductive
 *   cavity).  Sound waves vibrating the structure shift its resonance and
 *   modulate the re-radiated signal -- the RSSI stream IS the audio.  This is
 *   the principle behind the 1945 Great Seal bug.
 *
 * BOARDS
 *   Written for the PDDAXLQUE DX-LR20 LoRa dev kit (2x STM32 + SX1262,
 *   900 MHz) but works on any Arduino-compatible board with an SX1262 or
 *   LLCC68 via RadioLib.  Change the four pin defines below.
 *
 * WIRING
 *   SX1262 via SPI (change PIN_* defines for your board):
 *     NSS   -> PIN_CS    (default PA4 on STM32, 10 on Teensy/ESP32)
 *     DIO1  -> PIN_DIO1
 *     NRST  -> PIN_RST
 *     BUSY  -> PIN_BUSY
 *     MOSI / MISO / SCK -> board SPI defaults
 *
 *   Two boards, two antennas, facing each other with room for the test object.
 *   For cavity mode the TX antenna illuminates the cavity; the RX antenna
 *   picks up the re-radiated signal.
 *
 * COMMANDS (USB serial, 115200 8N1, newline-terminated)
 *   help
 *   mode tx | rx             operating mode (default: rx)
 *   freq <MHz>               150.0 .. 960.0
 *   power <dBm>              -9 .. +22 (TX only, default 10)
 *   bw <kHz>                 RX bandwidth: 7.8 10.4 15.6 20.8 31.25
 *                            41.7 62.5 125 250 500 (default 62.5)
 *   cw                       start CW (TX mode)
 *   off                      stop TX or RX
 *   rssi [N] [dwell_ms]      read RSSI, averaged over N readings
 *   sweep <f0> <f1> <pts> [dwell_ms]
 *                            freq sweep, output csv: freq_MHz,rssi_dBm
 *   peak                     peaks in last sweep (>6 dB above neighbours)
 *   baseline                 mark last sweep as baseline (no object)
 *   compare <f0> <f1> <pts> [dwell_ms]
 *                            sweep, subtract baseline, show difference
 *   cavity <MHz> <rate_Hz> [seconds]
 *                            high-rate RSSI at fixed freq (passive cavity)
 *   track <MHz> [seconds]    continuous RSSI at one frequency
 *   stat                     radio temperature, battery, errors
 *
 * DOCUMENTS
 *   Compromise of the Great Seal (CIA) -- passive-cavity principle
 *   The Thing, technical analysis (Crypto Museum)
 *   DIA 1975 (CIA-RDP96-00787R000500420001-2) -- field at 3 m
 *   SRI Geophysical Effects (CIA-RDP96-00788R001800300001-0) -- RF survey
 *
 * STATUS: compiles with arduino-cli, STM32 core 3.0.0 + RadioLib 7.7.1, for the
 *         BluePill F103C8 (63.8 KB, 97 % of flash: little room to grow) and the
 *         BlackPill F411CE (12 %). Core 3.x needs RadioLib's pin casts: build with
 *         build.bat in this folder. Not run on hardware. Pin defines may need
 *         updating for your board.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// --------------------------------------------------------------- pin mapping
// DX-LR20 (STM32) defaults -- change to match your board
#if defined(ARDUINO_ARCH_STM32)
  #define PIN_CS    PA4
  #define PIN_DIO1  PB1
  #define PIN_RST   PB0
  #define PIN_BUSY  PA3
#elif defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
  #define PIN_CS    10
  #define PIN_DIO1  3
  #define PIN_RST   5
  #define PIN_BUSY  4
#else                                       // ESP32 / generic
  #define PIN_CS    10
  #define PIN_DIO1  3
  #define PIN_RST   5
  #define PIN_BUSY  4
#endif

SX1262 radio = new Module(PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

// --------------------------------------------------------------- globals
const float  FREQ_MIN = 150.0f;
const float  FREQ_MAX = 960.0f;
// STM32F103C8 has only 20 KB RAM; 4 arrays × 500 × 4 B = 8 KB → fits.
// Increase to 2000 if your board has ≥64 KB RAM (F411, L476, WLE5, etc.)
const int    MAX_SWEEP = 500;

float  curFreq    = 868.0f;
float  curPower   = 10.0f;       // dBm
float  curBW      = 62.5f;       // kHz
bool   isTX       = false;       // false = RX mode
bool   cwActive   = false;

// sweep storage
float  sweepFreq[MAX_SWEEP];
float  sweepRSSI[MAX_SWEEP];
int    sweepPts   = 0;

// baseline
float  baseFreq[MAX_SWEEP];
float  baseRSSI[MAX_SWEEP];
int    basePts    = 0;
bool   hasBase    = false;

// command buffer
char   cmdBuf[256];
int    cmdLen     = 0;

// --------------------------------------------------------------- helpers
void pr(const char *s) { Serial.println(s); }
void prf(const char *fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print(buf);
}

int radioErr(int code, const char *what) {
  if (code != RADIOLIB_ERR_NONE) {
    prf("ERROR %s: %d\r\n", what, code);
  }
  return code;
}

// ----------------------------------------------------------- radio helpers
int initRadio() {
  // LoRa mode init, then we switch to FSK/direct for CW
  int r = radio.begin(curFreq, 125.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, curPower, 8, 1.6, false);
  if (r != RADIOLIB_ERR_NONE) return r;
  radio.setDio2AsRfSwitch(true);
  return RADIOLIB_ERR_NONE;
}

int setFreq(float mhz) {
  if (mhz < FREQ_MIN || mhz > FREQ_MAX) {
    prf("ERROR: freq %.3f out of range [%.0f, %.0f]\r\n", mhz, FREQ_MIN, FREQ_MAX);
    return -1;
  }
  curFreq = mhz;
  return radioErr(radio.setFrequency(mhz), "setFrequency");
}

int setPower(float dbm) {
  int p = (int)constrain(dbm, -9.0f, 22.0f);
  curPower = (float)p;
  return radioErr(radio.setOutputPower(p), "setOutputPower");
}

// read instantaneous RSSI while in RX continuous mode
float readRSSI() {
  // SX1262 instantaneous RSSI register
  return radio.getRSSI(true);
}

// switch to FSK RX for RSSI measurement
int enterRX() {
  radio.standby();
  // Reset to FSK mode for raw RSSI measurement
  int r = radio.beginFSK(curFreq, 4.8, 5.0, curBW, curPower, 8, 1.6, false);
  if (r != RADIOLIB_ERR_NONE) return radioErr(r, "beginFSK");
  radio.setDio2AsRfSwitch(true);
  r = radio.startReceive();
  return radioErr(r, "startReceive");
}

// start CW transmission
int startCW() {
  radio.standby();
  int r = radio.beginFSK(curFreq, 4.8, 5.0, 10.0, curPower, 8, 1.6, false);
  if (r != RADIOLIB_ERR_NONE) return radioErr(r, "beginFSK for CW");
  radio.setDio2AsRfSwitch(true);
  r = radio.transmitDirect();
  if (r == RADIOLIB_ERR_NONE) cwActive = true;
  return radioErr(r, "transmitDirect");
}

void stopRadio() {
  cwActive = false;
  radio.standby();
}

// ----------------------------------------------------------- measurement
void cmdRSSI(int samples, int dwell_ms) {
  if (isTX) { pr("switch to rx first"); return; }
  if (samples < 1) samples = 1;
  if (dwell_ms < 1) dwell_ms = 5;

  if (enterRX() != RADIOLIB_ERR_NONE) return;
  delay(dwell_ms);                       // let the RX settle

  float sum = 0;
  float mn  = 999.0f, mx = -999.0f;
  for (int i = 0; i < samples; i++) {
    float r = readRSSI();
    sum += r;
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    if (i < samples - 1) delay(dwell_ms);
  }
  float avg = sum / (float)samples;
  if (samples == 1) {
    prf("%.3f MHz  rssi %.1f dBm\r\n", curFreq, avg);
  } else {
    prf("%.3f MHz  rssi avg %.1f  min %.1f  max %.1f  (%d samples)\r\n",
        curFreq, avg, mn, mx, samples);
  }
  radio.standby();
}

// ----------------------------------------------------------- sweep
void cmdSweep(float f0, float f1, int pts, int dwell_ms, bool show) {
  if (isTX) { pr("switch to rx first"); return; }
  if (pts < 2) pts = 2;
  if (pts > MAX_SWEEP) pts = MAX_SWEEP;
  if (dwell_ms < 1) dwell_ms = 5;
  if (f0 < FREQ_MIN) f0 = FREQ_MIN;
  if (f1 > FREQ_MAX) f1 = FREQ_MAX;
  if (f0 > f1) { float t = f0; f0 = f1; f1 = t; }

  float step = (f1 - f0) / (float)(pts - 1);

  if (show) prf("# sweep %.3f -> %.3f MHz, %d pts, %d ms dwell\r\n", f0, f1, pts, dwell_ms);
  if (show) pr("freq_MHz,rssi_dBm");

  // init RX
  if (enterRX() != RADIOLIB_ERR_NONE) return;

  sweepPts = pts;
  for (int i = 0; i < pts; i++) {
    // check for abort
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 27 || c == 'q' || c == 'Q') {
        pr("# aborted");
        sweepPts = i;
        break;
      }
    }

    float f = f0 + step * (float)i;
    radio.setFrequency(f);
    delay(dwell_ms);

    // average a few samples for stability
    float sum = 0;
    const int avg = 4;
    for (int j = 0; j < avg; j++) {
      sum += readRSSI();
      if (j < avg - 1) delayMicroseconds(500);
    }
    float rssi = sum / (float)avg;

    sweepFreq[i] = f;
    sweepRSSI[i] = rssi;

    if (show) prf("%.4f,%.1f\r\n", f, rssi);
  }

  radio.standby();
  if (show) prf("# done, %d pts\r\n", sweepPts);
}

// ----------------------------------------------------------- peaks
void cmdPeak() {
  if (sweepPts < 5) { pr("no sweep data (run sweep first)"); return; }

  // find local maxima that are > 6 dB above both neighbours within +-3 pts
  pr("# peaks (>6 dB above local floor)");
  pr("freq_MHz,rssi_dBm,prominence_dB");
  int found = 0;
  for (int i = 3; i < sweepPts - 3; i++) {
    float here = sweepRSSI[i];
    // local floor = average of points 3 away on each side
    float floor = 0;
    for (int j = -3; j <= 3; j++) {
      if (j != 0) floor += sweepRSSI[i + j];
    }
    floor /= 6.0f;
    float prom = here - floor;
    if (prom > 6.0f && here >= sweepRSSI[i - 1] && here >= sweepRSSI[i + 1]) {
      prf("%.4f,%.1f,%.1f\r\n", sweepFreq[i], here, prom);
      found++;
    }
  }
  if (found == 0) pr("# no peaks found");
}

// ----------------------------------------------------------- baseline
void cmdBaseline() {
  if (sweepPts < 2) { pr("run a sweep first"); return; }
  basePts = sweepPts;
  for (int i = 0; i < basePts; i++) {
    baseFreq[i] = sweepFreq[i];
    baseRSSI[i] = sweepRSSI[i];
  }
  hasBase = true;
  prf("baseline set: %d pts, %.3f -> %.3f MHz\r\n",
      basePts, baseFreq[0], baseFreq[basePts - 1]);
}

void cmdCompare(float f0, float f1, int pts, int dwell_ms) {
  if (!hasBase) { pr("no baseline (run baseline first)"); return; }
  // run sweep silently
  cmdSweep(f0, f1, pts, dwell_ms, false);
  // interpolate baseline onto sweep frequencies and subtract
  pr("# compare: sweep - baseline");
  pr("freq_MHz,rssi_dBm,baseline_dBm,diff_dB");
  for (int i = 0; i < sweepPts; i++) {
    float f = sweepFreq[i];
    // linear interpolation in baseline
    float bval = baseRSSI[0];
    for (int j = 0; j < basePts - 1; j++) {
      if (f >= baseFreq[j] && f <= baseFreq[j + 1]) {
        float t = (f - baseFreq[j]) / (baseFreq[j + 1] - baseFreq[j]);
        bval = baseRSSI[j] + t * (baseRSSI[j + 1] - baseRSSI[j]);
        break;
      }
    }
    float diff = sweepRSSI[i] - bval;
    prf("%.4f,%.1f,%.1f,%.1f\r\n", f, sweepRSSI[i], bval, diff);
  }
}

// ------------------------------------------------------- cavity (passive cavity demodulator)
void cmdCavity(float freq_mhz, float rate_hz, float seconds) {
  if (isTX) { pr("switch to rx first"); return; }
  if (rate_hz < 1.0f) rate_hz = 100.0f;
  if (rate_hz > 10000.0f) rate_hz = 10000.0f;
  if (seconds <= 0) seconds = 10.0f;

  setFreq(freq_mhz);
  if (enterRX() != RADIOLIB_ERR_NONE) return;
  delay(10);

  unsigned long interval_us = (unsigned long)(1000000.0f / rate_hz);
  unsigned long total_us    = (unsigned long)(seconds * 1000000.0f);
  unsigned long start_us    = micros();

  prf("# cavity monitor: %.4f MHz, %.1f Hz rate, %.1f s\r\n", freq_mhz, rate_hz, seconds);
  pr("time_ms,rssi_dBm");

  unsigned long next_us = start_us;
  while ((micros() - start_us) < total_us) {
    // check abort
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 27 || c == 'q' || c == 'Q') {
        Serial.read();
        pr("# aborted");
        break;
      }
    }
    // wait for next sample
    while (micros() < next_us) { /* spin */ }
    next_us += interval_us;

    float rssi = readRSSI();
    float t_ms = (float)(micros() - start_us) / 1000.0f;
    prf("%.2f,%.1f\r\n", t_ms, rssi);
  }

  radio.standby();
  pr("# done");
}

// ------------------------------------------------------- track
void cmdTrack(float freq_mhz, float seconds) {
  if (isTX) { pr("switch to rx first"); return; }
  if (seconds <= 0) seconds = 60.0f;

  setFreq(freq_mhz);
  if (enterRX() != RADIOLIB_ERR_NONE) return;

  prf("# tracking %.4f MHz for %.0f s  (any key to stop)\r\n", freq_mhz, seconds);

  unsigned long start = millis();
  unsigned long dur   = (unsigned long)(seconds * 1000.0f);
  float mn = 999.0f, mx = -999.0f;
  double sum = 0;
  long count = 0;

  while ((millis() - start) < dur) {
    if (Serial.available()) { Serial.read(); break; }
    float r = readRSSI();
    sum += r;
    count++;
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    if (count % 100 == 0) {
      prf("\r%.4f MHz  rssi %.1f  avg %.1f  [%.1f .. %.1f]  n=%ld",
          freq_mhz, r, (float)(sum / count), mn, mx, count);
    }
    delay(5);
  }
  prf("\r\n# done: avg %.1f  min %.1f  max %.1f  (%ld readings)\r\n",
      (float)(sum / count), mn, mx, count);
  radio.standby();
}

// ------------------------------------------------------- command parser
void parseLine(const char *line) {
  // skip whitespace
  while (*line == ' ') line++;
  if (*line == 0 || *line == '#') return;

  char cmd[32];
  int n = 0;
  while (*line && *line != ' ' && n < 31) cmd[n++] = *line++;
  cmd[n] = 0;
  while (*line == ' ') line++;

  // ---- help
  if (strcmp(cmd, "help") == 0) {
    pr("rf_resonance -- SX1262 resonance probe & cavity demodulator");
    pr("");
    pr("  mode tx | rx        operating mode");
    pr("  freq <MHz>          150 .. 960");
    pr("  power <dBm>         -9 .. +22 (TX)");
    pr("  bw <kHz>            RX bandwidth");
    pr("  cw                  start CW (TX)");
    pr("  off                 stop");
    pr("  rssi [N] [dwell]    read RSSI");
    pr("  sweep f0 f1 pts [dwell]");
    pr("  peak                peaks in last sweep");
    pr("  baseline            save last sweep as baseline");
    pr("  compare f0 f1 pts [dwell]");
    pr("  cavity <MHz> <rate_Hz> [seconds]");
    pr("  track <MHz> [seconds]");
    pr("  stat");
    return;
  }

  // ---- mode
  if (strcmp(cmd, "mode") == 0) {
    if (strncmp(line, "tx", 2) == 0) {
      isTX = true;
      stopRadio();
      pr("mode: TX (use cw to start carrier)");
    } else if (strncmp(line, "rx", 2) == 0) {
      isTX = false;
      stopRadio();
      pr("mode: RX");
    } else {
      prf("mode: %s\r\n", isTX ? "TX" : "RX");
    }
    return;
  }

  // ---- freq
  if (strcmp(cmd, "freq") == 0) {
    float f = atof(line);
    if (setFreq(f) == RADIOLIB_ERR_NONE) {
      prf("freq: %.4f MHz\r\n", curFreq);
      if (cwActive) startCW();               // re-start CW at new freq
    }
    return;
  }

  // ---- power
  if (strcmp(cmd, "power") == 0) {
    float p = atof(line);
    setPower(p);
    prf("power: %.0f dBm\r\n", curPower);
    return;
  }

  // ---- bw
  if (strcmp(cmd, "bw") == 0) {
    curBW = atof(line);
    prf("bw: %.1f kHz\r\n", curBW);
    return;
  }

  // ---- cw
  if (strcmp(cmd, "cw") == 0) {
    if (!isTX) { pr("set mode tx first"); return; }
    prf("CW on at %.4f MHz, %+.0f dBm\r\n", curFreq, curPower);
    startCW();
    return;
  }

  // ---- off
  if (strcmp(cmd, "off") == 0) {
    stopRadio();
    pr("radio off");
    return;
  }

  // ---- rssi
  if (strcmp(cmd, "rssi") == 0) {
    int samples = 1; int dwell = 5;
    sscanf(line, "%d %d", &samples, &dwell);
    cmdRSSI(samples, dwell);
    return;
  }

  // ---- sweep
  if (strcmp(cmd, "sweep") == 0) {
    float f0, f1; int pts = 100; int dwell = 5;
    int got = sscanf(line, "%f %f %d %d", &f0, &f1, &pts, &dwell);
    if (got < 2) { pr("usage: sweep <f0> <f1> [pts] [dwell_ms]"); return; }
    cmdSweep(f0, f1, pts, dwell, true);
    return;
  }

  // ---- peak
  if (strcmp(cmd, "peak") == 0) { cmdPeak(); return; }

  // ---- baseline
  if (strcmp(cmd, "baseline") == 0) { cmdBaseline(); return; }

  // ---- compare
  if (strcmp(cmd, "compare") == 0) {
    float f0, f1; int pts = 100; int dwell = 5;
    int got = sscanf(line, "%f %f %d %d", &f0, &f1, &pts, &dwell);
    if (got < 2) { pr("usage: compare <f0> <f1> [pts] [dwell_ms]"); return; }
    cmdCompare(f0, f1, pts, dwell);
    return;
  }

  // ---- cavity
  if (strcmp(cmd, "cavity") == 0) {
    float freq, rate = 1000.0f, sec = 10.0f;
    int got = sscanf(line, "%f %f %f", &freq, &rate, &sec);
    if (got < 1) { pr("usage: cavity <MHz> [rate_Hz] [seconds]"); return; }
    cmdCavity(freq, rate, sec);
    return;
  }

  // ---- track
  if (strcmp(cmd, "track") == 0) {
    float freq, sec = 60.0f;
    sscanf(line, "%f %f", &freq, &sec);
    if (freq < FREQ_MIN) { pr("usage: track <MHz> [seconds]"); return; }
    cmdTrack(freq, sec);
    return;
  }

  // ---- stat
  if (strcmp(cmd, "stat") == 0) {
    prf("mode:  %s\r\n", isTX ? "TX" : "RX");
    prf("freq:  %.4f MHz\r\n", curFreq);
    prf("power: %.0f dBm\r\n", curPower);
    prf("bw:    %.1f kHz\r\n", curBW);
    prf("cw:    %s\r\n", cwActive ? "ON" : "off");
    prf("sweep: %d pts\r\n", sweepPts);
    prf("base:  %s\r\n", hasBase ? "set" : "none");
    return;
  }

  prf("unknown: %s  (try help)\r\n", cmd);
}

// --------------------------------------------------------------- setup/loop
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pr("");
  pr("rf_resonance -- SX1262 resonance probe & cavity demodulator");
  pr("type help for commands");

  if (initRadio() != RADIOLIB_ERR_NONE) {
    pr("FATAL: radio init failed -- check wiring and pin defines");
  } else {
    prf("radio ready: %.1f MHz, %+.0f dBm, RX bw %.1f kHz\r\n", curFreq, curPower, curBW);
  }
  pr("mode: RX (type 'mode tx' on the transmit board)");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      parseLine(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}
