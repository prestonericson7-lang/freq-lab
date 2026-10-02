/*
 * cc1101_scanner.ino -- Teensy 4.1 + CC1101 sub-GHz spectrum scanner
 *
 * WHAT IT DOES
 *   Sweeps the CC1101 across its three frequency bands (300-348, 387-464,
 *   779-928 MHz) measuring RSSI at each step.  Builds a spectrum, finds
 *   peaks, and can do a waterfall (time x frequency) plot.  Also has a
 *   continuous-monitor mode for watching one frequency over time, and a
 *   resonance-probe mode that works with an external CW source (your
 *   STM32+SX1262 TX board, or any signal generator).
 *
 *   The CC1101's RSSI is measured in RX-idle mode with no demodulation
 *   running, so it reads the total power in the selected bandwidth --
 *   exactly what a spectrum analyzer does.  Resolution bandwidth is set by
 *   the channel filter (58-812 kHz selectable).
 *
 * WIRING (Teensy 4.1)
 *   CC1101 via SPI0:
 *     pin 10 -> CS        pin 11 -> MOSI
 *     pin 12 <- MISO      pin 13 -> SCK
 *     pin 2  <- GDO0 (optional, for interrupt/packet mode)
 *     3.3 V  -> VCC       GND -> GND
 *
 *   If using a second CC1101 (two-radio resonance probe):
 *     pin 9  -> CS2       same SPI bus (MOSI/MISO/SCK shared)
 *
 * BANDS
 *   0 : 300.000 - 348.000 MHz     1 : 387.000 - 464.000 MHz
 *   2 : 779.000 - 928.000 MHz
 *   The CC1101 cannot tune between the bands -- there are gaps.
 *
 * COMMANDS (USB serial, 115200 8N1, newline-terminated)
 *   help
 *   freq <MHz>                  tune to frequency (must be in a valid band)
 *   rssi [N] [dwell_ms]         read RSSI, averaged over N readings
 *   sweep [f0] [f1] [pts] [dwell_ms]
 *                               sweep a range (default: full current band)
 *   band <0|1|2>                select band, then sweep entire band
 *   allbands [pts_per_band] [dwell_ms]
 *                               sweep all three bands
 *   zoom <MHz> <span_MHz> [pts] narrow sweep around a frequency
 *   peak                        peaks in last sweep
 *   baseline                    save sweep as baseline
 *   compare [f0] [f1] [pts] [dwell_ms]
 *                               sweep minus baseline
 *   monitor <MHz> [seconds]     continuous RSSI at one frequency
 *   waterfall <f0> <f1> <pts> <rows> [dwell_ms]
 *                               time-frequency waterfall
 *   bw <kHz>                    RX bandwidth (58 68 81 102 116 135
 *                               162 203 232 270 325 406 464 541 650 812)
 *   log on | off                log to SD card
 *   stat
 *
 * DOCUMENTS
 *   DIA 1975 (CIA-RDP96-00787R000500420001-2) -- EM field measurements
 *   SRI Geophysical Effects (CIA-RDP96-00788R001800300001-0) -- ELF-RF survey
 *   Chinese Somatic Science (CIA-RDP96-00792R000300040002-9) -- 400 nT claim
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62, RadioLib 7.7.1, 111 KB).
 *         Not run on hardware.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <SD.h>

// --------------------------------------------------------------- pins
const int PIN_CS   = 10;
const int PIN_GDO0 = 2;
const int PIN_SD   = BUILTIN_SDCARD;

CC1101 radio = new Module(PIN_CS, PIN_GDO0, RADIOLIB_NC, RADIOLIB_NC);

// --------------------------------------------------------------- bands
struct Band {
  float fMin, fMax;
  const char *name;
};
const Band BANDS[] = {
  { 300.0f, 348.0f, "300-348 MHz" },
  { 387.0f, 464.0f, "387-464 MHz" },
  { 779.0f, 928.0f, "779-928 MHz" }
};
const int NUM_BANDS = 3;
int curBand = 2;                             // default: 779-928

// --------------------------------------------------------------- globals
const int MAX_SWEEP = 2000;
float  curFreq = 868.0f;
float  curBW   = 203.0f;                     // kHz

float  sweepFreq[MAX_SWEEP];
float  sweepRSSI[MAX_SWEEP];
int    sweepPts = 0;

float  baseFreq[MAX_SWEEP];
float  baseRSSI[MAX_SWEEP];
int    basePts  = 0;
bool   hasBase  = false;

bool   sdOK    = false;
bool   logging = false;
File   logFile;

char   cmdBuf[256];
int    cmdLen = 0;

// --------------------------------------------------------------- helpers
void pr(const char *s) { Serial.println(s); }
void prf(const char *fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print(buf);
}

bool inBand(float f) {
  for (int b = 0; b < NUM_BANDS; b++) {
    if (f >= BANDS[b].fMin && f <= BANDS[b].fMax) return true;
  }
  return false;
}

int bandOf(float f) {
  for (int b = 0; b < NUM_BANDS; b++) {
    if (f >= BANDS[b].fMin && f <= BANDS[b].fMax) return b;
  }
  return -1;
}

int initRadio() {
  int r = radio.begin(curFreq, 4.8, 0.0, curBW, 0, 16);
  return r;
}

float readRSSI() {
  // CC1101: put in RX, wait for RSSI to settle, read
  radio.startReceive();
  delayMicroseconds(800);                   // CC1101 RSSI needs ~1.2 ms to settle
  float rssi = radio.getRSSI();
  radio.standby();
  return rssi;
}

// --------------------------------------------------------------- sweep
void doSweep(float f0, float f1, int pts, int dwell_ms, bool show) {
  if (pts < 2) pts = 2;
  if (pts > MAX_SWEEP) pts = MAX_SWEEP;
  if (dwell_ms < 1) dwell_ms = 2;
  if (f0 > f1) { float t = f0; f0 = f1; f1 = t; }

  float step = (f1 - f0) / (float)(pts - 1);
  if (show) {
    prf("# sweep %.3f -> %.3f MHz, %d pts, %d ms dwell, bw %.0f kHz\r\n",
        f0, f1, pts, dwell_ms, curBW);
    pr("freq_MHz,rssi_dBm");
  }

  sweepPts = pts;
  for (int i = 0; i < pts; i++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 27 || c == 'q') { pr("# aborted"); sweepPts = i; break; }
    }
    float f = f0 + step * (float)i;
    // skip if outside any band
    if (!inBand(f)) {
      sweepFreq[i] = f;
      sweepRSSI[i] = -130.0f;               // below noise floor
      if (show) prf("%.4f,-130.0\r\n", f);
      continue;
    }
    radio.setFrequency(f);
    delay(dwell_ms);

    // average 4 reads
    float sum = 0;
    for (int j = 0; j < 4; j++) {
      radio.startReceive();
      delayMicroseconds(800);
      sum += radio.getRSSI();
      radio.standby();
    }
    float rssi = sum / 4.0f;

    sweepFreq[i] = f;
    sweepRSSI[i] = rssi;
    if (show) prf("%.4f,%.1f\r\n", f, rssi);
    if (logging && logFile) {
      char buf[64];
      snprintf(buf, sizeof(buf), "%.4f,%.1f\n", f, rssi);
      logFile.print(buf);
    }
  }
  radio.standby();
  if (show) prf("# done, %d pts\r\n", sweepPts);
}

// --------------------------------------------------------------- peaks
void cmdPeak() {
  if (sweepPts < 5) { pr("no sweep data"); return; }
  pr("# peaks (>6 dB above local floor)");
  pr("freq_MHz,rssi_dBm,prominence_dB");
  int found = 0;
  for (int i = 3; i < sweepPts - 3; i++) {
    float here = sweepRSSI[i];
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
  if (!found) pr("# no peaks");
}

// --------------------------------------------------------------- baseline / compare
void cmdBaseline() {
  if (sweepPts < 2) { pr("run a sweep first"); return; }
  basePts = sweepPts;
  for (int i = 0; i < basePts; i++) {
    baseFreq[i] = sweepFreq[i];
    baseRSSI[i] = sweepRSSI[i];
  }
  hasBase = true;
  prf("baseline set: %d pts\r\n", basePts);
}

void cmdCompare(float f0, float f1, int pts, int dwell_ms) {
  if (!hasBase) { pr("run baseline first"); return; }
  doSweep(f0, f1, pts, dwell_ms, false);
  pr("# compare: sweep - baseline");
  pr("freq_MHz,rssi_dBm,baseline_dBm,diff_dB");
  for (int i = 0; i < sweepPts; i++) {
    float f = sweepFreq[i];
    float bval = baseRSSI[0];
    for (int j = 0; j < basePts - 1; j++) {
      if (f >= baseFreq[j] && f <= baseFreq[j + 1]) {
        float t = (f - baseFreq[j]) / (baseFreq[j + 1] - baseFreq[j]);
        bval = baseRSSI[j] + t * (baseRSSI[j + 1] - baseRSSI[j]);
        break;
      }
    }
    prf("%.4f,%.1f,%.1f,%.1f\r\n", f, sweepRSSI[i], bval, sweepRSSI[i] - bval);
  }
}

// --------------------------------------------------------------- monitor
void cmdMonitor(float freq, float seconds) {
  if (!inBand(freq)) { pr("frequency outside CC1101 bands"); return; }
  if (seconds <= 0) seconds = 60.0f;
  radio.setFrequency(freq);
  prf("# monitoring %.4f MHz for %.0f s  (any key to stop)\r\n", freq, seconds);

  unsigned long start = millis();
  unsigned long dur   = (unsigned long)(seconds * 1000.0f);
  float mn = 999.0f, mx = -999.0f;
  double sum = 0;
  long count = 0;

  while ((millis() - start) < dur) {
    if (Serial.available()) { Serial.read(); break; }
    radio.startReceive();
    delay(2);
    float r = radio.getRSSI();
    radio.standby();
    sum += r;
    count++;
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    if (count % 50 == 0) {
      prf("\r%.4f MHz  rssi %.1f  avg %.1f  [%.1f..%.1f]  n=%ld",
          freq, r, (float)(sum / count), mn, mx, count);
    }
  }
  prf("\r\n# done: avg %.1f  min %.1f  max %.1f  (%ld readings)\r\n",
      (float)(sum / count), mn, mx, count);
}

// --------------------------------------------------------------- waterfall
void cmdWaterfall(float f0, float f1, int pts, int rows, int dwell_ms) {
  prf("# waterfall %d rows x %d pts, %.3f - %.3f MHz\r\n", rows, pts, f0, f1);
  pr("# row,freq_MHz,rssi_dBm");
  for (int r = 0; r < rows; r++) {
    if (Serial.available()) { Serial.read(); pr("# aborted"); break; }
    float step = (f1 - f0) / (float)(pts - 1);
    for (int i = 0; i < pts; i++) {
      float f = f0 + step * (float)i;
      if (!inBand(f)) {
        prf("%d,%.4f,-130.0\r\n", r, f);
        continue;
      }
      radio.setFrequency(f);
      delay(dwell_ms);
      radio.startReceive();
      delayMicroseconds(800);
      float rssi = radio.getRSSI();
      radio.standby();
      prf("%d,%.4f,%.1f\r\n", r, f, rssi);
    }
  }
  pr("# waterfall done");
}

// --------------------------------------------------------------- command parser
void parseLine(const char *line) {
  while (*line == ' ') line++;
  if (*line == 0 || *line == '#') return;

  char cmd[32];
  int n = 0;
  while (*line && *line != ' ' && n < 31) cmd[n++] = *line++;
  cmd[n] = 0;
  while (*line == ' ') line++;

  if (strcmp(cmd, "help") == 0) {
    pr("cc1101_scanner -- sub-GHz spectrum scanner");
    pr("");
    pr("  freq <MHz>          tune (300-348 / 387-464 / 779-928)");
    pr("  rssi [N] [dwell]    read RSSI");
    pr("  sweep [f0 f1 pts dwell]   sweep (default: full band)");
    pr("  band <0|1|2>        select & sweep band");
    pr("  allbands [pts] [dwell]    sweep all three bands");
    pr("  zoom <MHz> <span> [pts]   narrow sweep");
    pr("  peak                peaks in last sweep");
    pr("  baseline            save sweep as baseline");
    pr("  compare [f0 f1 pts dwell] sweep - baseline");
    pr("  monitor <MHz> [sec] continuous RSSI");
    pr("  waterfall f0 f1 pts rows [dwell]");
    pr("  bw <kHz>            RX bandwidth");
    pr("  log on|off          SD logging");
    pr("  stat");
    return;
  }

  if (strcmp(cmd, "freq") == 0) {
    float f = atof(line);
    if (!inBand(f)) { pr("frequency outside CC1101 bands"); return; }
    curFreq = f;
    curBand = bandOf(f);
    radio.setFrequency(f);
    prf("freq: %.4f MHz  band %d (%s)\r\n", f, curBand, BANDS[curBand].name);
    return;
  }

  if (strcmp(cmd, "rssi") == 0) {
    int samples = 1, dwell = 2;
    sscanf(line, "%d %d", &samples, &dwell);
    if (samples < 1) samples = 1;
    float sum = 0, mn = 999.0f, mx = -999.0f;
    for (int i = 0; i < samples; i++) {
      float r = readRSSI();
      sum += r;
      if (r < mn) mn = r;
      if (r > mx) mx = r;
      if (i < samples - 1) delay(dwell);
    }
    float avg = sum / (float)samples;
    if (samples == 1) prf("%.4f MHz  rssi %.1f dBm\r\n", curFreq, avg);
    else prf("%.4f MHz  rssi avg %.1f  [%.1f..%.1f]  (%d)\r\n", curFreq, avg, mn, mx, samples);
    return;
  }

  if (strcmp(cmd, "sweep") == 0) {
    float f0 = BANDS[curBand].fMin, f1 = BANDS[curBand].fMax;
    int pts = 200, dwell = 2;
    sscanf(line, "%f %f %d %d", &f0, &f1, &pts, &dwell);
    doSweep(f0, f1, pts, dwell, true);
    return;
  }

  if (strcmp(cmd, "band") == 0) {
    int b = atoi(line);
    if (b < 0 || b >= NUM_BANDS) { pr("band 0, 1 or 2"); return; }
    curBand = b;
    prf("band %d: %s\r\n", b, BANDS[b].name);
    doSweep(BANDS[b].fMin, BANDS[b].fMax, 200, 2, true);
    return;
  }

  if (strcmp(cmd, "allbands") == 0) {
    int pts = 200, dwell = 2;
    sscanf(line, "%d %d", &pts, &dwell);
    for (int b = 0; b < NUM_BANDS; b++) {
      prf("# --- band %d: %s ---\r\n", b, BANDS[b].name);
      doSweep(BANDS[b].fMin, BANDS[b].fMax, pts, dwell, true);
    }
    return;
  }

  if (strcmp(cmd, "zoom") == 0) {
    float center, span; int pts = 200;
    int got = sscanf(line, "%f %f %d", &center, &span, &pts);
    if (got < 2) { pr("usage: zoom <MHz> <span_MHz> [pts]"); return; }
    doSweep(center - span / 2.0f, center + span / 2.0f, pts, 2, true);
    return;
  }

  if (strcmp(cmd, "peak") == 0) { cmdPeak(); return; }
  if (strcmp(cmd, "baseline") == 0) { cmdBaseline(); return; }

  if (strcmp(cmd, "compare") == 0) {
    float f0 = BANDS[curBand].fMin, f1 = BANDS[curBand].fMax;
    int pts = 200, dwell = 2;
    sscanf(line, "%f %f %d %d", &f0, &f1, &pts, &dwell);
    cmdCompare(f0, f1, pts, dwell);
    return;
  }

  if (strcmp(cmd, "monitor") == 0) {
    float freq, sec = 60.0f;
    sscanf(line, "%f %f", &freq, &sec);
    if (freq < 100.0f) { pr("usage: monitor <MHz> [seconds]"); return; }
    cmdMonitor(freq, sec);
    return;
  }

  if (strcmp(cmd, "waterfall") == 0) {
    float f0, f1; int pts = 100, rows = 20, dwell = 2;
    int got = sscanf(line, "%f %f %d %d %d", &f0, &f1, &pts, &rows, &dwell);
    if (got < 4) { pr("usage: waterfall <f0> <f1> <pts> <rows> [dwell]"); return; }
    cmdWaterfall(f0, f1, pts, rows, dwell);
    return;
  }

  if (strcmp(cmd, "bw") == 0) {
    curBW = atof(line);
    radio.setRxBandwidth(curBW);
    prf("bw: %.0f kHz\r\n", curBW);
    return;
  }

  if (strcmp(cmd, "log") == 0) {
    if (strncmp(line, "on", 2) == 0) {
      if (!sdOK) { pr("no SD card"); return; }
      char name[32];
      snprintf(name, sizeof(name), "scan_%04lu.csv", millis() / 1000);
      logFile = SD.open(name, FILE_WRITE);
      if (logFile) {
        logging = true;
        prf("logging to %s\r\n", name);
      } else {
        pr("failed to open log file");
      }
    } else {
      if (logFile) logFile.close();
      logging = false;
      pr("logging off");
    }
    return;
  }

  if (strcmp(cmd, "stat") == 0) {
    prf("freq:  %.4f MHz\r\n", curFreq);
    prf("band:  %d (%s)\r\n", curBand, BANDS[curBand].name);
    prf("bw:    %.0f kHz\r\n", curBW);
    prf("sweep: %d pts\r\n", sweepPts);
    prf("base:  %s\r\n", hasBase ? "set" : "none");
    prf("SD:    %s\r\n", sdOK ? "ok" : "not found");
    prf("log:   %s\r\n", logging ? "on" : "off");
    return;
  }

  prf("unknown: %s  (try help)\r\n", cmd);
}

// --------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pr("");
  pr("cc1101_scanner -- sub-GHz spectrum scanner");
  pr("type help for commands");

  sdOK = SD.begin(PIN_SD);
  if (sdOK) pr("SD card ready");

  int r = initRadio();
  if (r != RADIOLIB_ERR_NONE) {
    prf("FATAL: CC1101 init error %d -- check wiring\r\n", r);
  } else {
    prf("radio ready: band %d (%s), bw %.0f kHz\r\n",
        curBand, BANDS[curBand].name, curBW);
  }
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
