/*
 * rf_survey.ino -- Teensy 4.1 + AD8318 broadband RF power meter and envelope analyser
 *
 * WHAT IT DOES
 *   The AD8318 is a logarithmic detector: 1 MHz to 8 GHz in, a DC voltage out
 *   that is proportional to the input power in dB (about -25 mV per dB, from
 *   -60 dBm to 0 dBm). This sketch samples that voltage at 50,000 S/s and
 *   turns it into the three things a room survey needs:
 *     1. the level: mean and peak power per second in dBm, converted to
 *        uW/cm2 for the antenna you tell it about;
 *     2. the envelope spectrum: what MODULATES the RF around you -- Wi-Fi
 *        beacons every 102.4 ms (a 9.766 Hz line), LTE frames (100 Hz, 1 kHz),
 *        DECT (100 Hz), Bluetooth (1.6 kHz), PWM lights, 50/60 Hz -- from
 *        0.1 Hz to 25 kHz;
 *     3. the pulse-interval histogram: the time between RF bursts, which
 *        identifies the source even when the spectrum is messy.
 *   With a GPS it logs a position-tagged level once a second (`survey`), so
 *   a walk through a building becomes a map (tools/rf_survey_host.py).
 *
 *   The old files' "modulation window" claims (DIA 1976) and the Moscow
 *   Signal (2.5-4 GHz at 2-18 uW/cm2 on the embassy) were about exactly
 *   these quantities; this measures them. Numbers for reference: a phone
 *   at 1 m ~1-100 uW/cm2 while transmitting; a Wi-Fi router at 1 m ~0.1-1;
 *   the FCC general-population limit is 1000 uW/cm2 (1 mW/cm2) averaged
 *   over 30 min at 1.5-100 GHz. This is a survey meter, not a health device.
 *
 * WIRING (Teensy 4.1)
 *   AD8318 module: VCC 5 V, GND, VOUT -> 10 k -> pin 14 (A0), 10 nF from A0 to GND
 *   (the module's output is 0.5-2.1 V: inside the Teensy's 3.3 V range)
 *   Antenna/filter into the module's RF input (SMA). A band-pass or a horn in
 *   front selects the band; the detector itself cannot tell frequencies apart.
 *   GPS: pin 0 (RX1) <- NMEA 9600 (optional, for `survey`)
 *   pin 3: pulse marker (high while the detector is above the pulse threshold)
 *   SD card in the slot (optional)
 *
 * CALIBRATION
 *   `cal <dBm>` with a known level applied, twice at two levels at least 20 dB
 *   apart (a signal generator, or the SX1262 board at a set power through a
 *   known attenuator): the sketch fits slope and intercept. Defaults are the
 *   datasheet typicals at 2.2 GHz: -24.4 mV/dB, intercept +20 dBm.
 *   `ant <GHz> <dBi>`: the frequency and gain assumed for uW/cm2
 *   (S = P / A_eff, A_eff = G lambda^2 / 4 pi). A 2 dBi dipole at 2.44 GHz has
 *   A_eff = 19 cm2; -30 dBm received = 1 uW / 19 cm2 = 0.053 uW/cm2.
 *
 * COMMANDS (USB serial 115200, newline-terminated)
 *   help | stat
 *   level                    live line per second: mean dBm, peak dBm, uW/cm2, duty
 *   spec fast <seconds>      envelope spectrum 12 Hz .. 25 kHz (12.2 Hz bins), top lines named
 *   spec slow <seconds>      envelope spectrum 0.1 .. 100 Hz (0.061 Hz bins), top lines named
 *   pulses <seconds>         pulse-interval histogram, top intervals named
 *   thresh <dB>              pulse threshold above the running median (default 6)
 *   cal <dBm>                take a calibration point at the applied level (two needed)
 *   cal reset                back to the datasheet defaults
 *   ant <GHz> <dBi>          antenna assumed for uW/cm2 (default 2.44, 2.0)
 *   survey <on|off>          one GPS-tagged line per second to SURVEY.CSV and the serial port
 *   log <on|off>             level lines to LEVEL.CSV
 *   stop
 *
 * STATUS: written for Teensyduino 1.62 (Teensy 4.1). The pure-arithmetic parts
 *   (dBm/uW conversion, pulse-interval detection, histogram, line naming) are
 *   compiled and run on a PC by test/pc_test.cpp; see the README.
 */

#include "rf_analysis.h"

#ifndef RF_PC_TEST
#include <Arduino.h>
#include <SD.h>
#include <arm_math.h>

const int PIN_ADC = 14, PIN_MARK = 3;
const double FS = 50000.0;
const int NFAST = 4096;                      // 12.2 Hz bins, 82 ms per segment
const int DEC_SLOW = 200;                    // 50 kS/s -> 250 S/s
const int NSLOW = 4096;                      // 0.061 Hz bins, 16.4 s per segment

IntervalTimer adcTimer;
volatile uint16_t fifo[8192];
volatile uint32_t fHead = 0, fTail = 0, fDrops = 0;
volatile uint32_t nSamples = 0;

void adcISR() {
  uint16_t v = analogRead(PIN_ADC);
  uint32_t nx = (fHead + 1) % 8192;
  if (nx == fTail) fDrops++;
  else { fifo[fHead] = v; fHead = nx; }
  nSamples++;
}

RfCal cal;                                   // slope/intercept, antenna
PulseDetector pulses;
IntervalHistogram hist;
LevelAccumulator level;

// spectra
arm_rfft_fast_instance_f32 fftFast, fftSlow;
float winFast[NFAST], winSlow[NSLOW];
float ringFast[NFAST], ringSlow[NSLOW];
int   posFast = 0, posSlow = 0;
long  cntFast = 0, cntSlow = 0;
float fftIn[NFAST], fftOut[NFAST];
double PxxFast[NFAST / 2], PxxSlow[NSLOW / 2];
int   segFast = 0, segSlow = 0;
double slowAcc = 0; int slowPhase = 0;
bool  specFastOn = false, specSlowOn = false, pulsesOn = false, levelOn = false, surveyOn = false, logOn = false, sdOk = false;
elapsedMillis sinceSecond, runTimer;
uint32_t runMs = 0;
char nmea[100]; int nmeaLen = 0; bool gpsValid = false; double gpsLat = 0, gpsLon = 0; char gpsStamp[24] = ""; elapsedMillis gpsAge;

void parseNmea(char *s) {
  if (strlen(s) < 20 || s[0] != '$' || strncmp(s + 3, "RMC", 3) != 0) return;
  char *f[13]; int n = 0;
  for (char *p = s; n < 13 && p; n++) { f[n] = p; p = strchr(p, ','); if (p) *p++ = 0; }
  if (n < 10) return;
  gpsValid = (f[2][0] == 'A');
  if (gpsValid && strlen(f[3]) > 4 && strlen(f[5]) > 5) {
    double lat = atof(f[3]), lon = atof(f[5]);
    gpsLat = (int)(lat / 100) + fmod(lat, 100.0) / 60.0; if (f[4][0] == 'S') gpsLat = -gpsLat;
    gpsLon = (int)(lon / 100) + fmod(lon, 100.0) / 60.0; if (f[6][0] == 'W') gpsLon = -gpsLon;
  }
  if (strlen(f[1]) >= 6 && strlen(f[9]) >= 6) {
    snprintf(gpsStamp, sizeof(gpsStamp), "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
             f[9][4], f[9][5], f[9][2], f[9][3], f[9][0], f[9][1], f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
    gpsAge = 0;
  }
}
void pollGps() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') { if (nmeaLen > 0) { nmea[nmeaLen] = 0; nmeaLen = 0; parseNmea(nmea); } }
    else if (nmeaLen < (int)sizeof(nmea) - 1) nmea[nmeaLen++] = c;
  }
}
void timeStamp(char *buf, size_t n) {
  if (gpsValid && gpsStamp[0] && gpsAge < 3000) snprintf(buf, n, "%s", gpsStamp);
  else snprintf(buf, n, "up%lus", (unsigned long)(millis() / 1000));
}

void segment(arm_rfft_fast_instance_f32 &inst, float *ring, int pos, int nfft, float *win, double *pxx, int &nseg) {
  double mean = 0;
  for (int i = 0; i < nfft; i++) mean += ring[i];
  mean /= nfft;
  for (int i = 0; i < nfft; i++) fftIn[i] = (ring[(pos + i) % nfft] - (float)mean) * win[i];
  arm_rfft_fast_f32(&inst, fftIn, fftOut, 0);
  for (int b = 1; b < nfft / 2; b++) pxx[b] += (double)fftOut[2 * b] * fftOut[2 * b] + (double)fftOut[2 * b + 1] * fftOut[2 * b + 1];
  nseg++;
}

void printSpectrum(double *pxx, int nfft, int nseg, double fs, const char *what) {
  if (nseg == 0) { Serial.println("# no segment finished yet"); return; }
  // the envelope spectrum in dB relative to the strongest line, plus the named top lines
  int nb = nfft / 2;
  double pmax = 0;
  for (int b = 1; b < nb; b++) if (pxx[b] > pmax) pmax = pxx[b];
  Serial.printf("# %s envelope spectrum, %d averages, %.4f Hz bins. f_Hz,dB_rel\n", what, nseg, fs / nfft);
  for (int b = 1; b < nb; b++) Serial.printf("%.4f,%.1f\n", b * fs / nfft, 10 * log10(pxx[b] / pmax + 1e-12));
  LinePeak peaks[8];
  int np = findPeaks(pxx, nb, fs / nfft, peaks, 8);
  Serial.println("# strongest lines:");
  for (int i = 0; i < np; i++)
    Serial.printf("#   %9.3f Hz  %6.1f dB  %s\n", peaks[i].hz, peaks[i].db_rel, nameLine(peaks[i].hz));
}

void serviceSamples() {
  while (fTail != fHead) {
    uint16_t code = fifo[fTail];
    fTail = (fTail + 1) % 8192;
    double v = code * 3.3 / 4096.0;
    double dbm = cal.dbmOf(v);
    level.addVolts(v);
    level.add(dbm);
    bool above = pulses.feed(dbm, hist);
    digitalWriteFast(PIN_MARK, above ? HIGH : LOW);
    if (specFastOn) {
      ringFast[posFast] = (float)dbm; posFast = (posFast + 1) % NFAST; cntFast++;
      if (cntFast >= NFAST && ((cntFast - NFAST) % (NFAST / 2)) == 0) segment(fftFast, ringFast, posFast, NFAST, winFast, PxxFast, segFast);
    }
    if (specSlowOn) {
      slowAcc += dbm;
      if (++slowPhase >= DEC_SLOW) {
        slowPhase = 0;
        ringSlow[posSlow] = (float)(slowAcc / DEC_SLOW); slowAcc = 0; posSlow = (posSlow + 1) % NSLOW; cntSlow++;
        if (cntSlow >= NSLOW && ((cntSlow - NSLOW) % (NSLOW / 2)) == 0) segment(fftSlow, ringSlow, posSlow, NSLOW, winSlow, PxxSlow, segSlow);
      }
    }
  }
}

void everySecond() {
  LevelResult r = level.result(cal);
  char ts[24]; timeStamp(ts, sizeof(ts));
  if (levelOn || surveyOn) {
    Serial.printf("%s,%.2f,%.2f,%.4f,%.3f", ts, r.mean_dbm, r.peak_dbm, r.uw_cm2, r.duty);
    if (surveyOn) Serial.printf(",%.6f,%.6f,%d", gpsLat, gpsLon, gpsValid ? 1 : 0);
    Serial.println();
  }
  if (sdOk && (logOn || surveyOn)) {
    const char *name = surveyOn ? "SURVEY.CSV" : "LEVEL.CSV";
    bool isNew = !SD.exists(name);
    File f = SD.open(name, FILE_WRITE);
    if (f) {
      if (isNew) f.println(surveyOn ? "time,mean_dbm,peak_dbm,uw_cm2,duty,lat,lon,fix" : "time,mean_dbm,peak_dbm,uw_cm2,duty");
      f.printf("%s,%.2f,%.2f,%.4f,%.3f", ts, r.mean_dbm, r.peak_dbm, r.uw_cm2, r.duty);
      if (surveyOn) f.printf(",%.6f,%.6f,%d", gpsLat, gpsLon, gpsValid ? 1 : 0);
      f.println();
      f.close();
    }
  }
  level.reset();
}

void finishRun() {
  if (specFastOn) { printSpectrum(PxxFast, NFAST, segFast, FS, "fast (12 Hz - 25 kHz)"); specFastOn = false; }
  if (specSlowOn) { printSpectrum(PxxSlow, NSLOW, segSlow, FS / DEC_SLOW, "slow (0.06 - 125 Hz)"); specSlowOn = false; }
  if (pulsesOn) {
    Serial.printf("# %lu pulses in %.0f s, threshold %.1f dB above the median (%.1f dBm)\n",
                  (unsigned long)hist.count, runMs / 1000.0, pulses.thresh_db, pulses.median_dbm());
    IntervalPeak top[6];
    int n = hist.top(top, 6);
    Serial.println("# interval_ms,count,likely source");
    for (int i = 0; i < n; i++) Serial.printf("#   %9.3f,%6lu,%s\n", top[i].ms, (unsigned long)top[i].count, nameInterval(top[i].ms));
    pulsesOn = false;
  }
  runMs = 0;
}

void startRun(uint32_t seconds) {
  runMs = seconds * 1000UL;
  runTimer = 0;
  segFast = segSlow = 0; cntFast = cntSlow = 0;
  for (int b = 0; b < NFAST / 2; b++) PxxFast[b] = 0;
  for (int b = 0; b < NSLOW / 2; b++) PxxSlow[b] = 0;
  hist.reset();
}

void printHelp() {
  Serial.println(F("# rf_survey: level | spec fast|slow <s> | pulses <s> | thresh <dB> | cal <dBm>|reset | ant <GHz> <dBi> |\n"
                   "#   survey on|off | log on|off | stop | stat"));
}

void handleLine(char *line) {
  char *tok[4]; int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "level")) { levelOn = true; Serial.println("# time,mean_dbm,peak_dbm,uw_cm2,duty"); }
  else if (!strcmp(c, "spec") && n >= 3) {
    startRun(atoi(tok[2]));
    if (tok[1][0] == 'f') specFastOn = true; else specSlowOn = true;
    Serial.printf("# %s spectrum for %s s ...\n", tok[1], tok[2]);
  } else if (!strcmp(c, "pulses") && n >= 2) { startRun(atoi(tok[1])); pulsesOn = true; Serial.printf("# counting pulses for %s s ...\n", tok[1]); }
  else if (!strcmp(c, "thresh") && n >= 2) { pulses.thresh_db = atof(tok[1]); Serial.printf("# pulse threshold %.1f dB\n", pulses.thresh_db); }
  else if (!strcmp(c, "cal") && n >= 2) {
    if (!strcmp(tok[1], "reset")) { cal.reset(); Serial.println("# calibration back to datasheet typicals"); }
    else {
      double v = level.lastVolts();
      int k = cal.addPoint(atof(tok[1]), v);
      Serial.printf("# cal point %d: %.2f dBm at %.4f V", k, atof(tok[1]), v);
      if (k >= 2) Serial.printf(" -> slope %.2f mV/dB, intercept %.2f dBm", cal.slope_v_per_db * 1000, cal.intercept_dbm);
      Serial.println();
    }
  } else if (!strcmp(c, "ant") && n >= 3) { cal.setAntenna(atof(tok[1]), atof(tok[2])); Serial.printf("# antenna %.3f GHz, %.1f dBi: A_eff %.2f cm2\n", cal.freq_ghz, cal.gain_dbi, cal.aeff_cm2()); }
  else if (!strcmp(c, "survey") && n >= 2) { surveyOn = !strcmp(tok[1], "on"); if (surveyOn) Serial.println("# time,mean_dbm,peak_dbm,uw_cm2,duty,lat,lon,fix"); }
  else if (!strcmp(c, "log") && n >= 2) logOn = !strcmp(tok[1], "on");
  else if (!strcmp(c, "stop")) { levelOn = surveyOn = logOn = specFastOn = specSlowOn = pulsesOn = false; runMs = 0; Serial.println("# stopped"); }
  else if (!strcmp(c, "stat")) {
    char ts[24]; timeStamp(ts, sizeof(ts));
    Serial.printf("# %s, samples %lu, drops %lu, GPS %s, SD %s, cal slope %.2f mV/dB intercept %.1f dBm, antenna %.3f GHz %.1f dBi, last %.4f V = %.1f dBm\n",
                  ts, (unsigned long)nSamples, (unsigned long)fDrops, gpsValid ? "fix" : "no fix", sdOk ? "present" : "none",
                  cal.slope_v_per_db * 1000, cal.intercept_dbm, cal.freq_ghz, cal.gain_dbi, level.lastVolts(), cal.dbmOf(level.lastVolts()));
  } else Serial.println("# unknown command, type help");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(PIN_MARK, OUTPUT);
  analogReadResolution(12);
  analogReadAveraging(1);
  arm_rfft_fast_init_f32(&fftFast, NFAST);
  arm_rfft_fast_init_f32(&fftSlow, NSLOW);
  for (int i = 0; i < NFAST; i++) winFast[i] = 0.5f - 0.5f * cosf(TWO_PI * i / (float)NFAST);
  for (int i = 0; i < NSLOW; i++) winSlow[i] = 0.5f - 0.5f * cosf(TWO_PI * i / (float)NSLOW);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  adcTimer.priority(48);
  adcTimer.begin(adcISR, 1000000.0 / FS);
  Serial.println("# rf_survey -- AD8318 power meter / envelope analyser. Type help.");
  printHelp();
}

void loop() {
  static char line[64]; static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len > 0) { line[len] = 0; len = 0; handleLine(line); } }
    else if (len < (int)sizeof(line) - 1) line[len++] = ch;
  }
  pollGps();
  serviceSamples();
  if (sinceSecond >= 1000) { sinceSecond -= 1000; everySecond(); }
  if (runMs && runTimer >= runMs) finishRun();
}
#endif
