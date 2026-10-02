/*
 * body_resonance.ino -- Teensy 4.1 + up to six MPU-6050s: vibration spectra,
 *                       body transfer functions and heartbeat micro-motion
 *
 * WHAT IT DOES
 *   Reads up to six accelerometers at once and gives three measurements:
 *
 *   spec  Vibration spectrum at every sensor, 0.25 - 40 Hz.
 *   tf    How much of the vibration at a reference sensor (seat, floor) reaches
 *         each other sensor (hip, chest, head), frequency by frequency: gain,
 *         phase and coherence. A gain peak above 1 is a resonance of whatever
 *         is between the two sensors. Sit on a car seat or a running washing
 *         machine and the trunk's 4 - 8 Hz resonance shows up here.
 *   bcg   The body's own micro-motion from the heartbeat (ballistocardiogram),
 *         one sensor on the chest or on a platform under a still person:
 *         heart rate, the amplitude of every heartbeat harmonic, where their
 *         envelope peaks (the body's ringing frequency), and how much of the
 *         motion sits in 6.8 - 7.5 Hz. That last number is the Gateway report's
 *         claim (CIA-RDP96-00788R001700210016-5): the body rings at 6.8 - 7.5 Hz
 *         and about three times harder in deep meditation. Record the same
 *         person resting, holding their breath, and meditating, and compare.
 *
 *   Measurement only. Nothing here shakes anybody.
 *
 * WIRING (Teensy 4.1, every MPU-6050 on 3.3 V and GND)
 *   sensor 0: SDA 18, SCL 19, AD0 low  (0x68)     sensor 1: same bus, AD0 high (0x69)
 *   sensor 2: SDA 17, SCL 16, AD0 low             sensor 3: same bus, AD0 high
 *   sensor 4: SDA 25, SCL 24, AD0 low             sensor 5: same bus, AD0 high
 *   Use whichever you need; missing sensors are skipped. Keep I2C wires short
 *   (under about 50 cm) or drop to `rate 100`.
 *   SD card in the Teensy's slot for `log` and for saved results.
 *
 * COMMANDS (USB serial, newline-terminated)
 *   scan                      list sensors found
 *   rate <50..500>            sample rate in Hz (default 250)
 *   chan <vert|x|y|z>         what is analysed: vert = along gravity, whatever
 *                             way the sensor is mounted (default)
 *   spec <seconds>            spectra of all sensors
 *   tf <ref_sensor> <seconds> transfer functions from the reference sensor
 *   bcg <sensor> <seconds> [label]   heartbeat micro-motion analysis
 *   log <seconds> [label]     raw samples to the SD card
 *   save <on|off>             also write spec / tf / bcg output to the SD card
 *   stat
 *   Any key aborts a running measurement.
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62). Not yet run on hardware.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <arm_math.h>

const int MAX_SENS = 6;
const int NFFT     = 2048;
const int HOP      = 1024;
const int NBINS    = 400;

struct Sensor {
  bool     present;
  TwoWire *bus;
  uint8_t  addr;
  float    g[3];        // slow estimate of the gravity vector, in g
};
Sensor sens[MAX_SENS];
int    nPresent = 0;
const float LSB_PER_G = 16384.0f;     // +-2 g range

float fs = 250.0f;
int   chanSel = -1;                   // -1 = vertical, 0/1/2 = x/y/z
bool  sdOk = false;
bool  saveWanted = false;

IntervalTimer tickTimer;
volatile bool     tickFlag = false;
volatile uint32_t tickMicros = 0;
volatile uint32_t tickOverruns = 0;

// ----------------------------------------------------------------- Welch engine
arm_rfft_fast_instance_f32 fftInst;
float  win[NFFT];
double winSum = 0, winSumSq = 0;
float  ring[MAX_SENS][NFFT];
int    ringPos = 0;
long   ringCount = 0;
float  fftIn[NFFT], fftOut[NFFT];
float  segRe[MAX_SENS][NBINS], segIm[MAX_SENS][NBINS];
double Pxx[MAX_SENS][NBINS];
double PxyRe[MAX_SENS][NBINS], PxyIm[MAX_SENS][NBINS];
int    nSeg = 0;
int    refSensor = -1;

// ----------------------------------------------------------------- output tee
class TeeOut : public Print {
 public:
  File f;
  bool fileOn = false;
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (fileOn) f.write(c);
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    if (fileOn) f.write(b, n);
    return n;
  }
};
TeeOut out;

void tickISR() {
  if (tickFlag) tickOverruns++;
  tickMicros = micros();
  tickFlag = true;
}

// =================================================================== sensors
bool mpuWrite(Sensor &s, uint8_t reg, uint8_t val) {
  s.bus->beginTransmission(s.addr);
  s.bus->write(reg);
  s.bus->write(val);
  return s.bus->endTransmission() == 0;
}

bool mpuRead(Sensor &s, uint8_t reg, uint8_t *buf, uint8_t n) {
  s.bus->beginTransmission(s.addr);
  s.bus->write(reg);
  if (s.bus->endTransmission(false) != 0) return false;
  if (s.bus->requestFrom(s.addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = s.bus->read();
  return true;
}

void scanSensors(bool verbose) {
  TwoWire *buses[3] = {&Wire, &Wire1, &Wire2};
  nPresent = 0;
  for (int b = 0; b < 3; b++) {
    buses[b]->begin();
    buses[b]->setClock(400000);
    for (int a = 0; a < 2; a++) {
      int idx = b * 2 + a;
      Sensor &s = sens[idx];
      s.bus = buses[b];
      s.addr = 0x68 + a;
      s.present = false;
      uint8_t who = 0;
      if (mpuRead(s, 0x75, &who, 1)) {
        mpuWrite(s, 0x6B, 0x01);        // wake, gyro-X PLL clock
        delay(5);
        mpuWrite(s, 0x1A, 0x03);        // DLPF 3: 44 Hz accelerometer bandwidth (anti-alias)
        mpuWrite(s, 0x19, 0);           // 1 kHz internal rate
        mpuWrite(s, 0x1C, 0x00);        // +-2 g
        s.present = true;
        nPresent++;
      }
      if (verbose)
        Serial.printf("# sensor %d (bus %d, 0x%02X): %s\n", idx, b, 0x68 + a,
                      s.present ? "found" : "-");
    }
  }
}

bool readAccel(Sensor &s, float a[3]) {
  uint8_t b[6];
  if (!mpuRead(s, 0x3B, b, 6)) return false;
  a[0] = (float)(int16_t)((b[0] << 8) | b[1]) / LSB_PER_G;
  a[1] = (float)(int16_t)((b[2] << 8) | b[3]) / LSB_PER_G;
  a[2] = (float)(int16_t)((b[4] << 8) | b[5]) / LSB_PER_G;
  return true;
}

// Channel value for analysis. "vert" projects onto the measured gravity
// direction and removes 1 g, so the sensor can be mounted at any angle.
float channelValue(Sensor &s, const float a[3]) {
  if (chanSel >= 0) return a[chanSel];
  float gn = sqrtf(s.g[0] * s.g[0] + s.g[1] * s.g[1] + s.g[2] * s.g[2]);
  if (gn < 0.2f) return 0;
  return (a[0] * s.g[0] + a[1] * s.g[1] + a[2] * s.g[2]) / gn - gn;
}

void startSampling() {
  tickFlag = false;
  tickOverruns = 0;
  tickTimer.begin(tickISR, 1000000.0f / fs);
}

void stopSampling() { tickTimer.end(); }

// One second of still data to seed each sensor's gravity vector.
void seedGravity() {
  float sum[MAX_SENS][3] = {};
  int cnt[MAX_SENS] = {};
  elapsedMillis t = 0;
  while (t < 1000) {
    if (!tickFlag) continue;
    tickFlag = false;
    for (int k = 0; k < MAX_SENS; k++) {
      if (!sens[k].present) continue;
      float a[3];
      if (readAccel(sens[k], a)) {
        sum[k][0] += a[0]; sum[k][1] += a[1]; sum[k][2] += a[2];
        cnt[k]++;
      }
    }
  }
  for (int k = 0; k < MAX_SENS; k++)
    if (cnt[k]) for (int i = 0; i < 3; i++) sens[k].g[i] = sum[k][i] / cnt[k];
}

bool abortRequested() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    return true;
  }
  return false;
}

// =================================================================== Welch
void welchReset(int ref) {
  ringPos = 0;
  ringCount = 0;
  nSeg = 0;
  refSensor = ref;
  for (int k = 0; k < MAX_SENS; k++)
    for (int b = 0; b < NBINS; b++) { Pxx[k][b] = 0; PxyRe[k][b] = 0; PxyIm[k][b] = 0; }
}

void welchSegment() {
  for (int k = 0; k < MAX_SENS; k++) {
    if (!sens[k].present) continue;
    double mean = 0;
    for (int i = 0; i < NFFT; i++) mean += ring[k][i];
    mean /= NFFT;
    for (int i = 0; i < NFFT; i++) {
      int j = (ringPos + i) % NFFT;                 // oldest sample first
      fftIn[i] = (ring[k][j] - (float)mean) * win[i];
    }
    arm_rfft_fast_f32(&fftInst, fftIn, fftOut, 0);
    for (int b = 1; b < NBINS; b++) {
      segRe[k][b] = fftOut[2 * b];
      segIm[k][b] = fftOut[2 * b + 1];
    }
    segRe[k][0] = segIm[k][0] = 0;
  }
  for (int k = 0; k < MAX_SENS; k++) {
    if (!sens[k].present) continue;
    for (int b = 1; b < NBINS; b++) {
      Pxx[k][b] += (double)segRe[k][b] * segRe[k][b] + (double)segIm[k][b] * segIm[k][b];
      if (refSensor >= 0) {
        double rr = segRe[refSensor][b], ri = segIm[refSensor][b];
        PxyRe[k][b] += rr * segRe[k][b] + ri * segIm[k][b];      // conj(ref) * sensor
        PxyIm[k][b] += rr * segIm[k][b] - ri * segRe[k][b];
      }
    }
  }
  nSeg++;
}

void welchPush(const float v[MAX_SENS]) {
  for (int k = 0; k < MAX_SENS; k++) ring[k][ringPos] = v[k];
  ringPos = (ringPos + 1) % NFFT;
  ringCount++;
  if (ringCount >= NFFT && ((ringCount - NFFT) % HOP) == 0) welchSegment();
}

static inline float binHz(int b) { return (float)b * fs / (float)NFFT; }

// amplitude spectral density, g per root-Hz
float asd(int k, int b) {
  if (nSeg == 0) return 0;
  return (float)sqrt(2.0 * Pxx[k][b] / ((double)nSeg * fs * winSumSq));
}

// Amplitude of a steady sine line near this bin, g peak. A Hann window spreads
// a line over neighbouring bins; summing five bins recovers its full power
// (total = 1.5 x the on-bin peak) wherever the line sits between bins.
float lineAmp(int k, int b) {
  if (nSeg == 0) return 0;
  double s = 0;
  for (int d = -2; d <= 2; d++)
    if (b + d >= 1 && b + d < NBINS) s += Pxx[k][b + d];
  return (float)(2.0 * sqrt(s / nSeg / 1.5) / winSum);
}

// =================================================================== acquisition
// mode: 0 = spectra only, 1 = raw log to SD as well
bool acquire(float seconds, bool rawLog, File *rawFile) {
  startSampling();
  seedGravity();
  elapsedMillis t = 0;
  uint32_t t0 = micros();
  uint32_t lastReport = 0;
  bool ok = true;
  while (t < (uint32_t)(seconds * 1000.0f)) {
    if (abortRequested()) { ok = false; break; }
    if (!tickFlag) continue;
    noInterrupts();
    tickFlag = false;
    uint32_t tus = tickMicros;
    interrupts();
    float v[MAX_SENS] = {};
    float a[MAX_SENS][3] = {};
    for (int k = 0; k < MAX_SENS; k++) {
      if (!sens[k].present) continue;
      if (readAccel(sens[k], a[k])) {
        for (int i = 0; i < 3; i++) sens[k].g[i] += (a[k][i] - sens[k].g[i]) * (1.0f / (5.0f * fs));
        v[k] = channelValue(sens[k], a[k]);
      }
    }
    welchPush(v);
    if (rawLog && rawFile) {
      rawFile->print((unsigned long)(tus - t0));
      for (int k = 0; k < MAX_SENS; k++) {
        if (!sens[k].present) continue;
        rawFile->printf(",%.5f,%.5f,%.5f", a[k][0], a[k][1], a[k][2]);
      }
      rawFile->println();
    }
    if (t - lastReport >= 10000) {
      lastReport = t;
      Serial.printf("# %lu s of %.0f s, %d segments\n", (unsigned long)(t / 1000), seconds, nSeg);
    }
  }
  stopSampling();
  if (tickOverruns)
    Serial.printf("# %lu samples were late or skipped (lower the rate or use fewer sensors)\n",
                  (unsigned long)tickOverruns);
  return ok;
}

bool openSave(const char *prefix, const char *label) {
  if (!saveWanted || !sdOk) return false;
  char name[48];
  for (int i = 0; i < 1000; i++) {
    if (label && label[0]) snprintf(name, sizeof(name), "%s_%03d_%s.csv", prefix, i, label);
    else snprintf(name, sizeof(name), "%s_%03d.csv", prefix, i);
    char probe[16];
    snprintf(probe, sizeof(probe), "%s_%03d", prefix, i);
    // a number is taken if any file starts with prefix_NNN
    bool taken = false;
    File root = SD.open("/");
    while (true) {
      File e = root.openNextFile();
      if (!e) break;
      if (!strncmp(e.name(), probe, strlen(probe))) taken = true;
      e.close();
      if (taken) break;
    }
    root.close();
    if (!taken) {
      out.f = SD.open(name, FILE_WRITE);
      if (out.f) {
        out.fileOn = true;
        Serial.printf("# saving to %s\n", name);
        return true;
      }
      return false;
    }
  }
  return false;
}

void closeSave() {
  if (out.fileOn) {
    out.fileOn = false;
    out.f.close();
  }
}

// =================================================================== commands
void cmdSpec(float seconds) {
  if (!nPresent) { Serial.println("# no sensors"); return; }
  welchReset(-1);
  Serial.printf("# spec: %.0f s at %.0f Hz, resolution %.3f Hz\n", seconds, fs, fs / NFFT);
  if (!acquire(seconds, false, NULL) && nSeg == 0) return;
  if (nSeg == 0) { Serial.println("# too short: need at least 8.2 s at 250 Hz"); return; }
  openSave("spec", "");
  out.printf("# amplitude spectral density in micro-g per root-Hz, %d averages\n", nSeg);
  out.print("f_Hz");
  for (int k = 0; k < MAX_SENS; k++) if (sens[k].present) out.printf(",s%d", k);
  out.println();
  int bmax = (int)(40.0f * NFFT / fs);
  if (bmax > NBINS - 1) bmax = NBINS - 1;
  for (int b = 2; b <= bmax; b++) {
    out.printf("%.3f", binHz(b));
    for (int k = 0; k < MAX_SENS; k++) if (sens[k].present) out.printf(",%.2f", asd(k, b) * 1e6f);
    out.println();
  }
  for (int k = 0; k < MAX_SENS; k++) {
    if (!sens[k].present) continue;
    out.printf("# sensor %d strongest lines:", k);
    bool used[NBINS] = {};
    for (int n = 0; n < 5; n++) {
      int best = -1;
      for (int b = 4; b < bmax; b++) {
        if (used[b]) continue;
        if (Pxx[k][b] > Pxx[k][b - 1] && Pxx[k][b] >= Pxx[k][b + 1])
          if (best < 0 || Pxx[k][b] > Pxx[k][best]) best = b;
      }
      if (best < 0) break;
      for (int b = max(0, best - 3); b <= min(NBINS - 1, best + 3); b++) used[b] = true;
      out.printf("  %.2f Hz (%.1f ug)", binHz(best), lineAmp(k, best) * 1e6f);
    }
    out.println();
  }
  closeSave();
}

void cmdTf(int ref, float seconds) {
  if (ref < 0 || ref >= MAX_SENS || !sens[ref].present) { Serial.println("# reference sensor not present"); return; }
  welchReset(ref);
  Serial.printf("# tf: reference sensor %d, %.0f s at %.0f Hz\n", ref, seconds, fs);
  if (!acquire(seconds, false, NULL) && nSeg == 0) return;
  if (nSeg < 4) { Serial.println("# too short: need at least 4 segments (about 21 s at 250 Hz) for coherence"); return; }
  openSave("tf", "");
  out.printf("# gain = sensor / reference, lag in degrees (positive = sensor lags), coherence 0..1, %d averages\n", nSeg);
  out.print("f_Hz");
  for (int k = 0; k < MAX_SENS; k++)
    if (sens[k].present && k != ref) out.printf(",gain%d,lag%d,coh%d", k, k, k);
  out.println();
  int bmax = (int)(30.0f * NFFT / fs);
  if (bmax > NBINS - 1) bmax = NBINS - 1;
  for (int b = 2; b <= bmax; b++) {
    out.printf("%.3f", binHz(b));
    for (int k = 0; k < MAX_SENS; k++) {
      if (!sens[k].present || k == ref) continue;
      double pr = Pxx[ref][b], pk = Pxx[k][b];
      double re = PxyRe[k][b], im = PxyIm[k][b];
      double gain = (pr > 0) ? sqrt(re * re + im * im) / pr : 0;
      double lag  = -atan2(im, re) * 180.0 / PI;
      double coh  = (pr > 0 && pk > 0) ? (re * re + im * im) / (pr * pk) : 0;
      out.printf(",%.3f,%.1f,%.3f", gain, lag, coh);
    }
    out.println();
  }
  int b0 = (int)(2.0f * NFFT / fs), b1 = (int)(12.0f * NFFT / fs);
  for (int k = 0; k < MAX_SENS; k++) {
    if (!sens[k].present || k == ref) continue;
    int best = -1;
    double bestGain = 0;
    for (int b = b0; b <= b1 && b < NBINS; b++) {
      double pr = Pxx[ref][b], pk = Pxx[k][b];
      double re = PxyRe[k][b], im = PxyIm[k][b];
      if (pr <= 0 || pk <= 0) continue;
      double coh = (re * re + im * im) / (pr * pk);
      double gain = sqrt(re * re + im * im) / pr;
      if (coh > 0.5 && gain > bestGain) { bestGain = gain; best = b; }
    }
    if (best >= 0)
      out.printf("# sensor %d: highest gain between 2 and 12 Hz is %.2f at %.2f Hz\n", k, bestGain, binHz(best));
    else
      out.printf("# sensor %d: no coherent band between 2 and 12 Hz (too little vibration at the reference)\n", k);
  }
  closeSave();
}

// spectrum amplitude (sqrt of power) at an arbitrary frequency, linear interpolation
double ampAt(int k, double f) {
  double x = f * NFFT / fs;
  int b = (int)x;
  if (b < 1 || b >= NBINS - 1) return 0;
  double t = x - b;
  return sqrt(Pxx[k][b]) * (1 - t) + sqrt(Pxx[k][b + 1]) * t;
}

double bandPower(int k, float f0, float f1) {
  int b0 = (int)ceilf(f0 * NFFT / fs), b1 = (int)floorf(f1 * NFFT / fs);
  double s = 0;
  for (int b = max(1, b0); b <= b1 && b < NBINS; b++) s += Pxx[k][b];
  return s;
}

void cmdBcg(int k, float seconds, const char *label) {
  if (k < 0 || k >= MAX_SENS || !sens[k].present) { Serial.println("# sensor not present"); return; }
  welchReset(-1);
  Serial.printf("# bcg: sensor %d, %.0f s at %.0f Hz. Keep still.\n", k, seconds, fs);
  if (!acquire(seconds, false, NULL) && nSeg == 0) return;
  if (nSeg < 3) { Serial.println("# too short: use 60 s or more"); return; }
  openSave("bcg", label);
  out.printf("# bcg sensor %d, label '%s', %d averages, resolution %.3f Hz\n", k, label, nSeg, fs / NFFT);

  // heart rate: the fundamental whose harmonics carry the most amplitude
  double bestF = 0, bestScore = 0;
  for (double f = 0.7; f <= 3.0; f += 0.005) {
    double sc = 0;
    for (int h = 1; h <= 8; h++) sc += ampAt(k, f * h);
    if (sc > bestScore) { bestScore = sc; bestF = f; }
  }
  out.printf("# heart rate %.1f beats/min (fundamental %.3f Hz)\n", bestF * 60.0, bestF);

  // harmonic amplitudes and the envelope peak
  out.println("harmonic,f_Hz,amplitude_ug");
  double hAmp[32];
  int nh = 0;
  for (int h = 1; h < 32 && bestF * h <= 20.0; h++) {
    int b = (int)lround(bestF * h * NFFT / fs);
    hAmp[h] = lineAmp(k, b);
    nh = h;
    out.printf("%d,%.3f,%.2f\n", h, bestF * h, hAmp[h] * 1e6);
  }
  int hm = 0;
  for (int h = 2; h <= nh; h++)
    if (bestF * h >= 3.0 && bestF * h <= 15.0 && (hm == 0 || hAmp[h] > hAmp[hm])) hm = h;
  if (hm > 1 && hm < nh) {
    double y1 = hAmp[hm - 1], y2 = hAmp[hm], y3 = hAmp[hm + 1];
    double den = y1 - 2 * y2 + y3;
    double off = (fabs(den) > 1e-30) ? 0.5 * (y1 - y3) / den : 0;
    if (off > 1) off = 1;
    if (off < -1) off = -1;
    out.printf("# harmonic envelope peaks near %.2f Hz (harmonic %d, %.2f ug)\n",
               bestF * (hm + off), hm, hAmp[hm] * 1e6);
  } else {
    out.println("# no clear envelope peak between 3 and 15 Hz");
  }

  // the Gateway band
  double pBand = bandPower(k, 6.8f, 7.5f);
  double pAll  = bandPower(k, 1.0f, 20.0f);
  double pLo   = bandPower(k, 5.5f, 6.5f), pHi = bandPower(k, 8.0f, 9.0f);
  double nb = floor(7.5 * NFFT / fs) - ceil(6.8 * NFFT / fs) + 1;
  double nl = floor(6.5 * NFFT / fs) - ceil(5.5 * NFFT / fs) + 1;
  double nu = floor(9.0 * NFFT / fs) - ceil(8.0 * NFFT / fs) + 1;
  // band power = sum of PSD * bin width = 2 * sum(Pxx) / (segments * sum(w^2) * NFFT)
  double rmsBand = sqrt(2.0 * pBand / ((double)nSeg * winSumSq * NFFT));          // g rms in band
  out.printf("# 6.8-7.5 Hz band: %.2f ug rms, %.1f %% of the 1-20 Hz power\n",
             rmsBand * 1e6, 100.0 * pBand / (pAll > 0 ? pAll : 1));
  if (pLo > 0 && pHi > 0 && nb > 0)
    out.printf("# band contrast (6.8-7.5 Hz against 5.5-6.5 and 8-9 Hz, per bin): %.2f\n",
               (pBand / nb) / (0.5 * (pLo / nl + pHi / nu)));
  closeSave();
}

void cmdLog(float seconds, const char *label) {
  if (!sdOk) { Serial.println("# no SD card"); return; }
  if (!nPresent) { Serial.println("# no sensors"); return; }
  char name[48];
  File f;
  for (int i = 0; i < 1000; i++) {
    if (label && label[0]) snprintf(name, sizeof(name), "raw_%03d_%s.csv", i, label);
    else snprintf(name, sizeof(name), "raw_%03d.csv", i);
    if (!SD.exists(name)) { f = SD.open(name, FILE_WRITE); break; }
  }
  if (!f) { Serial.println("# could not open a log file"); return; }
  f.print("t_us");
  for (int k = 0; k < MAX_SENS; k++) if (sens[k].present) f.printf(",s%dx_g,s%dy_g,s%dz_g", k, k, k);
  f.println();
  Serial.printf("# logging %.0f s at %.0f Hz to %s\n", seconds, fs, name);
  welchReset(-1);
  acquire(seconds, true, &f);
  f.close();
  Serial.println("# log closed");
}

void printHelp() {
  Serial.println(F(
    "# body_resonance commands:\n"
    "#  scan | rate <50..500> | chan <vert|x|y|z> | save <on|off> | stat\n"
    "#  spec <seconds>\n"
    "#  tf <ref_sensor> <seconds>\n"
    "#  bcg <sensor> <seconds> [label]\n"
    "#  log <seconds> [label]\n"
    "#  any key aborts a running measurement"));
}

void handleLine(char *line) {
  char *tok[5];
  int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 5; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    printHelp();
  } else if (!strcmp(c, "scan")) {
    scanSensors(true);
  } else if (!strcmp(c, "rate") && n >= 2) {
    float r = atof(tok[1]);
    if (r < 50) r = 50;
    if (r > 500) r = 500;
    fs = r;
    Serial.printf("# rate %.0f Hz, resolution %.3f Hz, analysis to %.0f Hz\n", fs, fs / NFFT,
                  min(40.0f, (NBINS - 1) * fs / NFFT));
  } else if (!strcmp(c, "chan") && n >= 2) {
    chanSel = (tok[1][0] == 'x') ? 0 : (tok[1][0] == 'y') ? 1 : (tok[1][0] == 'z') ? 2 : -1;
    Serial.println("# channel set");
  } else if (!strcmp(c, "save") && n >= 2) {
    saveWanted = !strcmp(tok[1], "on");
    if (saveWanted && !sdOk) Serial.println("# no SD card found at start-up");
    else Serial.printf("# saving %s\n", saveWanted ? "on" : "off");
  } else if (!strcmp(c, "spec") && n >= 2) {
    cmdSpec(atof(tok[1]));
  } else if (!strcmp(c, "tf") && n >= 3) {
    cmdTf(atoi(tok[1]), atof(tok[2]));
  } else if (!strcmp(c, "bcg") && n >= 3) {
    cmdBcg(atoi(tok[1]), atof(tok[2]), (n >= 4) ? tok[3] : "");
  } else if (!strcmp(c, "log") && n >= 2) {
    cmdLog(atof(tok[1]), (n >= 3) ? tok[2] : "");
  } else if (!strcmp(c, "stat")) {
    Serial.printf("# %d sensors, rate %.0f Hz, channel %s, SD %s, saving %s\n", nPresent, fs,
                  chanSel < 0 ? "vert" : chanSel == 0 ? "x" : chanSel == 1 ? "y" : "z",
                  sdOk ? "present" : "none", saveWanted ? "on" : "off");
  } else {
    Serial.println("# unknown command, type help");
  }
}

// =================================================================== setup/loop
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Serial.println("# body_resonance -- multi-accelerometer spectra, transfer functions, heartbeat micro-motion");
  arm_rfft_fast_init_f32(&fftInst, NFFT);
  for (int i = 0; i < NFFT; i++) {
    win[i] = 0.5f - 0.5f * cosf(TWO_PI * (float)i / (float)NFFT);        // Hann
    winSum += win[i];
    winSumSq += (double)win[i] * win[i];
  }
  scanSensors(true);
  sdOk = SD.begin(BUILTIN_SDCARD);
  printHelp();
}

void loop() {
  static char line[96];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
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
}
