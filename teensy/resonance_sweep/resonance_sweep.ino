/*
 * resonance_sweep.ino -- Teensy 4.1 swept-sine resonance analyzer
 *
 * WHAT IT DOES
 *   Drives an exciter with a sine wave, reads an MPU-6050 accelerometer fixed
 *   to the object under test, and measures the response amplitude and phase AT
 *   THE DRIVE FREQUENCY ONLY (lock-in detection, least-squares fit). Sweeping
 *   the drive gives the object's resonance curve: where it rings, how sharply
 *   (Q), and how fast it builds up and dies away. "track" then phase-locks the
 *   drive onto the resonance and follows it, which is what Tesla's oscillator
 *   did mechanically.
 *
 * WIRING (Teensy 4.1)
 *   pin 2  -> 1 kOhm -> [node] -> amplifier input      (sine, 0..3.3 V, centred 1.65 V)
 *                        [node] -> 100 nF -> GND       (RC low-pass, ~1.6 kHz)
 *             Put a 1 uF capacitor in series to the amplifier to block the DC.
 *   pin 3  -> square wave at the drive frequency (scope trigger, or gate of a
 *             logic-level MOSFET switching a small motor / solenoid exciter)
 *   pin 18 -> MPU-6050 SDA      pin 19 -> MPU-6050 SCL
 *   pin 4  <- MPU-6050 INT      (exact sample timing; without it the sketch
 *                                falls back to polling and phase is +-0.5 ms)
 *   3.3 V  -> MPU-6050 VCC      GND -> MPU-6050 GND, AD0 low (address 0x68)
 *   Amplifier output -> speaker / bass shaker / voice coil fixed to the object.
 *
 * LIMITS
 *   Drive range 0.05 .. 400 Hz. The MPU-6050 samples at 1 kHz with a 260 Hz
 *   accelerometer bandwidth, so trust amplitudes up to about 200 Hz.
 *   Phase includes the amplifier, exciter and sensor delays. The resonance
 *   shows as a 180 degree swing across the peak, not as an absolute number.
 *
 * COMMANDS (USB serial, any baud, newline-terminated)
 *   help
 *   freq <Hz>            set drive frequency
 *   amp <0..1>           set drive level (start low)
 *   on | off             drive on / off
 *   read [seconds]       one lock-in measurement at the current frequency
 *   noise [seconds]      same with the drive off: shows the noise floor
 *   sweep <f0> <f1> <points> [lin|log] [settle_s] [integrate_s]
 *   peak                 re-print peak, bandwidth and Q from the last sweep
 *   track [seconds]      phase-lock the drive onto the last sweep's peak
 *   ringdown <f> <drive_s> <record_s>   drive, cut, fit the decay: tau, Q
 *   raw <seconds>        stream raw samples
 *   axis <x|y|z|auto>    axis used for peak / track / ringdown
 *   range <2|4|8|16>     accelerometer full scale in g
 *   log <on|off>         also write every result to the SD card
 *   stat                 sensor rate, overruns, settings
 *   Any key aborts a running sweep / track / ringdown.
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62). Not yet run on hardware.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>

// ----------------------------------------------------------------- pins
const int PIN_DRIVE   = 2;    // FlexPWM sine output
const int PIN_SYNC    = 3;    // square wave, high during the first half cycle
const int PIN_MPU_INT = 4;    // MPU-6050 data-ready interrupt

// ----------------------------------------------------------------- drive
const float  DDS_RATE_HZ    = 20000.0f;    // sine update rate (IntervalTimer, 50 us)
const float  PWM_CARRIER_HZ = 146484.38f;  // ideal 10-bit carrier on Teensy 4.x
const int    PWM_BITS       = 10;
const int    PWM_MID        = 512;
const int    PWM_SWING      = 500;         // counts of swing at amp = 1.0
const float  F_MIN          = 0.05f;
const float  F_MAX          = 400.0f;

const int LUT_BITS = 10;
const int LUT_SIZE = 1 << LUT_BITS;
int16_t sineLUT[LUT_SIZE];

IntervalTimer ddsTimer;
volatile uint32_t phaseAcc    = 0;   // 2^32 = one cycle
volatile uint32_t ftw         = 0;   // phase step per DDS tick
volatile int32_t  ampCounts   = 0;   // 0..PWM_SWING, 0 = drive off
volatile uint32_t lastTickCyc = 0;   // CPU cycle counter at the last DDS tick
uint32_t cycPerTick = 30000;

float driveFreq = 10.0f;
float driveAmp  = 0.3f;
bool  driveOn   = false;

// ----------------------------------------------------------------- sensor
const uint8_t MPU_ADDR = 0x68;
volatile uint32_t latchedPhase  = 0;
volatile uint32_t latchedMicros = 0;
volatile bool     sampleFlag    = false;
volatile uint32_t overruns      = 0;
volatile uint32_t intCount      = 0;
bool  pollMode    = false;
bool  mpuOk       = false;
int   accelRangeG = 2;
float lsbPerG     = 16384.0f;
elapsedMicros pollTimer;

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
bool sdOk = false;
bool logWanted = false;

// ----------------------------------------------------------------- sweep store
const int MAX_PTS = 400;
struct Meas {
  float f;
  float amp[3];    // g peak
  float lag[3];    // degrees, positive = response lags the drive
  float dc[3];     // g
  float resid[3];  // g rms left over after the fit
  uint32_t n;
};
Meas sweepPts[MAX_PTS];
int  sweepN = 0;
int  axisSel = -1;          // -1 = auto
bool  peakValid = false;
int   peakAxis = 2;
float peakFreq = 0, peakAmp = 0, peakLag = 0, peakBW = 0, peakQ = 0;

// =================================================================== DDS
FASTRUN void ddsISR() {
  uint32_t p = phaseAcc + ftw;
  phaseAcc = p;
  lastTickCyc = ARM_DWT_CYCCNT;
  int32_t s = sineLUT[p >> (32 - LUT_BITS)];
  int32_t duty = PWM_MID + ((s * ampCounts) >> 15);
  analogWrite(PIN_DRIVE, duty);
  digitalWriteFast(PIN_SYNC, (p & 0x80000000u) ? LOW : HIGH);
}

void setFreq(float f) {
  if (f < F_MIN) f = F_MIN;
  if (f > F_MAX) f = F_MAX;
  uint32_t w = (uint32_t)((double)f * 4294967296.0 / (double)DDS_RATE_HZ + 0.5);
  noInterrupts();
  ftw = w;
  interrupts();
  driveFreq = (float)((double)w * (double)DDS_RATE_HZ / 4294967296.0);
}

void applyAmp() {
  int32_t a = driveOn ? (int32_t)(driveAmp * PWM_SWING + 0.5f) : 0;
  noInterrupts();
  ampCounts = a;
  interrupts();
}

// Phase of the drive right now, interpolated between DDS ticks.
static inline uint32_t phaseNow() {
  uint32_t cyc = ARM_DWT_CYCCNT;
  uint32_t p = phaseAcc;
  uint32_t tick = lastTickCyc;
  uint32_t w = ftw;
  uint32_t dc = cyc - tick;
  if (dc > cycPerTick) dc = cycPerTick;
  return p + (uint32_t)(((uint64_t)w * dc) / cycPerTick);
}

// =================================================================== MPU-6050
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void mpuISR() {
  __disable_irq();
  uint32_t p = phaseNow();
  __enable_irq();
  latchedPhase = p;
  latchedMicros = micros();
  if (sampleFlag) overruns++;
  sampleFlag = true;
  intCount++;
}

bool setRange(int g) {
  uint8_t sel;
  switch (g) {
    case 2:  sel = 0; lsbPerG = 16384.0f; break;
    case 4:  sel = 1; lsbPerG = 8192.0f;  break;
    case 8:  sel = 2; lsbPerG = 4096.0f;  break;
    case 16: sel = 3; lsbPerG = 2048.0f;  break;
    default: return false;
  }
  accelRangeG = g;
  return mpuWrite(0x1C, sel << 3);
}

bool mpuInit() {
  Wire.begin();
  Wire.setClock(400000);
  uint8_t who = 0;
  if (!mpuRead(0x75, &who, 1)) return false;
  mpuWrite(0x6B, 0x80);            // device reset
  delay(100);
  mpuWrite(0x68, 0x07);            // signal path reset
  delay(100);
  mpuWrite(0x6B, 0x01);            // wake, clock = gyro X PLL
  delay(10);
  mpuWrite(0x1A, 0x00);            // DLPF off: accelerometer bandwidth 260 Hz
  mpuWrite(0x19, 7);               // 8 kHz / (1 + 7) = 1 kHz sample rate
  mpuWrite(0x1B, 0x00);            // gyro +-250 dps (unused)
  setRange(accelRangeG);
  mpuWrite(0x37, 0x10);            // INT: active high, push-pull, 50 us pulse, clear on read
  mpuWrite(0x38, 0x01);            // data-ready interrupt
  Serial.printf("# MPU WHO_AM_I = 0x%02X\n", who);
  return true;
}

// Returns true when a fresh sample was read. a[] in g, ph = drive phase at the
// instant the sample was taken, tus = micros() at that instant.
bool getSample(float a[3], uint32_t &ph, uint32_t &tus) {
  if (!mpuOk) return false;
  if (pollMode) {
    if (pollTimer < 1000) return false;
    pollTimer -= 1000;
    noInterrupts();
    ph = phaseNow();
    interrupts();
    tus = micros();
  } else {
    if (!sampleFlag) return false;
    noInterrupts();
    ph = latchedPhase;
    tus = latchedMicros;
    sampleFlag = false;
    interrupts();
  }
  uint8_t b[6];
  if (!mpuRead(0x3B, b, 6)) return false;
  a[0] = (float)(int16_t)((b[0] << 8) | b[1]) / lsbPerG;
  a[1] = (float)(int16_t)((b[2] << 8) | b[3]) / lsbPerG;
  a[2] = (float)(int16_t)((b[4] << 8) | b[5]) / lsbPerG;
  return true;
}

// =================================================================== lock-in
// Least-squares fit of  a(t) = c0 + X*sin(phase) + Y*cos(phase)  per axis.
// Exact DC (gravity) rejection and no need for evenly spaced samples.
struct LockIn {
  double n, Ss, Sc, Sss, Scc, Ssc;
  double Sa[3], Sas[3], Sac[3], Saa[3];
  void reset() {
    n = Ss = Sc = Sss = Scc = Ssc = 0;
    for (int i = 0; i < 3; i++) Sa[i] = Sas[i] = Sac[i] = Saa[i] = 0;
  }
  void add(float s, float c, const float a[3]) {
    n += 1; Ss += s; Sc += c; Sss += s * s; Scc += c * c; Ssc += s * c;
    for (int i = 0; i < 3; i++) {
      Sa[i] += a[i]; Sas[i] += a[i] * s; Sac[i] += a[i] * c; Saa[i] += a[i] * a[i];
    }
  }
  bool solve(int i, double &c0, double &X, double &Y, double &resid) const {
    if (n < 8) return false;
    // | n   Ss  Sc  | |c0|   |Sa |
    // | Ss  Sss Ssc | |X | = |Sas|
    // | Sc  Ssc Scc | |Y |   |Sac|
    double m00 = n, m01 = Ss, m02 = Sc, m11 = Sss, m12 = Ssc, m22 = Scc;
    double det = m00 * (m11 * m22 - m12 * m12) - m01 * (m01 * m22 - m12 * m02)
               + m02 * (m01 * m12 - m11 * m02);
    if (fabs(det) < 1e-9) return false;
    double b0 = Sa[i], b1 = Sas[i], b2 = Sac[i];
    c0 = (b0 * (m11 * m22 - m12 * m12) - m01 * (b1 * m22 - m12 * b2)
          + m02 * (b1 * m12 - m11 * b2)) / det;
    X  = (m00 * (b1 * m22 - b2 * m12) - b0 * (m01 * m22 - m12 * m02)
          + m02 * (m01 * b2 - b1 * m02)) / det;
    Y  = (m00 * (m11 * b2 - m12 * b1) - m01 * (m01 * b2 - b1 * m02)
          + b0 * (m01 * m12 - m11 * m02)) / det;
    double ss = Saa[i] - c0 * b0 - X * b1 - Y * b2;
    resid = (ss > 0) ? sqrt(ss / n) : 0;
    return true;
  }
};

bool abortRequested() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    return true;
  }
  return false;
}

static inline float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

// One lock-in measurement at the current drive frequency.
// Returns false if aborted or if the sensor gave no data.
bool measure(float int_s, Meas &m) {
  LockIn L;
  L.reset();
  float T = int_s;
  if (T < 3.0f / driveFreq) T = 3.0f / driveFreq;   // at least three cycles
  uint32_t Tus = (uint32_t)(T * 1e6f);
  elapsedMicros t = 0;
  float a[3];
  uint32_t ph, tus;
  while (t < Tus) {
    if (getSample(a, ph, tus)) {
      float ang = (float)ph * (TWO_PI / 4294967296.0f);
      L.add(sinf(ang), cosf(ang), a);
    }
    if (abortRequested()) return false;
  }
  m.f = driveFreq;
  m.n = (uint32_t)L.n;
  for (int i = 0; i < 3; i++) {
    double c0, X, Y, r;
    if (!L.solve(i, c0, X, Y, r)) return false;
    m.amp[i]   = (float)sqrt(X * X + Y * Y);
    m.lag[i]   = (float)(atan2(-Y, X) * 180.0 / PI);
    m.dc[i]    = (float)c0;
    m.resid[i] = (float)r;
  }
  return true;
}

void printMeasHeader() {
  out.println("f_Hz,x_mg,x_lag_deg,y_mg,y_lag_deg,z_mg,z_lag_deg,total_mg,noise_mg_rms,samples");
}

void printMeas(const Meas &m) {
  float tot = sqrtf(m.amp[0] * m.amp[0] + m.amp[1] * m.amp[1] + m.amp[2] * m.amp[2]);
  float nz  = sqrtf(m.resid[0] * m.resid[0] + m.resid[1] * m.resid[1] + m.resid[2] * m.resid[2]);
  out.printf("%.4f,%.3f,%.1f,%.3f,%.1f,%.3f,%.1f,%.3f,%.3f,%lu\n",
             m.f, m.amp[0] * 1000, m.lag[0], m.amp[1] * 1000, m.lag[1],
             m.amp[2] * 1000, m.lag[2], tot * 1000, nz * 1000, (unsigned long)m.n);
}

// =================================================================== SD log
bool openLog(const char *prefix) {
  if (!logWanted || !sdOk) return false;
  char name[16];
  for (int i = 0; i < 1000; i++) {
    snprintf(name, sizeof(name), "%s%03d.CSV", prefix, i);
    if (!SD.exists(name)) {
      out.f = SD.open(name, FILE_WRITE);
      if (out.f) {
        out.fileOn = true;
        Serial.printf("# logging to %s\n", name);
        return true;
      }
      return false;
    }
  }
  return false;
}

void closeLog() {
  if (out.fileOn) {
    out.fileOn = false;
    out.f.close();
  }
}

// =================================================================== analysis
int pickAxis() {
  if (axisSel >= 0) return axisSel;
  int best = 2;
  float bestAmp = -1;
  for (int k = 0; k < 3; k++)
    for (int i = 0; i < sweepN; i++)
      if (sweepPts[i].amp[k] > bestAmp) { bestAmp = sweepPts[i].amp[k]; best = k; }
  return best;
}

void analyzePeak() {
  peakValid = false;
  if (sweepN < 3) { out.println("# peak: need at least 3 sweep points"); return; }
  int k = pickAxis();
  int im = 0;
  for (int i = 1; i < sweepN; i++) if (sweepPts[i].amp[k] > sweepPts[im].amp[k]) im = i;
  float f0 = sweepPts[im].f, a0 = sweepPts[im].amp[k];
  if (im > 0 && im < sweepN - 1) {
    // parabola through the three points around the maximum
    float x1 = sweepPts[im - 1].f, x2 = sweepPts[im].f, x3 = sweepPts[im + 1].f;
    float y1 = sweepPts[im - 1].amp[k], y2 = a0, y3 = sweepPts[im + 1].amp[k];
    float d = (x1 - x2) * (x1 - x3) * (x2 - x3);
    if (fabsf(d) > 1e-12f) {
      float A = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / d;
      float B = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / d;
      float C = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / d;
      if (A < 0) {
        float fv = -B / (2 * A);
        if (fv > x1 && fv < x3) { f0 = fv; a0 = C - B * B / (4 * A); }
      }
    }
  }
  // -3 dB points either side
  float half = a0 * 0.70710678f;
  float fl = NAN, fh = NAN;
  for (int i = im; i > 0; i--) {
    if (sweepPts[i - 1].amp[k] <= half && sweepPts[i].amp[k] >= half) {
      float t = (half - sweepPts[i - 1].amp[k]) / (sweepPts[i].amp[k] - sweepPts[i - 1].amp[k]);
      fl = sweepPts[i - 1].f + t * (sweepPts[i].f - sweepPts[i - 1].f);
      break;
    }
  }
  for (int i = im; i < sweepN - 1; i++) {
    if (sweepPts[i + 1].amp[k] <= half && sweepPts[i].amp[k] >= half) {
      float t = (sweepPts[i].amp[k] - half) / (sweepPts[i].amp[k] - sweepPts[i + 1].amp[k]);
      fh = sweepPts[i].f + t * (sweepPts[i + 1].f - sweepPts[i].f);
      break;
    }
  }
  // lag at the peak, interpolated between the bracketing points
  float lag = sweepPts[im].lag[k];
  int j = (f0 >= sweepPts[im].f) ? im : im - 1;
  if (j >= 0 && j < sweepN - 1) {
    float la = sweepPts[j].lag[k];
    float lb = la + wrap180(sweepPts[j + 1].lag[k] - la);
    float t = (f0 - sweepPts[j].f) / (sweepPts[j + 1].f - sweepPts[j].f);
    lag = wrap180(la + t * (lb - la));
  }
  peakAxis = k; peakFreq = f0; peakAmp = a0; peakLag = lag;
  peakBW = (isnan(fl) || isnan(fh)) ? 0 : (fh - fl);
  peakQ  = (peakBW > 0) ? f0 / peakBW : 0;
  peakValid = true;
  out.printf("# peak: axis %c  f0 = %.4f Hz  amplitude = %.3f mg  lag at peak = %.1f deg\n",
             "xyz"[k], f0, a0 * 1000, lag);
  if (peakBW > 0)
    out.printf("# -3 dB band: %.4f .. %.4f Hz  width = %.4f Hz  Q = %.1f\n", fl, fh, peakBW, peakQ);
  else
    out.println("# -3 dB band not bracketed by this sweep: widen the range or add points");
}

// =================================================================== commands
void cmdSweep(float f0, float f1, int npts, bool logSpacing, float settle_s, float int_s) {
  if (npts < 2) npts = 2;
  if (npts > MAX_PTS) npts = MAX_PTS;
  if (f0 < F_MIN) f0 = F_MIN;
  if (f1 > F_MAX) f1 = F_MAX;
  openLog("SWEEP");
  out.printf("# sweep %.4f -> %.4f Hz, %d points, %s, settle %.2f s, integrate %.2f s, amp %.3f, range +-%d g\n",
             f0, f1, npts, logSpacing ? "log" : "lin", settle_s, int_s, driveAmp, accelRangeG);
  printMeasHeader();
  bool wasOn = driveOn;
  driveOn = true;
  applyAmp();
  sweepN = 0;
  for (int i = 0; i < npts; i++) {
    float frac = (float)i / (float)(npts - 1);
    float f = logSpacing ? f0 * powf(f1 / f0, frac) : f0 + (f1 - f0) * frac;
    setFreq(f);
    float st = settle_s;
    if (st < 5.0f / driveFreq) st = 5.0f / driveFreq;
    elapsedMillis w = 0;
    bool stop = false;
    float dump[3]; uint32_t ph, tus;
    while (w < (uint32_t)(st * 1000)) {
      getSample(dump, ph, tus);               // keep the sensor drained
      if (abortRequested()) { stop = true; break; }
    }
    if (stop) break;
    Meas m;
    if (!measure(int_s, m)) break;
    sweepPts[sweepN++] = m;
    printMeas(m);
  }
  driveOn = wasOn;
  applyAmp();
  analyzePeak();
  closeLog();
}

void cmdTrack(float seconds) {
  if (!peakValid) { out.println("# track: run a sweep first"); return; }
  int k = peakAxis;
  float f = peakFreq;
  // Hz of correction per degree of phase error. Across a resonance the phase
  // moves about 180 degrees over roughly twice the bandwidth.
  float kp = (peakBW > 0) ? 0.3f * (2.0f * peakBW / 180.0f) : 0.0005f * peakFreq;
  float fLo = peakFreq * 0.5f, fHi = peakFreq * 2.0f;
  openLog("TRACK");
  out.printf("# track: axis %c, start %.4f Hz, target lag %.1f deg, gain %.5f Hz/deg\n",
             "xyz"[k], f, peakLag, kp);
  out.println("t_s,f_Hz,amp_mg,lag_deg,err_deg");
  bool wasOn = driveOn;
  driveOn = true;
  applyAmp();
  setFreq(f);
  elapsedMillis t = 0;
  while (seconds <= 0 || t < (uint32_t)(seconds * 1000)) {
    float win = 10.0f / driveFreq;
    if (win < 0.25f) win = 0.25f;
    Meas m;
    if (!measure(win, m)) break;
    float err = wrap180(m.lag[k] - peakLag);
    out.printf("%.2f,%.4f,%.3f,%.1f,%.1f\n", t / 1000.0f, driveFreq, m.amp[k] * 1000, m.lag[k], err);
    f -= kp * err;                       // lag too large = above resonance = go down
    if (f < fLo) f = fLo;
    if (f > fHi) f = fHi;
    setFreq(f);
  }
  driveOn = wasOn;
  applyAmp();
  closeLog();
}

void cmdRingdown(float f, float drive_s, float rec_s) {
  int k = (axisSel >= 0) ? axisSel : (peakValid ? peakAxis : 2);
  openLog("RING");
  setFreq(f);
  out.printf("# ringdown: drive %.4f Hz for %.1f s, then record %.1f s on axis %c\n",
             driveFreq, drive_s, rec_s, "xyz"[k]);
  driveOn = true;
  applyAmp();
  elapsedMillis w = 0;
  float a[3]; uint32_t ph, tus;
  while (w < (uint32_t)(drive_s * 1000)) {
    getSample(a, ph, tus);
    if (abortRequested()) { driveOn = false; applyAmp(); closeLog(); return; }
  }
  // amplitude just before the cut
  Meas m0;
  float win = 5.0f / driveFreq;
  if (win < 0.05f) win = 0.05f;
  if (!measure(win, m0)) { driveOn = false; applyAmp(); closeLog(); return; }
  driveOn = false;
  applyAmp();                                   // the reference phase keeps running
  out.printf("# driven amplitude %.3f mg, noise %.3f mg rms\n", m0.amp[k] * 1000, m0.resid[k] * 1000);
  out.println("t_s,amp_mg,lag_deg");
  // decay envelope from back-to-back lock-in windows
  const int MAXW = 600;
  static float tt[MAXW], aa[MAXW], ll[MAXW];
  int nw = 0;
  elapsedMicros t = 0;
  float prevLag = 0, unwrap = 0;
  while (t < (uint32_t)(rec_s * 1e6f) && nw < MAXW) {
    uint32_t tstart = t;
    Meas m;
    if (!measure(win, m)) break;
    float tc = (tstart + (uint32_t)t) * 0.5e-6f;
    float lag = m.lag[k];
    if (nw > 0) unwrap += wrap180(lag - prevLag);
    else unwrap = lag;
    prevLag = lag;
    tt[nw] = tc; aa[nw] = m.amp[k]; ll[nw] = unwrap;
    nw++;
    out.printf("%.4f,%.4f,%.1f\n", tc, m.amp[k] * 1000, unwrap);
  }
  // fit ln(amp) = ln(A0) - t/tau over windows well above the noise floor
  float floorG = 3.0f * m0.resid[k] / sqrtf(win * 1000.0f / 2.0f);
  if (floorG < 0.0002f) floorG = 0.0002f;
  double Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, Sl = 0, Sxl = 0;
  int nf = 0;
  for (int i = 0; i < nw; i++) {
    if (aa[i] < floorG) break;
    double x = tt[i], y = log(aa[i]);
    Sx += x; Sy += y; Sxx += x * x; Sxy += x * y; Sl += ll[i]; Sxl += x * ll[i];
    nf++;
  }
  if (nf >= 3) {
    double den = nf * Sxx - Sx * Sx;
    double slope = (nf * Sxy - Sx * Sy) / den;          // 1/s, negative for a decay
    double lagRate = (nf * Sxl - Sx * Sl) / den;        // deg/s
    double fFree = driveFreq - lagRate / 360.0;
    if (slope < 0) {
      double tau = -1.0 / slope;
      out.printf("# decay: tau = %.4f s over %d windows, free frequency = %.4f Hz, Q = %.1f\n",
                 tau, nf, fFree, PI * fFree * tau);
    } else {
      out.println("# decay: amplitude did not fall, no fit");
    }
  } else {
    out.println("# decay: too few windows above the noise floor (raise amp or drive nearer resonance)");
  }
  closeLog();
}

void cmdRaw(float seconds) {
  openLog("RAW");
  out.println("t_us,x_g,y_g,z_g,drive_phase_deg");
  elapsedMicros t = 0;
  float a[3]; uint32_t ph, tus;
  uint32_t t0 = micros();
  while (t < (uint32_t)(seconds * 1e6f)) {
    if (getSample(a, ph, tus))
      out.printf("%lu,%.5f,%.5f,%.5f,%.2f\n", (unsigned long)(tus - t0), a[0], a[1], a[2],
                 (float)ph * (360.0f / 4294967296.0f));
    if (abortRequested()) break;
  }
  closeLog();
}

void cmdStat() {
  uint32_t c0 = intCount;
  delay(1000);
  uint32_t c1 = intCount;
  Serial.printf("# sensor: %s, %s, measured rate %lu Hz, overruns %lu, range +-%d g\n",
                mpuOk ? "MPU-6050 ok" : "NOT FOUND", pollMode ? "polling (no INT wire)" : "INT-timed",
                (unsigned long)(c1 - c0), (unsigned long)overruns, accelRangeG);
  Serial.printf("# drive: %.4f Hz, amp %.3f, %s\n", driveFreq, driveAmp, driveOn ? "ON" : "off");
  Serial.printf("# SD card: %s, logging %s, axis %s\n", sdOk ? "present" : "none",
                logWanted ? "on" : "off", axisSel < 0 ? "auto" : (axisSel == 0 ? "x" : axisSel == 1 ? "y" : "z"));
}

void printHelp() {
  Serial.println(F(
    "# resonance_sweep commands:\n"
    "#  freq <Hz> | amp <0..1> | on | off\n"
    "#  read [s] | noise [s]\n"
    "#  sweep <f0> <f1> <points> [lin|log] [settle_s] [integrate_s]\n"
    "#  peak | track [s] | ringdown <f> <drive_s> <record_s>\n"
    "#  raw <s> | axis <x|y|z|auto> | range <2|4|8|16> | log <on|off> | stat\n"
    "#  any key aborts a running sweep / track / ringdown"));
}

void handleLine(char *line) {
  char *tok[8];
  int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 8; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    printHelp();
  } else if (!strcmp(c, "freq") && n >= 2) {
    setFreq(atof(tok[1]));
    Serial.printf("# freq %.4f Hz\n", driveFreq);
  } else if (!strcmp(c, "amp") && n >= 2) {
    float a = atof(tok[1]);
    if (a < 0) a = 0;
    if (a > 1) a = 1;
    driveAmp = a;
    applyAmp();
    Serial.printf("# amp %.3f\n", driveAmp);
  } else if (!strcmp(c, "on")) {
    driveOn = true; applyAmp(); Serial.println("# drive ON");
  } else if (!strcmp(c, "off")) {
    driveOn = false; applyAmp(); Serial.println("# drive off");
  } else if (!strcmp(c, "read") || !strcmp(c, "noise")) {
    float s = (n >= 2) ? atof(tok[1]) : 2.0f;
    bool wasOn = driveOn;
    if (!strcmp(c, "noise")) { driveOn = false; applyAmp(); }
    Meas m;
    printMeasHeader();
    if (measure(s, m)) printMeas(m);
    else Serial.println("# no data (sensor missing or aborted)");
    driveOn = wasOn; applyAmp();
  } else if (!strcmp(c, "sweep") && n >= 4) {
    bool lg = (n >= 5 && !strcmp(tok[4], "log"));
    float st = (n >= 6) ? atof(tok[5]) : 1.0f;
    float it = (n >= 7) ? atof(tok[6]) : 2.0f;
    cmdSweep(atof(tok[1]), atof(tok[2]), atoi(tok[3]), lg, st, it);
  } else if (!strcmp(c, "peak")) {
    analyzePeak();
  } else if (!strcmp(c, "track")) {
    cmdTrack((n >= 2) ? atof(tok[1]) : 0);
  } else if (!strcmp(c, "ringdown") && n >= 4) {
    cmdRingdown(atof(tok[1]), atof(tok[2]), atof(tok[3]));
  } else if (!strcmp(c, "raw") && n >= 2) {
    cmdRaw(atof(tok[1]));
  } else if (!strcmp(c, "axis") && n >= 2) {
    if (tok[1][0] == 'x') axisSel = 0;
    else if (tok[1][0] == 'y') axisSel = 1;
    else if (tok[1][0] == 'z') axisSel = 2;
    else axisSel = -1;
    Serial.println("# axis set");
  } else if (!strcmp(c, "range") && n >= 2) {
    if (setRange(atoi(tok[1]))) Serial.printf("# range +-%d g\n", accelRangeG);
    else Serial.println("# range must be 2, 4, 8 or 16");
  } else if (!strcmp(c, "log") && n >= 2) {
    logWanted = !strcmp(tok[1], "on");
    if (logWanted && !sdOk) Serial.println("# no SD card found at start-up");
    else Serial.printf("# SD logging %s\n", logWanted ? "on" : "off");
  } else if (!strcmp(c, "stat")) {
    cmdStat();
  } else {
    Serial.println("# unknown command, type help");
  }
}

// =================================================================== setup/loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_SYNC, OUTPUT);
  pinMode(PIN_MPU_INT, INPUT);
  for (int i = 0; i < LUT_SIZE; i++)
    sineLUT[i] = (int16_t)lroundf(32767.0f * sinf(TWO_PI * (float)i / (float)LUT_SIZE));

  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PIN_DRIVE, PWM_CARRIER_HZ);
  analogWrite(PIN_DRIVE, PWM_MID);

  cycPerTick = (uint32_t)((float)F_CPU_ACTUAL / DDS_RATE_HZ);
  setFreq(driveFreq);
  applyAmp();
  ddsTimer.priority(32);
  ddsTimer.begin(ddsISR, 1000000.0f / DDS_RATE_HZ);

  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("# resonance_sweep -- Teensy 4.1 lock-in resonance analyzer");
  mpuOk = mpuInit();
  if (!mpuOk) {
    Serial.println("# MPU-6050 not found on pins 18/19 (address 0x68). Check wiring, then reset.");
  } else {
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuISR, RISING);
    delay(200);
    if (intCount < 20) {
      detachInterrupt(digitalPinToInterrupt(PIN_MPU_INT));
      pollMode = true;
      Serial.println("# no pulses on pin 4 (MPU INT): using polling, phase is +-0.5 ms uncertain");
    }
  }
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
  // keep the sensor's interrupt flag drained while idle
  float a[3]; uint32_t ph, tus;
  getSample(a, ph, tus);
}
