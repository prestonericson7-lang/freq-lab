/*
 * schumann_monitor.ino -- Teensy 4.1 + ADS1256 24-bit ELF monitor:
 *                         Schumann-resonance modes, E and H channels, E/H ratio
 *
 * WHAT IT DOES
 *   The product version of elf_logger: a 24-bit converter instead of the
 *   Teensy's 12-bit one, two channels (magnetic H from an induction coil,
 *   electric E from a ball antenna), and a report that tracks the first five
 *   Earth-ionosphere cavity modes (about 7.8, 14.3, 20.8, 27.3, 33.8 Hz):
 *   centre frequency, amplitude in pT/rtHz (H) or uV/m/rtHz (E), how far the
 *   mode stands above the floor, and the E/H wave impedance at the first two
 *   modes (NUSC 1986: the ratio runs ~1.2 dB higher by day at 76 Hz; here you
 *   measure it yourself at 7.8 and 14 Hz).
 *
 *   The Schumann modes are a guaranteed real signal, about 1 pT at 7.8 Hz,
 *   driven by the world's lightning; their daily power cycle follows the
 *   afternoon thunderstorm peaks of Asia, Africa and the Americas. A product
 *   that shows the live modes is rarer than one that merely generates 7.83 Hz.
 *
 * HARDWARE
 *   ADS1256 module (the common blue board, 7.68 MHz crystal, 2.5 V reference):
 *     Teensy 13 SCK, 11 MOSI (DIN), 12 MISO (DOUT), 10 CS, 9 DRDY, 8 RESET (optional)
 *     5 V to the module's 5 V, GND; AIN0/AIN1 = H channel, AIN2/AIN3 = E channel
 *     (differential, within 0..5 V of the module's analog ground; bias at 2.5 V)
 *   H: induction coil -> preamp (see docs/analog-front-ends.md) -> AIN0/AIN1, gain told with `gain h`
 *   E: ball antenna (insulated sphere on a mast) -> electrometer op-amp (10^12 ohm input,
 *      e.g. LMC6001/ADA4530, unity gain) -> AIN2/AIN3; `gain e` = that stage's gain
 *   GPS: pin 0 (RX1) <- NMEA 9600, pin 2 <- PPS (optional)
 *   SD card in the Teensy's slot (optional)
 *
 * SAMPLING
 *   ADS1256 at 2000 SPS, multiplexed H/E/H/E -> 1000 SPS per channel.
 *   Per channel: IIR notch at the mains frequency and its 2nd/3rd harmonics,
 *   8th-order Butterworth low-pass at 45 Hz, decimate by 4 -> 250 Hz.
 *   4096-point Welch spectra (0.061 Hz bins, Hann, 50 % overlap), plus the
 *   E*conj(H) cross-spectrum for the impedance.
 *
 * COMMANDS (USB serial 115200, newline-terminated)
 *   start [report_seconds]    begin (default report every 60 s; minimum 20)
 *   stop | live | stat | help
 *   gain <h|e> <x>            preamp voltage gain in front of that channel (default 1000 / 1)
 *   pga <h|e> <1|2|4|8|16|32|64>   ADS1256 gain for that channel (default 8 / 1)
 *   coil <turns> <area_m2>    H calibration by physics: V = 2 pi f N A B (air coil, below resonance)
 *   hflat <V_per_pT>          or a flat sensor (fluxgate/magnetometer analog out)
 *   ecal <h_eff_m>            E: effective height of the ball antenna (V_out = E * h_eff), default 1.0
 *   mains <50|60>             (default 60)
 *   json <on|off>             reports as one JSON line (for schumann_host.py), default off = CSV
 *   stream <on|off>           print each 16 s segment's five mode amplitudes as it happens
 *   raw <on|off>              250 Hz H and E waveforms to the SD card
 *
 * REPORT (CSV)
 *   time, segments, then for modes 1..5: f_Hz, H_pT_rtHz, H_x_floor, E_uVm_rtHz,
 *   then Z1_ohm, Z2_ohm (E/H at modes 1 and 2, |E|/|H| in V/m per T = ohm*m/s... reported as
 *   the impedance ratio in ohms: Z = E / (H/mu0)), coherence1, coherence2, and the input
 *   noise floor at 30 Hz for H (pT/rtHz) so a customer can see the sensor's quality.
 *
 * STATUS: written for Teensyduino 1.62 (Teensy 4.1). See the README for the compile check.
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <arm_math.h>

// ----------------------------------------------------------------- pins / constants
const int PIN_CS = 10, PIN_DRDY = 9, PIN_RESET = 8, PIN_PPS = 2;
const double FS_ADC = 1000.0;          // per channel after multiplexing (ADS1256 at 2000 SPS)
const int    DECIM = 4;
const float  FS_OUT = 250.0f;
const int    NFFT = 4096, HOP = 2048, NBINS = 760;
const double VREF = 2.5;               // ADS1256 reference: full scale = +/-2*Vref/PGA
const double MU0 = 4.0e-7 * PI;
const int    NMODE = 5;
const float  MODE_HZ[NMODE] = {7.83f, 14.3f, 20.8f, 27.3f, 33.8f};

// ADS1256 commands / registers / rates
enum { CMD_WAKEUP = 0x00, CMD_RDATA = 0x01, CMD_RREG = 0x10, CMD_WREG = 0x50, CMD_SELFCAL = 0xF0,
       CMD_SYNC = 0xFC, CMD_RESET = 0xFE, CMD_SDATAC = 0x0F };
enum { REG_STATUS = 0, REG_MUX = 1, REG_ADCON = 2, REG_DRATE = 3, REG_IO = 4 };
const uint8_t DRATE_2000 = 0xB0;

// ----------------------------------------------------------------- filters
struct Biquad {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  inline double step(double x) {
    double y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void notch(double f0, double q, double fs) {
    double w = 2.0 * PI * f0 / fs, al = sin(w) / (2.0 * q), a0 = 1.0 + al;
    b0 = 1.0 / a0; b1 = -2.0 * cos(w) / a0; b2 = 1.0 / a0;
    a1 = -2.0 * cos(w) / a0; a2 = (1.0 - al) / a0;
    z1 = z2 = 0;
  }
  void lowpass(double f0, double q, double fs) {
    double w = 2.0 * PI * f0 / fs, al = sin(w) / (2.0 * q), a0 = 1.0 + al;
    b0 = (1.0 - cos(w)) / 2.0 / a0; b1 = (1.0 - cos(w)) / a0; b2 = b0;
    a1 = -2.0 * cos(w) / a0; a2 = (1.0 - al) / a0;
    z1 = z2 = 0;
  }
};

struct Channel {
  const char *name;
  uint8_t mux;                 // ADS1256 MUX register value
  uint8_t pgaCode;             // 0..6 -> gain 1..64
  double  frontGain;           // external preamp gain
  Biquad  notchF[3], lpF[4];
  int     decimPhase = 0;
  // ISR -> main FIFO of filtered, decimated samples
  volatile float fifo[1024];
  volatile uint32_t head = 0, tail = 0, drops = 0;
  // 250 Hz ring for spectra
  float ring[NFFT];
  int   ringPos = 0;
  long  ringCount = 0;
  float fftIn[NFFT], fftOut[NFFT];
  double Pxx[NBINS];
  float  respCorr[NBINS];
  int32_t rawMin = 0x7FFFFFFF, rawMax = -0x7FFFFFFF;
};

Channel chH = {"H", 0x01, 3, 1000.0};      // AIN0-AIN1, PGA 8
Channel chE = {"E", 0x23, 0, 1.0};         // AIN2-AIN3, PGA 1
Channel *CH[2] = {&chH, &chE};

// cross-spectrum E * conj(H), accumulated per report
double Pxy_re[NBINS], Pxy_im[NBINS];
int    nSeg = 0;
arm_rfft_fast_instance_f32 fftInst;
float  win[NFFT];
double winSumSq = 0;

// calibration
bool   hByCoil = true;
double coilTurns = 20000, coilArea = 7.85e-3;      // 20 k turns on a 100 mm diameter... see kit doc
double hFlatVperT = 1.0e6;                         // V per T for a flat sensor (1 V/uT) when hByCoil = false
double eHeff = 1.0;                                // metres: V_out = E * h_eff
double mainsHz = 60.0;

// state
bool logging = false, jsonOut = false, streamOn = false, rawWanted = false, sdOk = false, rawOpen = false;
uint32_t reportMs = 60000;
elapsedMillis sinceReport;
File rawFile;
uint32_t outIndex = 0;
volatile int curCh = 0;
volatile uint32_t adcCount = 0, ppsCount = 0, ppsAdcIndex = 0;
volatile bool ppsFlag = false;

// GPS
char nmea[100]; int nmeaLen = 0; bool gpsValid = false; char gpsStamp[24] = ""; elapsedMillis gpsAge;

// =================================================================== ADS1256 low level
SPISettings adsSpi(1800000, MSBFIRST, SPI_MODE1);

static inline void adsCmd(uint8_t c) {
  SPI.beginTransaction(adsSpi);
  digitalWriteFast(PIN_CS, LOW);
  SPI.transfer(c);
  digitalWriteFast(PIN_CS, HIGH);
  SPI.endTransaction();
}

void adsWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(adsSpi);
  digitalWriteFast(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(val);
  digitalWriteFast(PIN_CS, HIGH);
  SPI.endTransaction();
}

uint8_t adsReadReg(uint8_t reg) {
  SPI.beginTransaction(adsSpi);
  digitalWriteFast(PIN_CS, LOW);
  SPI.transfer(CMD_RREG | reg);
  SPI.transfer(0x00);
  delayMicroseconds(7);                       // t6: 50 CLKIN cycles at 7.68 MHz
  uint8_t v = SPI.transfer(0xFF);
  digitalWriteFast(PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

// read the conversion that is ready, and set the mux + restart for the next channel
static inline int32_t adsReadAndSwitch(uint8_t nextMux, uint8_t nextAdcon) {
  SPI.beginTransaction(adsSpi);
  digitalWriteFast(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | REG_MUX);           // set the next input first (datasheet cycling sequence)
  SPI.transfer(0x01);                         // two registers: MUX, ADCON
  SPI.transfer(nextMux);
  SPI.transfer(nextAdcon);
  SPI.transfer(CMD_SYNC);
  delayMicroseconds(4);
  SPI.transfer(CMD_WAKEUP);
  SPI.transfer(CMD_RDATA);                    // then read the finished conversion of the old input
  delayMicroseconds(7);
  uint32_t v = ((uint32_t)SPI.transfer(0xFF) << 16) | ((uint32_t)SPI.transfer(0xFF) << 8) | SPI.transfer(0xFF);
  digitalWriteFast(PIN_CS, HIGH);
  SPI.endTransaction();
  if (v & 0x800000) v |= 0xFF000000;          // sign-extend 24 -> 32
  return (int32_t)v;
}

bool adsInit() {
  pinMode(PIN_CS, OUTPUT); digitalWriteFast(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_RESET, OUTPUT); digitalWriteFast(PIN_RESET, LOW); delay(2); digitalWriteFast(PIN_RESET, HIGH); delay(50);
  adsCmd(CMD_SDATAC);
  delay(2);
  adsWriteReg(REG_STATUS, 0x06);              // MSB first, auto-cal on, buffer off (coil preamp drives it fine)
  adsWriteReg(REG_MUX, chH.mux);
  adsWriteReg(REG_ADCON, chH.pgaCode);        // clock out off, sensor detect off, PGA
  adsWriteReg(REG_DRATE, DRATE_2000);
  adsWriteReg(REG_IO, 0x00);
  adsCmd(CMD_SELFCAL);
  delay(400);
  uint8_t st = adsReadReg(REG_STATUS);
  return (st >> 4) == 0x03 || (st & 0x0F) == 0x06 || st != 0xFF;   // ID bits read as 3 on a real chip
}

// =================================================================== ISR on DRDY: one conversion ready
void drdyISR() {
  int finished = curCh;
  int next = finished ^ 1;
  int32_t raw = adsReadAndSwitch(CH[next]->mux, CH[next]->pgaCode);
  curCh = next;
  Channel &c = *CH[finished];
  if (raw < c.rawMin) c.rawMin = raw;
  if (raw > c.rawMax) c.rawMax = raw;
  double x = (double)raw;
  x = c.notchF[0].step(x); x = c.notchF[1].step(x); x = c.notchF[2].step(x);
  x = c.lpF[0].step(x); x = c.lpF[1].step(x); x = c.lpF[2].step(x); x = c.lpF[3].step(x);
  if (finished == 0) adcCount++;
  if (++c.decimPhase >= DECIM) {
    c.decimPhase = 0;
    uint32_t nx = (c.head + 1) % 1024;
    if (nx == c.tail) c.drops++;
    else { c.fifo[c.head] = (float)x; c.head = nx; }
  }
}

void ppsISR() { ppsAdcIndex = adcCount; ppsCount++; ppsFlag = true; }

// =================================================================== filters / calibration
double biquadPower(const Biquad &q, double f, double fs) {
  double w = 2.0 * PI * f / fs, c1 = cos(w), s1 = sin(w), c2 = cos(2 * w), s2 = sin(2 * w);
  double nr = q.b0 + q.b1 * c1 + q.b2 * c2, ni = -(q.b1 * s1 + q.b2 * s2);
  double dr = 1.0 + q.a1 * c1 + q.a2 * c2, di = -(q.a1 * s1 + q.a2 * s2);
  return (nr * nr + ni * ni) / (dr * dr + di * di);
}

void setupFilters() {
  static const double BW8_Q[4] = {0.50979558, 0.60134489, 0.89997622, 2.56291545};
  noInterrupts();
  for (int i = 0; i < 2; i++) {
    for (int h = 0; h < 3; h++) CH[i]->notchF[h].notch(mainsHz * (h + 1), 30.0, FS_ADC);
    for (int k = 0; k < 4; k++) CH[i]->lpF[k].lowpass(45.0, BW8_Q[k], FS_ADC);
  }
  interrupts();
  for (int i = 0; i < 2; i++)
    for (int b = 0; b < NBINS; b++) {
      double f = (double)b * FS_OUT / NFFT, h2 = 1.0;
      for (int h = 0; h < 3; h++) h2 *= biquadPower(CH[i]->notchF[h], f, FS_ADC);
      for (int k = 0; k < 4; k++) h2 *= biquadPower(CH[i]->lpF[k], f, FS_ADC);
      CH[i]->respCorr[b] = (h2 > 1e-4) ? (float)(1.0 / h2) : 1e4f;
    }
}

static inline float binHz(int b) { return (float)b * FS_OUT / (float)NFFT; }

// volts at the ADC input per code: full scale +/- 2*Vref/PGA over 2^23 codes
double voltsPerCode(const Channel &c) { return 2.0 * VREF / (1 << c.pgaCode) / 8388608.0; }

// volts at the SENSOR (before the preamp) per code
double sensorVoltsPerCode(const Channel &c) { return voltsPerCode(c) / c.frontGain; }

// H channel: tesla per sensor volt at frequency f
double teslaPerVolt(double f) {
  if (hByCoil) return 1.0 / (2.0 * PI * f * coilTurns * coilArea);
  return 1.0 / hFlatVperT;
}

// =================================================================== spectra
void welchReset() {
  nSeg = 0;
  for (int b = 0; b < NBINS; b++) { chH.Pxx[b] = chE.Pxx[b] = 0; Pxy_re[b] = Pxy_im[b] = 0; }
}

void welchSegment() {
  for (int i = 0; i < 2; i++) {
    Channel &c = *CH[i];
    double mean = 0;
    for (int k = 0; k < NFFT; k++) mean += c.ring[k];
    mean /= NFFT;
    for (int k = 0; k < NFFT; k++) {
      int j = (c.ringPos + k) % NFFT;
      c.fftIn[k] = (c.ring[j] - (float)mean) * win[k];
    }
    arm_rfft_fast_f32(&fftInst, c.fftIn, c.fftOut, 0);
    for (int b = 1; b < NBINS; b++)
      c.Pxx[b] += (double)c.fftOut[2 * b] * c.fftOut[2 * b] + (double)c.fftOut[2 * b + 1] * c.fftOut[2 * b + 1];
  }
  for (int b = 1; b < NBINS; b++) {                  // E * conj(H)
    double er = chE.fftOut[2 * b], ei = chE.fftOut[2 * b + 1], hr = chH.fftOut[2 * b], hi = chH.fftOut[2 * b + 1];
    Pxy_re[b] += er * hr + ei * hi;
    Pxy_im[b] += ei * hr - er * hi;
  }
  nSeg++;
}

// amplitude spectral density at the sensor in sensor volts / rtHz
double asdSensor(const Channel &c, int b) {
  if (nSeg == 0) return 0;
  double p = c.Pxx[b] * c.respCorr[b];
  return sqrt(2.0 * p / ((double)nSeg * FS_OUT * winSumSq)) * sensorVoltsPerCode(c);
}

double smoothP(const Channel &c, int b) {
  double s = 0; int n = 0;
  for (int k = b - 4; k <= b + 4; k++) if (k >= 1 && k < NBINS) { s += c.Pxx[k] * c.respCorr[k]; n++; }
  return n ? s / n : 0;
}

// peak of the smoothed spectrum in [f0,f1]: centre frequency (parabolic), ASD, ratio to the surround
void findMode(const Channel &c, float f0, float f1, float &fPeak, double &asdPeak, float &ratio) {
  int b0 = (int)(f0 * NFFT / FS_OUT), b1 = (int)(f1 * NFFT / FS_OUT);
  int best = b0; double pBest = smoothP(c, b0);
  for (int b = b0 + 1; b <= b1 && b < NBINS; b++) { double p = smoothP(c, b); if (p > pBest) { pBest = p; best = b; } }
  double pl = smoothP(c, best - 1), pr = smoothP(c, best + 1);
  double den = pl - 2 * pBest + pr;
  double delta = (den != 0) ? 0.5 * (pl - pr) / den : 0;
  fPeak = binHz(best) + (float)delta * FS_OUT / NFFT;
  asdPeak = sqrt(2.0 * pBest / ((double)nSeg * FS_OUT * winSumSq)) * sensorVoltsPerCode(c);
  int w = (int)(2.0f * NFFT / FS_OUT);
  static float tmp[160]; int n = 0;
  for (int b = b0 - w; b <= b1 + w && n < 160; b++) {
    if (b < 1 || b >= NBINS || (b >= b0 && b <= b1)) continue;
    tmp[n++] = (float)(c.Pxx[b] * c.respCorr[b]);
  }
  if (n < 3) { ratio = 0; return; }
  for (int i = 1; i < n; i++) { float v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = v; }
  ratio = (tmp[n / 2] > 0) ? sqrtf((float)pBest / tmp[n / 2]) : 0;
}

// E/H at a frequency: |Pxy| / Pxx(H) gives E/H in (sensor V_E / sensor V_H); converted to V/m per T
void impedanceAt(float fc, double &z_ohm, double &coh) {
  int b = (int)(fc * NFFT / FS_OUT + 0.5f);
  double sxy_r = 0, sxy_i = 0, sxx = 0, syy = 0;
  for (int k = b - 4; k <= b + 4; k++) {
    if (k < 1 || k >= NBINS) continue;
    sxy_r += Pxy_re[k]; sxy_i += Pxy_im[k]; sxx += chH.Pxx[k]; syy += chE.Pxx[k];
  }
  if (sxx <= 0 || syy <= 0) { z_ohm = 0; coh = 0; return; }
  double mag = sqrt(sxy_r * sxy_r + sxy_i * sxy_i);
  coh = mag * mag / (sxx * syy);
  // E/H transfer in code units -> sensor volts -> field units
  double e_per_h_codes = mag / sxx;
  double e_vm_per_h_t = e_per_h_codes * (sensorVoltsPerCode(chE) / eHeff) / (sensorVoltsPerCode(chH) * teslaPerVolt(fc));
  z_ohm = e_vm_per_h_t * MU0;                        // Z = E / (B/mu0), ohms; free space = 377
}

// =================================================================== GPS
void parseNmea(char *s) {
  if (strlen(s) < 20 || s[0] != '$' || strncmp(s + 3, "RMC", 3) != 0) return;
  char *f[13]; int n = 0;
  for (char *p = s; n < 13 && p; n++) { f[n] = p; p = strchr(p, ','); if (p) *p++ = 0; }
  if (n < 10) return;
  gpsValid = (f[2][0] == 'A');
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

// =================================================================== report
void report() {
  if (nSeg == 0) return;
  char ts[24]; timeStamp(ts, sizeof(ts));
  float fH[NMODE], rH[NMODE], fE[NMODE], rE[NMODE];
  double aH[NMODE], aE[NMODE];
  for (int m = 0; m < NMODE; m++) {
    findMode(chH, MODE_HZ[m] - 1.5f, MODE_HZ[m] + 1.5f, fH[m], aH[m], rH[m]);
    findMode(chE, MODE_HZ[m] - 1.5f, MODE_HZ[m] + 1.5f, fE[m], aE[m], rE[m]);
  }
  double z1, c1, z2, c2;
  impedanceAt(fH[0], z1, c1);
  impedanceAt(fH[1], z2, c2);
  int b30 = (int)(30.0f * NFFT / FS_OUT);
  double floorH_pT = asdSensor(chH, b30) * teslaPerVolt(30.0) * 1e12;
  char line[700]; int n = 0;
  if (jsonOut) {
    n = snprintf(line, sizeof(line), "{\"t\":\"%s\",\"nseg\":%d,\"modes\":[", ts, nSeg);
    for (int m = 0; m < NMODE; m++)
      n += snprintf(line + n, sizeof(line) - n, "%s{\"f\":%.3f,\"H_pT\":%.4f,\"Hx\":%.2f,\"E_uVm\":%.4f,\"Ex\":%.2f}",
                    m ? "," : "", fH[m], aH[m] * teslaPerVolt(fH[m]) * 1e12, rH[m], aE[m] / eHeff * 1e6, rE[m]);
    n += snprintf(line + n, sizeof(line) - n, "],\"Z1\":%.1f,\"coh1\":%.3f,\"Z2\":%.1f,\"coh2\":%.3f,\"floor30_pT\":%.4f,\"drops\":%lu}",
                  z1, c1, z2, c2, floorH_pT, (unsigned long)(chH.drops + chE.drops));
  } else {
    n = snprintf(line, sizeof(line), "%s,%d", ts, nSeg);
    for (int m = 0; m < NMODE; m++)
      n += snprintf(line + n, sizeof(line) - n, ",%.3f,%.4f,%.2f,%.4f", fH[m], aH[m] * teslaPerVolt(fH[m]) * 1e12, rH[m], aE[m] / eHeff * 1e6);
    n += snprintf(line + n, sizeof(line) - n, ",%.1f,%.3f,%.1f,%.3f,%.4f", z1, c1, z2, c2, floorH_pT);
  }
  Serial.println(line);
  if (sdOk) {
    bool isNew = !SD.exists("SCHUMANN.CSV");
    File f = SD.open("SCHUMANN.CSV", FILE_WRITE);
    if (f) {
      if (isNew) {
        f.print("time,segments");
        for (int m = 1; m <= NMODE; m++) f.printf(",m%d_Hz,m%d_H_pT_rtHz,m%d_H_x_floor,m%d_E_uVm_rtHz", m, m, m, m);
        f.println(",Z1_ohm,coh1,Z2_ohm,coh2,H_floor30_pT_rtHz");
      }
      f.println(line);
      f.close();
    }
  }
  for (int i = 0; i < 2; i++) {
    Channel &c = *CH[i];
    if (c.rawMin <= -8380000 || c.rawMax >= 8380000)
      Serial.printf("# WARNING: %s channel hit full scale (codes %ld..%ld): lower pga/gain\n", c.name, (long)c.rawMin, (long)c.rawMax);
    noInterrupts(); c.rawMin = 0x7FFFFFFF; c.rawMax = -0x7FFFFFFF; interrupts();
  }
  welchReset();
}

void printSpectrum() {
  Serial.printf("# f_Hz,H_pT_rtHz,E_uVm_rtHz  (%d averages)\n", nSeg);
  for (int b = 4; b < NBINS; b++)
    Serial.printf("%.4f,%.5f,%.5f\n", binHz(b), asdSensor(chH, b) * teslaPerVolt(binHz(b)) * 1e12, asdSensor(chE, b) / eHeff * 1e6);
}

// =================================================================== service
void service() {
  // both channels advance together (same decimation phase); pull pairs
  while (chH.tail != chH.head && chE.tail != chE.head) {
    for (int i = 0; i < 2; i++) {
      Channel &c = *CH[i];
      float x = c.fifo[c.tail];
      c.tail = (c.tail + 1) % 1024;
      c.ring[c.ringPos] = x;
      c.ringPos = (c.ringPos + 1) % NFFT;
      c.ringCount++;
    }
    outIndex++;
    if (rawOpen) {
      rawFile.printf("%lu,%.1f,%.1f\n", (unsigned long)outIndex, chH.ring[(chH.ringPos + NFFT - 1) % NFFT], chE.ring[(chE.ringPos + NFFT - 1) % NFFT]);
      if (outIndex % 1250 == 0) rawFile.flush();
    }
    if (chH.ringCount >= NFFT && ((chH.ringCount - NFFT) % HOP) == 0) {
      welchSegment();
      if (streamOn) {
        Serial.printf("# seg %d:", nSeg);
        for (int m = 0; m < NMODE; m++) {
          float fp, r; double a;
          findMode(chH, MODE_HZ[m] - 1.5f, MODE_HZ[m] + 1.5f, fp, a, r);
          Serial.printf("  %.2f Hz %.3f pT (x%.1f)", fp, a * teslaPerVolt(fp) * 1e12, r);
        }
        Serial.println();
      }
    }
  }
  if (ppsFlag) {
    noInterrupts(); ppsFlag = false; uint32_t idx = ppsAdcIndex; interrupts();
    if (rawOpen && gpsValid) rawFile.printf("PPS,%s,%.4f\n", gpsStamp, (double)idx / DECIM);
  }
}

void startLogging() {
  welchReset();
  chH.ringCount = chE.ringCount = 0; chH.ringPos = chE.ringPos = 0;
  sinceReport = 0; logging = true;
  if (rawWanted && sdOk) {
    char name[16];
    for (int i = 0; i < 1000; i++) { snprintf(name, sizeof(name), "SRAW%03d.CSV", i); if (!SD.exists(name)) break; }
    rawFile = SD.open(name, FILE_WRITE);
    if (rawFile) { rawOpen = true; rawFile.println("index,H_codes,E_codes"); Serial.printf("# raw -> %s\n", name); }
  }
  Serial.printf("# logging: report every %lu s, mains %.0f Hz, H gain %.0f pga %d, E gain %.0f pga %d\n",
                (unsigned long)(reportMs / 1000), mainsHz, chH.frontGain, 1 << chH.pgaCode, chE.frontGain, 1 << chE.pgaCode);
}

void stopLogging() { logging = false; if (rawOpen) { rawOpen = false; rawFile.close(); } Serial.println("# stopped"); }

void printHelp() {
  Serial.println(F("# schumann_monitor: start [s] | stop | live | stat | gain <h|e> <x> | pga <h|e> <1..64> |\n"
                   "#   coil <turns> <area_m2> | hflat <V_per_pT> | ecal <h_eff_m> | mains <50|60> |\n"
                   "#   json <on|off> | stream <on|off> | raw <on|off>"));
}

uint8_t pgaCodeOf(int g) { int c = 0; while ((1 << c) < g && c < 6) c++; return c; }

void handleLine(char *line) {
  char *tok[4]; int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "start")) { if (n >= 2) { float s = atof(tok[1]); if (s < 20) s = 20; reportMs = (uint32_t)(s * 1000); } startLogging(); }
  else if (!strcmp(c, "stop")) stopLogging();
  else if (!strcmp(c, "live")) { if (nSeg == 0) Serial.println("# no finished segment yet (the first takes 16.4 s)"); else printSpectrum(); }
  else if (!strcmp(c, "gain") && n >= 3) { Channel &ch = (tok[1][0] == 'e') ? chE : chH; float g = atof(tok[2]); if (g > 0) ch.frontGain = g; Serial.printf("# %s front-end gain %.1f\n", ch.name, ch.frontGain); }
  else if (!strcmp(c, "pga") && n >= 3) { Channel &ch = (tok[1][0] == 'e') ? chE : chH; ch.pgaCode = pgaCodeOf(atoi(tok[2])); Serial.printf("# %s PGA %d\n", ch.name, 1 << ch.pgaCode); }
  else if (!strcmp(c, "coil") && n >= 3) { hByCoil = true; coilTurns = atof(tok[1]); coilArea = atof(tok[2]); Serial.printf("# H by coil: %.0f turns, %.5f m2 -> %.3f uV per pT at 7.83 Hz\n", coilTurns, coilArea, 2 * PI * 7.83 * coilTurns * coilArea * 1e-12 * 1e6); }
  else if (!strcmp(c, "hflat") && n >= 2) { hByCoil = false; hFlatVperT = atof(tok[1]) * 1e12; Serial.printf("# H flat sensor: %.3g V per pT\n", atof(tok[1])); }
  else if (!strcmp(c, "ecal") && n >= 2) { eHeff = atof(tok[1]); Serial.printf("# E effective height %.3f m\n", eHeff); }
  else if (!strcmp(c, "mains") && n >= 2) { mainsHz = (atoi(tok[1]) == 50) ? 50.0 : 60.0; setupFilters(); Serial.printf("# mains notch %.0f Hz\n", mainsHz); }
  else if (!strcmp(c, "json") && n >= 2) jsonOut = !strcmp(tok[1], "on");
  else if (!strcmp(c, "stream") && n >= 2) streamOn = !strcmp(tok[1], "on");
  else if (!strcmp(c, "raw") && n >= 2) { rawWanted = !strcmp(tok[1], "on"); Serial.printf("# raw %s (next start)\n", rawWanted ? "on" : "off"); }
  else if (!strcmp(c, "stat")) {
    char ts[24]; timeStamp(ts, sizeof(ts));
    Serial.printf("# %s, GPS %s, PPS %lu, SD %s, %s, segments %d, conversions %lu, drops %lu/%lu, H codes %ld..%ld, E codes %ld..%ld\n",
                  ts, gpsValid ? "fix" : "no fix", (unsigned long)ppsCount, sdOk ? "present" : "none", logging ? "LOGGING" : "idle", nSeg,
                  (unsigned long)adcCount, (unsigned long)chH.drops, (unsigned long)chE.drops,
                  (long)chH.rawMin, (long)chH.rawMax, (long)chE.rawMin, (long)chE.rawMax);
  } else Serial.println("# unknown command, type help");
}

// =================================================================== setup / loop
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(PIN_PPS, INPUT);
  SPI.begin();
  arm_rfft_fast_init_f32(&fftInst, NFFT);
  for (int i = 0; i < NFFT; i++) { win[i] = 0.5f - 0.5f * cosf(TWO_PI * (float)i / (float)NFFT); winSumSq += (double)win[i] * win[i]; }
  setupFilters();
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  bool ads = adsInit();
  Serial.printf("# schumann_monitor -- ADS1256 %s\n", ads ? "found" : "NOT responding (check SPI wiring and 5 V)");
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), drdyISR, FALLING);
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
  service();
  if (logging && sinceReport >= reportMs) { sinceReport = 0; report(); }
}
