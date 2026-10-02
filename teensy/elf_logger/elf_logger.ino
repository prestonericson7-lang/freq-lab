/*
 * elf_logger.ino -- Teensy 4.1 extremely-low-frequency logger, 0.25 - 45 Hz
 *
 * WHAT IT DOES
 *   A modern version of the ELF monitor SRI built in 1984
 *   (CIA-RDP96-00788R001800300001-0): a sensor, x1000 of gain, an FFT, and a
 *   line of numbers every half hour. SRI had an 8-bit converter and a home
 *   computer and logged 19 frequencies from 1 to 30 Hz. This samples at 4 kHz,
 *   notches out the mains, decimates to 250 Hz, runs 4096-point spectra
 *   (0.061 Hz resolution) continuously, stamps everything with GPS time, and
 *   keeps the raw waveform so two stations can be compared sample by sample.
 *
 *   Each report gives:
 *     - the same 19 bands SRI logged (1.5625 Hz steps up to 29.7 Hz), in
 *       microvolts rms at the sensor
 *     - the strongest line near each of the first three Earth-ionosphere
 *       (Schumann) modes, about 7.8, 14 and 20 Hz, and how far each stands
 *       above the spectrum around it
 *
 * WHAT YOU HAVE TO BUILD IN FRONT OF IT
 *   The Teensy's ADC needs a signal centred on 1.65 V, within 0..3.3 V.
 *   Sensor (induction coil, or two electrodes like SRI's tree) -> differential
 *   or instrumentation amplifier with a gain of about 1000 -> a 1.65 V bias ->
 *   pin A0. Tell the sketch the gain with `gain 1000` so results come out in
 *   volts at the sensor. Put a low-pass in the amplifier or right at the pin
 *   (1.6 kOhm in series, 1 uF from A0 to GND = 100 Hz corner): the converter
 *   samples at 4 kHz and anything left near 4, 8, 12 kHz lands in the band.
 *   Battery power and distance from house wiring matter more than anything in
 *   this file.
 *
 * WIRING (Teensy 4.1)
 *   pin 14 (A0) <- amplifier output, 0..3.3 V centred on 1.65 V
 *   pin 0 (RX1) <- GPS module TX (9600 baud NMEA)        optional
 *   pin 2       <- GPS module PPS                        optional
 *   3.3 V / GND -> GPS module
 *   SD card in the Teensy's slot.
 *
 * COMMANDS (USB serial, newline-terminated)
 *   start [report_seconds]   begin logging; default report every 300 s
 *   stop
 *   live                     print the full spectrum of the current average now
 *   gain <x>                 front-end voltage gain (default 1000)
 *   mains <50|60>            mains frequency to notch (default 60)
 *   raw <on|off>             also write the 250 Hz waveform to the SD card
 *   full <on|off>            also write every report's full spectrum to the SD card
 *   stat
 *
 * FILES ON THE SD CARD
 *   ELFSPEC.CSV   one line per report: time, 19 bands, three mode lines
 *   RAWnnn.CSV    waveform, with a "PPS,<UTC>,<sample position>" line at every GPS second
 *   SPCnnn.CSV    full spectra (when `full on`)
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62). Filters, spectra, report and
 *   GPS stamping run on a PC against a simulated front end. Not yet run on hardware.
 */

#include <Arduino.h>
#include <SD.h>
#include <arm_math.h>

const int    PIN_ADC  = 14;       // A0
const int    PIN_PPS  = 2;
const double FS_ADC   = 4000.0;   // ADC sample rate
const int    DECIM    = 16;
const float  FS_OUT   = 250.0f;   // after decimation
const int    NFFT     = 4096;
const int    HOP      = 2048;
const int    NBINS    = 760;      // up to 46 Hz
const float  ADC_LSB  = 3.3f / 4096.0f;

// ----------------------------------------------------------------- filters
struct Biquad {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  inline double step(double x) {                      // transposed direct form II
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
Biquad notchF[3];      // mains and its 2nd and 3rd harmonics
Biquad lpF[4];         // 8th-order Butterworth low-pass at 45 Hz: with 60 Hz mains the
                       // 240 Hz harmonic folds to 10 Hz after decimation, this puts it 116 dB down
double mainsHz = 60.0;

// ----------------------------------------------------------------- sampling
IntervalTimer adcTimer;
const int FIFO_N = 1024;
volatile float    fifo[FIFO_N];
volatile uint32_t fifoHead = 0, fifoTail = 0, fifoDrops = 0;
volatile uint32_t adcCount = 0;            // 4 kHz sample counter
volatile int      decimPhase = 0;
volatile int      adcMin = 4095, adcMax = 0;
volatile uint32_t ppsCount = 0, ppsAdcIndex = 0;
volatile bool     ppsFlag = false;

// ----------------------------------------------------------------- spectra
arm_rfft_fast_instance_f32 fftInst;
float  win[NFFT];
double winSumSq = 0, winSum = 0;
float  ring[NFFT];
int    ringPos = 0;
long   ringCount = 0;
float  fftIn[NFFT], fftOut[NFFT];
double Pxx[NBINS];
float  respCorr[NBINS];                     // 1 / |filter response|^2 at each bin
int    nSeg = 0;

// ----------------------------------------------------------------- state
float  gainFront = 1000.0f;
bool   logging = false, rawWanted = false, fullWanted = false, sdOk = false;
uint32_t reportMs = 300000;
elapsedMillis sinceReport;
File   rawFile;
bool   rawOpen = false;
uint32_t outIndex = 0;                      // 250 Hz sample counter
uint32_t reportNo = 0;

// GPS
char   nmea[100];
int    nmeaLen = 0;
bool   gpsValid = false;
char   gpsStamp[24] = "";                   // 20yy-mm-ddThh:mm:ssZ
elapsedMillis gpsAge;
bool   ppsPending = false;                  // a PPS edge is waiting for its time sentence
double ppsPendingIdx = 0;
elapsedMillis ppsPendingAge;

// =================================================================== ISRs
void adcISR() {
  int code = analogRead(PIN_ADC);
  if (code < adcMin) adcMin = code;
  if (code > adcMax) adcMax = code;
  double x = (double)code;
  x = notchF[0].step(x);
  x = notchF[1].step(x);
  x = notchF[2].step(x);
  x = lpF[0].step(x);
  x = lpF[1].step(x);
  x = lpF[2].step(x);
  x = lpF[3].step(x);
  adcCount++;
  if (++decimPhase >= DECIM) {
    decimPhase = 0;
    uint32_t next = (fifoHead + 1) % FIFO_N;
    if (next == fifoTail) fifoDrops++;
    else { fifo[fifoHead] = (float)x; fifoHead = next; }
  }
}

void ppsISR() {
  ppsAdcIndex = adcCount;
  ppsCount++;
  ppsFlag = true;
}

// |H|^2 of one biquad at frequency f
double biquadPower(const Biquad &q, double f) {
  double w = 2.0 * PI * f / FS_ADC, c1 = cos(w), s1 = sin(w), c2 = cos(2 * w), s2 = sin(2 * w);
  double nr = q.b0 + q.b1 * c1 + q.b2 * c2, ni = -(q.b1 * s1 + q.b2 * s2);
  double dr = 1.0 + q.a1 * c1 + q.a2 * c2, di = -(q.a1 * s1 + q.a2 * s2);
  return (nr * nr + ni * ni) / (dr * dr + di * di);
}

void setupFilters() {
  static const double BW8_Q[4] = {0.50979558, 0.60134489, 0.89997622, 2.56291545};
  noInterrupts();
  for (int h = 0; h < 3; h++) notchF[h].notch(mainsHz * (h + 1), 30.0, FS_ADC);
  for (int k = 0; k < 4; k++) lpF[k].lowpass(45.0, BW8_Q[k], FS_ADC);
  interrupts();
  // The filters' own response is known exactly, so take it back out of every spectrum.
  for (int b = 0; b < NBINS; b++) {
    double f = (double)b * FS_OUT / NFFT, h2 = 1.0;
    for (int h = 0; h < 3; h++) h2 *= biquadPower(notchF[h], f);
    for (int k = 0; k < 4; k++) h2 *= biquadPower(lpF[k], f);
    respCorr[b] = (h2 > 1e-4) ? (float)(1.0 / h2) : 1e4f;
  }
}

// =================================================================== GPS
// Minimal $..RMC parser: UTC time, validity flag, date.
void parseNmea(char *s) {
  if (strlen(s) < 20 || s[0] != '$' || strncmp(s + 3, "RMC", 3) != 0) return;
  char *f[13];
  int n = 0;
  for (char *p = s; n < 13 && p; n++) {
    f[n] = p;
    p = strchr(p, ',');
    if (p) *p++ = 0;
  }
  if (n < 10) return;
  gpsValid = (f[2][0] == 'A');
  if (strlen(f[1]) >= 6 && strlen(f[9]) >= 6) {
    snprintf(gpsStamp, sizeof(gpsStamp), "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
             f[9][4], f[9][5], f[9][2], f[9][3], f[9][0], f[9][1],
             f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
    gpsAge = 0;
    // The sentence that follows a PPS edge carries that edge's time.
    if (ppsPending && ppsPendingAge < 950 && gpsValid && rawOpen)
      rawFile.printf("PPS,%s,%.4f\n", gpsStamp, ppsPendingIdx);
    ppsPending = false;
  }
}

void pollGps() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (nmeaLen > 0) {
        nmea[nmeaLen] = 0;
        nmeaLen = 0;
        parseNmea(nmea);
      }
    } else if (nmeaLen < (int)sizeof(nmea) - 1) {
      nmea[nmeaLen++] = c;
    }
  }
}

// "2026-09-30T18:00:00Z" when GPS has a fix, otherwise seconds since power-up
void timeStamp(char *buf, size_t n) {
  if (gpsValid && gpsStamp[0] && gpsAge < 3000) snprintf(buf, n, "%s", gpsStamp);
  else snprintf(buf, n, "up%lus", (unsigned long)(millis() / 1000));
}

// =================================================================== spectra
void welchReset() {
  nSeg = 0;
  for (int b = 0; b < NBINS; b++) Pxx[b] = 0;
}

void welchSegment() {
  double mean = 0;
  for (int i = 0; i < NFFT; i++) mean += ring[i];
  mean /= NFFT;
  for (int i = 0; i < NFFT; i++) {
    int j = (ringPos + i) % NFFT;
    fftIn[i] = (ring[j] - (float)mean) * win[i];
  }
  arm_rfft_fast_f32(&fftInst, fftIn, fftOut, 0);
  for (int b = 1; b < NBINS; b++)
    Pxx[b] += (double)fftOut[2 * b] * fftOut[2 * b] + (double)fftOut[2 * b + 1] * fftOut[2 * b + 1];
  nSeg++;
}

static inline float binHz(int b) { return (float)b * FS_OUT / (float)NFFT; }

// volts per ADC count, referred to the sensor
static inline double scaleV() { return (double)ADC_LSB / (double)gainFront; }

// averaged power in one bin, with the filter response taken out
static inline double pw(int b) { return Pxx[b] * (double)respCorr[b]; }

// amplitude spectral density at the sensor, volts per root-Hz
double asdOf(double p) {
  if (nSeg == 0) return 0;
  return sqrt(2.0 * p / ((double)nSeg * FS_OUT * winSumSq)) * scaleV();
}
double asd(int b) { return asdOf(pw(b)); }

// rms voltage at the sensor between two frequencies
double bandRms(float f0, float f1) {
  if (nSeg == 0) return 0;
  int b0 = (int)ceilf(f0 * NFFT / FS_OUT), b1 = (int)floorf(f1 * NFFT / FS_OUT);
  double s = 0;
  for (int b = max(1, b0); b <= b1 && b < NBINS; b++) s += pw(b);
  return sqrt(2.0 * s / ((double)nSeg * winSumSq * NFFT)) * scaleV();
}

// Power averaged over +-4 bins (0.55 Hz). The Earth-ionosphere modes are humps a
// couple of hertz wide, so the single strongest bin would mostly be noise.
double smoothP(int b) {
  double s = 0;
  int n = 0;
  for (int k = b - 4; k <= b + 4; k++)
    if (k >= 1 && k < NBINS) { s += pw(k); n++; }
  return n ? s / n : 0;
}

// Highest point of the smoothed spectrum between f0 and f1, and how far it stands
// above the median of the two hertz either side. Flat noise gives 1.1 to 1.2
// (it is the highest of about 30 noisy points), so only larger numbers mean anything.
void findMode(float f0, float f1, float &fPeak, double &asdPeak, float &ratio) {
  int b0 = (int)(f0 * NFFT / FS_OUT), b1 = (int)(f1 * NFFT / FS_OUT);
  int best = b0;
  double pBest = smoothP(b0);
  for (int b = b0 + 1; b <= b1 && b < NBINS; b++) {
    double p = smoothP(b);
    if (p > pBest) { pBest = p; best = b; }
  }
  fPeak = binHz(best);
  asdPeak = asdOf(pBest);
  int w = (int)(2.0f * NFFT / FS_OUT);
  static float tmp[160];
  int n = 0;
  for (int b = b0 - w; b <= b1 + w && n < 160; b++) {
    if (b < 1 || b >= NBINS) continue;
    if (b >= b0 && b <= b1) continue;
    tmp[n++] = (float)pw(b);
  }
  if (n < 3) { ratio = 0; return; }
  for (int i = 1; i < n; i++) {                       // insertion sort, n is small
    float v = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  float med = tmp[n / 2];
  ratio = (med > 0) ? sqrtf((float)pBest / med) : 0;
}

void printSpectrum(Print &o) {
  o.printf("# amplitude spectral density at the sensor, nV per root-Hz, %d averages, gain %.0f\n", nSeg, gainFront);
  o.println("f_Hz,nV_rtHz");
  for (int b = 4; b < NBINS; b++) o.printf("%.4f,%.3f\n", binHz(b), asd(b) * 1e9);
}

void report() {
  if (nSeg == 0) return;
  char ts[24];
  timeStamp(ts, sizeof(ts));
  reportNo++;
  char line[420];
  int n = snprintf(line, sizeof(line), "%s,%d", ts, nSeg);
  for (int k = 1; k <= 19; k++) {
    float fc = 1.5625f * k;
    n += snprintf(line + n, sizeof(line) - n, ",%.4f", bandRms(fc - 0.78125f, fc + 0.78125f) * 1e6);
  }
  const float win0[3] = {7.0f, 13.0f, 19.0f}, win1[3] = {9.0f, 15.5f, 21.5f};
  for (int m = 0; m < 3; m++) {
    float fp, pr;
    double ap;
    findMode(win0[m], win1[m], fp, ap, pr);
    n += snprintf(line + n, sizeof(line) - n, ",%.3f,%.2f,%.2f", fp, ap * 1e9, pr);
  }
  Serial.println(line);
  if (sdOk) {
    bool isNew = !SD.exists("ELFSPEC.CSV");
    File f = SD.open("ELFSPEC.CSV", FILE_WRITE);
    if (f) {
      if (isNew) {
        f.print("time,averages");
        for (int k = 1; k <= 19; k++) f.printf(",uV_%.2fHz", 1.5625f * k);
        f.println(",m1_Hz,m1_nV_rtHz,m1_x_floor,m2_Hz,m2_nV_rtHz,m2_x_floor,m3_Hz,m3_nV_rtHz,m3_x_floor");
      }
      f.println(line);
      f.close();
    }
    if (fullWanted) {
      char name[16];
      snprintf(name, sizeof(name), "SPC%03lu.CSV", (unsigned long)(reportNo % 1000));
      File g = SD.open(name, FILE_WRITE);
      if (g) {
        g.printf("# %s\n", ts);
        printSpectrum(g);
        g.close();
      }
    }
  }
  uint32_t drops = fifoDrops;
  int lo = adcMin, hi = adcMax;
  noInterrupts();
  adcMin = 4095; adcMax = 0;
  interrupts();
  if (lo <= 8 || hi >= 4087)
    Serial.printf("# WARNING: the ADC hit a rail (codes %d..%d). Lower the front-end gain or the mains pickup.\n", lo, hi);
  if (drops) Serial.printf("# %lu samples dropped so far\n", (unsigned long)drops);
  welchReset();
}

// Pull decimated samples out of the ISR's queue and process them.
void service() {
  while (fifoTail != fifoHead) {
    float x = fifo[fifoTail];
    fifoTail = (fifoTail + 1) % FIFO_N;
    ring[ringPos] = x;
    ringPos = (ringPos + 1) % NFFT;
    ringCount++;
    outIndex++;
    if (rawOpen) {
      rawFile.printf("%lu,%.3f\n", (unsigned long)outIndex, x);
      if (outIndex % 1250 == 0) rawFile.flush();        // at most 5 s lost if power drops
    }
    if (ringCount >= NFFT && ((ringCount - NFFT) % HOP) == 0) welchSegment();
  }
  if (ppsFlag) {
    noInterrupts();
    ppsFlag = false;
    uint32_t idx = ppsAdcIndex;
    interrupts();
    // position of the GPS second in units of the 250 Hz samples written above
    ppsPendingIdx = (double)idx / DECIM;
    ppsPendingAge = 0;
    ppsPending = true;
  }
}

void startLogging() {
  welchReset();
  ringCount = 0;
  ringPos = 0;
  sinceReport = 0;
  logging = true;
  if (rawWanted && sdOk) {
    char name[16];
    for (int i = 0; i < 1000; i++) {
      snprintf(name, sizeof(name), "RAW%03d.CSV", i);
      if (!SD.exists(name)) break;
    }
    rawFile = SD.open(name, FILE_WRITE);
    if (rawFile) {
      rawOpen = true;
      rawFile.printf("# 250 Hz samples in ADC counts after mains notch and 45 Hz low-pass; gain %.0f; %.9g V per count at the sensor\n",
                     gainFront, scaleV());
      rawFile.println("# the filters delay the waveform by about 18 ms (4.6 samples) relative to the PPS marks");
      rawFile.println("index,counts");
      Serial.printf("# raw waveform -> %s\n", name);
    }
  }
  Serial.printf("# logging: report every %lu s, mains notch %.0f Hz, gain %.0f\n",
                (unsigned long)(reportMs / 1000), mainsHz, gainFront);
  Serial.print("time,averages");
  for (int k = 1; k <= 19; k++) Serial.printf(",uV_%.2fHz", 1.5625f * k);
  Serial.println(",m1_Hz,m1_nV_rtHz,m1_x_floor,m2_Hz,m2_nV_rtHz,m2_x_floor,m3_Hz,m3_nV_rtHz,m3_x_floor");
}

void stopLogging() {
  logging = false;
  if (rawOpen) {
    rawOpen = false;
    rawFile.close();
  }
  Serial.println("# stopped");
}

void printHelp() {
  Serial.println(F(
    "# elf_logger commands:\n"
    "#  start [report_seconds] | stop | live | stat\n"
    "#  gain <x> | mains <50|60> | raw <on|off> | full <on|off>"));
}

void handleLine(char *line) {
  char *tok[4];
  int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    printHelp();
  } else if (!strcmp(c, "start")) {
    if (n >= 2) {
      float s = atof(tok[1]);
      if (s < 30) s = 30;
      reportMs = (uint32_t)(s * 1000.0f);
    }
    startLogging();
  } else if (!strcmp(c, "stop")) {
    stopLogging();
  } else if (!strcmp(c, "live")) {
    if (nSeg == 0) Serial.println("# no finished segment yet (the first takes 16.4 s)");
    else printSpectrum(Serial);
  } else if (!strcmp(c, "gain") && n >= 2) {
    float g = atof(tok[1]);
    if (g > 0) gainFront = g;
    Serial.printf("# front-end gain %.1f\n", gainFront);
  } else if (!strcmp(c, "mains") && n >= 2) {
    mainsHz = (atoi(tok[1]) == 50) ? 50.0 : 60.0;
    setupFilters();
    Serial.printf("# mains notch %.0f Hz\n", mainsHz);
  } else if (!strcmp(c, "raw") && n >= 2) {
    rawWanted = !strcmp(tok[1], "on");
    Serial.printf("# raw waveform logging %s (takes effect at the next start)\n", rawWanted ? "on" : "off");
  } else if (!strcmp(c, "full") && n >= 2) {
    fullWanted = !strcmp(tok[1], "on");
    Serial.printf("# full spectra %s\n", fullWanted ? "on" : "off");
  } else if (!strcmp(c, "stat")) {
    char ts[24];
    timeStamp(ts, sizeof(ts));
    Serial.printf("# %s, GPS %s, PPS pulses %lu, SD %s, %s, segments %d, ADC codes %d..%d, dropped %lu\n",
                  ts, gpsValid ? "fix" : "no fix", (unsigned long)ppsCount, sdOk ? "present" : "none",
                  logging ? "LOGGING" : "idle", nSeg, (int)adcMin, (int)adcMax, (unsigned long)fifoDrops);
  } else {
    Serial.println("# unknown command, type help");
  }
}

// =================================================================== setup/loop
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(PIN_PPS, INPUT);
  analogReadResolution(12);
  analogReadAveraging(4);
  arm_rfft_fast_init_f32(&fftInst, NFFT);
  for (int i = 0; i < NFFT; i++) {
    win[i] = 0.5f - 0.5f * cosf(TWO_PI * (float)i / (float)NFFT);
    winSum += win[i];
    winSumSq += (double)win[i] * win[i];
  }
  setupFilters();
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsISR, RISING);
  adcTimer.priority(48);
  adcTimer.begin(adcISR, 1000000.0 / FS_ADC);
  Serial.println("# elf_logger -- 0.25 to 45 Hz spectrum logger");
  printHelp();
}

void loop() {
  static char line[64];
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
  pollGps();
  service();
  if (logging && sinceReport >= reportMs) {
    sinceReport = 0;
    report();
  }
}
