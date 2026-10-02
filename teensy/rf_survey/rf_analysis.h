// rf_analysis.h -- the arithmetic of rf_survey.ino, kept free of Arduino so a PC
// can compile and test it (test/pc_test.cpp). Included by the sketch.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

// ------------------------------------------------------------- detector calibration
struct RfCal {
  double slope_v_per_db = -0.0244;      // AD8318 typical at 2.2 GHz
  double intercept_dbm = 20.0;          // V = slope * (P - intercept)
  double freq_ghz = 2.44, gain_dbi = 2.0;
  double p1_dbm = 0, v1 = 0; int npts = 0;

  void reset() { slope_v_per_db = -0.0244; intercept_dbm = 20.0; npts = 0; }
  double dbmOf(double volts) const { return intercept_dbm + volts / slope_v_per_db; }
  double voltsOf(double dbm) const { return slope_v_per_db * (dbm - intercept_dbm); }
  // two points fit slope and intercept; returns how many points are held
  int addPoint(double dbm, double volts) {
    if (npts == 0) { p1_dbm = dbm; v1 = volts; npts = 1; return 1; }
    if (fabs(dbm - p1_dbm) < 5.0) { p1_dbm = dbm; v1 = volts; return 1; }   // too close: replace
    slope_v_per_db = (volts - v1) / (dbm - p1_dbm);
    intercept_dbm = dbm - volts / slope_v_per_db;
    npts = 2;
    return 2;
  }
  void setAntenna(double ghz, double dbi) { freq_ghz = ghz; gain_dbi = dbi; }
  // effective aperture in cm2: G * lambda^2 / (4 pi)
  double aeff_cm2() const {
    double lambda_m = 0.299792458 / freq_ghz;
    return pow(10.0, gain_dbi / 10.0) * lambda_m * lambda_m / (4.0 * M_PI) * 1e4;
  }
  double uw_cm2(double dbm) const { return pow(10.0, dbm / 10.0) * 1000.0 / aeff_cm2(); }   // dBm -> mW -> uW
};

// ------------------------------------------------------------- level per second
struct LevelResult { double mean_dbm, peak_dbm, uw_cm2, duty; };

struct LevelAccumulator {
  double sum_mw = 0, peak = -200, last_v = 0; uint32_t n = 0, n_above = 0;
  double duty_ref_dbm = -40;
  void add(double dbm) { sum_mw += pow(10.0, dbm / 10.0); if (dbm > peak) peak = dbm; n++; if (dbm > duty_ref_dbm) n_above++; }
  void addVolts(double v) { last_v = v; }
  double lastVolts() const { return last_v; }
  void reset() { sum_mw = 0; peak = -200; n = 0; n_above = 0; }
  LevelResult result(const RfCal &cal) const {
    LevelResult r;
    r.mean_dbm = n ? 10.0 * log10(sum_mw / n) : -200;
    r.peak_dbm = peak;
    r.uw_cm2 = cal.uw_cm2(r.mean_dbm);
    r.duty = n ? (double)n_above / n : 0;
    return r;
  }
};

// ------------------------------------------------------------- pulse intervals
struct IntervalPeak { double ms; uint32_t count; };

struct IntervalHistogram {
  // 120 log-spaced bins from 0.05 ms to 2000 ms (about 9.2 % per bin)
  static const int NB = 120;
  uint32_t bins[NB];
  uint32_t count = 0;
  double lo_ms = 0.05, hi_ms = 2000.0;
  IntervalHistogram() { reset(); }
  void reset() { memset(bins, 0, sizeof(bins)); count = 0; }
  int binOf(double ms) const {
    if (ms < lo_ms || ms >= hi_ms) return -1;
    return (int)(log(ms / lo_ms) / log(hi_ms / lo_ms) * NB);
  }
  double centre(int b) const { return lo_ms * pow(hi_ms / lo_ms, (b + 0.5) / NB); }
  void add(double ms) { int b = binOf(ms); if (b >= 0) { bins[b]++; count++; } }
  // the strongest local maxima, most counts first
  int top(IntervalPeak *out, int max) const {
    int n = 0;
    for (int b = 0; b < NB; b++) {
      uint32_t v = bins[b];
      if (v == 0) continue;
      uint32_t l = b ? bins[b - 1] : 0, r = (b + 1 < NB) ? bins[b + 1] : 0;
      if (v < l || v < r) continue;
      // merge the neighbours into the peak's count and weighted centre
      double w = (double)v * centre(b) + (double)l * (b ? centre(b - 1) : 0) + (double)r * (b + 1 < NB ? centre(b + 1) : 0);
      uint32_t tot = v + l + r;
      IntervalPeak p = {w / tot, tot};
      int i = n;
      while (i > 0 && out[i - 1].count < p.count) { if (i < max) out[i] = out[i - 1]; i--; }
      if (i < max) { out[i] = p; if (n < max) n++; }
    }
    return n;
  }
};

struct PulseDetector {
  double thresh_db = 6.0;
  double fs = 50000.0;
  // running median estimate (a cheap tracker: step toward the sample)
  double med = -60.0;
  bool above = false;
  uint32_t idx = 0, last_start = 0;
  bool have_last = false;
  double median_dbm() const { return med; }
  // returns whether the current sample is inside a pulse; records the interval at each pulse start
  bool feed(double dbm, IntervalHistogram &h) {
    // sign-step median tracker: settles where half the samples are above it;
    // 0.0005 dB per sample = up to 25 dB/s of slew at 50 kS/s
    med += (dbm > med) ? 0.0005 : -0.0005;
    // 3 dB of hysteresis: a burst sitting near the threshold is one pulse, not several
    bool now = above ? (dbm > med + thresh_db - 3.0) : (dbm > med + thresh_db);
    if (now && !above) {
      if (have_last) h.add((idx - last_start) * 1000.0 / fs);
      last_start = idx; have_last = true;
    }
    above = now;
    idx++;
    return now;
  }
  void reset() { above = false; have_last = false; }
};

// ------------------------------------------------------------- naming lines and intervals
struct LinePeak { double hz, db_rel; };

inline int findPeaks(const double *pxx, int nb, double hz_per_bin, LinePeak *out, int max) {
  double pmax = 0;
  for (int b = 1; b < nb; b++) if (pxx[b] > pmax) pmax = pxx[b];
  if (pmax <= 0) return 0;
  int n = 0;
  for (int b = 2; b < nb - 1; b++) {
    if (pxx[b] < pxx[b - 1] || pxx[b] < pxx[b + 1]) continue;
    double db = 10 * log10(pxx[b] / pmax + 1e-12);
    if (db < -30) continue;
    LinePeak p = {b * hz_per_bin, db};
    int i = n;
    while (i > 0 && out[i - 1].db_rel < p.db_rel) { if (i < max) out[i] = out[i - 1]; i--; }
    if (i < max) { out[i] = p; if (n < max) n++; }
  }
  return n;
}

struct KnownLine { double hz; double tol; const char *name; };
static const KnownLine KNOWN_LINES[] = {
  {9.766, 0.15, "Wi-Fi beacon (102.4 ms)"}, {19.53, 0.2, "2x Wi-Fi beacon"}, {50.0, 0.3, "50 Hz mains / light"},
  {60.0, 0.3, "60 Hz mains / light"}, {100.0, 0.5, "DECT frame / LTE 10 ms frame / 50 Hz lighting"},
  {120.0, 0.5, "60 Hz lighting (full-wave)"}, {217.0, 1.0, "GSM 4.615 ms frame"}, {1000.0, 5.0, "LTE 1 ms subframe"},
  {1600.0, 8.0, "Bluetooth 625 us slot"}, {2000.0, 10.0, "LTE 0.5 ms slot"}, {4000.0, 40.0, "Wi-Fi OFDM symbol"},
  {7812.5, 50.0, "USB microframe / 128 us"}, {15000.0, 100.0, "LTE SC-FDMA symbol (14 per ms)"},
};
inline const char *nameLine(double hz) {
  for (unsigned i = 0; i < sizeof(KNOWN_LINES) / sizeof(KNOWN_LINES[0]); i++)
    if (fabs(hz - KNOWN_LINES[i].hz) <= KNOWN_LINES[i].tol) return KNOWN_LINES[i].name;
  return "";
}

struct KnownInterval { double ms; double tol_pct; const char *name; };
static const KnownInterval KNOWN_INTERVALS[] = {
  {102.4, 3, "Wi-Fi beacon"}, {0.625, 5, "Bluetooth slot"}, {1.0, 5, "LTE subframe"}, {10.0, 4, "LTE/DECT frame"},
  {4.615, 4, "GSM frame"}, {0.5, 5, "LTE slot"}, {1000.0, 3, "1 s periodic (BLE advertising, beacons)"},
  {20.0, 5, "20 ms (BLE connection interval / Zigbee)"}, {100.0, 3, "100 ms periodic (BLE advertising)"},
  {0.1, 10, "100 us (radar / fast TDD)"},
};
inline const char *nameInterval(double ms) {
  for (unsigned i = 0; i < sizeof(KNOWN_INTERVALS) / sizeof(KNOWN_INTERVALS[0]); i++)
    if (fabs(ms - KNOWN_INTERVALS[i].ms) <= KNOWN_INTERVALS[i].ms * KNOWN_INTERVALS[i].tol_pct / 100.0) return KNOWN_INTERVALS[i].name;
  return "";
}
