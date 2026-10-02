// nuisance_analysis.h -- the veto logic of nuisance_logger (list item #159),
// free of Arduino so a PC tests it (test/pc_test.cpp).
//
// Every signal channel is logged beside the ordinary things that fake a signal:
// vibration (the 8 Hz mount resonance that sank the pilot studies, #133),
// stray B-field, sound, and 60 Hz mains. This decides, per window, whether an
// excursion in the signal is EXPLAINED by a coincident excursion in a nuisance,
// so the ordinary cause can be vetoed after the fact -- which is what every null
// in the files came down to.
#pragma once
#include <math.h>
#include <stddef.h>

namespace nuisance {

// Welford running mean/variance for z-scoring a channel online.
struct RunStat {
  double mean = 0, m2 = 0;
  long n = 0;
  void push(double x) { n++; double d = x - mean; mean += d / n; m2 += d * (x - mean); }
  double var() const { return n > 1 ? m2 / (n - 1) : 0; }
  double sd() const { return sqrt(var()); }
  double z(double x) const { double s = sd(); return s > 1e-12 ? (x - mean) / s : 0; }
};

// Windowed Pearson correlation between two streams, updated per sample over the
// last W samples (circular buffers; O(1) per sample via running sums).
struct WinCorr {
  static const int MAXW = 2048;
  int w;
  double bx[MAXW], by[MAXW];
  int pos = 0; long n = 0;
  double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  void init(int window) { w = window < 2 ? 2 : (window > MAXW ? MAXW : window); pos = 0; n = 0; sx = sy = sxx = syy = sxy = 0; for (int i = 0; i < w; i++) bx[i] = by[i] = 0; }
  void push(double x, double y) {
    if (n >= w) {                       // remove the sample leaving the window
      double ox = bx[pos], oy = by[pos];
      sx -= ox; sy -= oy; sxx -= ox * ox; syy -= oy * oy; sxy -= ox * oy;
    }
    bx[pos] = x; by[pos] = y;
    sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    pos = (pos + 1) % w;
    if (n < w) n++;
  }
  double r() const {
    if (n < 2) return 0;
    double cov = sxy - sx * sy / n;
    double vx = sxx - sx * sx / n, vy = syy - sy * sy / n;
    double den = sqrt(vx * vy);
    return den > 1e-12 ? cov / den : 0;
  }
};

// Per-sample veto decision for one signal and up to NCH nuisance channels.
// A nuisance "explains" the signal now when the signal is PERSISTENTLY
// CORRELATED with that nuisance over the window (|r| >= r_thr). Correlation,
// not a shared threshold crossing, is the right test: it catches both an
// impulsive nuisance that coincides with a signal burst (the 8 Hz mount
// resonance, #133) AND a continuous one that modulates the signal (60 Hz mains),
// the latter of which never trips an amplitude threshold because a steady sine
// peaks at only ~1.4 sd. The per-channel z-scores are kept for the log so the
// strength of each excursion is recorded, but the veto is the correlation.
template <int NCH>
struct Veto {
  RunStat sigStat, nuiStat[NCH];
  WinCorr corr[NCH];
  double z_thr = 3.0, r_thr = 0.5;
  bool active[NCH] = {false};
  long vetoed[NCH] = {0};
  double z_sig = 0, z_nui[NCH] = {0};
  void init(int window) { for (int i = 0; i < NCH; i++) corr[i].init(window); }
  // returns a bitmask of nuisance channels currently vetoing the signal
  unsigned step(double sig, const double *nui) {
    sigStat.push(sig);
    z_sig = sigStat.z(sig);
    unsigned mask = 0;
    for (int i = 0; i < NCH; i++) {
      nuiStat[i].push(nui[i]);
      corr[i].push(sig, nui[i]);
      z_nui[i] = nuiStat[i].z(nui[i]);
      active[i] = fabs(corr[i].r()) >= r_thr;
      if (active[i]) { mask |= (1u << i); vetoed[i]++; }
    }
    return mask;
  }
  // fraction of a run's signal excursions that any nuisance explained
  double contamination(long sig_excursions) const {
    if (sig_excursions <= 0) return 0;
    long v = 0;
    for (int i = 0; i < NCH; i++) v = v > vetoed[i] ? v : vetoed[i];
    return (double)v / sig_excursions;
  }
};

}  // namespace nuisance
