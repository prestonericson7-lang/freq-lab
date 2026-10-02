// sleep_analysis.h -- the signal processing of sleep_cue.ino, free of Arduino so a
// PC can compile and test it (test/pc_test.cpp). Everything runs sample by sample
// at FS (250 Hz): band trackers, a slow-oscillation detector, the sleep-stage
// gate, the cue scheduler, SHA-256 for the sealed word-pair key.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace sleep {

static const double FS = 250.0;

// ------------------------------------------------------------- IIR
struct Biquad {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  inline double step(double x) { double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
  void lowpass(double f0, double q, double fs) {
    double w = 2 * M_PI * f0 / fs, al = sin(w) / (2 * q), a0 = 1 + al;
    b0 = (1 - cos(w)) / 2 / a0; b1 = (1 - cos(w)) / a0; b2 = b0; a1 = -2 * cos(w) / a0; a2 = (1 - al) / a0; z1 = z2 = 0;
  }
  void highpass(double f0, double q, double fs) {
    double w = 2 * M_PI * f0 / fs, al = sin(w) / (2 * q), a0 = 1 + al;
    b0 = (1 + cos(w)) / 2 / a0; b1 = -(1 + cos(w)) / a0; b2 = b0; a1 = -2 * cos(w) / a0; a2 = (1 - al) / a0; z1 = z2 = 0;
  }
  // phase of the frequency response at f (radians)
  double phase(double f, double fs) const {
    double w = 2 * M_PI * f / fs, c1 = cos(w), s1 = sin(w), c2 = cos(2 * w), s2 = sin(2 * w);
    double nr = b0 + b1 * c1 + b2 * c2, ni = -(b1 * s1 + b2 * s2);
    double dr = 1 + a1 * c1 + a2 * c2, di = -(a1 * s1 + a2 * s2);
    return atan2(ni, nr) - atan2(di, dr);
  }
};

// group delay (seconds) of a cascade of biquads at frequency f, by central difference
// (the envelope delay; reported in the sketch's `stat` for reference)
inline double groupDelay(const Biquad *const *bq, int n, double f, double fs) {
  double df = 0.01, p1 = 0, p2 = 0;
  for (int i = 0; i < n; i++) { p1 += bq[i]->phase(f - df, fs); p2 += bq[i]->phase(f + df, fs); }
  double d = p2 - p1;
  while (d > M_PI) d -= 2 * M_PI;
  while (d < -M_PI) d += 2 * M_PI;
  return -d / (2 * M_PI * 2 * df);
}

// 4th-order Butterworth band-pass (two HP + two LP biquads) with an RMS tracker
struct BandTracker {
  Biquad hp[2], lp[2];
  double ms = 0, alpha = 0;       // exponential mean square
  double y = 0;
  void setup(double f_lo, double f_hi, double fs, double tau_s) {
    static const double Q[2] = {0.54119610, 1.30656296};
    for (int i = 0; i < 2; i++) { hp[i].highpass(f_lo, Q[i], fs); lp[i].lowpass(f_hi, Q[i], fs); }
    alpha = 1.0 - exp(-1.0 / (tau_s * fs));
    ms = 0;
  }
  inline double step(double x) {
    y = lp[1].step(lp[0].step(hp[1].step(hp[0].step(x))));
    ms += alpha * (y * y - ms);
    return y;
  }
  double rms() const { return sqrt(ms); }
};

// ------------------------------------------------------------- sleep-stage gate
// Not a polysomnograph: a conservative "deep enough to cue" decision from one
// frontal EEG channel (+ an EMG/EOG channel if present), decided per 30 s epoch:
//   deep  = delta fraction >= DELTA_FRAC and alpha fraction <= ALPHA_MAX and
//           EMG rms <= EMG_RATIO x the quietest EMG epoch seen so far,
//           for 2 consecutive epochs; any epoch failing badly (alpha > 0.3 or
//           EMG > 4x quiet) drops out at once.
struct StageGate {
  double delta_frac_min = 0.45, alpha_frac_max = 0.15, emg_ratio = 2.0;
  BandTracker delta, theta, alpha, sigma, beta, emg;
  double epoch_s = 30.0;
  int n_in_epoch = 0, epoch_len = 0;
  double acc_d = 0, acc_t = 0, acc_a = 0, acc_s = 0, acc_b = 0, acc_e = 0;
  double emg_quiet = -1;
  int consecutive = 0, epochs = 0;
  bool is_deep = false;
  // last finished epoch
  double ep_delta = 0, ep_alpha = 0, ep_sigma = 0, ep_emg = 0, ep_theta = 0;
  bool have_emg = false;

  void setup(double fs, bool with_emg) {
    delta.setup(0.5, 4.0, fs, 2.0); theta.setup(4.0, 8.0, fs, 2.0); alpha.setup(8.0, 12.0, fs, 2.0);
    sigma.setup(12.0, 15.0, fs, 2.0); beta.setup(16.0, 30.0, fs, 2.0); emg.setup(20.0, 100.0, fs, 2.0);
    epoch_len = (int)(epoch_s * fs);
    have_emg = with_emg;
  }
  // returns true when an epoch just finished
  bool step(double eeg_uV, double emg_uV) {
    delta.step(eeg_uV); theta.step(eeg_uV); alpha.step(eeg_uV); sigma.step(eeg_uV); beta.step(eeg_uV);
    emg.step(have_emg ? emg_uV : eeg_uV);
    acc_d += delta.ms; acc_t += theta.ms; acc_a += alpha.ms; acc_s += sigma.ms; acc_b += beta.ms; acc_e += emg.ms;
    if (++n_in_epoch < epoch_len) return false;
    double tot = acc_d + acc_t + acc_a + acc_s + acc_b + 1e-12;
    ep_delta = acc_d / tot; ep_theta = acc_t / tot; ep_alpha = acc_a / tot; ep_sigma = acc_s / tot;
    ep_emg = sqrt(acc_e / n_in_epoch);
    acc_d = acc_t = acc_a = acc_s = acc_b = acc_e = 0; n_in_epoch = 0; epochs++;
    if (emg_quiet < 0 || ep_emg < emg_quiet) emg_quiet = ep_emg;
    bool quiet = ep_emg <= emg_ratio * emg_quiet * 1.0 + 1e-9;
    bool ok = ep_delta >= delta_frac_min && ep_alpha <= alpha_frac_max && quiet;
    bool bad = ep_alpha > 0.30 || ep_emg > 4.0 * emg_quiet;
    if (ok) consecutive++; else consecutive = 0;
    if (bad) { consecutive = 0; is_deep = false; }
    else if (consecutive >= 2) is_deep = true;
    else if (!ok) is_deep = false;
    return true;
  }
};

// ------------------------------------------------------------- slow-oscillation detector
// On the band-limited signal (2nd-order high-pass at hp_hz, 4th-order low-pass at
// 4 Hz): a negative half-wave (down-crossing -> trough -> rising) whose trough is
// below -thresh_uV and whose duration so far is 0.125-1.0 s. The trough is
// confirmed when the signal has risen `confirm_uV` from its minimum; that sample
// is reported with the trough's index and depth.
// The high-pass is deliberately gentle: a 4th-order 0.5 Hz high-pass shrinks an
// isolated 0.5 s half-wave to 38 % of its depth, so a threshold on its output no
// longer means a physical amplitude. With 2nd order at 0.16 Hz the trough keeps
// about 90 % and thresh_uV is close to the raw-trough criterion used in the
// literature (Massimini 2004: -80 uV; Ngo 2013 cued at an adaptive ~-80 uV).
struct SoDetector {
  double thresh_uV = 75.0, confirm_uV = 5.0, hp_hz = 0.16;
  double min_half_s = 0.125, max_half_s = 1.0;
  Biquad hp[1], lp[2];
  double fs = 250.0;
  bool below = false;        // inside a negative half-wave
  long down_idx = 0, min_idx = 0, idx = 0;
  double min_v = 0, prev = 0, last_y = 0;
  bool reported = false;
  long delay_samples = 0;    // filter delay of a slow-oscillation trough, taken off the reported time
  // output of the last confirmed trough (index in RAW-signal time)
  long trough_idx = -1; double trough_uV = 0; long count = 0;

  void setup(double fs_) {
    fs = fs_;
    static const double Q[2] = {0.54119610, 1.30656296};
    hp[0].highpass(hp_hz, 0.70710678, fs);
    for (int i = 0; i < 2; i++) lp[i].lowpass(4.0, Q[i], fs);
    delay_samples = calibrateDelay();
  }

  // How late the filtered trough of a slow oscillation comes after the raw trough.
  // Not the group delay (144 ms at 1 Hz here: that is the envelope's delay); a
  // wave's trough moves by the PHASE delay, which the high-pass's phase lead cuts
  // to about 70 ms. Measured on a template wave (0.5 s down-state, 0.5 s up-state
  // of 0.6x, the shape of human N3 slow oscillations) through copies of the filters.
  long calibrateDelay() const {
    Biquad h = hp[0], l0 = lp[0], l1 = lp[1];
    h.z1 = h.z2 = l0.z1 = l0.z2 = l1.z1 = l1.z2 = 0;
    long n = (long)(0.5 * fs), start = (long)(2.0 * fs), total = start + 4 * n;
    long raw_min_i = start + n / 2, f_min_i = 0;
    double f_min = 1e9;
    for (long i = 0; i < total; i++) {
      double x = 0;
      long k = i - start;
      if (k >= 0 && k < n) x = -100.0 * sin(M_PI * k / (double)n);
      else if (k >= n && k < 2 * n) x = 60.0 * sin(M_PI * (k - n) / (double)n);
      double y = l1.step(l0.step(h.step(x)));
      if (y < f_min) { f_min = y; f_min_i = i; }
    }
    return f_min_i - raw_min_i;
  }
  // returns true on the sample that confirms a trough
  bool step(double eeg_uV) {
    double y = lp[1].step(lp[0].step(hp[0].step(eeg_uV)));
    last_y = y;
    bool out = false;
    if (!below) {
      if (prev >= 0 && y < 0) { below = true; down_idx = idx; min_idx = idx; min_v = y; reported = false; }
    } else {
      if (y < min_v) { min_v = y; min_idx = idx; }
      double half_s = (idx - down_idx) / fs;
      if (!reported && min_v <= -thresh_uV && y >= min_v + confirm_uV && half_s >= min_half_s && half_s <= max_half_s) {
        reported = true; trough_idx = min_idx - delay_samples; trough_uV = min_v; count++; out = true;
      }
      if (y >= 0 || half_s > max_half_s) below = false;
    }
    prev = y; idx++;
    return out;
  }
};

// ------------------------------------------------------------- cue scheduler
// cue = trough + delay (Ngo 2013 used the subject's SO half-period, ~0.5 s), only
// when the gate says deep, after the refractory time, under the rate cap, and
// not during an arousal pause.
// Arousal, checked on every sample: alpha rms above alpha_floor_uV (default 8 uV;
// deep-sleep alpha is 1-3 uV, relaxed wake alpha 10-50 uV) or EMG rms above
// emg_floor_uV (the sketch sets it to 3x the quietest EMG epoch it has seen).
// Any arousal pauses cueing for arousal_pause_s (5 min). Arousals within 10 s
// after a cue are also counted separately: that number is the safety metric
// (cues must not wake the sleeper).
struct CueScheduler {
  double delay_s = 0.5, refractory_s = 2.5;
  int max_per_min = 6;
  double arousal_pause_s = 300.0, alpha_floor_uV = 8.0, emg_floor_uV = 8.0;
  double fs = 250.0;
  long pending_idx = -1, last_cue_idx = -1000000, pause_until = -1, idx = 0;
  long minute_cues[64]; int mc_n = 0;
  bool in_arousal = false;
  long cues = 0, skipped_gate = 0, skipped_rate = 0, skipped_refractory = 0, skipped_pause = 0;
  long arousals = 0, arousals_after_cue = 0;

  void setup(double fs_) { fs = fs_; }
  void schedule(long trough_idx) { pending_idx = trough_idx + (long)(delay_s * fs); }
  // call every sample; returns true on the sample a cue must be delivered
  bool step(bool deep, double alpha_rms, double emg_rms) {
    bool fire = false;
    bool aroused = alpha_rms > alpha_floor_uV || emg_rms > emg_floor_uV;
    if (aroused) {
      if (!in_arousal) {
        arousals++;
        if (idx - last_cue_idx <= (long)(10 * fs)) arousals_after_cue++;
      }
      pause_until = idx + (long)(arousal_pause_s * fs);      // the pause runs from the END of the arousal
    }
    in_arousal = aroused;
    if (pending_idx >= 0 && idx >= pending_idx) {
      pending_idx = -1;
      if (!deep) skipped_gate++;
      else if (idx < pause_until) skipped_pause++;
      else if (idx - last_cue_idx < refractory_s * fs) skipped_refractory++;
      else {
        // rate cap over the trailing minute
        int keep = 0;
        for (int i = 0; i < mc_n; i++) if (idx - minute_cues[i] < 60 * fs) minute_cues[keep++] = minute_cues[i];
        mc_n = keep;
        if (mc_n >= max_per_min) skipped_rate++;
        else {
          fire = true; cues++; last_cue_idx = idx;
          if (mc_n < 64) minute_cues[mc_n++] = idx;
        }
      }
    }
    idx++;
    return fire;
  }
};

// ------------------------------------------------------------- SHA-256 (for the sealed key)
struct Sha256 {
  uint32_t h[8]; uint8_t buf[64]; uint64_t len = 0; int bl = 0;
  static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  Sha256() { reset(); }
  void reset() {
    static const uint32_t iv[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    memcpy(h, iv, sizeof(h)); len = 0; bl = 0;
  }
  void block(const uint8_t *p) {
    static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
      0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
      0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t w[64];
    for (int i = 0; i < 16; i++) w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
    for (int i = 16; i < 64; i++) {
      uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
      uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25), ch = (e & f) ^ (~e & g), t1 = hh + S1 + ch + k[i] + w[i];
      uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22), mj = (a & b) ^ (a & c) ^ (b & c), t2 = S0 + mj;
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  void update(const uint8_t *p, size_t n) {
    len += n;
    while (n--) { buf[bl++] = *p++; if (bl == 64) { block(buf); bl = 0; } }
  }
  void update(const char *s) { update((const uint8_t *)s, strlen(s)); }
  void finish(char *hex65) {
    uint64_t bits = len * 8;
    uint8_t pad = 0x80; update(&pad, 1);
    uint8_t z = 0; while (bl != 56) update(&z, 1);
    uint8_t l[8]; for (int i = 0; i < 8; i++) l[i] = (uint8_t)(bits >> (56 - 8 * i));
    update(l, 8);
    for (int i = 0; i < 8; i++) for (int j = 0; j < 4; j++) { uint8_t b = (uint8_t)(h[i] >> (24 - 8 * j)); hex65[8 * i + 2 * j] = "0123456789abcdef"[b >> 4]; hex65[8 * i + 2 * j + 1] = "0123456789abcdef"[b & 15]; }
    hex65[64] = 0;
  }
};

// ------------------------------------------------------------- word-pair split
// mask[i] = 1 if pair i is cued; exactly half cued; rng() gives uniform 32-bit words
template <typename RNG>
inline void splitPairs(int n, uint8_t *mask, RNG rng) {
  for (int i = 0; i < n; i++) mask[i] = (i < n / 2) ? 1 : 0;
  for (int i = n - 1; i > 0; i--) { int j = (int)(rng() % (uint32_t)(i + 1)); uint8_t t = mask[i]; mask[i] = mask[j]; mask[j] = t; }
}
inline void commitment(const uint8_t *mask, int n, const char *salt, char *hex65) {
  Sha256 s;
  for (int i = 0; i < n; i++) { char c = mask[i] ? '1' : '0'; s.update((const uint8_t *)&c, 1); }
  s.update(":"); s.update(salt);
  s.finish(hex65);
}

}  // namespace sleep
