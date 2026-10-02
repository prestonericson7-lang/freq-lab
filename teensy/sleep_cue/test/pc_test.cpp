// pc_test.cpp -- sleep_cue's signal processing on a PC.
//   g++ -std=c++17 -I.. pc_test.cpp -o pc_test && ./pc_test
// Synthetic night: 3 min wake (10 Hz alpha 25 uV + EMG + noise) then 15 min of
// slow-wave sleep (full slow oscillations of 50-130 uV every 1.25 s, spindles,
// noise). Checks the stage gate, the SO detector, the cue timing and gating
// (including the 5 min pause that follows any arousal, the initial wake too),
// SHA-256 and the pair split.
#include "../sleep_analysis.h"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>

using namespace sleep;
static int passes = 0, fails = 0;
static void check(bool c, const char *what) { if (c) { passes++; printf("  PASS  %s\n", what); } else { fails++; printf("  FAIL  %s\n", what); } }

int main() {
  const double fs = FS;
  std::mt19937 rng(11);
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const int T_WAKE = 180, T_SLEEP = 900;
  const int N = (int)((T_WAKE + T_SLEEP) * fs);
  std::vector<double> eeg(N), emg(N);
  std::vector<long> so_troughs;                        // true trough sample indices
  std::vector<double> so_amps;                         // their down-state amplitudes
  // slow oscillations as full waves, one every 1.25 s: a 0.5 s negative down-state of
  // A = 50-130 uV followed by a 0.5 s positive up-state of 0.6 A (as in human N3)
  for (int i = 0; i < N; i++) {
    double t = i / fs;
    double x = 0, e = 0;
    // pink-ish background: two low-passed noises
    static double n1 = 0, n2 = 0;
    n1 += 0.05 * (g(rng) * 4 - n1); n2 += 0.005 * (g(rng) * 4 - n2);
    x += n1 + n2;
    if (t < T_WAKE) {
      x += 25.0 * sin(2 * M_PI * 10.0 * t) + 3.0 * g(rng);
      e = 15.0 * g(rng);                                // tense muscles
    } else {
      x += 8.0 * g(rng) * 0.3 + 2.0 * g(rng);
      e = 2.0 * g(rng);                                 // atonia-ish
      if (fmod(t, 7.0) < 1.0) x += 15.0 * sin(2 * M_PI * 13.5 * t);   // a spindle every 7 s
    }
    eeg[i] = x; emg[i] = e;
  }
  for (double t0 = T_WAKE + 2.0; t0 < T_WAKE + T_SLEEP - 2; t0 += 1.25) {
    double amp = 50.0 + 80.0 * u(rng);
    long i0 = (long)(t0 * fs), n = (long)(0.5 * fs);
    for (long k = 0; k < n; k++) eeg[i0 + k] -= amp * sin(M_PI * k / (double)n);
    for (long k = 0; k < n; k++) eeg[i0 + n + k] += 0.6 * amp * sin(M_PI * k / (double)n);
    so_troughs.push_back(i0 + n / 2);
    so_amps.push_back(amp);
  }

  printf("[1] stage gate\n");
  StageGate gate; gate.setup(fs, true);
  SoDetector so; so.setup(fs);
  CueScheduler cue; cue.setup(fs);
  BandTracker alphaT; alphaT.setup(8, 12, fs, 2.0);
  BandTracker emgT; emgT.setup(20, 100, fs, 2.0);
  std::vector<int> deep_by_epoch; std::vector<long> cue_idx, det_idx, det_trough;
  long first_deep_epoch = -1;
  for (int i = 0; i < N; i++) {
    alphaT.step(eeg[i]); emgT.step(emg[i]);
    if (gate.step(eeg[i], emg[i])) {
      deep_by_epoch.push_back(gate.is_deep ? 1 : 0);
      if (gate.is_deep && first_deep_epoch < 0) first_deep_epoch = gate.epochs;
      if (gate.emg_quiet > 0) cue.emg_floor_uV = 3.0 * gate.emg_quiet;     // as the sketch does
    }
    if (so.step(eeg[i])) { cue.schedule(so.trough_idx); det_idx.push_back(i); det_trough.push_back(so.trough_idx); }
    if (cue.step(gate.is_deep, alphaT.rms(), emgT.rms())) cue_idx.push_back(i);
  }
  int wake_epochs = T_WAKE / 30, sleep_epochs = T_SLEEP / 30;
  bool wake_ok = true; for (int k = 0; k < wake_epochs; k++) if (deep_by_epoch[k]) wake_ok = false;
  int deep_sleep = 0; for (int k = wake_epochs; k < wake_epochs + sleep_epochs; k++) deep_sleep += deep_by_epoch[k];
  printf("    epochs: %d wake, %d sleep; first deep epoch %ld; deep sleep epochs %d/%d\n", wake_epochs, sleep_epochs, first_deep_epoch, deep_sleep, sleep_epochs);
  check(wake_ok, "no wake epoch is called deep (alpha + EMG)");
  check(first_deep_epoch >= wake_epochs + 2 && first_deep_epoch <= wake_epochs + 3, "deep after 2 consecutive qualifying sleep epochs");
  check(deep_sleep >= sleep_epochs - 3, "nearly every slow-wave epoch is deep");

  printf("[2] slow-oscillation detector (threshold %.0f uV)\n", so.thresh_uV);
  int n_true = (int)so_troughs.size();
  // which true troughs were detected (a detection within 60 ms: filter delay included)
  std::vector<int> hit(n_true, 0);
  int matched = 0; double worst = 0;
  for (long d : det_trough) {
    double best = 1e9; int bi = -1;
    for (int k = 0; k < n_true; k++) { double o = fabs((double)(d - so_troughs[k])); if (o < best) { best = o; bi = k; } }
    if (best <= 0.06 * fs) { matched++; hit[bi] = 1; }
    worst = fmax(worst, best);
  }
  int big = 0, big_hit = 0, small = 0, small_hit = 0;
  for (int k = 0; k < n_true; k++) {
    if (so_amps[k] >= 95) { big++; big_hit += hit[k]; }
    if (so_amps[k] <= 65) { small++; small_hit += hit[k]; }
  }
  printf("    %d true troughs, %ld detected, %d matched (worst offset %.0f ms)\n", n_true, so.count, matched, worst / fs * 1000);
  printf("    amplitude >= 95 uV: %d of %d detected; <= 65 uV: %d of %d detected\n", big_hit, big, small_hit, small);
  check(matched == (int)det_trough.size(), "every detection is a real slow oscillation (no false troughs)");
  check(big_hit >= 0.95 * big, "at least 95 % of waves of 95 uV or more are detected");
  check(small_hit <= 0.05 * small, "at most 5 % of waves of 65 uV or less are detected (below threshold)");
  bool wake_det = false; for (long d : det_idx) if (d < T_WAKE * fs) wake_det = true;
  check(!wake_det, "no slow-oscillation detections in wake");

  printf("    filter delay taken off the trough times: %ld samples (%.0f ms)\n", so.delay_samples, so.delay_samples / fs * 1000);

  printf("[3] cues\n");
  printf("    %ld cues, skipped: gate %ld, refractory %ld, rate %ld, pause %ld\n", cue.cues, cue.skipped_gate, cue.skipped_refractory, cue.skipped_rate, cue.skipped_pause);
  // wake ends at 180 s; the alpha/EMG trackers fall below their floors within a few
  // seconds; the pause then runs 300 s: cueing may start from about 485 s
  long first_cue = cue_idx.empty() ? -1 : cue_idx[0];
  printf("    first cue at %.1f s (wake ended at %d s; the 5 min arousal pause runs from the end of wake)\n", first_cue / fs, T_WAKE);
  check(first_cue >= (T_WAKE + 300) * fs && first_cue <= (T_WAKE + 330) * fs, "first cue right after the post-wake pause, not before");
  double cue_minutes = (N - first_cue) / fs / 60.0;
  double rate = (cue.cues - 1) / cue_minutes;
  printf("    %.2f cues per minute after the first one\n", rate);
  check(rate >= 5.0 && rate <= 6.0, "rate cap holds: 5-6 cues per minute once cueing runs (cap 6)");
  bool timing = true;
  for (long c : cue_idx) {
    // the cue must be delay_s after some detected trough
    bool ok = false;
    for (long t : det_trough) if (llabs(c - (t + (long)(0.5 * fs))) <= 1) ok = true;
    if (!ok) timing = false;
  }
  check(timing, "every cue is exactly 0.5 s after a detected trough");
  bool refr = true;
  for (size_t i = 1; i < cue_idx.size(); i++) if (cue_idx[i] - cue_idx[i - 1] < 2.5 * fs) refr = false;
  check(refr, "no two cues closer than the 2.5 s refractory time");
  bool none_wake = true; for (long c : cue_idx) if (c < (T_WAKE + 60) * fs) none_wake = false;
  check(none_wake, "no cue before deep sleep is established");
  check(cue.skipped_gate > 0, "troughs before the gate opened were skipped, not cued");
  long sleep_arousals = cue.arousals;          // the wake period itself counts as one arousal
  printf("    arousals over the whole record: %ld (the wake period is one), %ld within 10 s after a cue\n",
         sleep_arousals, cue.arousals_after_cue);
  check(sleep_arousals == 1 && cue.arousals_after_cue == 0, "no arousal in undisturbed slow-wave sleep (only the initial wake)");
  // every cue lands 0.5 s after a TRUE (raw-signal) trough, within 40 ms
  bool true_timing = true; double worst_cue = 0;
  for (long c : cue_idx) {
    double best = 1e9;
    for (long t : so_troughs) best = fmin(best, fabs((double)(c - (t + (long)(0.5 * fs)))));
    worst_cue = fmax(worst_cue, best);
    if (best > 0.04 * fs) true_timing = false;
  }
  printf("    worst cue offset from true trough + 0.5 s: %.0f ms\n", worst_cue / fs * 1000);
  check(true_timing, "every cue is within 40 ms of the true trough + 0.5 s (filter delay compensated)");

  printf("[3b] arousal: a 20 s burst of alpha + EMG in the middle of slow-wave sleep\n");
  {
    std::vector<double> e2 = eeg, m2 = emg;
    long a0 = (long)((T_WAKE + 500) * fs), a1 = a0 + (long)(20 * fs);
    for (long i = a0; i < a1; i++) { e2[i] += 25.0 * sin(2 * M_PI * 10.0 * i / fs); m2[i] += 15.0 * g(rng); }
    StageGate g2; g2.setup(fs, true); SoDetector s2; s2.setup(fs); CueScheduler c2; c2.setup(fs);
    BandTracker at; at.setup(8, 12, fs, 2.0); BandTracker et; et.setup(20, 100, fs, 2.0);
    long cues_before = 0, cues_paused = 0, cues_resumed = 0;
    long pause_end = a1 + (long)(300 * fs);
    for (int i = 0; i < N; i++) {
      at.step(e2[i]); et.step(m2[i]);
      if (g2.step(e2[i], m2[i]) && g2.emg_quiet > 0) c2.emg_floor_uV = 3.0 * g2.emg_quiet;
      if (s2.step(e2[i])) c2.schedule(s2.trough_idx);
      if (c2.step(g2.is_deep, at.rms(), et.rms())) {
        if (i < a0) cues_before++;
        else if (i < pause_end) cues_paused++;
        else cues_resumed++;
      }
    }
    printf("    arousals %ld (wake + burst), cues before the burst %ld, during burst + 5 min %ld, after %ld\n",
           c2.arousals, cues_before, cues_paused, cues_resumed);
    check(c2.arousals == 2, "the burst is detected as one arousal (plus the initial wake)");
    check(cues_before > 0 && cues_paused == 0, "no cue from the burst's start until 5 min after it ended");
    check(cues_resumed > 0, "cueing resumes after the pause");
  }

  printf("[4] SHA-256 and the sealed split\n");
  char hex[65];
  Sha256 s; s.update("abc"); s.finish(hex);
  check(strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0, "SHA-256(\"abc\") matches the standard vector");
  Sha256 s2; s2.update("The quick brown fox jumps over the lazy dog"); s2.finish(hex);
  check(strcmp(hex, "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592") == 0, "SHA-256 of a 43-byte string");
  uint8_t mask[20]; std::mt19937 r2(5);
  splitPairs(20, mask, [&]() { return (uint32_t)r2(); });
  int cued = 0; for (int i = 0; i < 20; i++) cued += mask[i];
  check(cued == 10, "20 pairs -> exactly 10 cued");
  char c1[65], c2[65];
  commitment(mask, 20, "saltsalt", c1);
  commitment(mask, 20, "saltsalt", c2);
  mask[0] ^= 1; mask[1] ^= 1;
  char c3[65]; commitment(mask, 20, "saltsalt", c3);
  check(strcmp(c1, c2) == 0 && strcmp(c1, c3) != 0, "commitment reproducible and sensitive to the mask");

  printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
  return fails ? 1 : 0;
}
