// pc_test.cpp -- the nuisance-veto logic on synthetic data.
//   g++ -std=c++17 -I.. pc_test.cpp -o t && ./t
#include "../nuisance_analysis.h"
#include <cstdio>
#include <cmath>
#include <random>
using namespace nuisance;
static int passes = 0, fails = 0;
static void check(bool c, const char *w) { if (c) { passes++; printf("  PASS  %s\n", w); } else { fails++; printf("  FAIL  %s\n", w); } }

int main() {
  const double fs = 1000.0;
  std::mt19937 rng(5);
  std::normal_distribution<double> g(0, 1);

  printf("[1] running stats z-score\n");
  RunStat rs;
  for (int i = 0; i < 10000; i++) rs.push(5.0 + 2.0 * g(rng));
  printf("    mean %.2f (5), sd %.2f (2)\n", rs.mean, rs.sd());
  check(fabs(rs.mean - 5) < 0.1 && fabs(rs.sd() - 2) < 0.1, "Welford recovers mean and sd");
  check(fabs(rs.z(9.0) - 2.0) < 0.1, "z of mean+2sd is ~2");

  printf("[2] windowed correlation tracks and forgets\n");
  WinCorr wc; wc.init(500);
  for (int i = 0; i < 1000; i++) { double x = g(rng); wc.push(x, x + 0.1 * g(rng)); }   // correlated
  double r_corr = wc.r();
  for (int i = 0; i < 1000; i++) { wc.push(g(rng), g(rng)); }                            // now independent
  double r_indep = wc.r();
  printf("    r while correlated %.2f, after 2 windows of independence %.2f\n", r_corr, r_indep);
  check(r_corr > 0.9, "near-identical streams correlate ~1");
  check(fabs(r_indep) < 0.15, "the window forgets: independent streams decorrelate");

  printf("[3] a real signal, uncorrelated with any nuisance -> not vetoed\n");
  {
    Veto<4> v; v.init(256);
    long sig_exc = 0, vetoed_any = 0;
    for (int i = 0; i < 20000; i++) {
      double t = i / fs;
      double sig = g(rng) + (fmod(t, 2.0) < 0.05 ? 8.0 : 0.0);   // genuine 50 ms bursts every 2 s
      double nui[4] = {g(rng), g(rng), sin(2 * M_PI * 60 * t) * 0.3 + g(rng) * 0.1, g(rng)};
      bool excursion = fabs(sig) > 5;
      unsigned m = v.step(sig, nui);
      if (excursion) { sig_exc++; if (m) vetoed_any++; }
    }
    printf("    %ld signal excursions, %ld vetoed\n", sig_exc, vetoed_any);
    check(sig_exc > 100 && vetoed_any <= 0.05 * sig_exc, "a genuine signal is almost never vetoed by uncorrelated nuisances");
  }

  printf("[4] the #133 case: the 'signal' IS the 8 Hz mount vibration -> vetoed\n");
  {
    Veto<4> v; v.init(256);
    long sig_exc = 0, vetoed = 0;
    for (int i = 0; i < 20000; i++) {
      double t = i / fs;
      double vib = (fmod(t, 2.0) < 0.1 ? 6.0 * sin(2 * M_PI * 8 * t) : 0.0) + g(rng) * 0.3;  // vibration bursts
      double sig = vib * 1.2 + g(rng) * 0.3;                     // the 'signal' is just the vibration, scaled
      double nui[4] = {vib, g(rng), g(rng), g(rng)};             // channel 0 is the accelerometer
      bool excursion = fabs(v.sigStat.z(sig)) > 3 && fabs(sig) > 3;
      unsigned m = v.step(sig, nui);
      if (fabs(sig) > 3 && i > 2000) { sig_exc++; if (m & 1u) vetoed++; }
    }
    printf("    %ld excursions, %ld vetoed by the vibration channel\n", sig_exc, vetoed);
    check(sig_exc > 50 && vetoed >= 0.6 * sig_exc, "when the signal is the vibration, the accelerometer channel vetoes most excursions");
    check(v.vetoed[1] < 0.2 * sig_exc && v.vetoed[2] < 0.2 * sig_exc, "the unrelated nuisance channels do not veto (no false blame)");
  }

  printf("[5] continuous 60 Hz mains contaminating the signal ('60 Hz everywhere') -> vetoed\n");
  {
    // the common real case: the 'signal' is just steady mains pickup plus noise.
    // A window matched to a few mains cycles (64 samples = ~4 cycles at 60 Hz).
    Veto<4> v; v.init(64);
    long samp = 0, vetoed = 0;
    for (int i = 0; i < 20000; i++) {
      double t = i / fs;
      double mains = sin(2 * M_PI * 60 * t);
      double sig = 2.0 * mains + g(rng) * 0.5;                   // continuously mains-contaminated
      double nui[4] = {g(rng), g(rng), g(rng), mains};           // channel 3 is the mains pickup
      unsigned m = v.step(sig, nui);
      if (i > 2000) { samp++; if (m & (1u << 3)) vetoed++; }
    }
    printf("    %ld samples, %ld vetoed by mains (%.0f %%)\n", samp, vetoed, 100.0 * vetoed / samp);
    check(vetoed >= 0.9 * samp, "a signal that is continuous mains pickup is vetoed almost all the time by the mains channel");
    check(v.vetoed[0] < 0.1 * samp, "an unrelated noise channel does not veto a mains-contaminated signal");
  }

  printf("[6] impulsive mains bursts with a window matched to the mains\n");
  {
    Veto<4> v; v.init(64);
    long exc = 0, vetoed = 0;
    for (int i = 0; i < 40000; i++) {
      double t = i / fs;
      double mains = sin(2 * M_PI * 60 * t);
      double burst = (fmod(t, 1.5) < 0.2) ? 5.0 : 0.0;          // 200 ms bursts, > the 64-sample window
      double sig = burst * mains + g(rng) * 0.2;
      double nui[4] = {g(rng), g(rng), g(rng), mains};
      unsigned m = v.step(sig, nui);
      if (fabs(v.z_sig) > 3 && i > 2000) { exc++; if (m & (1u << 3)) vetoed++; }
    }
    printf("    %ld excursions, %ld vetoed (%.0f %%)\n", exc, vetoed, exc ? 100.0 * vetoed / exc : 0);
    check(exc > 50 && vetoed >= 0.7 * exc, "mains-modulated bursts are vetoed once the window fits inside the burst");
  }

  printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
  return fails ? 1 : 0;
}
