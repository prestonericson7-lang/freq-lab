// pc_test.cpp -- the Kirlian moisture+pressure fit on synthetic exposures.
//   g++ -std=c++17 -I.. pc_test.cpp -o t && ./t
#include "../kirlian_analysis.h"
#include <cstdio>
#include <cmath>
#include <random>
using namespace kirlian;
static int passes = 0, fails = 0;
static void check(bool c, const char *w) { if (c) { passes++; printf("  PASS  %s\n", w); } else { fails++; printf("  FAIL  %s\n", w); } }

int main() {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 1.0);
  const int N = 40;
  double corona[N], moist[N], press[N];
  std::uniform_real_distribution<double> um(2, 20), up(20, 200);

  printf("[1] corona that is purely moisture + pressure\n");
  for (int i = 0; i < N; i++) {
    moist[i] = um(rng); press[i] = up(rng);
    corona[i] = 5.0 + 3.0 * moist[i] + 0.5 * press[i] + noise(rng) * 2.0;   // small noise
  }
  Fit f = fit(corona, moist, press, N);
  printf("    b0 %.2f (5), b_moist %.3f (3), b_press %.4f (0.5), R^2 %.3f\n", f.b0, f.b_moist, f.b_press, f.r2);
  check(f.ok && fabs(f.b_moist - 3.0) < 0.3 && fabs(f.b_press - 0.5) < 0.1, "recovers the moisture and pressure coefficients");
  check(f.r2 > 0.95, "R^2 > 0.95 when the corona IS moisture + pressure");

  printf("[2] corona with an extra hidden component\n");
  double extra[N];
  for (int i = 0; i < N; i++) {
    extra[i] = (i % 2 == 0) ? 20.0 : 0.0;             // a structured residual (a 'state')
    corona[i] = 5.0 + 3.0 * moist[i] + 0.5 * press[i] + extra[i] + noise(rng) * 2.0;
  }
  Fit f2 = fit(corona, moist, press, N);
  double var_resid = 0, mean_resid = 0;
  for (int i = 0; i < N; i++) mean_resid += residual(f2, corona[i], moist[i], press[i]);
  mean_resid /= N;
  // correlate the residual with the hidden component
  double cov = 0, vr = 0, ve = 0, me = 0;
  for (int i = 0; i < N; i++) me += extra[i]; me /= N;
  for (int i = 0; i < N; i++) {
    double r = residual(f2, corona[i], moist[i], press[i]) - mean_resid;
    cov += r * (extra[i] - me); vr += r * r; ve += (extra[i] - me) * (extra[i] - me);
  }
  double corr = cov / sqrt(vr * ve);
  printf("    R^2 now %.3f, residual correlates with the hidden component at r = %.2f\n", f2.r2, corr);
  check(f2.r2 < 0.95, "R^2 drops when something beyond moisture + pressure is present");
  check(corr > 0.8, "the residual recovers the hidden component (this is the part worth keeping)");

  printf("[3] guards\n");
  check(!fit(corona, moist, press, 3).ok, "fewer than 4 exposures: no fit");
  double cm[6], mm[6], pp[6];
  for (int i = 0; i < 6; i++) { cm[i] = i; mm[i] = 1.0; pp[i] = i; }    // moisture constant -> singular column
  check(!fit(cm, mm, pp, 6).ok, "a constant regressor is detected as singular, not a garbage fit");

  printf("[4] moisture and pressure moving together (the confound `vary` avoids)\n");
  // nearly collinear: pressure tracks moisture with a little independent jitter,
  // as it would if the subject pressed harder as the finger got damp
  for (int i = 0; i < N; i++) {
    moist[i] = 2 + i * 0.4 + noise(rng) * 0.3;
    press[i] = 20 + i * 4.0 + noise(rng) * 5.0;
    corona[i] = 5 + 3 * moist[i] + 0.5 * press[i] + noise(rng) * 2.0;
  }
  Fit f4 = fit(corona, moist, press, N);
  // the combined prediction and R^2 are good, but the coefficient split is unstable
  // (that is why `vary` walks one variable at a time)
  check(f4.ok && f4.r2 > 0.9, "near-collinear moisture/pressure still fits (but `vary` is needed to split the coefficients cleanly)");
  // and perfect collinearity IS correctly refused as singular
  double mc[8], pc[8], cc[8];
  for (int i = 0; i < 8; i++) { mc[i] = 2 + i; pc[i] = 20 + 10 * i; cc[i] = 5 + 3 * mc[i] + 0.5 * pc[i]; }
  check(!fit(cc, mc, pc, 8).ok, "perfectly collinear inputs are refused as singular, not fitted with garbage");

  printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
  return fails ? 1 : 0;
}
