// kirlian_analysis.h -- the analysis core of kirlian_logger: factor a corona
// measurement into the mundane causes (skin moisture, contact pressure) and the
// residual. Free of Arduino so a PC tests it (test/pc_test.cpp).
//
// The 1972 CIA test (Investigation of the Kirlian Phenomenon) and the SRI files
// found the corona tracks moisture and pressure. This fits
//     corona = b0 + b_moist * moisture + b_press * pressure
// by ordinary least squares over the logged exposures, and reports R^2 and the
// residual corona after moisture and pressure are removed. If the residual is
// flat noise, the "aura" was moisture + pressure; a structured residual
// (e.g. correlated with a claimed state) is the open question worth keeping.
#pragma once
#include <math.h>
#include <stddef.h>

namespace kirlian {

struct Fit {
  double b0, b_moist, b_press;   // coefficients
  double r2;                     // fraction of corona variance explained
  double rms_resid;              // rms of corona - prediction (same units as corona)
  int n;
  bool ok;
};

// 3-parameter OLS via normal equations (X = [1, moist, press]); n must be >= 4.
inline Fit fit(const double *corona, const double *moist, const double *press, int n) {
  Fit f = {0, 0, 0, 0, 0, n, false};
  if (n < 4) return f;
  // sums
  double S[3][4] = {{0}};             // augmented X'X | X'y, rows for [1, m, p]
  double meanY = 0;
  for (int i = 0; i < n; i++) meanY += corona[i];
  meanY /= n;
  for (int i = 0; i < n; i++) {
    double xi[3] = {1.0, moist[i], press[i]};
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) S[a][b] += xi[a] * xi[b];
      S[a][3] += xi[a] * corona[i];
    }
  }
  // Gauss-Jordan with partial pivoting on the 3x4 system
  for (int col = 0; col < 3; col++) {
    int piv = col;
    for (int r = col + 1; r < 3; r++) if (fabs(S[r][col]) > fabs(S[piv][col])) piv = r;
    if (fabs(S[piv][col]) < 1e-12) return f;         // singular (e.g. constant regressor)
    if (piv != col) for (int c = 0; c < 4; c++) { double t = S[col][c]; S[col][c] = S[piv][c]; S[piv][c] = t; }
    double d = S[col][col];
    for (int c = 0; c < 4; c++) S[col][c] /= d;
    for (int r = 0; r < 3; r++) if (r != col) { double m = S[r][col]; for (int c = 0; c < 4; c++) S[r][c] -= m * S[col][c]; }
  }
  f.b0 = S[0][3]; f.b_moist = S[1][3]; f.b_press = S[2][3];
  double ssres = 0, sstot = 0;
  for (int i = 0; i < n; i++) {
    double pred = f.b0 + f.b_moist * moist[i] + f.b_press * press[i];
    double e = corona[i] - pred;
    ssres += e * e;
    sstot += (corona[i] - meanY) * (corona[i] - meanY);
  }
  f.r2 = (sstot > 0) ? 1.0 - ssres / sstot : 0.0;
  f.rms_resid = sqrt(ssres / n);
  f.ok = true;
  return f;
}

// residual corona for one exposure, given a fit
inline double residual(const Fit &f, double corona, double moist, double press) {
  return corona - (f.b0 + f.b_moist * moist + f.b_press * press);
}

}  // namespace kirlian
