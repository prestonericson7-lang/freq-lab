// pc_test.cpp -- rf_survey's arithmetic on a PC.
//   g++ -std=c++17 -DRF_PC_TEST -I.. pc_test.cpp -o pc_test && ./pc_test
// Checks: AD8318 dBm/volt conversion and two-point calibration, uW/cm2 from
// the antenna aperture, the per-second level (log-mean), the pulse detector
// and interval histogram on a synthetic Wi-Fi-beacon-plus-LTE waveform, and
// the naming tables.
#define RF_PC_TEST
#include "../rf_analysis.h"
#include <cstdio>
#include <cmath>
#include <random>

static int passes = 0, fails = 0;
static void check(bool c, const char *what) { if (c) { passes++; printf("  PASS  %s\n", what); } else { fails++; printf("  FAIL  %s\n", what); } }

int main() {
  printf("[1] calibration\n");
  RfCal cal;
  // datasheet typicals: 0 dBm -> about 0.49 V, -60 dBm -> 1.95 V
  check(fabs(cal.voltsOf(0.0) - 0.488) < 0.01, "0 dBm reads 0.49 V with the default slope/intercept");
  check(fabs(cal.dbmOf(1.952) - (-60.0)) < 0.5, "1.952 V reads -60 dBm");
  cal.addPoint(-10.0, 0.80);                      // a real detector: -10 dBm gave 0.80 V ...
  int k = cal.addPoint(-50.0, 1.80);              // ... and -50 dBm gave 1.80 V
  check(k == 2 && fabs(cal.slope_v_per_db - (-0.025)) < 1e-9 && fabs(cal.intercept_dbm - 22.0) < 1e-9,
        "two-point fit: slope -25.0 mV/dB, intercept +22 dBm");
  check(fabs(cal.dbmOf(1.30) - (-30.0)) < 1e-9, "1.30 V now reads -30 dBm");
  int k2 = cal.addPoint(-12.0, 0.85);
  check(k2 == 1, "a point within 5 dB of the first replaces it instead of fitting");

  printf("[2] power density\n");
  cal.reset(); cal.setAntenna(2.44, 2.0);
  check(fabs(cal.aeff_cm2() - 19.05) < 0.3, "2 dBi at 2.44 GHz: A_eff 19 cm2");
  check(fabs(cal.uw_cm2(-30.0) - 0.0525) < 0.002, "-30 dBm into 19 cm2 = 0.0525 uW/cm2");
  cal.setAntenna(0.915, 2.15);
  check(fabs(cal.aeff_cm2() - 140.1) < 1.0 && fabs(cal.uw_cm2(0.0) - 7.14) < 0.05, "0 dBm at a 915 MHz dipole: 1 mW over 140 cm2 = 7.14 uW/cm2");

  printf("[3] level accumulator\n");
  LevelAccumulator lv;
  for (int i = 0; i < 1000; i++) lv.add(i < 100 ? -20.0 : -60.0);     // 10 % duty at -20 dBm
  LevelResult r = lv.result(cal);
  check(fabs(r.mean_dbm - (-30.0)) < 0.05, "10 % duty of -20 dBm over -60 dBm averages to -30.0 dBm (power mean, not dB mean)");
  check(fabs(r.peak_dbm - (-20.0)) < 1e-9 && fabs(r.duty - 0.10) < 1e-9, "peak -20 dBm, duty 0.10 above -40 dBm");

  printf("[4] pulses: Wi-Fi beacons every 102.4 ms + LTE 1 ms bursts\n");
  PulseDetector pd; pd.fs = 50000.0;
  IntervalHistogram h;
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.5);
  const double fs = 50000.0;
  int nsamp = (int)(fs * 10.0);                    // 10 s
  // powers add: floor -65 dBm, a 1 ms Wi-Fi beacon at -35 every 102.4 ms, a 0.2 ms LTE burst at -45 every 10 ms
  auto waveform = [&](int i, std::mt19937 &r) {
    double t_ms = i * 1000.0 / fs;
    double mw = pow(10.0, -6.5);
    if (fmod(t_ms, 102.4) < 1.0) mw += pow(10.0, -3.5);
    if (fmod(t_ms + 37.0, 10.0) < 0.2) mw += pow(10.0, -4.5);
    return 10.0 * log10(mw) + noise(r);
  };
  for (int i = 0; i < nsamp; i++) pd.feed(waveform(i, rng), h);
  IntervalPeak top[6];
  int n = h.top(top, 6);
  printf("    median tracker %.1f dBm, %lu pulses\n", pd.median_dbm(), (unsigned long)h.count);
  for (int i = 0; i < n; i++) printf("    %9.3f ms  x%lu  %s\n", top[i].ms, (unsigned long)top[i].count, nameInterval(top[i].ms));
  bool got10 = false;
  for (int i = 0; i < n; i++) if (fabs(top[i].ms - 10.0) < 0.6) got10 = true;
  check(pd.median_dbm() < -55.0, "median tracker sits near the -65 dBm floor, not on the pulses");
  check(h.count > 900 && h.count < 1200, "about 1000 LTE frames + 98 beacons counted in 10 s");
  check(got10, "the 10 ms LTE frame interval is the top peak at the 6 dB threshold (beacons only split it)");
  // the same waveform with the threshold above the LTE level: only the beacons cross -> 102.4 ms
  PulseDetector pd2; pd2.fs = fs; pd2.thresh_db = 25.0;
  IntervalHistogram h2;
  std::mt19937 rng2(7);
  for (int i = 0; i < nsamp; i++) pd2.feed(waveform(i, rng2), h2);
  IntervalPeak top2[3];
  int n2 = h2.top(top2, 3);
  printf("    at 25 dB: %lu pulses, top %.1f ms x%lu %s\n", (unsigned long)h2.count, n2 ? top2[0].ms : 0.0,
         (unsigned long)(n2 ? top2[0].count : 0), n2 ? nameInterval(top2[0].ms) : "");
  check(n2 >= 1 && fabs(top2[0].ms - 102.4) < 6.0 && h2.count >= 90 && h2.count <= 102,
        "with the threshold above the LTE bursts, the 102.4 ms Wi-Fi beacon interval is the top peak");
  check(strcmp(nameInterval(10.0), "LTE/DECT frame") == 0 && strcmp(nameInterval(102.4), "Wi-Fi beacon") == 0, "interval naming");

  printf("[5] spectral peak finder and line naming\n");
  const int NB = 2048; double pxx[NB]; double hz_per_bin = 250.0 / 4096;   // slow spectrum
  for (int b = 0; b < NB; b++) pxx[b] = 1.0;
  int b_beacon = (int)(9.766 / hz_per_bin + 0.5), b_mains = (int)(60.0 / hz_per_bin + 0.5);
  pxx[b_beacon] = 1000; pxx[b_beacon - 1] = 300; pxx[b_beacon + 1] = 300; pxx[b_mains] = 200;
  LinePeak pk[8];
  int np = findPeaks(pxx, NB, hz_per_bin, pk, 8);
  check(np >= 2 && fabs(pk[0].hz - 9.766) < 0.07 && fabs(pk[1].hz - 60.0) < 0.07, "the two planted lines come out first, strongest first");
  check(strcmp(nameLine(pk[0].hz), "Wi-Fi beacon (102.4 ms)") == 0 && strcmp(nameLine(pk[1].hz), "60 Hz mains / light") == 0, "line naming");
  check(fabs(pk[1].db_rel - (-6.99)) < 0.05, "second line reported at -7.0 dB relative");

  printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
  return fails ? 1 : 0;
}
