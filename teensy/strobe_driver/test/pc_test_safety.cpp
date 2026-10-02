// pc_test_safety.cpp -- strobe_driver's safety gate on a PC.
//   g++ -std=c++17 -I.. pc_test_safety.cpp -o t && ./t
#include "../strobe_safety.h"
#include <cstdio>
#include <cstring>
using namespace strobe_safety;
static int passes = 0, fails = 0;
static void check(bool c, const char *w) { if (c) { passes++; printf("  PASS  %s\n", w); } else { fails++; printf("  FAIL  %s\n", w); } }

int main() {
  printf("[1] ramp-in\n");
  Ramp r;
  r.start();
  check(r.ramping() && r.level(4096) == 0, "first start after power-up ramps from zero");
  int at1 = 0, at2 = 0, at3 = 0;
  for (uint32_t t = 0; t < (uint32_t)(4 * TICK_HZ); t++) {
    if (t == (uint32_t)(1.0 * TICK_HZ)) at1 = r.level(4096);
    if (t == (uint32_t)(2.0 * TICK_HZ)) at2 = r.level(4096);
    if (t == (uint32_t)(3.5 * TICK_HZ)) at3 = r.level(4096);
    r.tick(true);
  }
  printf("    level at 1 s %d, 2 s %d, 3.5 s %d of 4096\n", at1, at2, at3);
  check(at1 > 1300 && at1 < 1440 && at2 > 2680 && at2 < 2780 && at3 == 4096, "linear over 3 s: 1/3 at 1 s, 2/3 at 2 s, full after 3 s");
  for (uint32_t t = 0; t < (uint32_t)(5 * TICK_HZ); t++) r.tick(false);     // 5 s dark
  r.start();
  check(!r.ramping() && r.level(4096) == 4096, "a restart after only 5 s dark does not ramp again (trial gaps)");
  for (uint32_t t = 0; t < (uint32_t)(11 * TICK_HZ); t++) r.tick(false);    // 11 s dark
  r.start();
  check(r.ramping() && r.level(1000) == 0, "after 10 s or more of darkness the next start ramps again");
  int prev = -1; bool mono = true;
  for (uint32_t t = 0; t < RAMP_TICKS; t++) { int l = r.level(1000); if (l < prev) mono = false; prev = l; r.tick(true); }
  check(mono && r.level(1000) == 1000, "the ramp never steps down and ends exactly at the set level");

  printf("[2] gate\n");
  check(check(10, 10, 20, DEFAULT_CAP_PCT, false) != nullptr, "unscreened: 10 Hz at 20 % refused");
  check(check(10, 10, 5, DEFAULT_CAP_PCT, false) == nullptr, "unscreened: 10 Hz at 5 % allowed (that is the screen level)");
  check(check(1, 1, 40, DEFAULT_CAP_PCT, false) == nullptr, "unscreened: 1 Hz at 40 % allowed (outside 2-55 Hz)");
  check(check(70, 70, 40, DEFAULT_CAP_PCT, false) == nullptr, "unscreened: 70 Hz at 40 % allowed");
  check(check(1, 70, 20, DEFAULT_CAP_PCT, false) != nullptr, "unscreened: a glide from 1 to 70 Hz crosses the band and is refused at 20 %");
  check(check(70, 1, 20, DEFAULT_CAP_PCT, false) != nullptr, "... in either direction");
  check(check(0, 0, 20, DEFAULT_CAP_PCT, false) == nullptr, "dark trials (0 Hz) are always allowed");
  check(check(10, 10, 20, DEFAULT_CAP_PCT, true) == nullptr, "screened: 10 Hz at 20 % allowed");
  check(check(10, 10, 60, DEFAULT_CAP_PCT, true) != nullptr, "screened: 60 % refused under the default 50 % cap");
  check(check(10, 10, 60, 80, true) == nullptr, "screened, cap raised to 80 %: 60 % allowed");
  check(touchesRisk(2, 2) && touchesRisk(55, 55) && !touchesRisk(1.99f, 1.99f) && !touchesRisk(55.01f, 60), "band edges 2 and 55 Hz inclusive");
  printf("\n=== %d PASS, %d FAIL ===\n", passes, fails);
  return fails ? 1 : 0;
}
