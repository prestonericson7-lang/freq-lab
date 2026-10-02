// strobe_safety.h -- the seizure-safety gate of strobe_driver (list item #150),
// kept free of Arduino so a PC compiles and tests it (test/pc_test_safety.cpp).
//
//   * ramp-in: light that starts after DARK_RESET_S of darkness ramps from zero to
//     the set brightness over RAMP_S (linear in drive level), so nobody meets a
//     full-intensity flicker without warning
//   * brightness cap: levels above `cap` (default 50 %) are refused; the cap goes
//     higher only with `cap <pct> yes`
//   * the risky band: photosensitive epilepsy is provoked mostly by 3-30 Hz and
//     up to ~55-60 Hz (UK ITC / Harding: 3-60 Hz; Fisher 2005); here 2-55 Hz
//   * screen: until a low-intensity screen has been run and accepted, anything in
//     the risky band is limited to SCREEN_LEVEL_PCT (5 %)
//   * abort: a button (pin 8 to ground) or the Esc key turns the light off from
//     inside the 20 kHz interrupt (the sketch does that part)
#pragma once
#include <stdint.h>

namespace strobe_safety {

static const double TICK_HZ = 20000.0;
static const double RAMP_S = 3.0, DARK_RESET_S = 10.0;
static const float  RISK_LO_HZ = 2.0f, RISK_HI_HZ = 55.0f;
static const float  SCREEN_LEVEL_PCT = 5.0f, DEFAULT_CAP_PCT = 50.0f;
static const uint32_t RAMP_TICKS = (uint32_t)(RAMP_S * TICK_HZ);
static const uint32_t DARK_RESET_TICKS = (uint32_t)(DARK_RESET_S * TICK_HZ);

struct Ramp {
  uint32_t dark = DARK_RESET_TICKS;   // ticks dark so far (saturating); starts "long dark"
  uint32_t left = 0;                  // ramp ticks still to run
  // call once per tick
  inline void tick(bool lit_session) {
    if (lit_session) { dark = 0; if (left) left--; }
    else if (dark < DARK_RESET_TICKS) dark++;
  }
  // call when a flashing session starts (on, glide, ext, a trial)
  inline void start() { if (dark >= DARK_RESET_TICKS) left = RAMP_TICKS; }
  // drive level to output now
  inline int level(int code) const {
    if (!left) return code;
    uint64_t done = RAMP_TICKS - left;
    return (int)((uint64_t)code * done / RAMP_TICKS);
  }
  inline bool ramping() const { return left != 0; }
};

// does [lo, hi] (Hz, either order) touch the risky band?
inline bool touchesRisk(float lo, float hi) {
  if (lo > hi) { float t = lo; lo = hi; hi = t; }
  return hi >= RISK_LO_HZ && lo <= RISK_HI_HZ;
}

// nullptr if allowed, else the reason
inline const char *check(float lo_hz, float hi_hz, float level_pct, float cap_pct, bool screened) {
  if (level_pct > cap_pct + 1e-3f) return "level above the brightness cap (`cap <pct> yes` to raise it)";
  if (!screened && touchesRisk(lo_hz, hi_hz) && level_pct > SCREEN_LEVEL_PCT + 1e-3f)
    return "2-55 Hz above 5 % needs the one-time screen first (`screen`, then `screen ok`)";
  return nullptr;
}

}  // namespace strobe_safety
