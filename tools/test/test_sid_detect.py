#!/usr/bin/env python3
"""
test_sid_detect.py -- the flare detector on a synthetic day.

Builds 24 h of one-second station levels with:
  * a sunrise ramp (+9 dB over 70 min) and a sunset ramp (-9 dB over 70 min)
  * 0.25 dB rms noise, and lightning spikes (+6 dB for one second, 200 of them)
  * flare A at 14:00: +3.2 dB rise over 3 min on 5 channels, 40 min recovery
  * flare B at 16:30: -2.0 dB over 2 min on 3 channels, 25 min recovery
  * a one-channel-only step at 11:00 (must NOT be an event)
and a GOES series with a C4 at flare A, nothing at B.

Expected: exactly 2 events, onsets within 120 s of the true onsets, A GOES-
confirmed, B not; none during the ramps or at 11:00; a second station with the
same flare makes it multi-station.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import sid_detect  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


T0 = 1_780_000_000                          # some UTC midnight-ish epoch
CALLS = ["NLK", "NPM", "NAA", "NML", "NAU", "NWC", "JJI", "REF"]
BASE = {"NLK": 48, "NPM": 40, "NAA": 45, "NML": 38, "NAU": 30, "NWC": 25, "JJI": 28, "REF": 8}


def ramp(t, t0, dur, amount):
    if t < t0:
        return 0.0
    if t > t0 + dur:
        return amount
    x = (t - t0) / dur
    return amount * (0.5 - 0.5 * math.cos(math.pi * x))


def flare(t, t_on, rise_s, amp, decay_s):
    if t < t_on:
        return 0.0
    if t < t_on + rise_s:
        return amp * (t - t_on) / rise_s
    return amp * math.exp(-(t - t_on - rise_s) / decay_s)


def make_station(seed, flares, single_step=None):
    rng = random.Random(seed)
    rows = []
    spikes = set(rng.randrange(86400) for _ in range(200))
    for s in range(0, 86400, 1):
        t = T0 + s
        lv = {}
        day = ramp(s, 6 * 3600, 70 * 60, 9.0) - ramp(s, 18 * 3600, 70 * 60, 9.0)
        for c in CALLS:
            v = BASE[c] + (day if c != "REF" else 0) + rng.gauss(0, 0.25)
            for (t_on, rise, amp, decay, chans) in flares:
                if c in chans:
                    v += flare(s, t_on, rise, amp, decay)
            if single_step and c == single_step[0] and s >= single_step[1]:
                v += single_step[2]
            if s in spikes:
                v += 6.0
            lv[c] = v
        rows.append((t, lv))
    return rows


FLARE_A = (14 * 3600, 180, 3.2, 2400, ["NLK", "NPM", "NAA", "NML", "NAU"])
FLARE_B = (16 * 3600 + 1800, 120, -2.0, 1500, ["NLK", "NAA", "NML"])

print("[1] single station, synthetic day")
rows = make_station(1, [FLARE_A, FLARE_B], single_step=("NPM", 11 * 3600, 4.0))
ev = sid_detect.detect(rows)
for e in ev:
    print(f"    event: onset {sid_detect.fmt(e['t_onset'])} dev {e['dev_db']:+.2f} dB "
          f"on {e['n_ch']} ch {e['channels']}")
check(len(ev) == 2, f"{len(ev)} events found (expected 2)")
if len(ev) >= 2:
    a, b = sorted(ev, key=lambda e: e["t_onset"])[:2]
    check(abs(a["t_onset"] - (T0 + FLARE_A[0])) <= 120, f"flare A onset within 120 s ({a['t_onset'] - T0 - FLARE_A[0]:+.0f} s)")
    check(abs(b["t_onset"] - (T0 + FLARE_B[0])) <= 120, f"flare B onset within 120 s ({b['t_onset'] - T0 - FLARE_B[0]:+.0f} s)")
    check(a["sign"] == 1 and b["sign"] == -1, "signs: A up, B down")
    check(a["n_ch"] >= 4 and b["n_ch"] >= 2, f"channel counts A={a['n_ch']} (>=4), B={b['n_ch']} (>=2)")
    check(2.5 < a["dev_db"] < 4.0, f"A peak deviation {a['dev_db']:+.2f} dB (true +3.2)")
ramp_hits = [e for e in ev if (6 * 3600 <= e["t_onset"] - T0 <= 7.5 * 3600) or (18 * 3600 <= e["t_onset"] - T0 <= 19.5 * 3600)]
check(not ramp_hits, "no events during sunrise/sunset ramps")
step_hits = [e for e in ev if abs(e["t_onset"] - T0 - 11 * 3600) < 1800]
check(not step_hits, "the one-channel step at 11:00 is not an event")

print("[2] GOES confirmation")
goes = [(T0 + s, 2e-7 + (4e-6 * flare(s, FLARE_A[0] - 60, 240, 1.0, 1200))) for s in range(0, 86400, 60)]
sid_detect.goes_confirm(ev, goes)
if len(ev) >= 2:
    check(a.get("goes_confirmed") and a["goes_class"].startswith("C"), f"A confirmed by GOES ({a.get('goes_class')})")
    check(not b.get("goes_confirmed"), f"B not confirmed ({b.get('goes_class')})")

print("[3] second station, multi-station flag")
rows2 = make_station(2, [FLARE_A])
ev2 = sid_detect.detect(rows2)
by = sid_detect.cross_station({"HOME": ev, "FRIEND": ev2})
check(len(ev2) == 1, f"station 2 sees {len(ev2)} event (expected 1: only flare A)")
if ev2 and len(ev) >= 2:
    check(a["multi_station"] and a["other_stations"] == ["FRIEND"], "flare A multi-station (HOME + FRIEND)")
    check(not b["multi_station"], "flare B single-station")

print("[4] CSV round trip")
import csv  # noqa: E402
import tempfile  # noqa: E402
import datetime as dt  # noqa: E402
tmp = tempfile.mkdtemp()
path = os.path.join(tmp, "vlf_test.csv")
with open(path, "w", newline="") as fh:
    w = csv.writer(fh)
    cols = ["utc", "unix", "seq", "nsamp", "pps", "clip", "in_rms", "in_dc"]
    for c in CALLS:
        cols += [f"{c}_db", f"{c}_spike"]
    w.writerow(cols)
    for i, (t, lv) in enumerate(rows[13 * 3600:15 * 3600]):
        stamp = dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        r = [stamp, f"{t:.1f}", i, 390625, 1, 0, "120.0", "0.0"]
        for c in CALLS:
            r += [f"{lv[c]:.2f}", "3.0"]
        w.writerow(r)
rr = sid_detect.read_vlf_csv(path)
ev3 = sid_detect.detect(rr)
check(len(rr) == 7200 and len(ev3) == 1 and abs(ev3[0]["t_onset"] - (T0 + FLARE_A[0])) <= 120,
      f"CSV of 13:00-15:00 read back ({len(rr)} rows) and flare A found again")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
