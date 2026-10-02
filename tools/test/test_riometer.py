#!/usr/bin/env python3
"""
test_riometer.py -- riometer_host.py on a synthetic rtl_power log.

Three sidereal days of 60 s integrations at 30 MHz (100 bins of 10 kHz):
  * galactic background: -42 dB + 3 dB sinusoid with the SIDEREAL period
  * 0.05 dB rms noise per integration, 0.3 dB rms per bin (a real 10 kHz bin
    integrated for 60 s has about 0.006 dB of radiometer noise; the rest is
    gain drift and interference, so this is still pessimistic)
  * one narrowband interferer bin at +20 dB (must be rejected)
  * day 3, 30 min absorption of 2.5 dB
Expected: after the first sidereal day the quiet-day curve comes from the
history ("qdc"), absorption reads 2.5 +/- 0.2 dB during the event and stays
within +/-0.3 dB elsewhere on day 3, and the interferer leaves no trace
(kept, it would lift the band power by about 1.6 dB).
"""
import math
import os
import random
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import riometer_host as rh  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


rng = random.Random(3)
T0 = 1_780_000_000
SD = rh.SIDEREAL_DAY
N = int(3 * SD / 60)
T_ABS = T0 + 2 * SD + 5 * 3600
tmp = tempfile.mkdtemp()
path = os.path.join(tmp, "rtl_power.csv")
import datetime as dt  # noqa: E402
with open(path, "w") as f:
    for i in range(N):
        t = T0 + 60 * i
        sky = -42.0 + 3.0 * math.sin(2 * math.pi * (t - T0) / SD) + rng.gauss(0, 0.05)
        if T_ABS <= t < T_ABS + 1800:
            sky -= 2.5
        bins = [sky + rng.gauss(0, 0.3) for _ in range(100)]
        bins[37] = sky + 20.0                       # a narrowband interferer
        stamp = dt.datetime.fromtimestamp(t, dt.timezone.utc)
        f.write(f"{stamp:%Y-%m-%d}, {stamp:%H:%M:%S}, 29500000, 30500000, 10000.00, 600, "
                + ", ".join(f"{b:.2f}" for b in bins) + "\n")

print("[1] replay")
a = types.SimpleNamespace(file=path, freq=30.0, mode="absorb", qdc_days=7, csv=os.path.join(tmp, "out.csv"),
                          station="T", push=None)
p = rh.cmd_replay(a)
rows = p.rows
check(len(rows) == N, f"{len(rows)} integrations processed")
day1 = [r for r in rows if r["t"] < T0 + SD]
day3 = [r for r in rows if r["t"] >= T0 + 2 * SD]
check(all(r["quiet_source"] == "running-median" for r in day1[:60]), "first hour uses the running-median stand-in")
check(all(r["quiet_source"] == "qdc" for r in day3), "day 3 uses the quiet-day curve")

print("[2] interferer rejection")
# the true sky power is -42 + 3 sin; with the +20 dB bin kept, the linear mean would read ~+1.6 dB high
err = [abs(r["power_db"] - (-42.0 + 3.0 * math.sin(2 * math.pi * (r["t"] - T0) / SD))) for r in rows
       if not (T_ABS <= r["t"] < T_ABS + 1800)]
check(max(err) < 0.3, f"band power within {max(err):.2f} dB of the true sky (interferer bin dropped)")

print("[3] absorption")
during = [r["absorption_db"] for r in day3 if T_ABS + 120 <= r["t"] < T_ABS + 1800]
quiet = [r["absorption_db"] for r in day3 if not (T_ABS - 600 <= r["t"] < T_ABS + 2400)]
m = sum(during) / len(during)
check(abs(m - 2.5) < 0.2, f"mean absorption during the event {m:.2f} dB (true 2.5)")
check(max(abs(x) for x in quiet) < 0.3, f"largest |absorption| outside the event {max(abs(x) for x in quiet):.2f} dB (< 0.3)")

print("[4] CSV output")
with open(a.csv) as f:
    lines = f.read().strip().splitlines()
check(lines[0].startswith("utc,unix,freq_mhz,power_db,quiet_db,quiet_source,absorption_db") and len(lines) == N + 1,
      "CSV header and one row per integration")

print("[5] sidereal minute arithmetic")
check(rh.sidereal_minute(T0) == rh.sidereal_minute(T0 + SD) and rh.sidereal_minute(T0) != rh.sidereal_minute(T0 + 86400),
      "same sidereal minute one sidereal day later, different one solar day later")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
