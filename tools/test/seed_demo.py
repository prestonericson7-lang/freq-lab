#!/usr/bin/env python3
"""
seed_demo.py -- fill a demo database for the dashboard: two stations, 26 h of
one-second VLF rows with sunrise/sunset, a flare, lightning strokes (one seen
by both stations), GOES flux, a riometer series, and the detector's alerts.

  python tools/test/seed_demo.py demo.sqlite
  python tools/sid_server.py --db demo.sqlite          # then open http://localhost:8750/
"""
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import sid_server  # noqa: E402

db = sys.argv[1] if len(sys.argv) > 1 else "demo.sqlite"
if os.path.exists(db):
    os.remove(db)
store = sid_server.Store(db)
NOW = time.time()
T0 = NOW - 26 * 3600
CALLS = ["NLK", "NPM", "NAA", "NML", "NAU", "NWC", "JJI", "REF"]
BASE = {"NLK": 48, "NPM": 40, "NAA": 45, "NML": 38, "NAU": 30, "NWC": 25, "JJI": 28, "REF": 8}
T_FLARE = NOW - 5 * 3600


def ramp(x, x0, dur, amount):
    if x < x0:
        return 0.0
    if x > x0 + dur:
        return amount
    return amount * (0.5 - 0.5 * math.cos(math.pi * (x - x0) / dur))


for st, seed in (("HOME", 11), ("FRIEND", 12)):
    rng = random.Random(seed)
    rows = []
    for s in range(26 * 3600):
        t = T0 + s
        hour = ((t % 86400) / 3600.0 + 12) % 24          # pretend sunrise at 06:00 local = whatever UTC
        day = ramp(hour, 6, 1.2, 9.0) - ramp(hour, 18, 1.2, 9.0)
        ch = {}
        for c in CALLS:
            v = BASE[c] + (day if c != "REF" else 0) + rng.gauss(0, 0.25)
            if c in ("NLK", "NPM", "NAA", "NML") and t >= T_FLARE:
                d = t - T_FLARE
                v += 3.4 * (d / 180 if d < 180 else math.exp(-(d - 180) / 2400))
            if rng.random() < 0.002:
                v += 5.0
            ch[c] = round(v, 2)
        rows.append({"t": t, "pps": 1, "clip": 0, "rms": round(120 + 10 * day / 9, 1), "ch": ch, "spk": {}})
    for i in range(0, len(rows), 2000):
        store.ingest(st, "vlf", rows[i:i + 2000])

# lightning: a storm in the last 3 hours, HOME sees 400 strokes, FRIEND sees 250, 120 in common
rng = random.Random(5)
home, friend = [], []
for i in range(400):
    t = NOW - rng.uniform(0, 3 * 3600)
    home.append({"t": t, "pps": 0, "clk": int((t % 1) * 50e6), "off": 0, "peak": rng.randint(650, 2000),
                 "pol": rng.choice([1, -1]), "seq": i, "snap": []})
    if i < 120:
        friend.append({"t": t + rng.uniform(-0.0025, 0.0025), "pps": 0, "clk": 0, "off": 0,
                       "peak": rng.randint(650, 1800), "pol": home[-1]["pol"], "seq": i, "snap": []})
for i in range(130):
    t = NOW - rng.uniform(0, 3 * 3600)
    friend.append({"t": t, "pps": 0, "clk": 0, "off": 0, "peak": rng.randint(650, 1500), "pol": 1, "seq": 200 + i, "snap": []})
store.ingest("HOME", "sferic", home)
store.ingest("FRIEND", "sferic", friend)

# GOES: B-level background with a C5 flare
goes = []
for i in range(26 * 60):
    t = T0 + 60 * i
    f = 2.5e-7
    if t > T_FLARE - 300:
        d = t - (T_FLARE - 300)
        f += 5e-6 * (d / 480 if d < 480 else math.exp(-(d - 480) / 1500))
    goes.append((t, f))
store.add_goes(goes)

# riometer: sidereal sinusoid with an absorption dip at the flare
rio = []
for i in range(26 * 60):
    t = T0 + 60 * i
    p = -42 + 3 * math.sin(2 * math.pi * t / 86164.09) + random.gauss(0, 0.05)
    q = -42 + 3 * math.sin(2 * math.pi * t / 86164.09) + 0.05
    if T_FLARE < t < T_FLARE + 1800:
        p -= 1.2 * math.exp(-(t - T_FLARE) / 900)
    rio.append({"t": t, "freq_mhz": 30.0, "power_db": round(p, 3), "quiet_db": round(q, 3), "absorption_db": round(q - p, 3)})
store.ingest("HOME", "riometer", rio)

n = sid_server.run_detector(store, span_s=26 * 3600)
print(f"{db}: 2 stations x 26 h, {len(home) + len(friend)} strokes, {len(goes)} GOES points, {len(rio)} riometer rows, "
      f"{n} alerts")
