#!/usr/bin/env python3
"""
test_sid_server.py -- the station server end to end, in process, on a temp DB.

  python tools/test/test_sid_server.py

  1. two stations upload 3 h of one-second VLF rows (synthetic, flare at +2 h)
  2. two stations upload lightning stamps; one stroke seen by both 1.2 ms apart
  3. GOES rows added; the detector run creates GOES-confirmed, multi-station alerts
  4. every GET endpoint returns the right shape and numbers; / serves the dashboard
  5. bad requests are refused without crashing the server
"""
import json
import math
import os
import random
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import sid_server  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


tmp = tempfile.mkdtemp()
db = os.path.join(tmp, "test.sqlite")
dash = os.path.join(HERE, "..", "..", "dashboard", "index.html")
srv, store = sid_server.serve(0, db, dash, goes=False, bind="127.0.0.1", detector=False)
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
URL = f"http://127.0.0.1:{port}"


def post(path, obj):
    req = urllib.request.Request(URL + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def get(path):
    try:
        with urllib.request.urlopen(URL + path, timeout=10) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


NOW = time.time()
T0 = NOW - 3 * 3600
CALLS = ["NLK", "NPM", "NAA", "NML", "NAU", "NWC", "JJI", "REF"]
BASE = {"NLK": 48, "NPM": 40, "NAA": 45, "NML": 38, "NAU": 30, "NWC": 25, "JJI": 28, "REF": 8}
T_FLARE = T0 + 2 * 3600


def rows_for(seed, with_flare=True):
    rng = random.Random(seed)
    out = []
    for s in range(3 * 3600):
        t = T0 + s
        ch = {}
        for c in CALLS:
            v = BASE[c] + rng.gauss(0, 0.25)
            if with_flare and c in ("NLK", "NPM", "NAA", "NML") and t >= T_FLARE:
                dtf = t - T_FLARE
                v += 3.0 * (dtf / 180 if dtf < 180 else math.exp(-(dtf - 180) / 2000))
            ch[c] = round(v, 2)
        out.append({"t": t, "pps": 1, "clip": 0, "rms": 120.0, "ch": ch, "spk": {}})
    return out


print("[1] VLF ingest, two stations")
tot = 0
for st, seed in (("HOME", 1), ("FRIEND", 2)):
    rows = rows_for(seed)
    for i in range(0, len(rows), 600):
        code, rep = post("/api/ingest", {"station": st, "kind": "vlf", "rows": rows[i:i + 600]})
        tot += rep.get("stored", 0)
check(tot == 2 * 10800, f"{tot} rows stored (expected 21600)")
code, body = get("/api/status")
st = json.loads(body)
check(code == 200 and [s["station"] for s in st["stations"]] == ["FRIEND", "HOME"], "status lists both stations")
check(all(s["rows"] == 10800 and "vlf" in s["kinds"] for s in st["stations"]), "row counts and kinds per station")

print("[2] sferic ingest and time-of-arrival match")
t_stroke = NOW - 1800.123456
sf_home = [{"t": t_stroke, "pps": 5, "clk": 6172800, "off": -0.3, "peak": 1400, "pol": 1, "seq": 0, "snap": [2048] * 64},
           {"t": NOW - 900.5, "pps": 6, "clk": 1, "off": 0, "peak": 700, "pol": -1, "seq": 1, "snap": []}]
sf_friend = [{"t": t_stroke + 0.0012, "pps": 5, "clk": 6232800, "off": 0.1, "peak": 1100, "pol": 1, "seq": 0, "snap": []}]
post("/api/ingest", {"station": "HOME", "kind": "sferic", "rows": sf_home})
post("/api/ingest", {"station": "FRIEND", "kind": "sferic", "rows": sf_friend})
code, body = get(f"/api/sferics?from={NOW - 3600}&to={NOW}&bin=60")
sf = json.loads(body)
check(sf["total"] == 3 and len(sf["events"]) == 3, f"{sf['total']} strokes listed with raw events (span < 1 h)")
code, body = get(f"/api/toa?from={NOW - 3600}&to={NOW}")
toa = json.loads(body)
check(len(toa) == 1 and set(toa[0]["arrivals"]) == {"HOME", "FRIEND"}, "one stroke matched across both stations")
if toa:
    dt_us = toa[0]["arrivals"]["FRIEND"]["dt_us"]
    check(abs(dt_us - 1200.0) < 1.0, f"arrival difference {dt_us} us (expected 1200)")

print("[3] GOES + detector -> alerts")
goes = [(T0 + 60 * i, 3e-7 + (5e-6 if abs(T0 + 60 * i - T_FLARE - 300) < 900 else 0)) for i in range(181)]
store.add_goes(goes)
n = sid_server.run_detector(store)
code, body = get(f"/api/alerts?from={T0}&to={NOW}")
al = json.loads(body)
check(n == 2 and len(al) == 2, f"detector stored {len(al)} alerts (one per station)")
if len(al) == 2:
    a = al[0]
    check(abs(a["t_onset"] - T_FLARE) < 120, f"onset within 120 s of the true flare ({a['t_onset'] - T_FLARE:+.0f} s)")
    check(a["goes_confirmed"] == 1 and a["goes_class"].startswith("C"), f"GOES-confirmed, class {a['goes_class']}")
    check(a["multi_station"] == 1 and len(a["other_stations"]) == 1, "multi-station flag set")
    check(2.0 < a["dev_db"] < 4.0 and a["n_ch"] == 4, f"peak {a['dev_db']:+.2f} dB on {a['n_ch']} channels")
n2 = sid_server.run_detector(store)
code, body = get(f"/api/alerts?from={T0}&to={NOW}")
check(len(json.loads(body)) == 2, "a second detector pass updates the same alerts (no duplicates)")

print("[4] series, goes, riometer, dashboard")
code, body = get(f"/api/series?station=HOME&from={T0}&to={NOW}&step=10")
ser = json.loads(body)
check(len(ser["t"]) == 1080 and len(ser["ch"]["NLK"]) == 1080, f"series decimated by 10: {len(ser['t'])} points")
code, body = get(f"/api/goes?from={T0}&to={NOW}")
check(len(json.loads(body)) == 181, "GOES series returned")
post("/api/ingest", {"station": "HOME", "kind": "riometer", "rows": [{"t": NOW - 60, "freq_mhz": 30.0, "power_db": -40.1, "quiet_db": -39.5, "absorption_db": 0.6}]})
code, body = get(f"/api/riometer?station=HOME&from={T0}&to={NOW}")
check(len(json.loads(body)) == 1, "riometer row stored and returned")
post("/api/ingest", {"station": "HOME", "kind": "schumann", "rows": [
    {"t": NOW - 120, "m1_Hz": 7.81, "m1_H_pT": 1.04, "Z1": 371.0}, {"t": NOW - 60, "m1_Hz": 7.84, "m1_H_pT": 1.10, "Z1": 374.0}]})
code, body = get(f"/api/generic?station=HOME&kind=schumann&from={T0}&to={NOW}")
gen = json.loads(body)
check(len(gen) == 2 and gen[1]["m1_H_pT"] == 1.10, "schumann rows stored as generic kind and returned intact")
code, body = get("/")
check(code == 200 and b"<title>Space-weather stations</title>" in body and b"/api/series" in body, "dashboard served at /")
code, body = get("/api/status")
check("riometer" in [k for s in json.loads(body)["stations"] if s["station"] == "HOME" for k in s["kinds"]], "kinds updated to include riometer")

print("[5] bad input")
code, rep = post("/api/ingest", {"station": "X", "kind": "vlf", "rows": "nope"})
check(code == 400, "rows not a list -> 400")
code, rep = post("/api/ingest", {"station": "X", "kind": "vlf", "rows": [{"no_t": 1}, {"t": "bad"}]})
check(code == 200 and rep["stored"] == 0, "rows without usable fields are skipped, not fatal")
code, body = get("/api/nothing")
check(code == 404 or b"unknown" in body, "unknown path -> 404")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
srv.shutdown()
sys.exit(1 if fails else 0)
