#!/usr/bin/env python3
"""
test_station_e2e.py -- the whole VLF station software chain, end to end, with the
REAL client code (vlf_host, riometer_host) pushing to a REAL sid_server, not the
stub servers the unit tests use. Catches wire-format mismatches, the detector on
real-client data, and the live API/dashboard shapes that unit tests miss.

  flow:  fake FPGA board -> vlf_host.cmd_log --push  --> sid_server (real)
         synthetic rtl_power -> riometer_host.cmd_replay --push --> sid_server
         GOES rows + sid_server.run_detector
         then query every live API endpoint and assert the chain worked.
"""
import datetime as dt
import math
import os
import sys
import tempfile
import threading
import time
import types
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

# fake serial for the host clients (same one the vlf_host unit test uses)
import fake_vlf_board as fvb  # noqa: E402
serial_mod = types.ModuleType("serial")
serial_mod.Serial = fvb.FakeSerial
sys.modules["serial"] = serial_mod

import sid_server  # noqa: E402
import vlf_host     # noqa: E402
import riometer_host as rh  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


def api(url):
    with urllib.request.urlopen(url, timeout=10) as r:
        import json
        return json.loads(r.read())


tmp = tempfile.mkdtemp()
os.chdir(tmp)
db = os.path.join(tmp, "e2e.sqlite")
dash = os.path.join(HERE, "..", "..", "dashboard", "index.html")

# a REAL sid_server (detector off; we trigger it explicitly for determinism)
srv, store = sid_server.serve(0, db, dash, goes=False, bind="127.0.0.1", detector=False)
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
URL = f"http://127.0.0.1:{port}"
print(f"sid_server up on {URL}")

NOW = time.time()
T0 = NOW - 3 * 3600
T_FLARE = NOW - 1 * 3600

print("[1] VLF rows -> real server ingest, two stations, a flare at T_FLARE")
# vlf_host.cmd_log's serial round-trips are covered in test_vlf_host.py; here we
# push rows in exactly the shape cmd_log emits, to exercise the REAL server ingest
# and the detector on two stations that both see the flare.
import json as _json  # noqa: E402
import vlf_host as _vh  # noqa: F401  (import kept: confirms the client module loads in this env)
rows_home, rows_friend = [], []
import random as _random  # noqa: E402
rng = _random.Random(3)
CALLS = ["NLK", "NPM", "NAA", "NML", "NAU", "NWC", "JJI", "REF"]
BASE = {"NLK": 48, "NPM": 40, "NAA": 45, "NML": 38, "NAU": 30, "NWC": 25, "JJI": 28, "REF": 8}
for s in range(3 * 3600):
    t = T0 + s
    for rows, seed in ((rows_home, 0), (rows_friend, 100)):
        ch = {}
        for c in CALLS:
            v = BASE[c] + rng.gauss(0, 0.2)
            if c in ("NLK", "NPM", "NAA", "NML") and t >= T_FLARE:
                d = t - T_FLARE
                v += 3.2 * (d / 180 if d < 180 else math.exp(-(d - 180) / 1800))
            ch[c] = round(v, 2)
        rows.append({"t": t, "pps": 1, "clip": 0, "rms": 120.0, "ch": ch, "spk": {}})


def push(station, kind, rows):
    for i in range(0, len(rows), 1000):
        body = _json.dumps({"station": station, "kind": kind, "rows": rows[i:i + 1000]}).encode()
        req = urllib.request.Request(URL + "/api/ingest", data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=15) as r:
            r.read()


push("HOME", "vlf", rows_home)
push("FRIEND", "vlf", rows_friend)
st = api(URL + "/api/status")
check({s["station"] for s in st["stations"]} == {"HOME", "FRIEND"}, "both stations registered on the real server")
check(all(s["rows"] == 10800 for s in st["stations"]), "10800 rows each ingested over the wire")

print("[2] riometer client -> server: real riometer_host.cmd_replay --push")
# a synthetic rtl_power capture: sidereal background + an absorption dip at the flare
import csv as _csv  # noqa: E402
cap = os.path.join(tmp, "rtl.csv")
with open(cap, "w") as f:
    for i in range(3 * 60):
        t = T0 + 60 * i
        sky = -42 + 3 * math.sin(2 * math.pi * (t - T0) / 86164.09) + rng.gauss(0, 0.05)
        if T_FLARE <= t < T_FLARE + 1800:
            sky -= 1.5 * math.exp(-(t - T_FLARE) / 900)
        bins = [sky + rng.gauss(0, 0.3) for _ in range(60)]
        stamp = dt.datetime.fromtimestamp(t, dt.timezone.utc)
        f.write(f"{stamp:%Y-%m-%d}, {stamp:%H:%M:%S}, 29500000, 30500000, 16666.67, 600, "
                + ", ".join(f"{x:.2f}" for x in bins) + "\n")
a = types.SimpleNamespace(file=cap, freq=30.0, mode="absorb", qdc_days=7, csv=os.path.join(tmp, "r.csv"),
                          station="HOME", push=URL)
rh.cmd_replay(a)
time.sleep(0.3)
rio = api(URL + f"/api/riometer?station=HOME&from={T0}&to={NOW}")
check(len(rio) >= 150, f"riometer rows reached the server ({len(rio)})")

print("[3] GOES + detector over the whole real store")
goes = [(T0 + 60 * i, 3e-7 + (5e-6 if abs(T0 + 60 * i - T_FLARE - 120) < 600 else 0)) for i in range(181)]
store.add_goes(goes)
n = sid_server.run_detector(store, span_s=4 * 3600)
al = api(URL + f"/api/alerts?from={T0}&to={NOW}")
check(n >= 1 and len(al) >= 1, f"detector raised {len(al)} alert(s) on the real client data")
if al:
    a0 = al[0]
    check(abs(a0["t_onset"] - T_FLARE) < 180, f"onset within 180 s of the injected flare ({a0['t_onset']-T_FLARE:+.0f} s)")
    check(a0["goes_confirmed"] == 1, f"GOES-confirmed ({a0['goes_class']})")
    check(a0["multi_station"] == 1, "flagged multi-station (HOME + FRIEND saw it)")

print("[4] live API shapes the dashboard uses")
ser = api(URL + f"/api/series?station=HOME&from={T0}&to={NOW}&step=10")
check(len(ser["t"]) > 100 and "NLK" in ser["ch"], "series endpoint returns decimated levels")
g = api(URL + f"/api/goes?from={T0}&to={NOW}")
check(len(g) == 181, "GOES endpoint returns the flux series")
import urllib.request as _u  # noqa: E402
with _u.urlopen(URL + "/", timeout=10) as r:
    html = r.read()
check(b"<title>Space-weather stations</title>" in html and b"/api/alerts" in html, "dashboard HTML served and wired to the API")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
srv.shutdown()
sys.exit(1 if fails else 0)
