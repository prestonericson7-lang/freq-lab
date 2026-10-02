#!/usr/bin/env python3
"""
test_vlf_host.py -- vlf_host.py against the fake v2 board and a stub server.

  python tools/test/test_vlf_host.py

Checks:
  1. Board accepts the v2 ID and reports version 2 (and still accepts v1)
  2. `log --push`: N one-second rows reach the stub server's /api/ingest
     with the right station name and channel levels
  3. `sferics`: injected strokes come out with the right absolute time
     (GPS second + 20 ns clock), polarity, peak, snapshot, sub-sample
     arrival refinement, CSV row and upload
  4. upload outage: rows queue while the server is down and are delivered after
"""
import http.server
import json
import math
import os
import sys
import tempfile
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fake_vlf_board as fvb          # noqa: E402

# make `import serial` give our fake before vlf_host is imported
serial_mod = types.ModuleType("serial")
serial_mod.Serial = fvb.FakeSerial
sys.modules["serial"] = serial_mod

import vlf_host                        # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


# ------------------------------------------------------------------ stub server
received = []
accepting = [True]


class Handler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n)
        if not accepting[0]:
            self.send_response(503)
            self.end_headers()
            return
        received.append(json.loads(body))
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok")

    def log_message(self, *a):
        pass


srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
URL = f"http://127.0.0.1:{port}"

tmp = tempfile.mkdtemp()
os.chdir(tmp)

# ------------------------------------------------------------------ 1. ID / version
print("[1] board identification")
fvb.FakeSerial.board = fvb.FakeBoard(version=1)
b1 = vlf_host.Board("FAKE")
check(b1.version == 1, "v1 bitstream accepted, version 1")
board = fvb.FakeBoard(version=2)
fvb.FakeSerial.board = board
b = vlf_host.Board("FAKE")
check(b.version == 2, "v2 bitstream accepted, version 2")
check(abs(b.fs - 390625.0) < 1e-6, f"sample rate from FS_CLKS = {b.fs}")

# ------------------------------------------------------------------ 2. log --push
print("[2] log --station --push (4 rows)")
args = types.SimpleNamespace(csv="vlf_test.csv", goes=False, no_pps=False, station="HOME",
                             push=URL, max_rows=4)
t_start = time.time()
vlf_host.cmd_log(b, args)
rows = [r for batch in received for r in batch["rows"]]
check(len(received) >= 1 and all(bt["station"] == "HOME" and bt["kind"] == "vlf" for bt in received),
      f"{len(received)} batch(es) uploaded as station HOME, kind vlf")
check(len(rows) == 4, f"{len(rows)} rows delivered (expected 4)")
check(all(abs(r["ch"]["NLK"] - 48.0) < 1.0 for r in rows), "NLK level about 48 dB in every row")
check(all(abs(r["t"] - round(r["t"])) < 1e-9 for r in rows), "row times are whole GPS seconds")
with open("vlf_test.csv") as f:
    lines = f.read().strip().splitlines()
check(len(lines) == 5 and lines[0].startswith("utc,unix,seq"), "CSV has a header and 4 data rows")

# ------------------------------------------------------------------ 3. sferics
print("[2b] GPS station bridge: windows stamped with GPS UTC, not the PC clock")
board.gps = dict(utc="2031-02-03T04:05:06Z", pps=42, valid=True, sats=9)   # a time far from 'now'
check(b.has_gps_bridge(), "the bridge answers @id")
gt = b.gps_time()
check(gt and gt["utc"] == "2031-02-03T04:05:06Z" and gt["pps"] == 42 and gt["valid"], "gps_time() parses @GPS")
received.clear()
args = types.SimpleNamespace(csv="vlf_gps.csv", goes=False, no_pps=False, station="HOME",
                             push=URL, max_rows=3, gps_bridge=True)
vlf_host.cmd_log(b, args)
time.sleep(0.2)
grows = [r for batch in received if batch["kind"] == "vlf" for r in batch["rows"]]
import datetime as _dt
base = _dt.datetime(2031, 2, 3, 4, 5, 6, tzinfo=_dt.timezone.utc).timestamp()
check(len(grows) == 3, f"{len(grows)} rows logged with the bridge")
check(all(abs(r["t"] - base) < 5 for r in grows), "rows carry the GPS time (2031), not the PC clock (now)")
board.gps = None
check(b.gps_time() is None, "gps_time() returns None when the bridge has no fix")

print("[3] sferics (3 injected strokes)")
received.clear()
board.regs[0x40] = 600
t_now = time.time()
strokes = [(t_now - 0.7, 1400, +1), (t_now - 0.3, 900, -1), (t_now - 0.1, 1200, +1)]
evs = [board.inject_sferic(t, pk, pol, thresh=600) for t, pk, pol in strokes]
check(board.read(0x43) == 3, "3 events waiting in the fake FIFO")
args = types.SimpleNamespace(thresh=600, holdoff=1953, csv="sf_test.csv", station="HOME", push=URL,
                             max_events=3)
vlf_host.cmd_sferics(b, args)
with open("sf_test.csv") as f:
    sf_rows = f.read().strip().splitlines()[1:]
check(len(sf_rows) == 3, f"{len(sf_rows)} sferic rows in the CSV")
ok = True
for (t_true, pk, pol), ev, line in zip(strokes, evs, sf_rows):
    c = line.split(",")
    t_logged = float(c[1])
    off = float(c[4])
    # logged time = GPS second + clk/50e6 + sub-sample refinement; the refinement is
    # taken out again here to compare the raw stamp with the injected time (20 ns)
    t_raw = t_logged - off / 390625.0
    if abs(t_raw - t_true) > 25e-9:
        ok = False
        print(f"    stamp {t_raw:.9f} vs injected {t_true:.9f}: off by {(t_raw - t_true) * 1e9:.0f} ns")
    if int(c[5]) != ev["peak"] or int(c[6]) != pol:
        ok = False
        print(f"    peak/polarity {c[5]}/{c[6]} vs {ev['peak']}/{pol}")
    snap = [int(s) for s in c[9].split(";")]
    if snap != ev["snap"]:
        ok = False
        print("    snapshot differs")
    if not (-1.0 <= off <= 0.0):
        ok = False
        print(f"    refinement offset {off} not within one sample before the trigger")
check(ok, "every stroke: raw stamp within 20 ns of the injected time, peak, polarity, snapshot, refinement")
time.sleep(0.2)
sf_batches = [bt for bt in received if bt["kind"] == "sferic"]
up = [r for bt in sf_batches for r in bt["rows"]]
check(len(up) == 3 and len(sf_batches) >= 1, "3 sferic rows uploaded as kind sferic")
check(board.read(0x43) == 0, "FIFO empty after the run")

# ------------------------------------------------------------------ 4. outage
print("[4] upload outage and recovery")
received.clear()
p = vlf_host.Pusher(URL, "HOME", "vlf", interval=0.2)
accepting[0] = False
for i in range(5):
    p.add({"t": 1000 + i, "ch": {}})
ok1 = p.flush_once()
check(not ok1 and len(p.q) == 5, "server down: batch kept in the queue")
accepting[0] = True
ok2 = p.flush_once()
check(ok2 and len(p.q) == 0 and p.sent == 5, "server back: all 5 queued rows delivered")
got = [r["t"] for r in received[0]["rows"]]
check(got == [1000, 1001, 1002, 1003, 1004], "rows delivered in their original order")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
srv.shutdown()
sys.exit(1 if fails else 0)
