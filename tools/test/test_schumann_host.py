#!/usr/bin/env python3
"""
test_schumann_host.py -- schumann_host.py against a fake monitor and a stub server.

The fake board answers `json on` / `start` by emitting three JSON report lines
of the monitor's exact format (plus a comment line). Checks: CSV rows, the
summary parse, the upload as kind "schumann", and `plot` on the CSV.
"""
import http.server
import json
import os
import sys
import tempfile
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

REPORTS = [
    '{"t":"2026-10-02T03:00:00Z","nseg":6,"modes":[{"f":7.812,"H_pT":1.0421,"Hx":4.1,"E_uVm":0.2110,"Ex":3.0},'
    '{"f":14.221,"H_pT":0.6120,"Hx":3.2,"E_uVm":0.1500,"Ex":2.5},{"f":20.733,"H_pT":0.4310,"Hx":2.4,"E_uVm":0.1000,"Ex":2.0},'
    '{"f":27.125,"H_pT":0.3100,"Hx":1.9,"E_uVm":0.0700,"Ex":1.6},{"f":33.600,"H_pT":0.2500,"Hx":1.5,"E_uVm":0.0500,"Ex":1.3}],'
    '"Z1":371.2,"coh1":0.812,"Z2":365.0,"coh2":0.700,"floor30_pT":0.0210,"drops":0}',
    '# a comment from the board',
    '{"t":"2026-10-02T03:01:00Z","nseg":6,"modes":[{"f":7.830,"H_pT":1.1000,"Hx":4.3,"E_uVm":0.2200,"Ex":3.1},'
    '{"f":14.250,"H_pT":0.6300,"Hx":3.3,"E_uVm":0.1520,"Ex":2.6},{"f":20.800,"H_pT":0.4400,"Hx":2.5,"E_uVm":0.1010,"Ex":2.0},'
    '{"f":27.300,"H_pT":0.3150,"Hx":1.9,"E_uVm":0.0710,"Ex":1.6},{"f":33.800,"H_pT":0.2550,"Hx":1.5,"E_uVm":0.0510,"Ex":1.3}],'
    '"Z1":374.0,"coh1":0.830,"Z2":368.1,"coh2":0.710,"floor30_pT":0.0208,"drops":0}',
    '{"t":"2026-10-02T03:02:00Z","nseg":6,"modes":[{"f":7.845,"H_pT":1.0800,"Hx":4.2,"E_uVm":0.2150,"Ex":3.0},'
    '{"f":14.240,"H_pT":0.6200,"Hx":3.2,"E_uVm":0.1510,"Ex":2.5},{"f":20.790,"H_pT":0.4350,"Hx":2.4,"E_uVm":0.1005,"Ex":2.0},'
    '{"f":27.280,"H_pT":0.3120,"Hx":1.9,"E_uVm":0.0705,"Ex":1.6},{"f":33.750,"H_pT":0.2520,"Hx":1.5,"E_uVm":0.0505,"Ex":1.3}],'
    '"Z1":372.5,"coh1":0.820,"Z2":366.4,"coh2":0.705,"floor30_pT":0.0209,"drops":0}',
]


class FakeSerial:
    def __init__(self, port, baud, timeout=1.0):
        self.out = []
        self.started = False

    def write(self, data):
        line = data.decode().strip()
        if line.startswith("start"):
            self.out = [r.encode() + b"\n" for r in REPORTS]
        elif line == "live":
            self.out = [f"{0.25 * k:.4f},{1.0 / (1 + k):.5f},{0.2 / (1 + k):.5f}\n".encode() for k in range(1, 760)]

    def readline(self):
        return self.out.pop(0) if self.out else b""

    def reset_input_buffer(self):
        self.out = []


serial_mod = types.ModuleType("serial")
serial_mod.Serial = FakeSerial
sys.modules["serial"] = serial_mod
import schumann_host  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


received = []


class Handler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        received.append(json.loads(self.rfile.read(n)))
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok")

    def log_message(self, *a):
        pass


srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
threading.Thread(target=srv.serve_forever, daemon=True).start()
URL = f"http://127.0.0.1:{srv.server_address[1]}"
tmp = tempfile.mkdtemp()
os.chdir(tmp)

print("[1] parse")
r = schumann_host.parse_report(REPORTS[0])
check(r and r["m1_Hz"] == 7.812 and r["m5_H_pT"] == 0.25 and r["Z1"] == 371.2, "report line parsed into flat columns")
check(schumann_host.parse_report(REPORTS[1]) is None and schumann_host.parse_report("{bad") is None, "comments and bad JSON ignored")

print("[2] log --push")
a = types.SimpleNamespace(port="FAKE", report=60, csv="s.csv", station="HOME", push=URL, max_reports=3)
schumann_host.cmd_log(a)
with open("s.csv") as f:
    lines = f.read().strip().splitlines()
check(len(lines) == 4 and lines[0].startswith("utc,unix,t_board,nseg,m1_Hz"), "CSV: header + 3 rows")
rows = [row for b in received for row in b["rows"]]
check(len(rows) == 3 and all(b["kind"] == "schumann" and b["station"] == "HOME" for b in received), "3 reports uploaded as kind schumann")
check(abs(rows[1]["m1_H_pT"] - 1.1) < 1e-9 and rows[2]["Z2"] == 366.4, "uploaded rows carry the mode values")

print("[3] plot")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
schumann_host.cmd_plot(types.SimpleNamespace(csv="s.csv", no_show=True))
check(os.path.exists("s.png") and os.path.getsize("s.png") > 10000, "history plot written")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
srv.shutdown()
sys.exit(1 if fails else 0)
