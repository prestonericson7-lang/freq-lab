#!/usr/bin/env python3
"""test_rf_survey_host.py -- rf_survey_host.py against a fake meter: survey CSV + map, spectrum collection."""
import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

SURVEY = [f"2026-10-02T10:00:{i:02d}Z,{-45 - i * 0.5:.2f},{-30 - i * 0.5:.2f},{0.01 * (1 + i):.4f},0.12,{37.7749 + i * 1e-5:.6f},{-122.4194 + i * 1e-5:.6f},1" for i in range(12)]
SPEC = ["# slow (0.06 - 125 Hz) envelope spectrum, 3 averages, 0.0610 Hz bins. f_Hz,dB_rel"] + \
       [f"{0.061 * k:.4f},{-30 + (30 if abs(0.061 * k - 9.766) < 0.04 else 0):.1f}" for k in range(1, 2048)] + \
       ["# strongest lines:", "#     9.766 Hz     0.0 dB  Wi-Fi beacon (102.4 ms)", "#    60.010 Hz   -12.0 dB  60 Hz mains / light"]


class FakeSerial:
    def __init__(self, port, baud, timeout=1.0):
        self.out = []

    def write(self, data):
        line = data.decode().strip()
        if line == "survey on":
            self.out = [(s + "\n").encode() for s in ["# time,mean_dbm,peak_dbm,uw_cm2,duty,lat,lon,fix"] + SURVEY]
        elif line.startswith("spec"):
            self.out = [(s + "\n").encode() for s in SPEC + ["", ""]]

    def readline(self):
        return self.out.pop(0) if self.out else b""

    def reset_input_buffer(self):
        self.out = []


serial_mod = types.ModuleType("serial"); serial_mod.Serial = FakeSerial; sys.modules["serial"] = serial_mod
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import rf_survey_host as rh  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


tmp = tempfile.mkdtemp(); os.chdir(tmp)
print("[1] survey")
out = rh.cmd_survey(types.SimpleNamespace(port="FAKE", csv="walk.csv", seconds=2))
with open("walk.csv") as f:
    lines = f.read().strip().splitlines()
check(len(lines) == 13 and lines[0].startswith("utc,board_time,mean_dbm"), "12 survey rows + header")
check(lines[1].split(",")[2] == "-45.0" and lines[1].split(",")[8] == "1", "fields in the right columns")
print("[2] map")
rh.cmd_map(types.SimpleNamespace(csv="walk.csv", no_show=True))
check(os.path.exists("walk_map.png") and os.path.getsize("walk_map.png") > 10000, "map PNG written from GPS rows")
print("[3] spectrum")
link = rh.Link("FAKE"); link.cmd("spec slow 1")
f, db, names = rh.collect_spectrum(link, "spectrum", timeout=5)
check(len(f) == 2047 and max(db) == 0.0, f"{len(f)} spectrum points collected")
check(len(names) == 2 and "Wi-Fi beacon" in names[0], "named lines captured")
print("[4] parse")
check(rh.parse_survey_line("# comment") is None and rh.parse_survey_line("x,1,2") is None, "comments and short lines ignored")
d = rh.parse_survey_line("up12s,-50.1,-40.2,0.0030,0.05")
check(d and d["fix"] == 0 and d["uw_cm2"] == 0.003, "line without GPS fields parsed with fix=0")
print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
