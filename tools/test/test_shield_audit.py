#!/usr/bin/env python3
"""test_shield_audit.py -- the shield analysis on synthetic sweeps and skin-depth theory."""
import csv
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import shield_audit as sa  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


tmp = tempfile.mkdtemp()


def lockin_csv(path, pairs):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["f_Hz", "amp_mV", "lag_deg", "dc_mV", "samples"])
        for f, mv in pairs:
            w.writerow([f"{f:.6f}", f"{mv:.4f}", "0.0", "0.0", "100"])


def cc_csv(path, pairs):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["f_MHz", "rssi_dBm"])
        for f_hz, dbm in pairs:
            w.writerow([f"{f_hz/1e6:.4f}", f"{dbm:.1f}"])


print("[1] read formats")
lockin_csv(os.path.join(tmp, "o.csv"), [(1, 100), (60, 100), (1000, 100), (1e5, 100)])
r = sa.read_sweep(os.path.join(tmp, "o.csv"))
check(len(r) == 4 and r[0] == (1.0, 100.0), "lock-in CSV read as linear amplitude")
cc_csv(os.path.join(tmp, "cc.csv"), [(100e6, -40), (200e6, -46)])
rc = sa.read_sweep(os.path.join(tmp, "cc.csv"))
check(len(rc) == 2 and abs(rc[1][0] - 200e6) < 1 and abs(rc[1][1] - 10 ** (-46 / 20)) < 1e-9, "CC1101 dBm CSV read and MHz scaled")

print("[2] attenuation = 20log10(open/shield)")
# a shield that drops the signal 10x at every frequency -> 20 dB
lockin_csv(os.path.join(tmp, "s20.csv"), [(1, 10), (60, 10), (1000, 10), (1e5, 10)])
att = sa.attenuation(r, sa.read_sweep(os.path.join(tmp, "s20.csv")))
check(len(att) == 4 and all(abs(d - 20.0) < 1e-6 for _, d in att), "a 10x drop is 20 dB at every matched frequency")

print("[3] a realistic mesh: lots at RF, little at ELF")
# open ref across ELF..RF
oref = [(1, 100), (60, 100), (1000, 100), (1e4, 100), (1e5, 100)]
lockin_csv(os.path.join(tmp, "o2.csv"), oref)
# mesh: 3 dB at 60 Hz rising to 60 dB at 100 kHz
import numpy as np
mesh = [(f, 100 / 10 ** (att_db / 20)) for f, att_db in [(1, 2), (60, 3), (1000, 15), (1e4, 35), (1e5, 60)]]
lockin_csv(os.path.join(tmp, "mesh.csv"), mesh)
att2 = sa.attenuation(sa.read_sweep(os.path.join(tmp, "o2.csv")), sa.read_sweep(os.path.join(tmp, "mesh.csv")))
d = dict((round(f), v) for f, v in att2)
check(abs(d[60] - 3) < 0.1 and abs(d[100000] - 60) < 0.1, "the mesh reads 3 dB at 60 Hz and 60 dB at 100 kHz")
check(d[60] < d[100000] - 40, "far more attenuation at RF than at mains -- the hole the FOIA files warn about")

print("[4] frequency matching tolerance")
# shield sampled at slightly different frequencies (within 2 %) still matches
shifted = [(f * 1.015, 10.0) for f in (1, 60, 1000, 1e5)]
lockin_csv(os.path.join(tmp, "shift.csv"), shifted)
att3 = sa.attenuation(r, sa.read_sweep(os.path.join(tmp, "shift.csv")))
check(len(att3) == 4, "frequencies within 2 % are matched")
far = [(f * 1.5, 10.0) for f in (1, 60, 1000, 1e5)]
lockin_csv(os.path.join(tmp, "far.csv"), far)
check(len(sa.attenuation(r, sa.read_sweep(os.path.join(tmp, "far.csv")))) == 0, "frequencies 50 % off are not matched")

print("[5] skin depth and absorption")
# copper skin depth at 60 Hz ~ 8.5 mm, at 100 kHz ~ 0.21 mm
d60 = sa.skin_depth(60, 1.0, 5.96e7)
d100k = sa.skin_depth(1e5, 1.0, 5.96e7)
print(f"    copper skin depth: 60 Hz {d60*1000:.2f} mm, 100 kHz {d100k*1000:.3f} mm")
check(abs(d60 * 1000 - 8.4) < 0.5 and abs(d100k * 1000 - 0.206) < 0.02, "copper skin depth matches the textbook values")
ab60, _ = sa.theory_db(60, "copper", 0.0002)
ab100k, _ = sa.theory_db(1e5, "copper", 0.0002)
print(f"    0.2 mm copper absorption: 60 Hz {ab60:.3f} dB, 100 kHz {ab100k:.1f} dB")
check(ab60 < 0.3 and ab100k > 5, "0.2 mm copper absorbs ~0.2 dB at 60 Hz but >8 dB at 100 kHz (negligible ELF shielding)")
# mu-metal: high permeability -> much smaller skin depth at low f (that's why it's used for magnetic shielding)
check(sa.skin_depth(60, 20000, 1.6e6) < sa.skin_depth(60, 1.0, 5.96e7), "mu-metal's skin depth at 60 Hz is far smaller than copper's")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
