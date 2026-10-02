#!/usr/bin/env python3
"""make_frames_cal.py -- harder tracker tests than make_frames.py.

  cal  : the `calibrate` step. 10 s of a small swing, then the knob turns 10 deg
         (174.5 mrad): the rotor swings about its new rest angle (T 8 s, Q 15).
  gap N: the rotor hung N mm above the paper instead of 20 (small swing + 10 mrad step).

Writes frames_<name>.bin and truth_<name>.csv (same layout as make_frames.py).
"""
import math
import sys
from multiprocessing import Pool

import numpy as np

import make_frames as mf

mf.SS = 3                                    # faster; this test is about staying locked on, not 0.1 mrad
T, Q = 8.0, 15.0
W0 = 2 * math.pi / T
TAU = 2 * Q / W0


def theta_cal(k):
    t = k / 10.0
    th = math.pi / 2 + 0.004 * math.sin(2 * math.pi * t / T)
    if t >= 10.0:
        tp = t - 10.0
        th = math.pi / 2 + 0.1745 - 0.1745 * math.exp(-tp / TAU) * math.cos(W0 * tp)
    return th


def theta_gap(k, n):
    t = k / 10.0
    th = math.pi / 2 + 0.004 * math.sin(2 * math.pi * t / T)
    return th + (0.010 if k >= n // 2 else 0.0)


def render(job):
    k, th, z_rotor = job
    mf.Z_ROTOR = z_rotor
    mf.rng = np.random.default_rng(1000 + k)
    ddx = 0.3 * math.sin(2 * math.pi * (k / 10.0) / 1.3)
    return k, mf.yuyv(mf.frame(th, 0.0, ddx, 0.0)), th, ddx


def run(name, jobs):
    with Pool(2) as p:
        out = sorted(p.map(render, jobs, chunksize=4))
    with open(f"frames_{name}.bin", "wb") as fb:
        for _, b, _, _ in out:
            fb.write(b)
    with open(f"truth_{name}.csv", "w") as f:
        f.write("frame,theta,roll,dx,dy\n")
        for k, _, th, ddx in out:
            f.write(f"{k},{th:.9f},0.000000000,{ddx:.9f},0.000000000\n")
    print(f"wrote {len(out)} frames for {name}")


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "cal":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 550
        run("cal", [(k, theta_cal(k), mf.Z_PAPER - 20.0) for k in range(n)])
    else:
        gap = float(sys.argv[2])
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 60
        run(f"gap{int(gap)}", [(k, theta_gap(k, n), mf.Z_PAPER - gap) for k in range(n)])
