#!/usr/bin/env python3
"""fake_cam.py -- stands in for the ESP32-S3 CAM on a pseudo-terminal (tests torsion_host.py).

Simulated pendulum (T 8 s, Q 15) at 10 frames/s. The equilibrium steps +174.5 mrad at
t = 25 s and back at t = 70 s (what the calibrate prompts ask for). Answers snap/info.
"""
import base64
import math
import os
import random
import sys
import threading
import time
import tty

import numpy as np

master, slave = os.openpty()
tty.setraw(slave)
print(os.ttyname(slave), flush=True)
lock = threading.Lock()
cmds = []


def reader():
    buf = b""
    while True:
        buf += os.read(master, 1024)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            with lock:
                cmds.append(line.decode().strip())


threading.Thread(target=reader, daemon=True).start()
T, Q = 8.0, 15.0
w0 = 2 * math.pi / T
th, om = 0.0, 0.0
t0 = time.time()
frame = 0
rng = random.Random(3)
while True:
    time.sleep(0.1)
    t = time.time() - t0
    eq = 174.5 if 25 <= t < 70 else 0.0
    for _ in range(10):
        h = 0.01
        acc = -w0 ** 2 * (th - eq) - (w0 / Q) * om + rng.gauss(0, 0.08) / math.sqrt(h)
        om += acc * h
        th += om * h
    frame += 1
    meas = th + rng.gauss(0, 0.2)
    out = [f"A,{int(t * 1000)},{frame},{int(meas * 1000)},{int((meas + 3.1) * 1000)},3100,3,3200,2400,9000,90,1"]
    with lock:
        cs, cmds[:] = list(cmds), []
    for c in cs:
        if c in ("snap", "snapfull"):
            img = np.full((120, 160), 200, np.uint8)
            img[50:70, 40:120] = 25                      # a fake rotor across the middle
            img[20:28, 76:84] = 30; img[92:100, 76:84] = 30
            out.append("SNAP 160 120 4 320.0 240.0 82.0 60.0 0 2 320.0 96.0 320.0 384.0 20.0 320.0 240.0 160.0")
            b64 = base64.b64encode(img.tobytes()).decode()
            out += [b64[i:i + 76] for i in range(0, len(b64), 76)]
            out.append("SNAPEND")
        elif c == "info":
            out.append("I,sensor_pid=0x2145,layout=ESP32S3_EYE,frame=640x480,yoff=0,fps=10.0,psram=8388608,acq=1,refs=2")
    os.write(master, ("\n".join(out) + "\n").encode())
