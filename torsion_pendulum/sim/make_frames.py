#!/usr/bin/env python3
"""
make_frames.py -- synthetic camera frames of the pendulum, for testing the tracker.

Pinhole camera 26 mm off the bottle axis looking straight down (as mounted on
the lid), 53 deg horizontal field of view, VGA 640x480, YUV422 (YUYV) like the
GC2145. Scene: the printed rotor (dumbbell, 76 x 28 mm) 20 mm above the paper,
the paper on the base's floor ZPAPER mm below the camera (205 by default; a
150-230 mm sleeve gives 160-240), lit from below through the 93 mm window
(brightest in the middle), two 8 mm reference squares at x = +/-30 mm, the unlit
2 mm rim of the floor beyond the window and the dark bottle wall outside that.

Environment: ZPAPER (mm), FOV (deg, horizontal), NOISE (counts), SS (supersampling).
Usage: make_frames.py [n_frames] [camera_roll_deg]
Writes frames.bin (N frames x 640*480*2 bytes) and truth.csv
(frame, rotor angle rad, camera roll rad, rotor dx mm, rotor dy mm).
"""
import math
import os
import sys

import numpy as np

W, H = 640, 480
FOV_H = math.radians(float(os.environ.get("FOV", "53")))
F = (W / 2) / math.tan(FOV_H / 2)
CAM_X = 26.0
Z_PAPER = float(os.environ.get("ZPAPER", "205"))
Z_ROTOR = Z_PAPER - 20.0                 # rotor hangs 20 mm above the paper
WINDOW_R = 46.5                          # backlit window
FLOOR_R = 48.5                           # the printed floor's unlit rim
REF_X = 30.0
ARM, DISC_R, BAR_HW, HUB_R = 24.0, 14.0, 4.0, 4.0
SS = int(os.environ.get("SS", "5"))      # supersampling per axis

rng = np.random.default_rng(2)


def rays(roll):
    """Unit-depth ray directions (dx, dy) in world axes for every subpixel."""
    u = (np.arange(W * SS) + 0.5) / SS - 0.5
    v = (np.arange(H * SS) + 0.5) / SS - 0.5
    uu, vv = np.meshgrid(u, v)
    xc = (uu - W / 2 + 0.5) / F               # camera frame, per unit depth
    yc = (vv - H / 2 + 0.5) / F
    c, s = math.cos(roll), math.sin(roll)     # camera mounted rotated by `roll` about its axis
    return xc * c - yc * s, xc * s + yc * c


def frame(theta, roll, dx=0.0, dy=0.0, noise=2.0):
    rx, ry = rays(roll)
    # rotor plane hit, rotor-centred coordinates
    px = CAM_X + rx * Z_ROTOR - dx
    py = 0.0 + ry * Z_ROTOR - dy
    c, s = math.cos(-theta), math.sin(-theta)
    qx, qy = px * c - py * s, px * s + py * c
    rotor = ((np.abs(qx) <= ARM) & (np.abs(qy) <= BAR_HW)) | \
            ((qx - ARM) ** 2 + qy ** 2 <= DISC_R ** 2) | ((qx + ARM) ** 2 + qy ** 2 <= DISC_R ** 2) | \
            (qx ** 2 + qy ** 2 <= HUB_R ** 2)
    # paper plane
    wx = CAM_X + rx * Z_PAPER
    wy = 0.0 + ry * Z_PAPER
    r = np.hypot(wx, wy)
    lum = 70.0 + 160.0 * np.exp(-(r / 50.0) ** 2)            # backlit paper: one LED 17 mm under it, brightest in the middle
    lum = np.where(r > WINDOW_R, 70.0, lum)                   # the floor's unlit rim: paper lit only by scatter
    lum = np.where(r > FLOOR_R, 40.0, lum)                    # beyond the floor: the bottle wall and the lid, dark
    cr, sr = math.cos(math.radians(12.0)), math.sin(math.radians(12.0))   # squares printed slightly turned
    for mx in (-REF_X, REF_X):
        ux, uy = (wx - mx) * cr + wy * sr, -(wx - mx) * sr + wy * cr
        sq = (np.abs(ux) <= 4.0) & (np.abs(uy) <= 4.0)
        lum = np.where(sq, 28.0, lum)
    lum = np.where(rotor, 22.0, lum)
    img = lum.reshape(H, SS, W, SS).mean(axis=(1, 3))
    img = img + rng.normal(0, noise, img.shape)
    return np.clip(np.round(img), 0, 255).astype(np.uint8)


def yuyv(y):
    out = np.full((H, W * 2), 128, np.uint8)
    out[:, 0::2] = y
    return out.tobytes()


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    roll0 = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0
    T = 8.0                                   # pendulum period, s; 10 frames/s
    rows = []
    with open("frames.bin", "wb") as fb:
        for k in range(n):
            t = k / 10.0
            theta = math.pi / 2 + 0.004 * math.sin(2 * math.pi * t / T)    # rotor across the arrow, 4 mrad swing
            if k >= n // 2:
                theta += 0.010                                            # 10 mrad step halfway
            roll = roll0
            if k >= 3 * n // 4:
                roll += 0.002                                             # camera itself turns 2 mrad
            ddx = 0.3 * math.sin(2 * math.pi * t / 1.3)                   # 0.3 mm swing of the whole rotor
            fb.write(yuyv(frame(theta, roll, ddx, 0.0, noise=float(os.environ.get("NOISE", "2")))))
            rows.append((k, theta, roll, ddx, 0.0))
    with open("truth.csv", "w") as f:
        f.write("frame,theta,roll,dx,dy\n")
        for r in rows:
            f.write(",".join(f"{v:.9f}" if isinstance(v, float) else str(v) for v in r) + "\n")
    print(f"wrote {n} frames")
