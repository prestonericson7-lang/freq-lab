#!/usr/bin/env python3
"""
test_cavity_readout.py -- the cavity-readout DSP on synthetic captures.

Builds a CW carrier at a known offset, modulated in amplitude and phase by a
test tone (as a vibrating cavity diaphragm would), writes it as 8-bit IQ like
rtl_sdr, and checks: carrier finding, AM/PM demod recovering the tone, the SNR
measure, the RSSI-series path, and the power-step threshold logic.
"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import cavity_readout_host as c  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


def make_capture(path, fs, seconds, carrier_off, tone, am_depth, pm_depth, noise):
    n = int(fs * seconds)
    t = np.arange(n) / fs
    m = np.sin(2 * np.pi * tone * t)
    env = 1.0 + am_depth * m
    ph = 2 * np.pi * carrier_off * t + pm_depth * m
    z = env * np.exp(1j * ph)
    z += noise * (np.random.randn(n) + 1j * np.random.randn(n))
    z = z / np.max(np.abs(z)) * 0.9
    i = np.clip(np.round(z.real * 127.5 + 127.5), 0, 255).astype(np.uint8)
    q = np.clip(np.round(z.imag * 127.5 + 127.5), 0, 255).astype(np.uint8)
    inter = np.empty(2 * n, dtype=np.uint8)
    inter[0::2] = i; inter[1::2] = q
    inter.tofile(path)


np.random.seed(3)
tmp = tempfile.mkdtemp()
fs = 1_024_000.0

print("[1] carrier finding")
p = os.path.join(tmp, "c.u8")
make_capture(p, fs, 0.5, 12345.0, 1000.0, 0.10, 0.0, 0.01)
iq = c.load_u8_iq(p)
fc = c.find_carrier(iq, fs)
check(abs(fc - 12345.0) < 50, f"carrier found at {fc:.0f} Hz (true 12345)")

print("[2] AM demod recovers the test tone")
am, pm, fsa, fcx = c.demod_iq(iq, fs, fs_audio=8000.0)
snr_am = c.tone_snr(am, fsa, 1000.0)
print(f"    audio fs {fsa:.0f}, AM tone SNR {snr_am:.1f} dB")
check(snr_am > 20, "10 % AM of a 1 kHz tone recovers at > 20 dB SNR")
check(c.tone_snr(am, fsa, 1700.0) < snr_am - 15, "no spurious tone at an unrelated frequency")

print("[3] PM demod recovers a phase-modulated tone")
p2 = os.path.join(tmp, "pm.u8")
make_capture(p2, fs, 0.5, -8000.0, 1500.0, 0.0, 0.3, 0.01)
am2, pm2, fsa2, _ = c.demod_iq(c.load_u8_iq(p2), fs, fs_audio=8000.0)
snr_pm = c.tone_snr(pm2, fsa2, 1500.0)
print(f"    PM tone SNR {snr_pm:.1f} dB (AM path {c.tone_snr(am2, fsa2, 1500.0):.1f})")
check(snr_pm > 20, "0.3 rad PM of a 1.5 kHz tone recovers on the PM path at > 20 dB")
check(snr_pm > c.tone_snr(am2, fsa2, 1500.0) + 10, "the PM path beats the AM path for a phase-modulated cavity")

print("[4] SNR falls with modulation depth")
depths, snrs = [0.10, 0.03, 0.01, 0.003], []
for dpt in depths:
    pp = os.path.join(tmp, f"d{dpt}.u8")
    make_capture(pp, fs, 0.5, 5000.0, 1000.0, dpt, 0.0, 0.02)
    a2, _, fa, _ = c.demod_iq(c.load_u8_iq(pp), fs)
    snrs.append(c.tone_snr(a2, fa, 1000.0))
print("    depth/SNR: " + ", ".join(f"{d:.3f}:{s:.0f}dB" for d, s in zip(depths, snrs)))
check(all(snrs[i] > snrs[i + 1] - 1 for i in range(len(snrs) - 1)), "SNR decreases monotonically as the cavity modulation gets weaker")
check(snrs[0] - snrs[-1] > 15, "a 30x weaker modulation is clearly worse (the #191 trend)")

print("[5] RSSI series path (low-rate test tone)")
tms = np.arange(0, 8000, 1.25)                      # 800 Hz for 8 s
rssi = -50 + 2.0 * np.sin(2 * np.pi * 50.0 * tms / 1000.0) + np.random.randn(len(tms)) * 0.1
x, fsr = c.rssi_to_series(tms, rssi)
check(abs(fsr - 800) < 20 and c.tone_snr(x, fsr, 50.0) > 20, f"a 50 Hz tone in 800/s RSSI recovers ({c.tone_snr(x, fsr, 50.0):.0f} dB)")

print("[6] power-step threshold")
check(c.lowest_power([(0, 30), (-3, 25), (-6, 18), (-9, 6)], 10.0) == -6, "lowest power above 10 dB is -6 (monotone)")
check(c.lowest_power([(0, 30), (-3, 8), (-6, 25)], 10.0) == 0, "a dip below threshold stops the descent (no lucky -6)")
check(c.lowest_power([(0, 5)], 10.0) is None, "none clearing the threshold -> None")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
