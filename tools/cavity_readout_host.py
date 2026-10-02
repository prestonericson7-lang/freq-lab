#!/usr/bin/env python3
"""
cavity_readout_host.py -- passive-cavity resonator demonstration (list items
#190, #191: the 1945 Theremin "Great Seal" passive cavity, a documented piece
of Cold War radio history; Crypto Museum, declassified CIA files).

This is an educational physics demo, not a surveillance tool. A battery-less
tuned cavity (a quarter-wave antenna coupled to a thin conductive diaphragm) is
lit by a CW carrier. When a LOUDSPEAKER PLAYING A KNOWN TEST TONE vibrates the
diaphragm, the cavity's resonance shifts and modulates the carrier it
re-radiates. A receiver recovers that test tone. The measurement is how well a
known tone comes back, and (item #191) how that scales as the illuminator power
is stepped down -- i.e. the minimum beam power a modern coherent receiver needs,
versus the 1950s detectors that made the CIA abandon the technique.

Everything here uses a calibrated test tone from a signal generator or a phone
tone app at the diaphragm. There is no voice or room-audio path.

  tone     SX1262 pair (stm32/rf_resonance): one board `cw` (illuminator), one
           streaming RSSI with `cavity`. Reports the test tone's recovered
           amplitude and SNR vs the known drive. RSSI rate ~800/s over the link,
           so test tones up to ~300 Hz.
  demod    an RTL-SDR IQ capture (rtl_sdr -f <Hz> -s <fs> file.u8): carrier
           found, mixed to DC, decimated, AM + PM demodulated, test-tone SNR.
  power    #191: step the illuminator power down (SX1262 `power`, +22..-9 dBm,
           plus named external attenuators), capture at each step, report the
           test-tone SNR vs power, and the lowest power that still clears a set
           SNR. Needs an rtl_sdr capture per step (pass a prefix).

    python cavity_readout_host.py tone COM5 COM6 --freq 915.0 --power 0 --rate 800 --seconds 10 --tone 200
    python cavity_readout_host.py demod cap.u8 --fs 2048000 --tone 1000 --wav tone.wav
    python cavity_readout_host.py power --captures caps/step --powers 0,-3,-6,-9 --fs 2048000 --tone 1000

LEGAL / SAFETY: 902-928 MHz CW without a licence must stay under FCC 15.249
(~ -1 dBm EIRP, 50 mV/m at 3 m). Higher power needs an amateur licence on the
33 cm band, or a fully enclosed shielded test box. Item #191 is about going
DOWN in power, which fits the unlicensed limit. Do this with your own test tone
and your own cavity on your own bench.
"""
import argparse
import math
import sys
import time
import wave

import numpy as np


# ---------------------------------------------------------------------- IQ / tone maths
def load_u8_iq(path, max_samples=None):
    raw = np.fromfile(path, dtype=np.uint8, count=-1 if max_samples is None else 2 * max_samples)
    n = (len(raw) // 2) * 2
    iq = (raw[0:n:2].astype(np.float32) - 127.5) + 1j * (raw[1:n:2].astype(np.float32) - 127.5)
    return iq / 127.5


def find_carrier(iq, fs, nfft=65536):
    n = min(len(iq), nfft)
    seg = iq[:n] * np.hanning(n)
    sp = np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs))
    k = int(np.argmax(sp))
    if 0 < k < n - 1:
        a, b, c = (math.log(sp[k - 1] + 1e-30), math.log(sp[k] + 1e-30), math.log(sp[k + 1] + 1e-30))
        den = a - 2 * b + c
        d = 0.5 * (a - c) / den if den != 0 else 0.0
        return float(f[k] + d * fs / n)
    return float(f[k])


def lowpass_decimate(x, fs, m, cutoff):
    """windowed-sinc FIR low-pass, then decimate by integer m."""
    if m <= 1:
        return x, fs
    ntaps = 8 * m + 1
    t = np.arange(ntaps) - (ntaps - 1) / 2
    h = np.sinc(2 * cutoff / fs * t) * np.blackman(ntaps)
    h /= h.sum()
    y = np.convolve(x, h, mode="same")
    return y[::m], fs / m


def demod_iq(iq, fs, fs_audio=8000.0, audio_bw=3400.0):
    """carrier -> DC -> decimate to ~fs_audio -> AM (|z|) and PM (angle), drift removed."""
    fc = find_carrier(iq, fs)
    n = np.arange(len(iq))
    z = iq * np.exp(-2j * np.pi * fc / fs * n)
    m1 = max(1, int(fs // 64000))
    z1, fs1 = lowpass_decimate(z, fs, m1, 20000.0)
    m2 = max(1, int(round(fs1 / fs_audio)))
    z2, fs2 = lowpass_decimate(z1, fs1, m2, audio_bw)
    am = np.abs(z2)
    pm = np.unwrap(np.angle(z2))

    def hp(x):
        k = max(3, int(fs2 / 50))                 # remove < ~50 Hz drift (carrier wander, temperature)
        return x - np.convolve(x, np.ones(k) / k, mode="same")

    return hp(am - am.mean()), hp(pm - pm.mean()), fs2, fc


def tone_snr(x, fs, f_tone, bw=20.0):
    """test-tone power in +-bw/2 vs the median noise density in 100-3000 Hz, dB."""
    n = len(x)
    if n < 64:
        return float("nan")
    sp = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    f = np.fft.rfftfreq(n, 1.0 / fs)
    df = f[1] - f[0]
    sig = sp[(f >= f_tone - bw / 2) & (f <= f_tone + bw / 2)].sum()
    band = (f >= 100) & (f <= min(3000, fs / 2 - 10)) & (np.abs(f - f_tone) > 3 * bw)
    noise = (np.median(sp[band]) if band.any() else 1e-30) * max(1.0, bw / df)
    return 10 * math.log10(max(sig, 1e-30) / max(noise, 1e-30))


def write_wav(path, x, fs):
    peak = float(np.max(np.abs(x))) or 1.0
    y = np.int16(np.clip(x / peak * 0.9, -1, 1) * 32767)
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(int(round(fs)))
        w.writeframes(y.tobytes())


def rssi_to_series(t_ms, rssi_dbm, fs_out=None):
    t = np.asarray(t_ms, float) / 1000.0
    a = 10 ** (np.asarray(rssi_dbm, float) / 20.0)
    if fs_out is None:
        fs_out = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 800.0
    tu = np.arange(t[0], t[-1], 1.0 / fs_out)
    au = np.interp(tu, t, a)
    return au - au.mean(), fs_out


def lowest_power(results, min_snr_db=10.0):
    """results: [(power_dbm, snr_db)]. Lowest power whose SNR, and every higher
    power's, clears min_snr_db (a monotone threshold, not a single lucky step)."""
    low = None
    for p, s in sorted(results, key=lambda x: -x[0]):
        if s >= min_snr_db:
            low = p
        else:
            break
    return low


# ---------------------------------------------------------------------- serial
class Board:
    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is missing: pip install pyserial")
        self.s = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def cmd(self, c, wait=0.2):
        self.s.write((c + "\n").encode())
        time.sleep(wait)
        return self.s.read(self.s.in_waiting or 1).decode("ascii", "replace")


# ---------------------------------------------------------------------- commands
def cmd_tone(a):
    tx, rx = Board(a.tx), Board(a.rx)
    tx.cmd("mode tx"); tx.cmd(f"freq {a.freq}"); tx.cmd(f"power {a.power}"); tx.cmd("cw")
    rx.cmd("mode rx")
    rx.s.write(f"cavity {a.freq} {a.rate} {a.seconds}\n".encode())
    t, r = [], []
    t0 = time.time()
    while time.time() - t0 < a.seconds + 5:
        raw = rx.s.readline()
        if not raw:
            continue
        line = raw.decode("ascii", "replace").strip()
        if line.startswith("# done") or line.startswith("# aborted"):
            break
        p = line.split(",")
        if len(p) == 2:
            try:
                t.append(float(p[0])); r.append(float(p[1]))
            except ValueError:
                pass
    tx.cmd("off")
    if len(t) < 100:
        sys.exit(f"only {len(t)} RSSI samples received -- check the link and that both boards answered")
    x, fs = rssi_to_series(t, r)
    print(f"{len(t)} samples, {fs:.0f}/s, RSSI {min(r):.1f}..{max(r):.1f} dBm")
    if a.tone:
        if a.tone > fs / 2:
            print(f"note: {a.tone} Hz is above the RSSI Nyquist ({fs/2:.0f} Hz); use a lower test tone or the demod path")
        else:
            print(f"test tone {a.tone} Hz: SNR {tone_snr(x, fs, a.tone):.1f} dB in 20 Hz")
    if a.wav:
        write_wav(a.wav, x, fs); print("wrote", a.wav)


def cmd_demod(a):
    iq = load_u8_iq(a.capture)
    if len(iq) < 1000:
        sys.exit("capture too short")
    am, pm, fs, fc = demod_iq(iq, a.fs)
    print(f"{len(iq)/a.fs:.2f} s of IQ at {a.fs/1e6:.3f} MS/s; carrier {fc:+.0f} Hz from tuned")
    best, best_snr = None, -1e9
    for name, x in (("AM", am), ("PM", pm)):
        s = tone_snr(x, fs, a.tone) if a.tone else float("nan")
        print(f"{name}: rms {np.sqrt(np.mean(x**2)):.3e}" + (f", test tone {a.tone} Hz SNR {s:.1f} dB" if a.tone else ""))
        if a.tone and s > best_snr:
            best, best_snr = x, s
    if a.wav and best is not None:
        write_wav(a.wav, best, fs); print("wrote", a.wav)
    return (am, pm, fs, fc)


def cmd_power(a):
    powers = [float(x) for x in a.powers.split(",")]
    results = []
    print("step  power_dBm  AM_SNR  PM_SNR")
    for i, p in enumerate(powers):
        path = f"{a.captures}_{i}.u8"
        try:
            iq = load_u8_iq(path)
        except FileNotFoundError:
            print(f"  (missing {path}: capture it with  rtl_sdr -f {int(a.freq*1e6)} -s {int(a.fs)} {path}  at power {p} dBm)")
            continue
        am, pm, fs, _ = demod_iq(iq, a.fs)
        sam, spm = tone_snr(am, fs, a.tone), tone_snr(pm, fs, a.tone)
        results.append((p, max(sam, spm)))
        print(f"  {i:3d}  {p:8.1f}  {sam:6.1f}  {spm:6.1f}")
    if results:
        low = lowest_power(results, a.min_snr)
        print(f"\nlowest illuminator power clearing {a.min_snr:.0f} dB SNR: "
              + (f"{low:.1f} dBm" if low is not None else "none of the steps did"))
    else:
        print("no captures found; this command reads rtl_sdr files you capture at each power step")


def main():
    ap = argparse.ArgumentParser(description="passive-cavity resonator test-tone readout (educational)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("tone"); t.add_argument("tx"); t.add_argument("rx")
    t.add_argument("--freq", type=float, default=915.0); t.add_argument("--power", type=float, default=0.0)
    t.add_argument("--rate", type=float, default=800.0); t.add_argument("--seconds", type=float, default=10.0)
    t.add_argument("--tone", type=float, default=200.0); t.add_argument("--wav")
    d = sub.add_parser("demod"); d.add_argument("capture")
    d.add_argument("--fs", type=float, default=2048000.0); d.add_argument("--tone", type=float, default=1000.0); d.add_argument("--wav")
    pw = sub.add_parser("power"); pw.add_argument("--captures", required=True, help="prefix: <prefix>_0.u8, _1.u8, ...")
    pw.add_argument("--powers", required=True, help="comma list matching the captures, e.g. 0,-3,-6,-9")
    pw.add_argument("--freq", type=float, default=915.0); pw.add_argument("--fs", type=float, default=2048000.0)
    pw.add_argument("--tone", type=float, default=1000.0); pw.add_argument("--min-snr", type=float, default=10.0)
    a = ap.parse_args()
    {"tone": cmd_tone, "demod": cmd_demod, "power": cmd_power}[a.cmd](a)


if __name__ == "__main__":
    main()
