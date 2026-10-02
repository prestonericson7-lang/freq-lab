#!/usr/bin/env python3
"""
vlf_model.py -- reference models for the vlf_sid FPGA receiver.

Two models of the same signal chain:

  * bit-exact integer model  -- mirrors the HDL arithmetic step for step
    (quarter-wave sine ROM, 12-bit phase, 12x16 mixer, 3-stage CIC with
    57-bit wraparound registers, rounding to 24 bits, I^2+Q^2). The
    simulation output must match it exactly.

  * float model              -- ideal LO, ideal CIC, no rounding. The
    integer model must agree with it to a small fraction of a dB, which
    shows the fixed-point design has no overflow or scaling mistake.

Also: the sine ROM table (used to generate hdl/sine_qrom.v), the default
station table and tuning words.
"""
import math

FS_NOM = 390_625            # XADC: 50 MHz / 4 = 12.5 MHz ADCCLK, 32 ADCCLK per conversion
LOG2R = 10                  # CIC decimation 1024 -> 381.47 Hz output rate
R = 1 << LOG2R
N_STAGES = 3
W = 27 + N_STAGES * LOG2R   # CIC register width: |x*lo| < 2^26, gain R^3
SH = W - 24                 # output slice [W-1:SH] -> 24-bit I/Q
K_AMP = 32767.0 / 16.0      # 24-bit I/Q magnitude per ADC count of tone amplitude

# Default channels (index -> (call sign, kHz, site)).  Frequencies from the
# Stanford SOLAR Center SID list and the Wikipedia VLF transmitter list.
STATIONS = [
    ("NLK", 24.80, "Jim Creek, WA"),
    ("NPM", 21.40, "Lualualei, HI"),
    ("NAA", 24.00, "Cutler, ME"),
    ("NML", 25.20, "LaMoure, ND"),
    ("NAU", 40.75, "Aguada, PR"),
    ("NWC", 19.80, "Exmouth, Australia"),
    ("JJI", 22.20, "Ebino, Japan"),
    ("REF", 30.00, "no station: noise/sferic reference"),
]


def ftw_for(f_hz, fs=FS_NOM):
    """32-bit NCO tuning word: f = ftw * fs / 2^32."""
    return int(round(f_hz * (1 << 32) / fs)) & 0xFFFFFFFF


def qrom_table():
    """Quarter-wave table, half-LSB phase offset so the quadrant symmetry is exact.
    T[i] = round(32767 * sin(2*pi*(i + 0.5) / 4096)), i = 0..1023 (15-bit unsigned)."""
    return [int(round(32767.0 * math.sin(2.0 * math.pi * (i + 0.5) / 4096.0))) for i in range(1024)]


_T = qrom_table()


def lo_sin(idx12):
    """Signed 16-bit LO value for a 12-bit phase index, exactly as the HDL builds it."""
    q = (idx12 >> 10) & 3
    a = idx12 & 1023
    m = _T[a ^ 1023] if (q & 1) else _T[a]
    return -m if (q & 2) else m


def _wrap(v, bits=W):
    """Two's-complement wrap to `bits` bits, returned as a signed Python int."""
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def channel_int(codes, ftw):
    """Bit-exact model of one vlf_channel. codes: raw 12-bit XADC codes from the
    first sample after reset. Returns the list of (I, Q) 24-bit decimated outputs
    and the sample index n at which each decimation happened."""
    rnd = 1 << (SH - 1)
    mask32 = 0xFFFFFFFF
    phase = 0
    ii1 = ii2 = ii3 = qi1 = qi2 = qi3 = 0
    iz0 = iz1 = iz2 = qz0 = qz1 = qz2 = 0
    out = []
    for n, code in enumerate(codes):
        xs = code - 2048
        idx = phase >> 20
        lo_s = lo_sin(idx)
        lo_c = lo_sin((idx + 1024) & 4095)
        phase = (phase + ftw) & mask32
        p_i = xs * lo_c
        p_q = xs * lo_s
        if (n & (R - 1)) == R - 1:              # decimation: capture i3 *before* this sample's update
            ic0, qc0 = ii3, qi3
            iy1 = _wrap(ic0 - iz0); iz0 = ic0
            iy2 = _wrap(iy1 - iz1); iz1 = iy1
            iy3 = _wrap(iy2 - iz2); iz2 = iy2
            qy1 = _wrap(qc0 - qz0); qz0 = qc0
            qy2 = _wrap(qy1 - qz1); qz1 = qy1
            qy3 = _wrap(qy2 - qz2); qz2 = qy2
            i24 = _wrap(iy3 + rnd) >> SH        # arithmetic shift == slice [W-1:SH]
            q24 = _wrap(qy3 + rnd) >> SH
            out.append((n, i24, q24))
        # pipelined integrators: each stage adds the previous stage's OLD value
        ii3 = _wrap(ii3 + ii2); ii2 = _wrap(ii2 + ii1); ii1 = _wrap(ii1 + p_i)
        qi3 = _wrap(qi3 + qi2); qi2 = _wrap(qi2 + qi1); qi1 = _wrap(qi1 + p_q)
    return out


def channel_float(codes, f_hz, fs=FS_NOM, ftw=None):
    """Ideal (floating point) version of the same chain, same timing. If ftw is
    given the LO frequency is the exact NCO frequency ftw*fs/2^32 and the LO
    phase matches the NCO (minus the half-LSB ROM offset, which only rotates I/Q)."""
    import numpy as np
    x = np.asarray(codes, dtype=np.float64) - 2048.0
    n = np.arange(len(x), dtype=np.float64)
    if ftw is not None:
        ph = 2.0 * np.pi * ((np.arange(len(x), dtype=np.uint64) * np.uint64(ftw)) & np.uint64(0xFFFFFFFF)).astype(np.float64) / 2.0 ** 32
    else:
        ph = 2.0 * np.pi * f_hz / fs * n
    scale = 32767.0
    p_i = x * scale * np.cos(ph)
    p_q = x * scale * np.sin(ph)
    # integrator chain with the same one-sample pipeline delays as the HDL
    def integ(v):
        a1 = np.cumsum(v)                                 # value after sample n
        a2 = np.concatenate(([0.0], np.cumsum(a1)[:-1]))  # sum of a1[m-1]
        a3 = np.concatenate(([0.0], np.cumsum(a2)[:-1]))
        return a3
    i3 = integ(p_i)
    q3 = integ(p_q)
    dec_n = np.arange(R - 1, len(x), R)
    # captured value = a3 *before* sample n's update = a3[n-1]
    ic0 = i3[dec_n - 1]
    qc0 = q3[dec_n - 1]
    def comb(c):
        y = c.copy()
        for _ in range(N_STAGES):
            y = y - np.concatenate(([0.0], y[:-1]))
        return y
    iy = comb(ic0) / 2.0 ** SH
    qy = comb(qc0) / 2.0 ** SH
    return dec_n, iy, qy


def amp_counts(pow_sum, ndec):
    """Window register values -> tone-equivalent amplitude in ADC counts."""
    if ndec <= 0 or pow_sum <= 0:
        return 0.0
    return math.sqrt(pow_sum / ndec) / K_AMP


if __name__ == "__main__":
    print(f"fs={FS_NOM}  R={R}  out rate={FS_NOM / R:.3f} Hz  W={W}  slice=[{W - 1}:{SH}]")
    for k, (call, khz, site) in enumerate(STATIONS):
        f = khz * 1000.0
        t = ftw_for(f)
        print(f"ch{k}  {call:4s} {khz:6.2f} kHz  FTW=0x{t:08X} ({t})  actual {t * FS_NOM / 2 ** 32:.4f} Hz  {site}")
