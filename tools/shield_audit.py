#!/usr/bin/env python3
"""
shield_audit.py -- measure what a "shielded" enclosure actually blocks, per band
(list items #134, #160). The point the FOIA files make repeatedly: a cage that
gives 120 dB against RF can give 3 dB against a 60 Hz magnetic field, so every
"shielded room" psi/bioeffect claim has a hole at ELF.

Low frequencies (0.01 Hz - 100 kHz): drive a transmit coil, measure a receive
coil with the FPGA lock-in (tools/lockin_host.py sweep --out curve.csv), once
with the shield between the coils and once without. Attenuation(f) =
20*log10(open / shielded). High frequencies (100 kHz - 1 GHz): the CC1101
scanner's sweep CSV or an SX1262 insertion-loss sweep, same idea.

    # with the lock-in:
    python tools/lockin_host.py COM7 sweep 1 100000 200 --log --out open.csv
    (put the shield between the coils)
    python tools/lockin_host.py COM7 sweep 1 100000 200 --log --out mesh.csv
    python shield_audit.py open.csv mesh.csv --name "copper mesh" --theory copper --thick-mm 0.2 --plot

    # several materials at once, against theory:
    python shield_audit.py open.csv mesh.csv alu.csv steel.csv mumetal.csv --plot

A shielding-effectiveness number alone is marketing; this prints it per
frequency with the material's own skin-depth prediction beside it, so the
customer sees WHERE the shield works and where it does not.
"""
import argparse
import csv
import math
import os
import sys

MU0 = 4e-7 * math.pi
# (relative permeability, conductivity S/m) at low frequency, typical
MATERIALS = {
    "copper":  (1.0, 5.96e7),
    "aluminum": (1.0, 3.5e7),
    "aluminium": (1.0, 3.5e7),
    "steel":   (100.0, 1.0e7),      # mild steel, mu_r falls at HF and with field; this is a low-field guide
    "mumetal": (20000.0, 1.6e6),    # annealed; mu_r is very field- and frequency-dependent
    "nickel":  (100.0, 1.4e7),
}


def read_sweep(path):
    """Accepts the lock-in CSV (f_Hz,amp_mV,...), the CC1101 CSV (f_MHz,rssi_dBm),
    or the VLF scan CSV (khz,level_db). Returns [(hz, linear_amplitude)]."""
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().lower().split(",")
        rdr = csv.reader(fh)
        # figure out the frequency column and whether the level is dB or linear
        if "f_hz" in head:
            fi, vi, is_db, fscale = head.index("f_hz"), head.index("amp_mv"), False, 1.0
        elif "f_mhz" in head:
            fi, vi, is_db, fscale = head.index("f_mhz"), head.index("rssi_dbm"), True, 1e6
        elif "khz" in head:
            fi, vi, is_db, fscale = head.index("khz"), head.index("level_db"), True, 1e3
        elif "freq_mhz" in head:
            fi, vi, is_db, fscale = head.index("freq_mhz"), head.index("rssi_dbm"), True, 1e6
        else:
            raise ValueError(f"{path}: unrecognised header {head}")
        for p in rdr:
            if not p or p[0].startswith("#"):
                continue
            try:
                f = float(p[fi]) * fscale
                v = float(p[vi])
            except (ValueError, IndexError):
                continue
            amp = 10 ** (v / 20.0) if is_db else v
            rows.append((f, amp))
    return rows


def attenuation(open_rows, shield_rows, tol=0.02):
    """match frequencies (within tol fractional) and return [(hz, dB)]; dB>0 = attenuated."""
    out = []
    sh = sorted(shield_rows)
    sf = [f for f, _ in sh]
    import bisect
    for f, a0 in sorted(open_rows):
        if a0 <= 0:
            continue
        i = bisect.bisect_left(sf, f)
        best = None
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(sf) and abs(sf[j] - f) <= tol * f:
                if best is None or abs(sf[j] - f) < abs(sf[best] - f):
                    best = j
        if best is None:
            continue
        a1 = sh[best][1]
        if a1 <= 0:
            continue
        out.append((f, 20 * math.log10(a0 / a1)))
    return out


def skin_depth(f, mu_r, sigma):
    return math.sqrt(1.0 / (math.pi * f * mu_r * MU0 * sigma)) if f > 0 else float("inf")


def theory_db(f, material, thick_m):
    """plane-wave shielding effectiveness: absorption (8.686 t/delta dB) + a simple
    reflection term for the magnetic near field of a coil. A guide, not a spec."""
    mu_r, sigma = MATERIALS[material]
    delta = skin_depth(f, mu_r, sigma)
    absorption = 8.686 * thick_m / delta
    # magnetic-field reflection loss is small for thin high-permeability sheets;
    # approximate the low-frequency magnetic SE of a thin shell: 20 log10(mu_r t / (3 r))
    # is geometry-dependent, so report absorption only (the part that is material, not geometry)
    return absorption, delta


def summarize(att, label):
    if not att:
        print(f"{label}: no overlapping frequencies")
        return
    bands = [(0.01, 1, "sub-Hz"), (1, 60, "ELF to mains"), (60, 1e3, "mains-1 kHz"),
             (1e3, 1e5, "1-100 kHz"), (1e5, 1e7, "0.1-10 MHz"), (1e7, 1e10, "10 MHz+")]
    print(f"\n{label}: attenuation by band (dB, median)")
    for lo, hi, name in bands:
        vals = [d for f, d in att if lo <= f < hi]
        if vals:
            vals.sort()
            print(f"  {name:14s} {len(vals):3d} pts   {vals[len(vals)//2]:6.1f} dB   "
                  f"(min {min(vals):.1f}, max {max(vals):.1f})")


def main():
    ap = argparse.ArgumentParser(description="shielding attenuation vs frequency, with skin-depth theory")
    ap.add_argument("open_csv", help="the no-shield reference sweep")
    ap.add_argument("shield_csv", nargs="+", help="one or more shielded sweeps")
    ap.add_argument("--name", action="append", help="label per shield file (repeatable)")
    ap.add_argument("--theory", help="material for the theory curve: " + ", ".join(sorted(MATERIALS)))
    ap.add_argument("--thick-mm", type=float, default=0.1, help="shield thickness for the theory curve")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out", help="write the per-frequency table to this CSV")
    a = ap.parse_args()
    open_rows = read_sweep(a.open_csv)
    results = []
    for i, sc in enumerate(a.shield_csv):
        label = (a.name[i] if a.name and i < len(a.name) else os.path.splitext(os.path.basename(sc))[0])
        att = attenuation(open_rows, read_sweep(sc))
        results.append((label, att))
        summarize(att, label)
    if a.theory and a.theory in MATERIALS and results:
        print(f"\ntheory ({a.theory}, {a.thick_mm} mm): absorption loss and skin depth")
        for f in (1, 60, 1e3, 1e4, 1e5, 1e6, 1e8):
            ab, dl = theory_db(f, a.theory, a.thick_mm / 1000.0)
            print(f"  {f:10.0f} Hz   absorption {ab:6.1f} dB   skin depth {dl*1000:8.3f} mm")
        print("  (absorption only; reflection and geometry add more at RF, little at ELF --")
        print("   which is exactly why a mesh that is 120 dB at 100 MHz is a few dB at 60 Hz)")
    if a.out:
        with open(a.out, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["label", "hz", "atten_db"])
            for label, att in results:
                for f, d in att:
                    w.writerow([label, f"{f:.4f}", f"{d:.2f}"])
        print(f"\nwrote {a.out}")
    if a.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            sys.exit("matplotlib missing: pip install matplotlib")
        fig, ax = plt.subplots(figsize=(11, 6))
        for label, att in results:
            if att:
                ax.semilogx([f for f, _ in att], [d for _, d in att], lw=0.9, marker=".", ms=3, label=label)
        if a.theory and a.theory in MATERIALS:
            fs = [10 ** (x / 4) for x in range(0, 33)]
            ax.semilogx(fs, [theory_db(f, a.theory, a.thick_mm / 1000.0)[0] for f in fs], "k--", lw=0.8,
                        label=f"{a.theory} {a.thick_mm} mm (absorption)")
        ax.set_xlabel("Hz"); ax.set_ylabel("attenuation, dB"); ax.grid(alpha=0.3, which="both"); ax.legend()
        ax.set_title("shielding effectiveness vs frequency")
        fig.tight_layout()
        png = (a.out and os.path.splitext(a.out)[0] + ".png") or "shield_audit.png"
        fig.savefig(png, dpi=120); print("saved", png)
        plt.show()


if __name__ == "__main__":
    main()
