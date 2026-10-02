#!/usr/bin/env python3
"""
lockin_host.py -- run the PZ7020-StarLite lock-in (fpga/lockin) from a PC.

Talks to the FPGA's UART (JM1 pin 14 = FPGA TX, pin 16 = FPGA RX, 115200 8N1)
through a 3.3 V USB-serial adapter, or through a Teensy 4.1 running
teensy/fpga_uart_bridge.

    pip install pyserial

Examples (Windows: COM7, Linux: /dev/ttyUSB0)
    python lockin_host.py COM7 id
    python lockin_host.py COM7 cal                       # loop-back delay calibration
    python lockin_host.py COM7 read 1000 --amp 0.2
    python lockin_host.py COM7 sweep 50 2000 200 --log --amp 0.2 --out curve.csv
    python lockin_host.py COM7 track 440 --amp 0.2 --step 0.05 --seconds 60
    python lockin_host.py COM7 off

Amplitudes are reported in millivolts at the ADC pin (0..1 V input range).
Lag is in degrees, positive = the response lags the drive. The raw lag includes
the RC filter on the drive pin and the ADC's conversion delay; `cal` measures
that with the drive looped straight back to the ADC and stores it, and later
runs subtract it.

STATUS: protocol and math checked against the HDL in simulation and against a
software model. Not yet run on hardware.
"""

import argparse
import json
import math
import os
import sys
import time

CLK_HZ = 50.0e6
TWO48 = float(1 << 48)
REF_AMP = 32700.0            # CORDIC reference amplitude
AMP_MAX = 58982              # register value for 90 % of full scale
ADC_VOLTS_PER_COUNT = 1.0 / 4096.0
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lockin_cal.json")

# register addresses (see fpga/lockin/hdl/cmd_regs.v)
R_ID, R_CTRL, R_FTW_LO, R_FTW_HI, R_AMP, R_CYCLES, R_DCSHIFT, R_START = range(8)
R_STATUS, R_N, R_I_LO, R_I_HI, R_Q_LO, R_Q_HI, R_SX_LO, R_SX_HI = range(8, 16)
R_ADC_RAW, R_DC_EST, R_TRK_STEP, R_TRK_COS, R_TRK_SIN, R_FNOW_LO, R_FNOW_HI = range(16, 23)

CTRL_DAC, CTRL_CONT, CTRL_TRACK, CTRL_INVERT, CTRL_SYNC = 1, 2, 4, 8, 16


def wrap180(d):
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


class LockIn:
    """Register-level access plus the measurement arithmetic."""

    def __init__(self, link):
        self.link = link                # object with write(bytes) and readline() -> bytes
        self.delay_s = 0.0              # drive-to-ADC delay from `cal`
        self.freq = 0.0

    # ---------------------------------------------------------------- transport
    def _cmd(self, text):
        self.link.write((text + "\n").encode("ascii"))
        reply = self.link.readline().decode("ascii", "replace").strip()
        if reply == "":
            raise IOError("no reply from the FPGA (check wiring, bitstream loaded, port)")
        if reply == "E":
            raise IOError("FPGA rejected command %r" % text)
        return reply

    def wr(self, addr, value):
        if self._cmd("W%02X%08X" % (addr, value & 0xFFFFFFFF)) != "K":
            raise IOError("write to register %02X not acknowledged" % addr)

    def rd(self, addr):
        return int(self._cmd("R%02X" % addr), 16)

    # ---------------------------------------------------------------- settings
    def ident(self):
        return self.rd(R_ID)

    def set_freq(self, f_hz):
        w = int(round(f_hz * TWO48 / CLK_HZ))
        w = max(1, min(w, (1 << 47) - 1))
        self.wr(R_FTW_LO, w & 0xFFFFFFFF)
        self.wr(R_FTW_HI, w >> 32)                 # writing HI loads the pair
        self.freq = w * CLK_HZ / TWO48
        return self.freq

    def set_amp(self, frac):
        frac = max(0.0, min(1.0, frac))
        self.wr(R_AMP, int(round(frac * AMP_MAX)))

    def set_ctrl(self, bits):
        self.wr(R_CTRL, bits)

    def freq_now(self):
        self.rd(R_STATUS)                          # freezes the shadow registers
        w = (self.rd(R_FNOW_HI) << 32) | self.rd(R_FNOW_LO)
        return w * CLK_HZ / TWO48

    # ---------------------------------------------------------------- measuring
    def _collect(self):
        n = self.rd(R_N)
        i = (self.rd(R_I_HI) << 32) | self.rd(R_I_LO)
        q = (self.rd(R_Q_HI) << 32) | self.rd(R_Q_LO)
        if i >= 1 << 63:
            i -= 1 << 64
        if q >= 1 << 63:
            q -= 1 << 64
        sx = (self.rd(R_SX_HI) << 32) | self.rd(R_SX_LO)
        w = (self.rd(R_FNOW_HI) << 32) | self.rd(R_FNOW_LO)
        f = w * CLK_HZ / TWO48
        if n == 0:
            raise IOError("window finished with zero samples: is the ADC running?")
        amp_counts = 2.0 * math.hypot(i, q) / (n * REF_AMP)
        lag = math.degrees(math.atan2(-q, i)) - 360.0 * f * self.delay_s
        return {
            "f": f,
            "mV": amp_counts * ADC_VOLTS_PER_COUNT * 1000.0,
            "lag": wrap180(lag),
            "n": n,
            "mean_mV": (sx / n) * ADC_VOLTS_PER_COUNT * 1000.0,
        }

    def measure(self, cycles):
        """One triggered window of `cycles` drive cycles."""
        cycles = max(1, min(int(cycles), (1 << 24) - 1))
        self.wr(R_CYCLES, cycles)
        st0 = self.rd(R_STATUS)
        self.wr(R_START, 1)
        deadline = time.time() + 2.0 * (cycles + 2) / max(self.freq, 1e-3) + 2.0
        while True:
            st = self.rd(R_STATUS)                 # also freezes the results
            if ((st >> 8) & 0xFFFF) != ((st0 >> 8) & 0xFFFF):
                break
            if time.time() > deadline:
                raise IOError("lock-in window did not finish (status %08X)" % st)
            time.sleep(0.002)
        if not (st & 2):
            raise IOError("the ADC is not producing samples")
        return self._collect()

    def latest(self):
        """Most recent finished window in continuous mode."""
        self.rd(R_STATUS)
        return self._collect()


# -------------------------------------------------------------------- analysis
def analyze_peak(rows):
    """rows: list of dicts with f, mV, lag. Returns peak frequency, Q, lag at peak."""
    if len(rows) < 3:
        return None
    im = max(range(len(rows)), key=lambda k: rows[k]["mV"])
    f0, a0 = rows[im]["f"], rows[im]["mV"]
    if 0 < im < len(rows) - 1:
        x1, x2, x3 = rows[im - 1]["f"], rows[im]["f"], rows[im + 1]["f"]
        y1, y2, y3 = rows[im - 1]["mV"], rows[im]["mV"], rows[im + 1]["mV"]
        d = (x1 - x2) * (x1 - x3) * (x2 - x3)
        if abs(d) > 1e-30:
            a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / d
            b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / d
            c = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / d
            if a < 0:
                fv = -b / (2 * a)
                if x1 < fv < x3:
                    f0, a0 = fv, c - b * b / (4 * a)
    half = a0 / math.sqrt(2.0)
    fl = fh = None
    for k in range(im, 0, -1):
        if rows[k - 1]["mV"] <= half <= rows[k]["mV"]:
            t = (half - rows[k - 1]["mV"]) / (rows[k]["mV"] - rows[k - 1]["mV"])
            fl = rows[k - 1]["f"] + t * (rows[k]["f"] - rows[k - 1]["f"])
            break
    for k in range(im, len(rows) - 1):
        if rows[k + 1]["mV"] <= half <= rows[k]["mV"]:
            t = (rows[k]["mV"] - half) / (rows[k]["mV"] - rows[k + 1]["mV"])
            fh = rows[k]["f"] + t * (rows[k + 1]["f"] - rows[k]["f"])
            break
    j = im if f0 >= rows[im]["f"] else im - 1
    lag = rows[im]["lag"]
    if 0 <= j < len(rows) - 1:
        la = rows[j]["lag"]
        lb = la + wrap180(rows[j + 1]["lag"] - la)
        t = (f0 - rows[j]["f"]) / (rows[j + 1]["f"] - rows[j]["f"])
        lag = wrap180(la + t * (lb - la))
    bw = (fh - fl) if (fl is not None and fh is not None) else None
    return {"f0": f0, "mV": a0, "lag": lag, "fl": fl, "fh": fh, "bw": bw,
            "Q": (f0 / bw) if bw else None}


def sweep_points(f0, f1, n, log):
    for k in range(n):
        t = k / (n - 1) if n > 1 else 0.0
        yield f0 * (f1 / f0) ** t if log else f0 + (f1 - f0) * t


# -------------------------------------------------------------------- commands
def load_cal(dev):
    try:
        with open(CAL_FILE) as fh:
            dev.delay_s = float(json.load(fh)["delay_s"])
    except (OSError, ValueError, KeyError):
        dev.delay_s = 0.0


def cmd_cal(dev, args):
    """Drive looped straight back to the ADC: fit lag = 360 * f * delay."""
    print("# cal: wire the filtered drive output to the ADC input through a divider")
    print("#      (keep the ADC pin under 1.0 V). Fitting lag against frequency...")
    dev.delay_s = 0.0
    dev.set_amp(args.amp)
    dev.set_ctrl(CTRL_DAC | CTRL_SYNC)
    freqs = [200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0]
    num = den = 0.0
    prev = None
    for f in freqs:
        dev.set_freq(f)
        time.sleep(0.05)
        m = dev.measure(max(20, int(f * 0.05)))
        lag = m["lag"]
        if prev is not None:
            lag = prev + wrap180(lag - prev)          # unwrap
        prev = lag
        print("#   %8.1f Hz  %8.2f mV  lag %7.2f deg" % (m["f"], m["mV"], lag))
        num += lag * m["f"]
        den += m["f"] * m["f"]
    delay = num / den / 360.0
    with open(CAL_FILE, "w") as fh:
        json.dump({"delay_s": delay}, fh)
    print("# equivalent delay %.3f microseconds, saved to %s" % (delay * 1e6, CAL_FILE))
    print("# (an RC filter is not a pure delay; above about a tenth of its corner frequency")
    print("#  expect a few degrees of residual lag)")


def cmd_read(dev, args):
    dev.set_amp(args.amp)
    dev.set_ctrl(CTRL_DAC | CTRL_SYNC)
    f = dev.set_freq(args.freq)
    time.sleep(args.settle)
    m = dev.measure(max(1, int(round(args.integrate * f))))
    print("f_Hz,amp_mV,lag_deg,dc_mV,samples")
    print("%.6f,%.4f,%.2f,%.2f,%d" % (m["f"], m["mV"], m["lag"], m["mean_mV"], m["n"]))


def cmd_sweep(dev, args):
    dev.set_amp(args.amp)
    dev.set_ctrl(CTRL_DAC | CTRL_SYNC)
    out = open(args.out, "w") if args.out else None
    rows = []
    header = "f_Hz,amp_mV,lag_deg,dc_mV,samples"
    print(header)
    if out:
        out.write(header + "\n")
    try:
        for f in sweep_points(args.f0, args.f1, args.points, args.log):
            fa = dev.set_freq(f)
            time.sleep(max(args.settle, 5.0 / fa))
            m = dev.measure(max(3, int(round(args.integrate * fa))))
            rows.append(m)
            line = "%.6f,%.4f,%.2f,%.2f,%d" % (m["f"], m["mV"], m["lag"], m["mean_mV"], m["n"])
            print(line)
            if out:
                out.write(line + "\n")
    except KeyboardInterrupt:
        print("# stopped")
    finally:
        if out:
            out.close()
    pk = analyze_peak(rows)
    if pk:
        print("# peak: f0 = %.4f Hz, %.3f mV, lag at peak %.1f deg" % (pk["f0"], pk["mV"], pk["lag"]))
        if pk["Q"]:
            print("# -3 dB band %.4f .. %.4f Hz, width %.4f Hz, Q = %.1f"
                  % (pk["fl"], pk["fh"], pk["bw"], pk["Q"]))
        else:
            print("# -3 dB band not bracketed: widen the range or add points")


def cmd_track(dev, args):
    """Hardware resonance tracking: the FPGA steps the frequency every window."""
    dev.set_amp(args.amp)
    dev.set_ctrl(CTRL_DAC | CTRL_SYNC)
    f = dev.set_freq(args.freq)
    time.sleep(args.settle)
    cycles = max(3, int(round(args.window * f)))
    if args.lag is None:
        m = dev.measure(cycles)
        target = m["lag"] + 360.0 * m["f"] * dev.delay_s      # the FPGA works on the raw lag
        print("# target lag taken at %.4f Hz: %.1f deg (raw)" % (m["f"], target))
    else:
        target = args.lag + 360.0 * f * dev.delay_s
    dev.wr(R_TRK_COS, int(round(32767 * math.cos(math.radians(target)))) & 0xFFFF)
    dev.wr(R_TRK_SIN, int(round(32767 * math.sin(math.radians(target)))) & 0xFFFF)
    dev.wr(R_TRK_STEP, max(1, int(round(args.step * TWO48 / CLK_HZ))))
    dev.wr(R_CYCLES, cycles)
    ctrl = CTRL_DAC | CTRL_SYNC | CTRL_CONT | CTRL_TRACK | (CTRL_INVERT if args.invert else 0)
    dev.set_ctrl(ctrl)
    print("t_s,f_Hz,amp_mV,lag_deg")
    t0 = time.time()
    try:
        while args.seconds <= 0 or time.time() - t0 < args.seconds:
            time.sleep(args.period)
            m = dev.latest()
            print("%.2f,%.6f,%.4f,%.2f" % (time.time() - t0, m["f"], m["mV"], m["lag"]))
    except KeyboardInterrupt:
        pass
    dev.set_ctrl(CTRL_DAC | CTRL_SYNC)
    print("# tracking stopped at %.6f Hz" % dev.freq_now())


def main():
    ap = argparse.ArgumentParser(description="PZ7020-StarLite lock-in host")
    ap.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("id")
    sub.add_parser("off")

    p = sub.add_parser("cal")
    p.add_argument("--amp", type=float, default=0.2)

    p = sub.add_parser("read")
    p.add_argument("freq", type=float)
    p.add_argument("--amp", type=float, default=0.2)
    p.add_argument("--settle", type=float, default=0.2)
    p.add_argument("--integrate", type=float, default=0.5, help="seconds")

    p = sub.add_parser("sweep")
    p.add_argument("f0", type=float)
    p.add_argument("f1", type=float)
    p.add_argument("points", type=int)
    p.add_argument("--log", action="store_true")
    p.add_argument("--amp", type=float, default=0.2)
    p.add_argument("--settle", type=float, default=0.2)
    p.add_argument("--integrate", type=float, default=0.5, help="seconds per point")
    p.add_argument("--out", help="also write CSV to this file")

    p = sub.add_parser("track")
    p.add_argument("freq", type=float, help="start frequency, near the resonance")
    p.add_argument("--amp", type=float, default=0.2)
    p.add_argument("--lag", type=float, default=None,
                   help="lag to hold, degrees (default: the lag measured at the start frequency)")
    p.add_argument("--step", type=float, default=0.01, help="Hz per window")
    p.add_argument("--window", type=float, default=0.1, help="seconds per window")
    p.add_argument("--settle", type=float, default=0.5)
    p.add_argument("--period", type=float, default=0.5, help="seconds between printed lines")
    p.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    p.add_argument("--invert", action="store_true", help="reverse the stepping direction")

    args = ap.parse_args()

    import serial                                  # pyserial
    link = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.1)
    link.reset_input_buffer()
    dev = LockIn(link)
    load_cal(dev)

    if args.cmd == "id":
        v = dev.ident()
        print("ID %08X %s" % (v, "(lock-in v1)" if v == 0x4C4B0001 else "(unexpected)"))
        st = dev.rd(R_STATUS)
        print("ADC %s, raw code %d, DC estimate %d counts"
              % ("alive" if st & 2 else "NOT RUNNING", dev.rd(R_ADC_RAW), dev.rd(R_DC_EST)))
    elif args.cmd == "off":
        dev.set_ctrl(0)
        print("drive off")
    elif args.cmd == "cal":
        cmd_cal(dev, args)
    elif args.cmd == "read":
        cmd_read(dev, args)
    elif args.cmd == "sweep":
        cmd_sweep(dev, args)
    elif args.cmd == "track":
        cmd_track(dev, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
