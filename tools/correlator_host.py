#!/usr/bin/env python3
"""
correlator_host.py -- host-side control for the FPGA cross-correlator

Talks to the PZ7020-StarLite correlator design over UART (115200 8N1).
Computes DC-removed correlation coefficients, coherence, and cross-
correlation at arbitrary lags.

Usage:
    python correlator_host.py COM5          # Windows
    python correlator_host.py /dev/ttyUSB0  # Linux

Commands:
    id          Read device ID
    adc         Read live ADC values (both channels)
    window N    Set window length to N samples
    start       Start acquisition
    status      Read status (busy, sequence number)
    results     Read full result set (auto, cross, sums)
    coherence   Compute normalised coherence from last results
    lag N       Query cross-correlation at lag N
    lagplot M   Sweep lags -M..+M and plot cross-correlation
    pps         Read GPS PPS sample counter
    fan         Read fan RPM
    stream N    Run N windows continuously, print coherence each time
    csv FILE N  Run N windows, save CSV with columns:
                seq, N, raa, rbb, rab, sum_a, sum_b, coherence
    help        Show this help
    quit        Exit

Theory:
    Given N sample pairs (a_i, b_i):
      mean_a = sum_a / N
      mean_b = sum_b / N
      var_a  = raa/N - mean_a^2   (auto-correlation at lag 0, DC-removed)
      var_b  = rbb/N - mean_b^2
      cov_ab = rab/N - mean_a*mean_b
      coherence = cov_ab / sqrt(var_a * var_b)

    The FPGA computes raa, rbb, rab, sum_a, sum_b as raw sums (no DC
    removal).  The host does the DC subtraction using the identities above.
"""

import sys
import time
import math
import serial

# ---- register addresses ----
REG_ID         = 0x00
REG_CTRL       = 0x01
REG_SAMPLES    = 0x02
REG_STATUS     = 0x03
REG_N          = 0x04
REG_RAA_LO     = 0x05
REG_RAA_HI     = 0x06
REG_RBB_LO     = 0x07
REG_RBB_HI     = 0x08
REG_RAB_LO     = 0x09
REG_RAB_HI     = 0x0A
REG_SUMA_LO    = 0x0B
REG_SUMA_HI    = 0x0C
REG_SUMB_LO    = 0x0D
REG_SUMB_HI    = 0x0E
REG_ADC_A      = 0x0F
REG_ADC_B      = 0x10
REG_QUERY_LAG  = 0x11
REG_QUERY_GO   = 0x12
REG_XCORR_LO   = 0x13
REG_XCORR_HI   = 0x14
REG_PPS_LAST   = 0x15
REG_FAN_RPM    = 0x16


def reg_write(ser, addr, data):
    """Write a 32-bit value to a register."""
    cmd = f"W{addr:02X}{data:08X}\n"
    ser.write(cmd.encode())
    resp = ser.readline().decode().strip()
    if resp != "K":
        print(f"  WRITE ERROR: expected K, got '{resp}'")
    return resp == "K"


def reg_read(ser, addr):
    """Read a 32-bit value from a register."""
    cmd = f"R{addr:02X}\n"
    ser.write(cmd.encode())
    resp = ser.readline().decode().strip()
    if len(resp) != 8:
        print(f"  READ ERROR: expected 8 hex chars, got '{resp}'")
        return 0
    return int(resp, 16)


def read_64(ser, addr_lo, addr_hi):
    """Read a 64-bit value from two consecutive registers."""
    lo = reg_read(ser, addr_lo)
    hi = reg_read(ser, addr_hi)
    val = (hi << 32) | lo
    # sign-extend from 64 bits
    if val >= (1 << 63):
        val -= (1 << 64)
    return val


def cmd_id(ser):
    val = reg_read(ser, REG_ID)
    tag = chr((val >> 24) & 0xFF) + chr((val >> 16) & 0xFF)
    ver = val & 0xFFFF
    print(f"  ID: 0x{val:08X}  ({tag} v{ver})")


def cmd_adc(ser):
    a = reg_read(ser, REG_ADC_A) & 0xFFF
    b = reg_read(ser, REG_ADC_B) & 0xFFF
    va = a / 4096.0  # XADC unipolar: 0..1.0 V
    vb = b / 4096.0
    print(f"  ADC A: {a:4d}  ({va:.4f} V)")
    print(f"  ADC B: {b:4d}  ({vb:.4f} V)")


def cmd_window(ser, n):
    reg_write(ser, REG_SAMPLES, n & 0xFFFFFF)
    rb = reg_read(ser, REG_SAMPLES)
    print(f"  Window set to {rb & 0xFFFFFF} samples")


def cmd_start(ser):
    reg_write(ser, REG_CTRL, 1)
    print("  Acquisition started")


def cmd_status(ser):
    val = reg_read(ser, REG_STATUS)
    busy = val & 1
    seq  = (val >> 16) & 0xFFFF
    print(f"  Busy: {busy}  Seq: {seq}")
    return busy, seq


def wait_done(ser, timeout=10.0):
    """Poll STATUS until not busy, return sequence number."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        val = reg_read(ser, REG_STATUS)
        if not (val & 1):
            return (val >> 16) & 0xFFFF
        time.sleep(0.05)
    print("  TIMEOUT waiting for acquisition to finish")
    return None


def cmd_results(ser, verbose=True):
    """Read STATUS (freezes snapshot), then all result registers."""
    # Reading STATUS freezes the snapshot registers
    status = reg_read(ser, REG_STATUS)
    busy = status & 1
    seq  = (status >> 16) & 0xFFFF

    n     = reg_read(ser, REG_N)
    raa   = read_64(ser, REG_RAA_LO, REG_RAA_HI)
    rbb   = read_64(ser, REG_RBB_LO, REG_RBB_HI)
    rab   = read_64(ser, REG_RAB_LO, REG_RAB_HI)
    sum_a = read_64(ser, REG_SUMA_LO, REG_SUMA_HI)
    sum_b = read_64(ser, REG_SUMB_LO, REG_SUMB_HI)

    if verbose:
        print(f"  Seq: {seq}  N: {n}  Busy: {busy}")
        print(f"  Raa:   {raa:>20d}")
        print(f"  Rbb:   {rbb:>20d}")
        print(f"  Rab:   {rab:>20d}")
        print(f"  Sum_a: {sum_a:>20d}")
        print(f"  Sum_b: {sum_b:>20d}")

    return {
        "seq": seq, "n": n, "busy": busy,
        "raa": raa, "rbb": rbb, "rab": rab,
        "sum_a": sum_a, "sum_b": sum_b
    }


def compute_coherence(r):
    """Compute normalised coherence from result dict."""
    n = r["n"]
    if n == 0:
        return 0.0

    mean_a = r["sum_a"] / n
    mean_b = r["sum_b"] / n
    var_a  = r["raa"] / n - mean_a * mean_a
    var_b  = r["rbb"] / n - mean_b * mean_b
    cov_ab = r["rab"] / n - mean_a * mean_b

    denom = math.sqrt(abs(var_a) * abs(var_b))
    if denom < 1e-12:
        return 0.0
    return cov_ab / denom


def cmd_coherence(ser):
    r = cmd_results(ser, verbose=True)
    c = compute_coherence(r)
    print(f"  Coherence: {c:+.6f}")
    return c


def cmd_lag(ser, lag):
    """Query cross-correlation at a specific lag."""
    # Write lag value (signed 16-bit)
    lag_u = lag & 0xFFFF  # two's complement
    reg_write(ser, REG_QUERY_LAG, lag_u)
    # Pulse query go
    reg_write(ser, REG_QUERY_GO, 1)
    # Wait a moment for computation (MAX_LAG * 2 clocks + overhead)
    time.sleep(0.01)
    # Read result
    xcorr = read_64(ser, REG_XCORR_LO, REG_XCORR_HI)
    print(f"  Xcorr(lag={lag:+d}) = {xcorr}")
    return xcorr


def cmd_lagplot(ser, max_lag):
    """Sweep lags and print ASCII plot."""
    lags = range(-max_lag, max_lag + 1)
    values = []
    print(f"  Sweeping lags -{max_lag}..+{max_lag}...")
    for lag in lags:
        lag_u = lag & 0xFFFF
        reg_write(ser, REG_QUERY_LAG, lag_u)
        reg_write(ser, REG_QUERY_GO, 1)
        time.sleep(0.005)
        val = read_64(ser, REG_XCORR_LO, REG_XCORR_HI)
        values.append(val)

    # ASCII plot
    vmin = min(values)
    vmax = max(values)
    span = vmax - vmin if vmax != vmin else 1
    width = 50
    print()
    for i, lag in enumerate(lags):
        bar_len = int((values[i] - vmin) / span * width)
        bar = "#" * bar_len
        print(f"  {lag:+4d} | {bar:<{width}} | {values[i]}")
    print()


def cmd_pps(ser):
    val = reg_read(ser, REG_PPS_LAST)
    print(f"  PPS last sample count: {val}")


def cmd_fan(ser):
    val = reg_read(ser, REG_FAN_RPM) & 0xFFFF
    print(f"  Fan RPM: {val}")


def cmd_stream(ser, count):
    """Run multiple windows and print coherence each time."""
    for i in range(count):
        cmd_start(ser)
        seq = wait_done(ser)
        if seq is None:
            break
        r = cmd_results(ser, verbose=False)
        c = compute_coherence(r)
        print(f"  [{i+1:4d}/{count}] seq={r['seq']} N={r['n']} "
              f"coherence={c:+.6f}")


def cmd_csv(ser, filename, count):
    """Run windows and save to CSV."""
    import csv
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["seq", "n", "raa", "rbb", "rab",
                          "sum_a", "sum_b", "coherence"])
        for i in range(count):
            cmd_start(ser)
            seq = wait_done(ser)
            if seq is None:
                break
            r = cmd_results(ser, verbose=False)
            c = compute_coherence(r)
            writer.writerow([r["seq"], r["n"], r["raa"], r["rbb"],
                             r["rab"], r["sum_a"], r["sum_b"],
                             f"{c:.8f}"])
            print(f"  [{i+1:4d}/{count}] coherence={c:+.6f}")
    print(f"  Saved to {filename}")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print("Usage: correlator_host.py <serial-port> [baud]")
        print("  e.g. correlator_host.py COM5")
        print("       correlator_host.py /dev/ttyUSB0 115200")
        sys.exit(0 if len(sys.argv) > 1 else 1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    ser = serial.Serial(port, baud, timeout=1.0)
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"Correlator host -- {port} @ {baud}")
    print("Type 'help' for commands.\n")

    # auto-read ID on connect
    cmd_id(ser)
    print()

    while True:
        try:
            line = input("corr> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "id":
                cmd_id(ser)
            elif cmd == "adc":
                cmd_adc(ser)
            elif cmd == "window" and len(parts) > 1:
                cmd_window(ser, int(parts[1]))
            elif cmd == "start":
                cmd_start(ser)
            elif cmd == "status":
                cmd_status(ser)
            elif cmd == "results":
                cmd_results(ser)
            elif cmd == "coherence":
                cmd_coherence(ser)
            elif cmd == "lag" and len(parts) > 1:
                cmd_lag(ser, int(parts[1]))
            elif cmd == "lagplot" and len(parts) > 1:
                cmd_lagplot(ser, int(parts[1]))
            elif cmd == "pps":
                cmd_pps(ser)
            elif cmd == "fan":
                cmd_fan(ser)
            elif cmd == "stream" and len(parts) > 1:
                cmd_stream(ser, int(parts[1]))
            elif cmd == "csv" and len(parts) > 2:
                cmd_csv(ser, parts[1], int(parts[2]))
            elif cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print(__doc__)
            else:
                print(f"  Unknown command: {line}")
                print("  Type 'help' for commands")
        except Exception as e:
            print(f"  ERROR: {e}")

    ser.close()
    print("Bye.")


if __name__ == "__main__":
    main()
