#!/usr/bin/env python3
"""
photon_host.py -- host-side control for the FPGA coincidence photon counter

Talks to the PZ7020-StarLite photon counter design over UART (115200 8N1).
Reads singles counts, coincidence counts, and computes true coincidence rates
after subtracting accidentals.

Usage:
    python photon_host.py COM5          # Windows
    python photon_host.py /dev/ttyUSB0  # Linux

Commands:
    id           Read device ID
    window N     Set coincidence window to N clock ticks (1 tick = 20 ns)
    start        Start counting
    stop         Stop counting
    status       Read status (busy, sequence number)
    read         Read all counters (singles A/B, coincidences, elapsed)
    rate         Compute singles and coincidence rates (counts/second)
    accidental   Compute accidental coincidence rate and true coincidences
    fan          Read fan RPM
    run T        Run for T seconds, then stop and print results
    stream T N   Run N acquisitions of T seconds each, print rates
    csv FILE T N Save N runs of T seconds to CSV
    help         Show this help
    quit         Exit

Theory (accidental subtraction):
    Two independent detectors with rates R_a and R_b produce accidental
    coincidences at rate:
        R_acc = 2 * tau * R_a * R_b
    where tau is the coincidence window in seconds.

    True coincidence rate:
        R_true = R_measured - R_acc

    If R_true > 0 with statistical significance, the detectors are seeing
    correlated photons (not just random dark counts).

    For biophoton detection:
        - Dark count rates: ~100-1000 cps per SiPM channel
        - Expected signal: ~1-100 photons/sec/cm^2 from living tissue
        - Coincidence window: 10-100 ns typical
        - Accidental rate with 100ns window, 1000 cps each:
          R_acc = 2 * 100e-9 * 1000 * 1000 = 0.0002 cps
        - So any measured coincidence rate >> 0.0002 cps is real signal
"""

import sys
import time
import math
import serial

CLK_HZ = 50_000_000  # 50 MHz FPGA clock

# ---- register addresses ----
REG_ID          = 0x00
REG_CTRL        = 0x01
REG_WINDOW      = 0x02
REG_STATUS      = 0x03
REG_SINGLES_A_LO = 0x04
REG_SINGLES_A_HI = 0x05
REG_SINGLES_B_LO = 0x06
REG_SINGLES_B_HI = 0x07
REG_COINC_LO    = 0x08
REG_COINC_HI    = 0x09
REG_ELAPSED     = 0x0A
REG_FAN_RPM     = 0x0B


def reg_write(ser, addr, data):
    cmd = f"W{addr:02X}{data:08X}\n"
    ser.write(cmd.encode())
    resp = ser.readline().decode().strip()
    if resp != "K":
        print(f"  WRITE ERROR: expected K, got '{resp}'")
    return resp == "K"


def reg_read(ser, addr):
    cmd = f"R{addr:02X}\n"
    ser.write(cmd.encode())
    resp = ser.readline().decode().strip()
    if len(resp) != 8:
        print(f"  READ ERROR: expected 8 hex chars, got '{resp}'")
        return 0
    return int(resp, 16)


def read_48(ser, addr_lo, addr_hi):
    lo = reg_read(ser, addr_lo)
    hi = reg_read(ser, addr_hi) & 0xFFFF
    return (hi << 32) | lo


def cmd_id(ser):
    val = reg_read(ser, REG_ID)
    tag = chr((val >> 24) & 0xFF) + chr((val >> 16) & 0xFF)
    ver = val & 0xFFFF
    print(f"  ID: 0x{val:08X}  ({tag} v{ver})")


def cmd_window(ser, ticks):
    reg_write(ser, REG_WINDOW, ticks & 0xFFFF)
    rb = reg_read(ser, REG_WINDOW)
    ns = (rb & 0xFFFF) * 20  # 20 ns per tick at 50 MHz
    print(f"  Window: {rb & 0xFFFF} ticks = {ns} ns")


def cmd_start(ser):
    reg_write(ser, REG_CTRL, 0x01)  # bit 0 = start
    print("  Counting started")


def cmd_stop(ser):
    reg_write(ser, REG_CTRL, 0x02)  # bit 1 = stop
    print("  Counting stopped")


def cmd_status(ser):
    val = reg_read(ser, REG_STATUS)
    busy = val & 1
    seq  = (val >> 16) & 0xFFFF
    print(f"  Busy: {busy}  Seq: {seq}")
    return busy, seq


def cmd_read(ser, verbose=True):
    """Read STATUS (freezes snapshot), then all counters."""
    status = reg_read(ser, REG_STATUS)
    busy = status & 1
    seq  = (status >> 16) & 0xFFFF

    sa   = read_48(ser, REG_SINGLES_A_LO, REG_SINGLES_A_HI)
    sb   = read_48(ser, REG_SINGLES_B_LO, REG_SINGLES_B_HI)
    co   = read_48(ser, REG_COINC_LO, REG_COINC_HI)
    el   = reg_read(ser, REG_ELAPSED)

    if verbose:
        elapsed_s = el / CLK_HZ
        print(f"  Seq: {seq}  Busy: {busy}")
        print(f"  Singles A:    {sa:>15d}")
        print(f"  Singles B:    {sb:>15d}")
        print(f"  Coincidences: {co:>15d}")
        print(f"  Elapsed:      {el:>15d} ticks  ({elapsed_s:.4f} s)")

    return {
        "seq": seq, "busy": busy,
        "singles_a": sa, "singles_b": sb,
        "coincidences": co, "elapsed": el
    }


def compute_rates(r):
    """Compute rates from raw counts."""
    el = r["elapsed"]
    if el == 0:
        return None
    t = el / CLK_HZ

    ra = r["singles_a"] / t
    rb = r["singles_b"] / t
    rc = r["coincidences"] / t
    return {"time_s": t, "rate_a": ra, "rate_b": rb, "rate_coinc": rc}


def compute_accidentals(r):
    """Compute accidental rate and true coincidence rate."""
    el = r["elapsed"]
    if el == 0:
        return None
    t = el / CLK_HZ

    # Read the window setting
    sa = r["singles_a"]
    sb = r["singles_b"]
    co = r["coincidences"]

    ra = sa / t
    rb = sb / t
    rc = co / t

    return {"time_s": t, "rate_a": ra, "rate_b": rb, "rate_coinc": rc}


def cmd_rate(ser):
    r = cmd_read(ser, verbose=True)
    rates = compute_rates(r)
    if rates:
        print(f"  Rate A:     {rates['rate_a']:.2f} cps")
        print(f"  Rate B:     {rates['rate_b']:.2f} cps")
        print(f"  Rate Coinc: {rates['rate_coinc']:.4f} cps")


def cmd_accidental(ser):
    """Compute accidental rate using the coincidence window."""
    r = cmd_read(ser, verbose=True)
    window_ticks = reg_read(ser, REG_WINDOW) & 0xFFFF
    tau = window_ticks / CLK_HZ  # window in seconds

    el = r["elapsed"]
    if el == 0:
        print("  No data")
        return
    t = el / CLK_HZ

    ra = r["singles_a"] / t
    rb = r["singles_b"] / t
    rc = r["coincidences"] / t

    # the FPGA counts |tA - tB| <= window ticks: a window 2*window+1 ticks wide
    r_acc = (2 * window_ticks + 1) / CLK_HZ * ra * rb
    r_true = rc - r_acc

    print(f"\n  Window:      {window_ticks} ticks = {tau*1e9:.1f} ns")
    print(f"  Rate A:      {ra:.2f} cps")
    print(f"  Rate B:      {rb:.2f} cps")
    print(f"  Measured:    {rc:.6f} cps")
    print(f"  Accidental:  {r_acc:.6f} cps")
    print(f"  True coinc:  {r_true:.6f} cps")

    if r["coincidences"] > 0:
        # Poisson error on coincidence count
        sigma = math.sqrt(r["coincidences"]) / t
        snr = r_true / sigma if sigma > 0 else 0
        print(f"  Poisson σ:   {sigma:.6f} cps")
        print(f"  SNR:         {snr:.2f} σ")
    print()


def cmd_run(ser, duration):
    """Run for a specified duration in seconds."""
    cmd_start(ser)
    print(f"  Running for {duration} seconds...")
    time.sleep(duration)
    cmd_stop(ser)
    time.sleep(0.1)
    cmd_read(ser)


def cmd_stream(ser, duration, count):
    """Run multiple acquisitions."""
    for i in range(count):
        cmd_start(ser)
        time.sleep(duration)
        cmd_stop(ser)
        time.sleep(0.1)
        r = cmd_read(ser, verbose=False)
        rates = compute_rates(r)
        if rates:
            print(f"  [{i+1:4d}/{count}] "
                  f"A={rates['rate_a']:.1f} B={rates['rate_b']:.1f} "
                  f"Coinc={rates['rate_coinc']:.4f} cps  "
                  f"t={rates['time_s']:.3f}s")


def cmd_csv(ser, filename, duration, count):
    """Run and save to CSV."""
    import csv

    window_ticks = reg_read(ser, REG_WINDOW) & 0xFFFF
    tau = window_ticks / CLK_HZ

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run", "time_s", "singles_a", "singles_b",
                          "coincidences", "rate_a", "rate_b", "rate_coinc",
                          "rate_accidental", "rate_true", "window_ns"])

        for i in range(count):
            cmd_start(ser)
            time.sleep(duration)
            cmd_stop(ser)
            time.sleep(0.1)
            r = cmd_read(ser, verbose=False)
            rates = compute_rates(r)
            if rates:
                r_acc = 2.0 * tau * rates["rate_a"] * rates["rate_b"]
                r_true = rates["rate_coinc"] - r_acc
                writer.writerow([
                    i + 1, f"{rates['time_s']:.4f}",
                    r["singles_a"], r["singles_b"], r["coincidences"],
                    f"{rates['rate_a']:.2f}", f"{rates['rate_b']:.2f}",
                    f"{rates['rate_coinc']:.6f}",
                    f"{r_acc:.6f}", f"{r_true:.6f}",
                    f"{tau*1e9:.1f}"
                ])
                print(f"  [{i+1:4d}/{count}] true={r_true:.6f} cps")

    print(f"  Saved to {filename}")


def cmd_fan(ser):
    val = reg_read(ser, REG_FAN_RPM) & 0xFFFF
    print(f"  Fan RPM: {val}")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print("Usage: photon_host.py <serial-port> [baud]")
        print("  e.g. photon_host.py COM5")
        print("       photon_host.py /dev/ttyUSB0 115200")
        sys.exit(0 if len(sys.argv) > 1 else 1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    ser = serial.Serial(port, baud, timeout=1.0)
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"Photon counter host -- {port} @ {baud}")
    print("Type 'help' for commands.\n")

    cmd_id(ser)
    print()

    while True:
        try:
            line = input("photon> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "id":
                cmd_id(ser)
            elif cmd == "window" and len(parts) > 1:
                cmd_window(ser, int(parts[1]))
            elif cmd == "start":
                cmd_start(ser)
            elif cmd == "stop":
                cmd_stop(ser)
            elif cmd == "status":
                cmd_status(ser)
            elif cmd == "read":
                cmd_read(ser)
            elif cmd == "rate":
                cmd_rate(ser)
            elif cmd == "accidental":
                cmd_accidental(ser)
            elif cmd == "fan":
                cmd_fan(ser)
            elif cmd == "run" and len(parts) > 1:
                cmd_run(ser, float(parts[1]))
            elif cmd == "stream" and len(parts) > 2:
                cmd_stream(ser, float(parts[1]), int(parts[2]))
            elif cmd == "csv" and len(parts) > 3:
                cmd_csv(ser, parts[1], float(parts[2]), int(parts[3]))
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
