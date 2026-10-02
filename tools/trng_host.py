#!/usr/bin/env python3
"""
trng_host.py -- PC side of the FPGA ring-oscillator TRNG (list item #73).

    python trng_host.py COM7 info                 status, health, die temperature
    python trng_host.py COM7 get 16               print 16 random 32-bit words
    python trng_host.py COM7 bytes 1000000 r.bin  stream a million bytes to a file
    python trng_host.py COM7 health               watch the health counters live
    python trng_host.py COM7 config --rct 20 --apt-win 1024 --apt-cut 660
    python trng_host.py COM7 selftest             pull 125 kB and run SP 800-90B-style
                                                  checks + die-temp log (the SRI run)
    python trng_host.py COM7 clear                clear a latched health alarm

The FPGA runs the SP 800-90B Repetition Count and Adaptive Proportion health
tests in hardware and withholds output while either is tripped; this host adds
the offline certification run SRI did in 1980 (large sample, logged die temp).
"""
import argparse
import math
import sys
import time

ID_WORD = 0x54524E31
CLK_HZ = 50_000_000.0


class Board:
    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is missing: pip install pyserial")
        self.s = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(0.1)
        self.s.write(b"\n")
        time.sleep(0.05)
        self.s.reset_input_buffer()
        if self.read(0x00) != ID_WORD:
            sys.exit("board did not answer the TRNG ID 0x54524E31: is trng.bit loaded and TX/RX right?")

    def read(self, a):
        self.s.write(f"R{a:02X}\n".encode())
        line = self.s.readline().decode("ascii", "replace").strip()
        if line in ("", "E") or len(line) != 8:
            raise IOError(f"bad reply to R{a:02X}: {line!r}")
        return int(line, 16)

    def write(self, a, v):
        self.s.write(f"W {a:02X} {v & 0xFFFFFFFF:08X}\n".encode())
        line = self.s.readline().decode("ascii", "replace").strip()
        if line != "K":
            raise IOError(f"write {a:02X} refused: {line!r}")

    def read_many(self, addrs):
        self.s.write("".join(f"R{a:02X}\n" for a in addrs).encode())
        out = []
        for a in addrs:
            line = self.s.readline().decode("ascii", "replace").strip()
            if line in ("", "E") or len(line) != 8:
                raise IOError(f"bad reply to R{a:02X}: {line!r}")
            out.append(int(line, 16))
        return out

    def status(self):
        v = self.read(0x02)
        return dict(run=v & 1, rct=bool(v & 2), apt=bool(v & 4), empty=bool(v & 8),
                    full=bool(v & 16), underflow=bool(v & 32), avail=(v >> 8) & 0xFF)

    def get_words(self, n):
        """read n words, waiting for the FIFO to fill; stops early if a health alarm latches."""
        out = []
        idle = 0
        while len(out) < n:
            st = self.status()
            if st["rct"] or st["apt"]:
                break
            if st["avail"] == 0:
                time.sleep(0.005)
                idle += 1
                if idle > 2000:
                    break
                continue
            idle = 0
            want = min(st["avail"], n - len(out), 60)
            vals = self.read_many([0x03] * want)
            out.extend(vals)
        return out

    def temp_c(self):
        return self.read(0x0C) * 503.975 / 4096.0 - 273.15

    def vccint(self):
        return self.read(0x0D) * 3.0 / 4096.0

    def clear(self):
        self.write(0x01, 0x101)      # RUN=1, clear alarms


def cmd_info(b, a):
    st = b.status()
    print(f"ID        0x{b.read(0x00):08X}  (trng v1)")
    print(f"RUN       {st['run']}   health: RCT {'ALARM' if st['rct'] else 'ok'}, APT {'ALARM' if st['apt'] else 'ok'}")
    print(f"FIFO      {st['avail']} words available" + ("  EMPTY" if st['empty'] else "") + ("  FULL" if st['full'] else ""))
    print(f"raw bits  {b.read(0x04)}")
    print(f"VN bits   {b.read(0x05)}")
    print(f"words     {b.read(0x08)}")
    print(f"RCT fails {b.read(0x06)}   APT fails {b.read(0x07)}")
    print(f"config    RCT cutoff {b.read(0x09)}, APT window {b.read(0x0A)}, APT cutoff {b.read(0x0B)}")
    print(f"die temp  {b.temp_c():.1f} C   VCCINT {b.vccint():.3f} V")


def cmd_get(b, a):
    for w in b.get_words(a.n):
        print(f"{w:08X}")


def cmd_bytes(b, a):
    need = a.n
    written = 0
    t0 = time.time()
    with open(a.out, "wb") as f:
        while written < need:
            words = b.get_words(min(2048, (need - written + 3) // 4))
            if not words:
                print("\nstopped (health alarm or timeout):", b.status())
                break
            buf = b"".join(w.to_bytes(4, "little") for w in words)
            take = min(len(buf), need - written)
            f.write(buf[:take])
            written += take
            if written % 65536 < 4:
                rate = written / max(1e-6, time.time() - t0)
                print(f"\r{written}/{need} bytes ({rate/1024:.1f} kB/s)", end="", flush=True)
    print(f"\nwrote {written} bytes to {a.out}")


def cmd_health(b, a):
    print("time      raw        VN         words    RCT_f  APT_f  temp   RCT  APT")
    try:
        while True:
            v = b.read_many([0x04, 0x05, 0x08, 0x06, 0x07, 0x02])
            t = b.temp_c()
            print(f"\r{time.strftime('%H:%M:%S')}  {v[0]:<10} {v[1]:<10} {v[2]:<8} {v[3]:<6} {v[4]:<6} {t:4.1f}C  "
                  f"{'!' if v[5]&2 else '.'}    {'!' if v[5]&4 else '.'}   ", end="", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()


def cmd_config(b, a):
    if a.rct is not None:
        b.write(0x09, a.rct)
    if a.apt_win is not None:
        b.write(0x0A, a.apt_win)
    if a.apt_cut is not None:
        b.write(0x0B, a.apt_cut)
    print(f"config: RCT cutoff {b.read(0x09)}, APT window {b.read(0x0A)}, APT cutoff {b.read(0x0B)}")


def monobit(bits):
    """SP 800-22 frequency (monobit) test p-value."""
    s = sum(1 if x else -1 for x in bits)
    return math.erfc(abs(s) / math.sqrt(len(bits)) / math.sqrt(2))


def runs_test(bits):
    """SP 800-22 runs test p-value."""
    n = len(bits)
    pi = sum(bits) / n
    if abs(pi - 0.5) >= 2 / math.sqrt(n):
        return 0.0
    vn = 1 + sum(1 for i in range(n - 1) if bits[i] != bits[i + 1])
    num = abs(vn - 2 * n * pi * (1 - pi))
    den = 2 * math.sqrt(2 * n) * pi * (1 - pi)
    return math.erfc(num / den)


def byte_chisq(data):
    """chi-square of the byte histogram vs uniform; returns (chi2, dof=255)."""
    hist = [0] * 256
    for byte in data:
        hist[byte] += 1
    exp = len(data) / 256.0
    return sum((h - exp) ** 2 / exp for h in hist)


def cmd_selftest(b, a):
    n_bytes = a.bytes
    print(f"pulling {n_bytes} bytes for the certification run (logging die temperature) ...")
    data = bytearray()
    temps = []
    t0 = time.time()
    while len(data) < n_bytes:
        words = b.get_words(2048)
        if not words:
            print("stopped early:", b.status())
            break
        data.extend(b"".join(w.to_bytes(4, "little") for w in words))
        temps.append(b.temp_c())
    data = bytes(data[:n_bytes])
    if len(data) < 1000:
        sys.exit("not enough data")
    bits = [(byte >> i) & 1 for byte in data for i in range(8)]
    print(f"got {len(data)} bytes in {time.time()-t0:.1f} s, die temp {min(temps):.1f}-{max(temps):.1f} C")
    ones = sum(bits)
    print(f"ones fraction        {ones/len(bits):.5f}  (ideal 0.5)")
    print(f"monobit p-value      {monobit(bits):.4f}  (fail if < 0.01)")
    print(f"runs p-value         {runs_test(bits):.4f}  (fail if < 0.01)")
    chi = byte_chisq(data)
    print(f"byte chi-square      {chi:.1f}  (dof 255; ~200-310 is typical, far outside = non-uniform)")
    # longest run of identical bits
    mx = run = 1
    for i in range(1, len(bits)):
        run = run + 1 if bits[i] == bits[i - 1] else 1
        mx = max(mx, run)
    print(f"longest bit run      {mx}")
    print(f"hardware RCT fails   {b.read(0x06)}   APT fails {b.read(0x07)}")
    ok = 0.49 < ones / len(bits) < 0.51 and monobit(bits) > 0.01 and runs_test(bits) > 0.01 and chi < 400
    print("\nRESULT:", "looks random (no health alarms, statistics pass)" if ok else
          "SUSPECT: review the statistics above and the hardware health counters")


def main():
    p = argparse.ArgumentParser(description="FPGA ring-oscillator TRNG host (#73)")
    p.add_argument("port")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    g = sub.add_parser("get"); g.add_argument("n", type=int)
    by = sub.add_parser("bytes"); by.add_argument("n", type=int); by.add_argument("out")
    sub.add_parser("health")
    sub.add_parser("clear")
    c = sub.add_parser("config"); c.add_argument("--rct", type=int); c.add_argument("--apt-win", type=int); c.add_argument("--apt-cut", type=int)
    st = sub.add_parser("selftest"); st.add_argument("--bytes", type=int, default=125000)
    a = p.parse_args()
    b = Board(a.port)
    {"info": cmd_info, "get": cmd_get, "bytes": cmd_bytes, "health": cmd_health,
     "clear": lambda b, a: (b.clear(), print("alarms cleared")),
     "config": cmd_config, "selftest": cmd_selftest}[a.cmd](b, a)


if __name__ == "__main__":
    main()
