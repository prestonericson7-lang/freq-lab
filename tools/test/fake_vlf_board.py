#!/usr/bin/env python3
"""
fake_vlf_board.py -- a software vlf_sid v2 board behind a fake serial port.

Implements the register file of fpga/vlf_sid_v2 (same text protocol, same
register map) well enough to exercise vlf_host.py end to end without hardware:
GPS-second windows with 8 station powers, the STATUS snapshot, the FS_CLKS
sample-rate word, PPS count, and the sferic event FIFO with 20 ns stamps and
64-sample snapshots. The window clock runs on real time so `log` produces one
row per second, like the hardware.

Use: patch serial.Serial with FakeSerial (see test_vlf_host.py).
"""
import math
import random
import threading
import time

FS = 390_625.0
K_AMP = 32767.0 / 16.0
STATION_DB = [48.0, 40.0, 45.0, 38.0, 30.0, 25.0, 28.0, 10.0]     # dB re 1 count, per channel


class FakeBoard:
    def __init__(self, version=2, start_time=None):
        self.version = version
        self.regs = {0x01: 3, 0x02: 390625, 0x18: 60, 0x40: 0, 0x41: 1953}
        for k in range(8):
            self.regs[0x10 + k] = 0
        self.t0 = start_time if start_time is not None else time.time()
        self.seq = 0
        self.snap = None
        self.events = []            # committed, waiting
        self.count = self.lost = 0
        self.lock = threading.Lock()
        self.last_window_t = math.floor(self.t0)
        self.live = self._window(self.last_window_t)
        self._tick()

    # -- windows: one per real-time second, powers from STATION_DB with a slow wobble
    def _tick(self):
        now = time.time()
        sec = math.floor(now)
        while self.last_window_t < sec:
            self.last_window_t += 1
            self.seq += 1
            self.live = self._window(self.last_window_t)

    def _window(self, t_end):
        ndec = 381
        pows, pmax = [], []
        for k, db in enumerate(STATION_DB):
            amp = 10 ** ((db + 0.5 * math.sin(t_end / 300.0 + k)) / 20.0)
            p = (amp * K_AMP) ** 2 * ndec
            pows.append(int(p))
            pmax.append(int(p / ndec * 1.5))
        rms = 120.0
        return dict(seq=self.seq, n=390625, ndec=ndec, flags=3, clip=0, xmin=1600, xmax=2500,
                    sq=int(rms * rms * 390625), sum=-1200, pow=pows, pmax=pmax, t_end=t_end)

    # -- sferic events
    def inject_sferic(self, t_abs, peak=1400, polarity=+1, thresh=None):
        """Create an event as the hardware would for a stroke at absolute time t_abs."""
        thresh = thresh if thresh is not None else self.regs[0x40]
        if thresh == 0:
            return None
        pps_cnt = int(math.floor(t_abs) - math.floor(self.t0))
        clk = int(round((t_abs - math.floor(t_abs)) * 50e6))
        snap = []
        for i in range(64):
            n = i - 16
            if n < 0:
                snap.append(2048 + random.randint(-3, 3))
            else:
                v = polarity * peak * math.exp(-n / 60.0) * math.sin(2 * math.pi * 8000.0 * (n + 7) / FS)
                snap.append(max(0, min(4095, int(round(2048 + v)))))
        pk = max(abs(s - 2048) for s in snap[16:])
        ev = dict(pps=pps_cnt, clk=clk, peak=pk, pol=1 if polarity < 0 else 0, seq=self.count, snap=snap)
        with self.lock:
            if len(self.events) >= 32:
                self.lost += 1
                return None
            self.events.append(ev)
            self.count += 1
        return ev

    # -- register access
    def read(self, a):
        self._tick()
        if a == 0x00:
            return 0x564C0001 if self.version == 1 else 0x564C0002
        if a == 0x03:
            self.snap = dict(self.live)
            st = (self.seq & 0xFFFF) << 16 | (1 << 3) | (1 << 1) | 1
            return st
        if a in self.regs:
            return self.regs[a]
        s = self.snap or self.live
        if a == 0x04: return s["seq"]
        if a == 0x05: return s["n"]
        if a == 0x06: return s["ndec"]
        if a == 0x07: return s["flags"]
        if a == 0x08: return s["clip"]
        if a == 0x09: return (s["xmax"] << 16) | s["xmin"]
        if a == 0x0A: return s["sq"] & 0xFFFFFFFF
        if a == 0x0B: return (s["sq"] >> 32) & 0xFFFFFFFF
        if a == 0x0C: return s["sum"] & 0xFFFFFFFF
        if a == 0x0D: return (s["sum"] >> 32) & 0xFFFFFFFF
        if a == 0x0E: return 2048
        if a == 0x0F: return int(math.floor(time.time()) - math.floor(self.t0))
        if a == 0x19: return 2400
        if a == 0x1A: return 8388608
        if 0x20 <= a < 0x40:
            k, j = (a - 0x20) >> 2, (a - 0x20) & 3
            if j == 0: return s["pow"][k] & 0xFFFFFFFF
            if j == 1: return (s["pow"][k] >> 32) & 0xFFFFFFFF
            if j == 2: return (s["pmax"][k] >> 16) & 0xFFFFFFFF
            return 0
        if self.version >= 2:
            with self.lock:
                ev = self.events[0] if self.events else None
                if a == 0x42: return self.count
                if a == 0x43: return len(self.events)
                if a == 0x44:
                    n = len(self.events)
                    if n:
                        self.events.pop(0)
                    return n
                if a == 0x49: return self.lost
                if ev is None:
                    return 0
                if a == 0x45: return ev["pps"]
                if a == 0x46: return (1 << 31) | ev["clk"]
                if a == 0x47: return (16 << 24) | (64 << 16) | (ev["pol"] << 12) | ev["peak"]
                if a == 0x48: return ev["seq"]
                if 0x60 <= a < 0x80:
                    k = a - 0x60
                    return ev["snap"][2 * k] | (ev["snap"][2 * k + 1] << 16)
        return 0

    def write(self, a, v):
        if a in (0x01, 0x02, 0x18, 0x40, 0x41) or 0x10 <= a <= 0x17:
            self.regs[a] = v
            return True
        return False


class FakeSerial:
    """Just enough of pyserial's Serial for vlf_host.Board: write(), readline(),
    reset_input_buffer(), with the board's text protocol and queued replies."""
    board = None            # set by the test before constructing Board

    def __init__(self, port, baud, timeout=0.5):
        self.port = port
        self.out = bytearray()
        self.inbuf = b""

    def write(self, data):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self._exec(line.decode("ascii", "replace").strip())

    def _exec(self, line):
        if not line:
            return
        # vlf_station bridge commands (answered by the bridge, not the FPGA)
        if line.startswith("@"):
            if line == "@id":
                self.out += b"@VLFSTATION 1\n"
            elif line == "@gps":
                g = getattr(self.board, "gps", None)
                if g:
                    self.out += f"@GPS {g['utc']} {g['pps']} {1 if g['valid'] else 0} {g['sats']}\n".encode()
                else:
                    self.out += b"@GPS NONE 0 0 0\n"
            else:
                self.out += b"@E\n"
            return
        p = line.replace(" ", "")
        try:
            if p[0] in "Rr" and len(p) == 3:
                v = self.board.read(int(p[1:3], 16))
                self.out += f"{v & 0xFFFFFFFF:08X}\n".encode()
            elif p[0] in "Ww" and len(p) == 11:
                ok = self.board.write(int(p[1:3], 16), int(p[3:11], 16))
                self.out += b"K\n" if ok else b"E\n"
            else:
                self.out += b"E\n"
        except ValueError:
            self.out += b"E\n"

    def readline(self):
        if b"\n" not in self.out:
            return b""
        i = self.out.index(b"\n") + 1
        line, self.out = bytes(self.out[:i]), self.out[i:]
        return line

    def reset_input_buffer(self):
        self.out = bytearray()

    def close(self):
        pass
