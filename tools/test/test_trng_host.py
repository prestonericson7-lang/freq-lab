#!/usr/bin/env python3
"""
test_trng_host.py -- trng_host.py against a software model of the TRNG board.

The fake board serves the register protocol over a fake serial port, generates
words from a seeded RNG into a 64-deep FIFO, and can be put into a stuck/biased
mode that trips the health bits. Checks: ID, word reads, FIFO availability,
health-alarm handling (get stops), config writes, clear, byte streaming, and the
statistical selftest on good vs biased data.
"""
import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import random as _random

ID_WORD = 0x54524E31


class FakeTRNG:
    def __init__(self):
        self.regs = {0x00: ID_WORD, 0x01: 1, 0x09: 20, 0x0A: 1024, 0x0B: 660,
                     0x0C: 0x97C, 0x0D: 0x555, 0x19: 1234}
        self.rng = _random.Random(12345)
        self.fifo = []
        self.raw = self.vn = self.words = self.rct_f = self.apt_f = 0
        self.rct = self.apt = self.underflow = False
        self.mode = "good"            # good | stuck | biased

    def _produce(self, k):
        if self.rct or self.apt:
            return
        for _ in range(k):
            if len(self.fifo) >= 64:
                break
            self.fifo.append(self.rng.getrandbits(32))
            self.words += 1

    def read(self, a):
        if a == 0x02:                 # STATUS: top up the FIFO a bit on each poll
            if self.mode == "good":
                self._produce(8)
            elif self.mode == "stuck":
                self.rct = True; self.rct_f = 5
            elif self.mode == "biased":
                self.apt = True; self.apt_f = 112
            avail = len(self.fifo)
            return (avail << 8) | (int(self.underflow) << 5) | (int(len(self.fifo) >= 64) << 4) \
                | (int(avail == 0) << 3) | (int(self.apt) << 2) | (int(self.rct) << 1) | 1
        if a == 0x03:
            if self.fifo:
                return self.fifo.pop(0)
            self.underflow = True
            return 0
        if a == 0x04: return self.raw
        if a == 0x05: return self.vn
        if a == 0x06: return self.rct_f
        if a == 0x07: return self.apt_f
        if a == 0x08: return self.words
        return self.regs.get(a, 0)

    def write(self, a, v):
        if a == 0x01:
            self.regs[0x01] = v & 1
            if v & 0x100:
                self.rct = self.apt = self.underflow = False
                self.rct_f = self.apt_f = 0
                self.mode = "good"
            return True
        if a in (0x09, 0x0A, 0x0B, 0x18):
            self.regs[a] = v
            return True
        return False


class FakeSerial:
    board = None

    def __init__(self, port, baud, timeout=1.0):
        self.out = bytearray()
        self.inbuf = b""

    def write(self, data):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            p = line.decode().strip().replace(" ", "")
            if not p:
                continue
            try:
                if p[0] in "Rr" and len(p) == 3:
                    self.out += f"{self.board.read(int(p[1:3], 16)) & 0xFFFFFFFF:08X}\n".encode()
                elif p[0] in "Ww" and len(p) == 11:
                    self.out += b"K\n" if self.board.write(int(p[1:3], 16), int(p[3:11], 16)) else b"E\n"
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


serial_mod = types.ModuleType("serial")
serial_mod.Serial = FakeSerial
sys.modules["serial"] = serial_mod
import trng_host as th  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


print("[1] ID + info")
FakeSerial.board = FakeTRNG()
b = th.Board("FAKE")
check(b.read(0x00) == ID_WORD, "TRNG ID accepted")
check(abs(b.temp_c() - 27.0) < 3, f"die temp decodes to ~27 C ({b.temp_c():.1f})")
check(abs(b.vccint() - 1.0) < 0.05, "VCCINT decodes to ~1.0 V")

print("[2] get words")
w = b.get_words(100)
check(len(w) == 100 and all(0 <= x < 2**32 for x in w), "100 32-bit words read across FIFO refills")
check(len(set(w)) == 100, "the words are distinct (not a stuck FIFO)")

print("[3] health alarm stops output")
FakeSerial.board = FakeTRNG(); FakeSerial.board.mode = "stuck"
b2 = th.Board("FAKE")
w2 = b2.get_words(100)
check(len(w2) == 0 and b2.status()["rct"], "a stuck source: RCT alarm, get returns nothing")
FakeSerial.board = FakeTRNG(); FakeSerial.board.mode = "biased"
b3 = th.Board("FAKE")
check(b3.get_words(100) == [] and b3.status()["apt"], "a biased source: APT alarm, get returns nothing")
b3.clear()
check(not b3.status()["apt"], "clear() clears the latched alarm")

print("[4] config")
FakeSerial.board = FakeTRNG()
b4 = th.Board("FAKE")
b4.write(0x09, 32); b4.write(0x0A, 512); b4.write(0x0B, 410)
check(b4.read(0x09) == 32 and b4.read(0x0A) == 512 and b4.read(0x0B) == 410, "RCT/APT config writes read back")

print("[5] byte stream")
tmp = tempfile.mkdtemp()
out = os.path.join(tmp, "r.bin")
FakeSerial.board = FakeTRNG()
b5 = th.Board("FAKE")
a = types.SimpleNamespace(n=40000, out=out)
th.cmd_bytes(b5, a)
check(os.path.getsize(out) == 40000, "40000 bytes streamed to a file")

print("[6] statistical selftest")
# good data: the fake words are from Python's MT, which passes these cheap tests
FakeSerial.board = FakeTRNG()
b6 = th.Board("FAKE")
data = bytes(b"".join(w.to_bytes(4, "little") for w in b6.get_words(8000))[:30000])
bits = [(byte >> i) & 1 for byte in data for i in range(8)]
check(0.48 < sum(bits) / len(bits) < 0.52, "good data: ones fraction near 0.5")
check(th.monobit(bits) > 0.01 and th.runs_test(bits) > 0.01, "good data passes monobit and runs")
# a biased byte stream should fail monobit
biased = bytes([0xFF if i % 4 else 0x00 for i in range(30000)])
bbits = [(byte >> i) & 1 for byte in biased for i in range(8)]
check(th.monobit(bbits) < 0.01, "a biased stream fails the monobit test")
check(th.byte_chisq(biased) > 1000, "a biased stream fails the byte chi-square")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
