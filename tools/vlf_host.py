#!/usr/bin/env python3
"""
vlf_host.py -- PC side of the freq-lab VLF solar-flare (SID) receiver.

Talks to the vlf_sid FPGA design over a 3.3 V USB-serial adapter
(JM1-14 = FPGA TX -> adapter RX, JM1-16 = FPGA RX <- adapter TX, GND to GND).

    python vlf_host.py COM7 info              what the board is doing right now
    python vlf_host.py COM7 level             live input level: set your preamp gain with this
    python vlf_host.py COM7 scan              VLF spectrum 10-50 kHz in ~10 s: which stations you hear
    python vlf_host.py COM7 log               log all 8 stations once per second to a CSV (Ctrl+C stops)
    python vlf_host.py COM7 log --goes        ... and pull the GOES X-ray flux so flares line up
    python vlf_host.py COM7 tune 7 23.4       point channel 7 at 23.4 kHz (DHO38, say)
    python vlf_host.py COM7 raw               type R/W register commands by hand
    python vlf_host.py plot vlf_2026-10-01.csv [--goes goes_2026-10-01.csv]

Station network (works with sid_server.py):
    python vlf_host.py COM7 log --station HOME --push http://server:8750
                                              ... and upload every second's row to the server
    python vlf_host.py COM7 sferics --thresh 600
                                              v2 bitstream: log lightning strokes with their
                                              GPS second + 20 ns clock stamp + 64-sample snapshot
    python vlf_host.py COM7 sferics --thresh 600 --station HOME --push http://server:8750

Needs: pip install pyserial        (plot also needs: pip install matplotlib)

What the numbers mean: each station's amplitude is given in dB relative to one
ADC count (1 count = 1 V / 4096 = 244 uV at the XADC pin). Only changes
matter for SID work: a solar flare makes daytime stations jump or dip by a few
dB within minutes, in step with the GOES X-ray curve.
"""
import argparse
import collections
import csv
import datetime as dt
import json
import math
import os
import statistics
import sys
import threading
import time
import urllib.request

FS_NOM = 390_625.0                      # XADC sample rate with the 50 MHz crystal
CLK_HZ = 50_000_000.0                   # fabric clock: the sferic stamps count these (20 ns)
K_AMP = 32767.0 / 16.0                  # sqrt(I^2+Q^2) per ADC count of tone amplitude
ID_WORDS = {0x564C0001: 1, 0x564C0002: 2}   # v1: stations only; v2: + sferic (lightning) capture
ID_WORD = 0x564C0001                    # kept for scripts that import it

# channel -> (call sign, kHz, site). Matches the FPGA's power-up defaults.
STATIONS = [
    ("NLK", 24.80, "Jim Creek, WA"),
    ("NPM", 21.40, "Lualualei, HI"),
    ("NAA", 24.00, "Cutler, ME"),
    ("NML", 25.20, "LaMoure, ND"),
    ("NAU", 40.75, "Aguada, PR"),
    ("NWC", 19.80, "Exmouth, Australia"),
    ("JJI", 22.20, "Ebino, Japan"),
    ("REF", 30.00, "no station: noise / lightning reference"),
]

# for naming peaks in a scan
KNOWN = [
    (16.40, "JXN Norway"), (17.00, "VTX India"), (17.80, "NAA (alt)"), (18.30, "HWU France"),
    (19.58, "GQD UK"), (19.80, "NWC Australia"), (20.27, "ICV Italy"), (21.40, "NPM Hawaii"),
    (21.75, "HWU France"), (22.10, "GQD UK"), (22.20, "JJI Japan"), (23.40, "DHO38 Germany"),
    (24.00, "NAA Maine"), (24.80, "NLK Washington"), (25.20, "NML North Dakota"),
    (26.70, "TBB Turkey"), (37.50, "TFK/NRK Iceland"), (40.75, "NAU Puerto Rico"),
    (45.90, "NSY Italy"),
]

GOES_URLS = [
    "https://services.swpc.noaa.gov/json/goes/primary/xrays-6-hour.json",
    "https://services.swpc.noaa.gov/json/goes/primary/xrays-1-day.json",
]


# ---------------------------------------------------------------------- serial link
class Board:
    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is missing: run  pip install pyserial")
        try:
            self.s = serial.Serial(port, 115200, timeout=0.5)
        except Exception as e:
            sys.exit(f"cannot open {port}: {e}")
        time.sleep(0.05)
        self.s.write(b"\n")                 # finish any half-sent command
        time.sleep(0.05)
        self.s.reset_input_buffer()
        ident = self.read(0x00)
        if ident not in ID_WORDS:
            sys.exit(f"board answered ID 0x{ident:08X}, expected 0x564C0001 (v1) or 0x564C0002 (v2): "
                     "is the vlf_sid bitstream loaded and TX/RX the right way round?")
        self.version = ID_WORDS[ident]
        # the ADC sample rate as measured by the FPGA against its 50 MHz crystal
        self.fs = FS_NOM
        for _ in range(10):
            clks = self.read(0x1A)
            if clks:
                fs = 65536 * 50e6 / clks
                if 100e3 < fs < 1.1e6:
                    self.fs = fs
                break
            time.sleep(0.05)

    def _line(self):
        line = self.s.readline().decode("ascii", "replace").strip()
        if not line:
            raise IOError("no reply from the board (check wiring and that the bitstream is loaded)")
        return line

    def read(self, addr):
        self.s.write(f"R{addr:02X}\n".encode())
        line = self._line()
        if line == "E" or len(line) != 8:
            raise IOError(f"bad reply to R{addr:02X}: {line!r}")
        return int(line, 16)

    def write(self, addr, value):
        self.s.write(f"W {addr:02X} {value & 0xFFFFFFFF:08X}\n".encode())
        line = self._line()
        if line != "K":
            raise IOError(f"write {addr:02X} refused: {line!r}")

    def gps_time(self):
        """Ask a vlf_station Teensy bridge for the GPS time (returns None if the
        port is a plain adapter, not the bridge, or there is no fix yet).
        Reply: '@GPS <utc|NONE> <pps_count> <valid> <sats>'."""
        self.s.write(b"@gps\n")
        line = self.s.readline().decode("ascii", "replace").strip()
        if not line.startswith("@GPS"):
            return None
        p = line.split()
        if len(p) < 5 or p[1] == "NONE":
            return None
        return dict(utc=p[1], pps=int(p[2]), valid=p[3] == "1", sats=int(p[4]))

    def has_gps_bridge(self):
        self.s.write(b"@id\n")
        return self.s.readline().decode("ascii", "replace").strip() == "@VLFSTATION 1"

    def read_many(self, addrs):
        """Send a batch of reads in one go; the FPGA queues the replies in order.
        One USB round trip instead of one per register (USB-serial adapters can
        add up to 16 ms of latency to every round trip)."""
        self.s.write("".join(f"R{a:02X}\n" for a in addrs).encode())
        vals = []
        for a in addrs:
            line = self._line()
            if line == "E" or len(line) != 8:
                raise IOError(f"bad reply to R{a:02X}: {line!r}")
            vals.append(int(line, 16))
        return vals

    SNAP_ADDRS = [0x03] + list(range(0x04, 0x0E)) + [0x20 + 4 * k + j for k in range(8) for j in range(3)]

    def snapshot(self):
        """Read STATUS (which freezes the last finished window) and everything in it."""
        v = self.read_many(self.SNAP_ADDRS)
        r = dict(zip(self.SNAP_ADDRS, v))
        snap = dict(status=r[0x03], seq=r[0x04], n=r[0x05], ndec=r[0x06], flags=r[0x07], clip=r[0x08])
        snap["xmin"], snap["xmax"] = r[0x09] & 0xFFF, (r[0x09] >> 16) & 0xFFF
        snap["sq"] = r[0x0A] | (r[0x0B] << 32)
        sm = r[0x0C] | (r[0x0D] << 32)
        snap["sum"] = sm - (1 << 64) if sm >> 63 else sm
        snap["pow"] = [r[0x20 + 4 * k] | (r[0x21 + 4 * k] << 32) for k in range(8)]
        snap["pmax"] = [r[0x22 + 4 * k] << 16 for k in range(8)]
        return snap

    # ---- v2 sferic (lightning) capture: registers 0x40-0x49, snapshot 0x60-0x7F
    EV_ADDRS = [0x45, 0x46, 0x47, 0x48] + list(range(0x60, 0x80))

    def sferic_arm(self, thresh, holdoff):
        if self.version < 2:
            sys.exit("this bitstream is v1 (ID 0x564C0001): sferic capture needs fpga/vlf_sid_v2")
        self.write(0x40, int(thresh) & 0xFFF)
        self.write(0x41, int(holdoff) & 0xFFFF)

    def sferic_waiting(self):
        return self.read(0x43)

    def sferic_pop(self):
        """Read the oldest event (header + 64 samples) in one round trip, then drop it."""
        v = self.read_many(self.EV_ADDRS)
        ev = dict(pps=v[0], clk=v[1] & 0x3FFFFFF, pps_seen=bool(v[1] >> 31),
                  pre=(v[2] >> 24) & 0xFF, length=(v[2] >> 16) & 0xFF,
                  polarity=-1 if (v[2] >> 12) & 1 else +1, peak=v[2] & 0xFFF, seq=v[3])
        snap = []
        for w in v[4:]:
            snap += [w & 0xFFF, (w >> 16) & 0xFFF]
        ev["snapshot"] = snap[:ev["length"]] if ev["length"] else snap
        self.read(0x44)
        return ev


def ftw_for(khz, fs=FS_NOM):
    return int(round(khz * 1000.0 * 2 ** 32 / fs)) & 0xFFFFFFFF


def khz_of(ftw, fs=FS_NOM):
    return ftw * fs / 2 ** 32 / 1000.0


def amp_db(pow_sum, ndec):
    if ndec <= 0 or pow_sum <= 0:
        return float("nan")
    return 20.0 * math.log10(math.sqrt(pow_sum / ndec) / K_AMP)


def spike_db(pmax, pow_sum, ndec):
    """largest single reading vs the window average: lightning sferics push this up"""
    if ndec <= 0 or pow_sum <= 0 or pmax <= 0:
        return float("nan")
    return 10.0 * math.log10(pmax / (pow_sum / ndec))


def input_stats(snap):
    n = max(snap["n"], 1)
    mean = snap["sum"] / n
    rms = math.sqrt(max(snap["sq"] / n - mean * mean, 0.0))
    return rms, mean


class Pusher(threading.Thread):
    """Uploads rows to sid_server.py (POST <url>/api/ingest, JSON) in batches; keeps
    rows across outages (up to a day of seconds) and retries."""

    def __init__(self, url, station, kind, interval=5.0):
        super().__init__(daemon=True)
        self.url = url.rstrip("/") + "/api/ingest"
        self.station, self.kind, self.interval = station, kind, interval
        self.q = collections.deque(maxlen=100_000)
        self.lock = threading.Lock()
        self.sent = self.failed = 0
        self.last_error = ""

    def add(self, row):
        with self.lock:
            self.q.append(row)

    def flush_once(self):
        with self.lock:
            batch = [self.q.popleft() for _ in range(min(600, len(self.q)))]
        if not batch:
            return True
        body = json.dumps({"station": self.station, "kind": self.kind, "rows": batch}).encode()
        req = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                r.read()
            self.sent += len(batch)
            return True
        except Exception as e:
            self.last_error = str(e)
            self.failed += 1
            with self.lock:
                self.q.extendleft(reversed(batch))
            return False

    def run(self):
        backoff = self.interval
        while True:
            time.sleep(backoff)
            ok = self.flush_once()
            backoff = self.interval if ok else min(backoff * 2, 300)


def wait_new_window(b, last_seq, timeout=5.0):
    """poll STATUS until the window counter moves; returns the snapshot"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ((b.read(0x03) >> 16) & 0xFFFF) != (last_seq & 0xFFFF):
            return b.snapshot()
        time.sleep(0.05)
    raise IOError("no new window: is CTRL.RUN off, or PPS mode on with a stalled PPS?")


# ---------------------------------------------------------------------- commands
def cmd_info(b, args):
    ctrl, win, st = b.read(0x01), b.read(0x02), b.read(0x03)
    print(f"ID       0x{b.read(0x00):08X}  (vlf_sid v{b.version}"
          + (", sferic capture)" if b.version >= 2 else ")"))
    if b.version >= 2:
        th, ho = b.read(0x40), b.read(0x41)
        print(f"SFERIC   threshold {th} counts ({'off' if th == 0 else 'armed'}), hold-off {ho} samples, "
              f"{b.read(0x42)} captured, {b.read(0x43)} waiting, {b.read(0x49)} lost")
    print(f"ADC      {b.fs:.1f} samples/s by the board crystal (design value {FS_NOM:.0f})")
    print(f"CTRL     run={ctrl & 1} pps_mode={(ctrl >> 1) & 1}   window={win} samples "
          f"({win / b.fs:.3f} s without GPS)")
    print(f"STATUS   windows done={st >> 16}  PPS alive={(st >> 3) & 1}  "
          f"last window ended on PPS={(st >> 1) & 1} / on timeout={(st >> 2) & 1}")
    print(f"PPS      {b.read(0x0F)} edges since power-up")
    print(f"ADC      live code {b.read(0x0E)} (mid-scale 2048 = 0.5 V)")
    print(f"FAN      {b.read(0x18)} %  {b.read(0x19)} rpm")
    snap = b.snapshot()
    rms, dc = input_stats(snap)
    print(f"WINDOW   #{snap['seq']}: {snap['n']} samples, {snap['ndec']} outputs/channel, "
          f"clipped {snap['clip']}, input RMS {rms:.1f} counts, DC {dc:+.1f}")
    print("\n ch  call    kHz    level dB   spike dB   site")
    for k in range(8):
        f = khz_of(b.read(0x10 + k), b.fs)
        call = next((c for c, khz, _ in STATIONS if abs(khz - f) < 0.01), "--")
        site = next((s for c, khz, s in STATIONS if abs(khz - f) < 0.01), "")
        print(f" {k}   {call:4s} {f:8.3f}   {amp_db(snap['pow'][k], snap['ndec']):8.2f}   "
              f"{spike_db(snap['pmax'][k], snap['pow'][k], snap['ndec']):8.2f}   {site}")


def cmd_tune(b, args):
    if not 0 <= args.channel <= 7:
        sys.exit("channel is 0..7")
    if not 1.0 <= args.khz <= 190.0:
        sys.exit("frequency must be 1..190 kHz (the ADC samples at 390.6 kS/s)")
    b.write(0x10 + args.channel, ftw_for(args.khz, b.fs))
    print(f"channel {args.channel} -> {khz_of(b.read(0x10 + args.channel), b.fs):.4f} kHz")


def cmd_defaults(b):
    for k, (_, khz, _) in enumerate(STATIONS):
        b.write(0x10 + k, ftw_for(khz, b.fs))


def cmd_level(b, args):
    """one line per window: everything needed to set the preamp gain"""
    b.write(0x02, int(round(b.fs)))
    b.write(0x01, 0x3)
    cmd_defaults(b)
    print("Aim for: RMS 30-300 counts, CLIP 0, min/max well inside 0..4095, DC near 0.")
    print("   time      RMS   %FS     DC   min   max  clip   strongest station")
    last = b.read(0x03) >> 16
    try:
        while True:
            snap = wait_new_window(b, last)
            last = snap["seq"]
            rms, dc = input_stats(snap)
            levels = [amp_db(snap["pow"][k], snap["ndec"]) for k in range(8)]
            k = max(range(7), key=lambda i: levels[i] if not math.isnan(levels[i]) else -999)
            warn = "  <-- CLIPPING: lower the gain" if snap["clip"] else (
                "  <-- very low: raise the gain" if rms < 5 else "")
            print(f"{time.strftime('%H:%M:%S')}  {rms:7.1f} {100 * rms / 2048:5.1f} {dc:+6.1f} "
                  f"{snap['xmin']:5d} {snap['xmax']:5d} {snap['clip']:5d}   "
                  f"{STATIONS[k][0]} {levels[k]:.1f} dB{warn}")
    except KeyboardInterrupt:
        print()


def cmd_scan(b, args):
    """Sweep the 8 channels across a band with clean, separately started windows."""
    saved = [b.read(0x10 + k) for k in range(8)]
    saved_ctrl, saved_win = b.read(0x01), b.read(0x02)
    win = int(b.fs * args.dwell)
    freqs = []
    f = args.start
    while f <= args.stop + 1e-9:
        freqs.append(round(f, 4))
        f += args.step
    result = []
    print(f"scanning {args.start}-{args.stop} kHz in {args.step * 1000:.0f} Hz steps, "
          f"{args.dwell * 1000:.0f} ms per step, 8 frequencies at a time ...")
    try:
        b.write(0x02, win)
        for i in range(0, len(freqs), 8):
            batch = freqs[i:i + 8]
            b.write(0x01, 0x0)                      # stop + clear the accumulators
            for k, fk in enumerate(batch):
                b.write(0x10 + k, ftw_for(fk, b.fs))
            time.sleep(0.012)                       # let the filters forget the old frequencies
            last = b.read(0x03) >> 16
            b.write(0x01, 0x1)                      # fresh window starts now (PPS ignored)
            snap = wait_new_window(b, last, timeout=args.dwell * 4 + 2)
            for k, fk in enumerate(batch):
                result.append((fk, amp_db(snap["pow"][k], snap["ndec"])))
            print(f"\r  {min(100, 100 * (i + 8) // len(freqs)):3d} %", end="", flush=True)
        print()
    finally:
        for k in range(8):
            b.write(0x10 + k, saved[k])
        b.write(0x02, saved_win)
        b.write(0x01, saved_ctrl)
    floor = statistics.median(v for _, v in result if not math.isnan(v))
    out = args.csv or time.strftime("vlf_scan_%Y%m%d_%H%M%S.csv")
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["khz", "level_db"])
        for fk, v in result:
            w.writerow([f"{fk:.4f}", f"{v:.2f}"])
    print(f"noise floor (median) {floor:.1f} dB.  Peaks more than 6 dB above it:")
    for j, (fk, v) in enumerate(result):
        left = result[j - 1][1] if j > 0 else -999
        right = result[j + 1][1] if j + 1 < len(result) else -999
        if v > floor + 6 and v >= left and v >= right:
            near = [(abs(kf - fk), nm) for kf, nm in KNOWN if abs(kf - fk) <= 0.15]
            name = min(near)[1] if near else ""
            bar = "#" * int(min(60, v - floor))
            print(f"  {fk:7.2f} kHz  {v:6.1f} dB  +{v - floor:4.1f}  {bar} {name}")
    print(f"saved {out}")


def flare_class(flux):
    if not flux or flux <= 0:
        return "?"
    for letter, lo in (("X", 1e-4), ("M", 1e-5), ("C", 1e-6), ("B", 1e-7), ("A", 1e-8)):
        if flux >= lo:
            return f"{letter}{flux / lo:.1f}"
    return f"A{flux / 1e-8:.2f}"


class Goes(threading.Thread):
    """Fetches the GOES 0.1-0.8 nm X-ray flux once a minute (NOAA SWPC JSON)."""

    def __init__(self, path):
        super().__init__(daemon=True)
        self.path = path
        self.latest = (None, None)
        self.seen = set()

    def run(self):
        new = not os.path.exists(self.path)
        with open(self.path, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["utc", "flux_w_m2", "class"])
            while True:
                for url in GOES_URLS:
                    try:
                        with urllib.request.urlopen(url, timeout=15) as r:
                            data = json.load(r)
                        rows = [d for d in data if d.get("energy") == "0.1-0.8nm" and d.get("flux")]
                        for d in rows:
                            if d["time_tag"] not in self.seen:
                                self.seen.add(d["time_tag"])
                                w.writerow([d["time_tag"], d["flux"], flare_class(d["flux"])])
                        fh.flush()
                        if rows:
                            self.latest = (rows[-1]["time_tag"], rows[-1]["flux"])
                        break
                    except Exception:
                        continue
                time.sleep(60)


def cmd_log(b, args):
    b.write(0x02, int(round(b.fs)))
    b.write(0x01, 0x3 if not args.no_pps else 0x1)
    cmd_defaults(b)
    day = time.strftime("%Y-%m-%d", time.gmtime())
    out = args.csv or f"vlf_{day}.csv"
    goes = None
    if args.goes:
        goes = Goes(os.path.join(os.path.dirname(os.path.abspath(out)), f"goes_{day}.csv"))
        goes.start()
    new = not os.path.exists(out)
    fh = open(out, "a", newline="")
    w = csv.writer(fh)
    if new:
        cols = ["utc", "unix", "seq", "nsamp", "pps", "clip", "in_rms", "in_dc"]
        for call, khz, _ in STATIONS:
            cols += [f"{call}_db", f"{call}_spike"]
        w.writerow(cols)
    pusher = None
    if getattr(args, "push", None):
        pusher = Pusher(args.push, args.station or "station", "vlf")
        pusher.start()
    print(f"logging to {out}" + (" (+ GOES X-rays)" if goes else "")
          + (f", uploading as '{pusher.station}' to {args.push}" if pusher else "") + ".  Ctrl+C stops.")
    print("utc       " + " ".join(f"{c:>6s}" for c, _, _ in STATIONS) + "   in_rms clip pps")
    good_n = []
    fs_used = b.fs
    last = b.read(0x03) >> 16
    first = True
    anchor = None                      # (utc second, seq) for labelling GPS-second windows
    n_rows = 0
    try:
        while True:
            snap = wait_new_window(b, last, timeout=4.0)
            jump = (snap["seq"] - last) & 0xFFFF
            last = snap["seq"]
            if first:                                   # the first window may be partial
                first = False
                continue
            n_rows += 1
            if getattr(args, "max_rows", None) and n_rows > args.max_rows:
                break
            now = time.time()
            # the time reference for labelling: GPS from the station bridge if we
            # have it (true UTC), otherwise the PC clock (needs NTP).
            ref = now - 0.1
            if getattr(args, "gps_bridge", False):
                gt = b.gps_time()
                if gt and gt["valid"]:
                    try:
                        ref = dt.datetime.strptime(gt["utc"], "%Y-%m-%dT%H:%M:%SZ").replace(
                            tzinfo=dt.timezone.utc).timestamp()
                    except ValueError:
                        pass
            pps_window = (snap["flags"] & 3) == 3
            # A PPS window ends exactly on a GPS second; we see it 0-0.15 s later.
            # Label consecutive PPS windows by counting seconds from an anchor so
            # polling jitter can never duplicate or skip a second; re-anchor if
            # the reference and the count disagree by more than 0.6 s.
            if pps_window:
                if anchor is None or abs(anchor[0] + (snap["seq"] - anchor[1]) - ref) > 0.6:
                    anchor = (round(ref), snap["seq"])
                t_end = anchor[0] + (snap["seq"] - anchor[1])
            else:
                anchor = None
                t_end = round(ref)
            stamp = dt.datetime.fromtimestamp(t_end, dt.timezone.utc)
            # GPS calibration of the sample clock: retune if the station frequencies drift > 0.2 Hz
            if pps_window and abs(snap["n"] - b.fs) < 0.001 * b.fs:
                good_n.append(snap["n"])
                good_n = good_n[-30:]
                if len(good_n) >= 10:
                    fs_est = statistics.mean(good_n)
                    if abs(fs_est - fs_used) * 41.0 / fs_used > 0.0002:
                        fs_used = fs_est
                        for k, (_, khz, _) in enumerate(STATIONS):
                            b.write(0x10 + k, ftw_for(khz, fs_used))
                        print(f"# GPS: sample clock {fs_used:.1f} Hz "
                              f"(crystal {(fs_used / b.fs - 1) * 1e6:+.1f} ppm vs GPS), stations retuned")
            rms, dc = input_stats(snap)
            levels = [amp_db(snap["pow"][k], snap["ndec"]) for k in range(8)]
            spikes = [spike_db(snap["pmax"][k], snap["pow"][k], snap["ndec"]) for k in range(8)]
            row = [stamp.strftime("%Y-%m-%dT%H:%M:%SZ"), f"{t_end:.1f}", snap["seq"], snap["n"],
                   int(pps_window), snap["clip"], f"{rms:.2f}", f"{dc:.2f}"]
            for lv, sp in zip(levels, spikes):
                row += [f"{lv:.2f}", f"{sp:.1f}"]
            w.writerow(row)
            fh.flush()
            if pusher:
                pusher.add({"t": t_end, "pps": int(pps_window), "clip": snap["clip"], "rms": round(rms, 2),
                            "ch": {c: (None if math.isnan(lv) else round(lv, 2))
                                   for (c, _, _), lv in zip(STATIONS, levels)},
                            "spk": {c: (None if math.isnan(sp) else round(sp, 1))
                                    for (c, _, _), sp in zip(STATIONS, spikes)}})
            line = (stamp.strftime("%H:%M:%S  ") + " ".join(f"{lv:6.1f}" for lv in levels)
                    + f"   {rms:6.1f} {snap['clip']:4d} {'yes' if pps_window else 'no'}")
            if jump > 1:
                line += f"  ({jump - 1} window(s) missed)"
            if goes and goes.latest[1]:
                line += f"  GOES {flare_class(goes.latest[1])}"
            if pusher and pusher.last_error and pusher.failed and n_rows % 60 == 0:
                line += f"  [upload: {pusher.failed} failures, {len(pusher.q)} queued: {pusher.last_error[:40]}]"
            print(line)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        fh.close()
        if pusher:
            pusher.flush_once()
            print(f"# uploaded {pusher.sent} rows, {len(pusher.q)} not delivered")


def pps_anchor(b):
    """Map the board's PPS count to UTC: the PPS edge that just happened is the
    PC's current second (the PC clock must be within +/-0.5 s, e.g. NTP-synced).
    Returns (utc_second, pps_cnt)."""
    cnt = b.read(0x0F)
    now = time.time()
    return (math.floor(now), cnt)


def event_time(ev, anchor):
    """absolute time of a sferic event: the GPS second it was in + clocks/50 MHz"""
    sec = anchor[0] + (ev["pps"] - anchor[1])
    return sec + ev["clk"] / CLK_HZ


def refine_arrival(snapshot, pre, thresh):
    """Sub-sample arrival: linear interpolation of the first crossing of +/-thresh
    around the trigger sample (index `pre`). Returns the offset in samples
    relative to the trigger sample (negative = the crossing was earlier)."""
    x = [s - 2048 for s in snapshot]
    i = pre
    while i > 0 and abs(x[i - 1]) >= thresh:
        i -= 1
    if i == 0 or abs(x[i]) < thresh:
        return 0.0
    a, b_ = abs(x[i - 1]), abs(x[i])
    frac = (thresh - a) / (b_ - a) if b_ != a else 1.0
    return (i - 1 + frac) - pre


def cmd_sferics(b, args):
    """Arm the v2 capture and log every lightning stroke with its 20 ns stamp."""
    b.sferic_arm(args.thresh, args.holdoff)
    out = args.csv or f"sferics_{time.strftime('%Y-%m-%d', time.gmtime())}.csv"
    new = not os.path.exists(out)
    fh = open(out, "a", newline="")
    w = csv.writer(fh)
    if new:
        w.writerow(["utc", "unix", "pps_cnt", "clk", "arrival_offset_samples", "peak", "polarity",
                    "seq", "pps_seen", "snapshot"])
    pusher = None
    if getattr(args, "push", None):
        pusher = Pusher(args.push, args.station or "station", "sferic", interval=2.0)
        pusher.start()
    anchor = pps_anchor(b)
    print(f"sferic capture armed: threshold {args.thresh} counts, hold-off {args.holdoff} samples "
          f"({args.holdoff / b.fs * 1000:.1f} ms); logging to {out}.  Ctrl+C stops.")
    print("PPS count now", anchor[1], "(PC clock must be within 0.5 s of UTC for absolute times)")
    n = 0
    try:
        while True:
            waiting = b.sferic_waiting()
            if waiting == 0:
                if getattr(args, "max_events", None) and n >= args.max_events:
                    break
                time.sleep(0.05)
                if n % 50 == 0:
                    anchor = pps_anchor(b)          # follow the PC clock, re-anchor regularly
                continue
            ev = b.sferic_pop()
            n += 1
            t = event_time(ev, anchor)
            off = refine_arrival(ev["snapshot"], ev["pre"], args.thresh)
            t_ref = t + off / b.fs
            stamp = dt.datetime.fromtimestamp(t_ref, dt.timezone.utc)
            w.writerow([stamp.strftime("%Y-%m-%dT%H:%M:%S.") + f"{stamp.microsecond:06d}Z", f"{t_ref:.9f}",
                        ev["pps"], ev["clk"], f"{off:.3f}", ev["peak"], ev["polarity"], ev["seq"],
                        int(ev["pps_seen"]), ";".join(str(s) for s in ev["snapshot"])])
            fh.flush()
            if pusher:
                pusher.add({"t": t_ref, "pps": ev["pps"], "clk": ev["clk"], "off": round(off, 3),
                            "peak": ev["peak"], "pol": ev["polarity"], "seq": ev["seq"],
                            "snap": ev["snapshot"]})
            print(f"{stamp.strftime('%H:%M:%S.%f')}  #{ev['seq']:<6d} peak {ev['peak']:4d} "
                  f"{'+' if ev['polarity'] > 0 else '-'}  clk {ev['clk']:8d} ({ev['clk'] / CLK_HZ * 1e3:8.3f} ms "
                  f"into GPS second {ev['pps']})  waiting {waiting - 1}"
                  + ("" if ev["pps_seen"] else "  NO PPS: stamp is relative to power-up only"))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        fh.close()
        if pusher:
            pusher.flush_once()
        lost = b.read(0x49)
        print(f"# {n} events logged, {lost} lost in the FPGA FIFO" + (f", uploaded {pusher.sent}" if pusher else ""))


def cmd_raw(b, args):
    print("Type commands like  R 03  or  W 01 00000003 ; empty line quits.")
    while True:
        try:
            line = input("vlf> ").strip()
        except EOFError:
            break
        if not line:
            break
        b.s.write((line + "\n").encode())
        print(b.s.readline().decode("ascii", "replace").strip())


def cmd_plot(args):
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        import matplotlib.dates as mdates
    except ImportError:
        sys.exit("matplotlib is missing: run  pip install matplotlib")
    t, cols = [], {}
    with open(args.csv) as fh:
        r = csv.DictReader(fh)
        for row in r:
            t.append(dt.datetime.strptime(row["utc"], "%Y-%m-%dT%H:%M:%SZ"))
            for call, _, _ in STATIONS:
                v = row.get(f"{call}_db", "nan")
                cols.setdefault(call, []).append(float(v) if v not in ("", "nan") else float("nan"))
    n_panels = 2 if args.goes else 1
    fig, axes = plt.subplots(n_panels, 1, sharex=True, figsize=(13, 7 if args.goes else 5),
                             squeeze=False)
    ax = axes[0][0]
    for call, _, _ in STATIONS:
        if call == "REF" and not args.ref:
            continue
        ax.plot(t, cols[call], lw=0.8, label=call)
    ax.set_ylabel("station level, dB re 1 ADC count")
    ax.legend(ncol=8, fontsize=8, loc="upper left")
    ax.grid(alpha=0.3)
    ax.set_title(os.path.basename(args.csv))
    if args.goes:
        gt, gf = [], []
        with open(args.goes) as fh:
            for row in csv.DictReader(fh):
                gt.append(dt.datetime.strptime(row["utc"], "%Y-%m-%dT%H:%M:%SZ"))
                gf.append(float(row["flux_w_m2"]))
        a2 = axes[1][0]
        a2.semilogy(gt, gf, color="k", lw=1)
        for letter, lo in (("A", 1e-8), ("B", 1e-7), ("C", 1e-6), ("M", 1e-5), ("X", 1e-4)):
            a2.axhline(lo, color="r", lw=0.4, alpha=0.5)
            a2.text(t[0] if t else gt[0], lo * 1.3, letter, color="r", fontsize=8)
        a2.set_ylabel("GOES 0.1-0.8 nm, W/m^2")
        a2.grid(alpha=0.3)
        if t:
            a2.set_xlim(t[0] - dt.timedelta(minutes=10), t[-1] + dt.timedelta(minutes=10))
    axes[-1][0].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    axes[-1][0].set_xlabel("UTC")
    fig.tight_layout()
    png = os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(png, dpi=120)
    print(f"saved {png}")
    if not args.no_show:
        plt.show()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "plot":
        p = argparse.ArgumentParser(prog="vlf_host.py plot")
        p.add_argument("cmd")
        p.add_argument("csv")
        p.add_argument("--goes", help="goes_YYYY-MM-DD.csv written by  log --goes")
        p.add_argument("--ref", action="store_true", help="also plot the noise reference channel")
        p.add_argument("--no-show", action="store_true", help="only save the PNG")
        cmd_plot(p.parse_args())
        return
    p = argparse.ArgumentParser(description="freq-lab VLF SID receiver host")
    p.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyUSB0")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("level")
    s = sub.add_parser("scan")
    s.add_argument("--start", type=float, default=10.0, help="kHz (default 10)")
    s.add_argument("--stop", type=float, default=50.0, help="kHz (default 50)")
    s.add_argument("--step", type=float, default=0.1, help="kHz (default 0.1)")
    s.add_argument("--dwell", type=float, default=0.1, help="seconds per step (default 0.1)")
    s.add_argument("--csv")
    lg = sub.add_parser("log")
    lg.add_argument("--csv", help="output file (default vlf_YYYY-MM-DD.csv)")
    lg.add_argument("--goes", action="store_true", help="also log GOES X-ray flux (needs internet)")
    lg.add_argument("--no-pps", action="store_true", help="ignore the GPS PPS input")
    lg.add_argument("--gps-bridge", action="store_true",
                    help="the port is a vlf_station Teensy bridge: stamp windows with its GPS UTC, not the PC clock")
    lg.add_argument("--station", help="this station's name for the server (e.g. HOME)")
    lg.add_argument("--push", help="sid_server.py URL, e.g. http://192.168.1.10:8750")
    lg.add_argument("--max-rows", type=int, help=argparse.SUPPRESS)      # for tests
    sf = sub.add_parser("sferics", help="v2: log lightning strokes with 20 ns GPS stamps")
    sf.add_argument("--thresh", type=int, default=600, help="trigger level in ADC counts (default 600)")
    sf.add_argument("--holdoff", type=int, default=1953, help="dead time in samples (default 1953 = 5 ms)")
    sf.add_argument("--csv", help="output file (default sferics_YYYY-MM-DD.csv)")
    sf.add_argument("--station")
    sf.add_argument("--push")
    sf.add_argument("--max-events", type=int, help=argparse.SUPPRESS)    # for tests
    t = sub.add_parser("tune")
    t.add_argument("channel", type=int)
    t.add_argument("khz", type=float)
    sub.add_parser("defaults", help="put all 8 channels back on the default stations")
    sub.add_parser("raw")
    args = p.parse_args()
    b = Board(args.port)
    if args.cmd == "info":
        cmd_info(b, args)
    elif args.cmd == "level":
        cmd_level(b, args)
    elif args.cmd == "scan":
        cmd_scan(b, args)
    elif args.cmd == "log":
        cmd_log(b, args)
    elif args.cmd == "sferics":
        cmd_sferics(b, args)
    elif args.cmd == "tune":
        cmd_tune(b, args)
    elif args.cmd == "defaults":
        cmd_defaults(b)
        print("channels back on " + ", ".join(c for c, _, _ in STATIONS))
    elif args.cmd == "raw":
        cmd_raw(b, args)


if __name__ == "__main__":
    main()
