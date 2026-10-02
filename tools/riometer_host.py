#!/usr/bin/env python3
"""
riometer_host.py -- cosmic-noise absorption (riometer) and solar radio burst
logger on an RTL-SDR, through the rtl_power tool that ships with rtl-sdr.

A riometer listens to the galactic radio background at about 30 MHz. The sky
noise repeats every sidereal day (23 h 56 min 04 s). When a solar flare's
X-rays (or a polar-cap proton event) ionise the D region, the noise coming
down through it is absorbed: the received power dips below the quiet-day
curve by 0.1-5 dB. That dip is the measurement. At 245 MHz the same tool
logs solar radio bursts, which show as power ABOVE the quiet curve.

    python riometer_host.py run  --freq 30.0 --span 1.0 --gain 40 --int 60
                                 [--station HOME --push http://server:8750]
    python riometer_host.py run  --freq 245.0 --span 2.0 --mode burst ...
    python riometer_host.py replay rtl_power_log.csv [--freq 30.0]   (offline)
    python riometer_host.py plot riometer_2026-10-02.csv

`run` starts  rtl_power -f <lo>:<hi>:<bin> -g <gain> -i <int> -e 1000h <tmp.csv>
and reads its output as it grows. Each integration gives one band power:
the linear mean of the bins (strong narrowband interferers, i.e. bins more
than 6 dB above the band median, are dropped first).

Quiet-day curve (QDC): for every sidereal minute of the day keep the
upper quartile of the powers seen at that sidereal minute over the last
--qdc-days days (default 7). Absorption = QDC - power (positive = absorbed).
Before a full sidereal day of data exists the QDC is the running median of
the last 2 hours (a coarse stand-in, marked in the CSV). Burst mode reports
enhancement = power - QDC instead.

Antenna: a half-wave dipole for 30 MHz is 4.7 m per leg; a 2-element Yagi or
a turnstile improves it; a 245 MHz Yagi has 0.58 m elements. Keep the SDR
gain below the point where any bin clips (the tool warns) and away from
switching supplies: the galactic noise is only a few dB above the SDR's own.
"""
import argparse
import bisect
import collections
import csv
import datetime as dt
import json
import math
import os
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

SIDEREAL_DAY = 86164.0905          # seconds


def sidereal_minute(t_unix):
    """minute of the sidereal day at Greenwich, 0..1435 (phase is arbitrary but constant)"""
    return int((t_unix % SIDEREAL_DAY) / SIDEREAL_DAY * 1436) % 1436


def band_power_db(bins_db, reject_db=6.0):
    """linear mean of the bins after dropping narrowband interferers"""
    vals = [v for v in bins_db if v is not None and math.isfinite(v)]
    if not vals:
        return None
    med = statistics.median(vals)
    keep = [v for v in vals if v <= med + reject_db]
    lin = sum(10 ** (v / 10.0) for v in keep) / len(keep)
    return 10.0 * math.log10(lin)


class QuietDayCurve:
    """upper-quartile power per sidereal minute over the last n days"""

    def __init__(self, days=7):
        self.days = days
        self.hist = collections.defaultdict(list)       # minute -> [(t, p)]
        self.n = 0
        self.t_first = None

    def add(self, t, p):
        self.n += 1
        if self.t_first is None:
            self.t_first = t
        m = sidereal_minute(t)
        lst = self.hist[m]
        lst.append((t, p))
        cutoff = t - self.days * SIDEREAL_DAY
        while lst and lst[0][0] < cutoff:
            lst.pop(0)

    def complete(self, t):
        return self.t_first is not None and t - self.t_first >= SIDEREAL_DAY

    def value(self, t):
        """QDC at time t from neighbouring sidereal minutes (+/-2) of previous days"""
        m = sidereal_minute(t)
        vals = []
        for dm in (-2, -1, 0, 1, 2):
            for tt, p in self.hist[(m + dm) % 1436]:
                if t - tt > 3600:                       # previous days only, not this pass
                    vals.append(p)
        if len(vals) < 3:
            return None
        vals.sort()
        return vals[int(len(vals) * 0.75)]


class Processor:
    """turns rtl_power rows into band powers, QDC, absorption; logs and uploads"""

    def __init__(self, mode="absorb", qdc_days=7, csv_path=None, pusher=None, station="station", freq_mhz=30.0):
        self.mode = mode
        self.qdc = QuietDayCurve(qdc_days)
        self.recent = collections.deque()               # (t, p) last 2 h, for the stand-in baseline
        self.csv_path = csv_path
        self.fh = self.w = None
        self.pusher = pusher
        self.station = station
        self.freq = freq_mhz
        self.rows = []

    def open(self):
        if self.csv_path:
            new = not os.path.exists(self.csv_path)
            self.fh = open(self.csv_path, "a", newline="")
            self.w = csv.writer(self.fh)
            if new:
                self.w.writerow(["utc", "unix", "freq_mhz", "power_db", "quiet_db", "quiet_source",
                                 "absorption_db" if self.mode == "absorb" else "enhancement_db", "nbins"])

    def feed(self, t, bins_db):
        p = band_power_db(bins_db)
        if p is None:
            return None
        self.recent.append((t, p))
        while self.recent and self.recent[0][0] < t - 7200:
            self.recent.popleft()
        q = self.qdc.value(t)
        src = "qdc"
        if q is None:
            src = "running-median"
            vals = sorted(v for _, v in self.recent)
            q = vals[len(vals) // 2] if len(vals) >= 5 else None
        self.qdc.add(t, p)
        delta = None if q is None else ((q - p) if self.mode == "absorb" else (p - q))
        row = dict(t=t, freq_mhz=self.freq, power_db=round(p, 3), quiet_db=None if q is None else round(q, 3),
                   quiet_source=src, absorption_db=None if delta is None else round(delta, 3), nbins=len(bins_db))
        self.rows.append(row)
        if self.w:
            stamp = dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            self.w.writerow([stamp, f"{t:.1f}", self.freq, row["power_db"], row["quiet_db"], src,
                             row["absorption_db"], len(bins_db)])
            self.fh.flush()
        if self.pusher:
            self.pusher.add({"t": t, "freq_mhz": self.freq, "power_db": row["power_db"],
                             "quiet_db": row["quiet_db"], "absorption_db": row["absorption_db"]})
        return row


def parse_rtl_power_line(line):
    """rtl_power CSV: date, time, hz_low, hz_high, hz_step, samples, db, db, ... -> (t, bins)"""
    p = [x.strip() for x in line.split(",")]
    if len(p) < 7:
        return None
    try:
        t = dt.datetime.strptime(p[0] + " " + p[1], "%Y-%m-%d %H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
        bins = [float(x) for x in p[6:] if x not in ("", "nan", "-nan")]
    except ValueError:
        return None
    return t, bins


class Pusher:
    def __init__(self, url, station, kind="riometer"):
        self.url = url.rstrip("/") + "/api/ingest"
        self.station, self.kind = station, kind
        self.q = collections.deque(maxlen=50000)
        self.lock = threading.Lock()
        self.sent = 0

    def add(self, row):
        with self.lock:
            self.q.append(row)

    def flush(self):
        with self.lock:
            batch = [self.q.popleft() for _ in range(min(500, len(self.q)))]
        if not batch:
            return True
        body = json.dumps({"station": self.station, "kind": self.kind, "rows": batch}).encode()
        try:
            req = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=15) as r:
                r.read()
            self.sent += len(batch)
            return True
        except Exception:
            with self.lock:
                self.q.extendleft(reversed(batch))
            return False


def cmd_run(a):
    lo, hi = a.freq - a.span / 2, a.freq + a.span / 2
    tmp = os.path.join(tempfile.gettempdir(), f"rtl_power_{int(time.time())}.csv")
    cmd = ["rtl_power", "-f", f"{lo}M:{hi}M:{a.bin}k", "-g", str(a.gain), "-i", str(a.int), "-e", "1000h", tmp]
    if a.device:
        cmd[1:1] = ["-d", str(a.device)]
    print("starting:", " ".join(cmd))
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        sys.exit("rtl_power not found: install the rtl-sdr tools (it is in the same package as rtl_sdr/rtl_fm)")
    pusher = Pusher(a.push, a.station) if a.push else None
    out = a.csv or f"riometer_{time.strftime('%Y-%m-%d', time.gmtime())}.csv"
    proc_ = Processor(a.mode, a.qdc_days, out, pusher, a.station, a.freq)
    proc_.open()
    print(f"{'absorption' if a.mode == 'absorb' else 'burst'} mode at {a.freq} MHz, {a.span} MHz span, "
          f"{a.int} s integrations -> {out}.  Ctrl+C stops.")
    pos = 0
    last_push = time.time()
    try:
        while proc.poll() is None:
            if os.path.exists(tmp):
                with open(tmp) as fh:
                    fh.seek(pos)
                    chunk = fh.read()
                    pos = fh.tell()
                # rtl_power writes one line per hop; a sweep of one hop = one row
                for line in chunk.splitlines():
                    r = parse_rtl_power_line(line)
                    if not r:
                        continue
                    row = proc_.feed(*r)
                    if row:
                        q = row["quiet_db"]
                        print(f"{dt.datetime.fromtimestamp(row['t'], dt.timezone.utc).strftime('%H:%M:%S')}  "
                              f"power {row['power_db']:7.2f} dB  quiet {q if q is None else f'{q:7.2f}'} "
                              f"({row['quiet_source']})  {'absorption' if a.mode == 'absorb' else 'enhancement'} "
                              f"{row['absorption_db'] if row['absorption_db'] is None else f'{row['absorption_db']:+6.2f}'} dB"
                              + ("  <-- bins clipping, lower the gain" if max(r[1]) > -5 else ""))
            if pusher and time.time() - last_push > 10:
                pusher.flush()
                last_push = time.time()
            time.sleep(1.0)
        err = proc.stderr.read()
        print("rtl_power exited:", err[-400:])
    except KeyboardInterrupt:
        proc.terminate()
        print("\nstopped")
    finally:
        if pusher:
            pusher.flush()


def cmd_replay(a):
    pusher = Pusher(a.push, a.station) if a.push else None
    out = a.csv or os.path.splitext(a.file)[0] + "_riometer.csv"
    p = Processor(a.mode, a.qdc_days, out, pusher, a.station, a.freq)
    p.open()
    n = 0
    with open(a.file) as fh:
        for line in fh:
            r = parse_rtl_power_line(line)
            if r:
                p.feed(*r)
                n += 1
    if pusher:
        while pusher.q:
            if not pusher.flush():
                break
    print(f"{n} integrations -> {out}")
    return p


def cmd_plot(a):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.dates as mdates
    except ImportError:
        sys.exit("matplotlib is missing: run  pip install matplotlib")
    t, pw, q, ab = [], [], [], []
    with open(a.file) as fh:
        for r in csv.DictReader(fh):
            t.append(dt.datetime.fromtimestamp(float(r["unix"]), dt.timezone.utc))
            pw.append(float(r["power_db"]))
            q.append(float(r["quiet_db"]) if r["quiet_db"] not in ("", "None") else float("nan"))
            k = "absorption_db" if "absorption_db" in r else "enhancement_db"
            ab.append(float(r[k]) if r[k] not in ("", "None") else float("nan"))
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    ax[0].plot(t, pw, lw=0.8, label="band power")
    ax[0].plot(t, q, lw=0.8, label="quiet-day curve")
    ax[0].set_ylabel("dB")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].plot(t, ab, lw=0.8, color="r")
    ax[1].set_ylabel("absorption dB")
    ax[1].grid(alpha=0.3)
    ax[1].xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
    fig.tight_layout()
    png = os.path.splitext(a.file)[0] + ".png"
    fig.savefig(png, dpi=120)
    print("saved", png)
    if not a.no_show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="RTL-SDR riometer / solar burst logger")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("run", "replay"):
        s = sub.add_parser(name)
        if name == "replay":
            s.add_argument("file", help="an rtl_power CSV")
        s.add_argument("--freq", type=float, default=30.0, help="centre MHz (30 riometer, 245 bursts)")
        s.add_argument("--span", type=float, default=1.0, help="MHz")
        s.add_argument("--bin", type=float, default=10.0, help="kHz per bin")
        s.add_argument("--gain", type=float, default=40.0)
        s.add_argument("--int", type=int, default=60, help="integration seconds")
        s.add_argument("--device", type=int)
        s.add_argument("--mode", choices=["absorb", "burst"], default="absorb")
        s.add_argument("--qdc-days", type=int, default=7)
        s.add_argument("--csv")
        s.add_argument("--station", default="station")
        s.add_argument("--push")
    pl = sub.add_parser("plot")
    pl.add_argument("file")
    pl.add_argument("--no-show", action="store_true")
    a = ap.parse_args()
    if a.cmd == "run":
        cmd_run(a)
    elif a.cmd == "replay":
        cmd_replay(a)
    else:
        cmd_plot(a)


if __name__ == "__main__":
    main()
