#!/usr/bin/env python3
"""
schumann_host.py -- PC side of teensy/schumann_monitor (and elf_logger's mode columns).

    python schumann_host.py COM8 log [--report 60] [--station HOME --push http://server:8750]
          sets `json on`, starts the monitor, appends every report to schumann_YYYY-MM-DD.csv,
          prints a one-line summary per report, uploads to sid_server.py as kind "schumann"
    python schumann_host.py COM8 live        one spectrum from the board, plotted (matplotlib)
    python schumann_host.py plot schumann_2026-10-02.csv
          24 h+ history: the five modes' amplitude and centre frequency, the E/H impedance

The daily Schumann power cycle (peaks when the big thunderstorm regions have
their afternoons: ~08, ~14 and ~20 UTC) is the first thing a customer sees in
the plot; the centre frequencies move by a few tenths of a hertz over the day
and with solar activity.
"""
import argparse
import collections
import csv
import datetime as dt
import json
import os
import sys
import threading
import time
import urllib.request

MODES = 5


class Link:
    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is missing: run  pip install pyserial")
        self.s = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def cmd(self, c):
        self.s.write((c + "\n").encode())
        time.sleep(0.05)

    def lines(self):
        while True:
            raw = self.s.readline()
            if not raw:
                yield None
                continue
            yield raw.decode("ascii", "replace").strip()


class Pusher(threading.Thread):
    def __init__(self, url, station):
        super().__init__(daemon=True)
        self.url = url.rstrip("/") + "/api/ingest"
        self.station = station
        self.q = collections.deque(maxlen=20000)
        self.lock = threading.Lock()
        self.sent = 0

    def add(self, row):
        with self.lock:
            self.q.append(row)

    def flush(self):
        with self.lock:
            batch = [self.q.popleft() for _ in range(min(200, len(self.q)))]
        if not batch:
            return True
        body = json.dumps({"station": self.station, "kind": "schumann", "rows": batch}).encode()
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

    def run(self):
        while True:
            time.sleep(10)
            self.flush()


def parse_report(line):
    """a JSON report line from the monitor -> flat dict, or None"""
    if not line.startswith("{"):
        return None
    try:
        d = json.loads(line)
    except ValueError:
        return None
    if "modes" not in d:
        return None
    row = {"t_board": d.get("t", ""), "nseg": d.get("nseg", 0), "Z1": d.get("Z1"), "coh1": d.get("coh1"),
           "Z2": d.get("Z2"), "coh2": d.get("coh2"), "floor30_pT": d.get("floor30_pT"), "drops": d.get("drops", 0)}
    for i, m in enumerate(d["modes"][:MODES], 1):
        row[f"m{i}_Hz"] = m.get("f")
        row[f"m{i}_H_pT"] = m.get("H_pT")
        row[f"m{i}_Hx"] = m.get("Hx")
        row[f"m{i}_E_uVm"] = m.get("E_uVm")
        row[f"m{i}_Ex"] = m.get("Ex")
    return row


COLS = ["utc", "unix", "t_board", "nseg"] + [f"m{i}_{k}" for i in range(1, MODES + 1) for k in ("Hz", "H_pT", "Hx", "E_uVm", "Ex")] \
    + ["Z1", "coh1", "Z2", "coh2", "floor30_pT", "drops"]


def cmd_log(a):
    link = Link(a.port)
    out = a.csv or f"schumann_{time.strftime('%Y-%m-%d', time.gmtime())}.csv"
    new = not os.path.exists(out)
    fh = open(out, "a", newline="")
    w = csv.DictWriter(fh, fieldnames=COLS)
    if new:
        w.writeheader()
    pusher = None
    if a.push:
        pusher = Pusher(a.push, a.station or "station")
        pusher.start()
    link.cmd("json on")
    link.cmd(f"start {a.report}")
    print(f"logging reports every {a.report} s to {out}" + (f", uploading to {a.push}" if pusher else "") + ".  Ctrl+C stops.")
    print("utc        m1 Hz   m1 pT  x  | m2 Hz   m2 pT  | m3 pT | Z1 ohm coh")
    n = 0
    try:
        for line in link.lines():
            if line is None:
                continue
            if line.startswith("#"):
                print(line)
                continue
            row = parse_report(line)
            if not row:
                continue
            now = time.time()
            row["utc"] = dt.datetime.fromtimestamp(now, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            row["unix"] = f"{now:.0f}"
            w.writerow(row)
            fh.flush()
            n += 1
            if pusher:
                pusher.add({"t": now, **{k: row[k] for k in COLS if k not in ("utc", "unix", "t_board")}})
            print(f"{row['utc'][11:19]}  {row['m1_Hz']:6.2f} {row['m1_H_pT']:7.3f} {row['m1_Hx']:4.1f} | "
                  f"{row['m2_Hz']:6.2f} {row['m2_H_pT']:7.3f} | {row['m3_H_pT']:6.3f} | {row['Z1']:6.0f} {row['coh1']:.2f}")
            if a.max_reports and n >= a.max_reports:
                break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        link.cmd("stop")
        fh.close()
        if pusher:
            pusher.flush()
            print(f"# uploaded {pusher.sent} reports")


def cmd_live(a):
    link = Link(a.port)
    link.cmd("live")
    f, h, e = [], [], []
    t0 = time.time()
    for line in link.lines():
        if line is None:
            if time.time() - t0 > 5:
                break
            continue
        if line.startswith("#"):
            if "no finished segment" in line:
                sys.exit("the board has no spectrum yet: wait 17 s after `start`")
            continue
        p = line.split(",")
        if len(p) == 3:
            try:
                f.append(float(p[0])); h.append(float(p[1])); e.append(float(p[2]))
            except ValueError:
                pass
        if f and len(f) >= 750:
            break
    if not f:
        sys.exit("no spectrum received")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
    ax[0].semilogy(f, h, lw=0.8); ax[0].set_ylabel("H, pT/rtHz"); ax[0].grid(alpha=0.3)
    ax[1].semilogy(f, e, lw=0.8, color="g"); ax[1].set_ylabel("E, uV/m/rtHz"); ax[1].set_xlabel("Hz"); ax[1].grid(alpha=0.3)
    for m in (7.83, 14.3, 20.8, 27.3, 33.8):
        ax[0].axvline(m, color="r", lw=0.4, alpha=0.5)
    ax[0].set_title("live spectrum")
    fig.tight_layout()
    plt.show()


def cmd_plot(a):
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    t, rows = [], []
    with open(a.csv) as fh:
        for r in csv.DictReader(fh):
            t.append(dt.datetime.strptime(r["utc"], "%Y-%m-%dT%H:%M:%SZ"))
            rows.append(r)
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    for i in range(1, MODES + 1):
        ax[0].plot(t, [float(r[f"m{i}_H_pT"]) for r in rows], lw=0.8, label=f"mode {i}")
        ax[1].plot(t, [float(r[f"m{i}_Hz"]) for r in rows], lw=0.8)
    ax[0].set_ylabel("H, pT/rtHz"); ax[0].legend(ncol=5, fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("centre Hz"); ax[1].grid(alpha=0.3)
    ax[2].plot(t, [float(r["Z1"]) for r in rows], lw=0.8, label="Z at mode 1")
    ax[2].plot(t, [float(r["Z2"]) for r in rows], lw=0.8, label="Z at mode 2")
    ax[2].set_ylabel("E/H impedance, ohm"); ax[2].legend(); ax[2].grid(alpha=0.3)
    ax[2].xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
    fig.suptitle(os.path.basename(a.csv))
    fig.tight_layout()
    png = os.path.splitext(a.csv)[0] + ".png"
    fig.savefig(png, dpi=120)
    print("saved", png)
    if not a.no_show:
        plt.show()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "plot":
        p = argparse.ArgumentParser(prog="schumann_host.py plot")
        p.add_argument("cmd")
        p.add_argument("csv")
        p.add_argument("--no-show", action="store_true")
        cmd_plot(p.parse_args())
        return
    p = argparse.ArgumentParser(description="Schumann monitor host")
    p.add_argument("port")
    sub = p.add_subparsers(dest="cmd", required=True)
    lg = sub.add_parser("log")
    lg.add_argument("--report", type=int, default=60)
    lg.add_argument("--csv")
    lg.add_argument("--station")
    lg.add_argument("--push")
    lg.add_argument("--max-reports", type=int, help=argparse.SUPPRESS)
    sub.add_parser("live")
    a = p.parse_args()
    if a.cmd == "log":
        cmd_log(a)
    else:
        cmd_live(a)


if __name__ == "__main__":
    main()
