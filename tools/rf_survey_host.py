#!/usr/bin/env python3
"""
rf_survey_host.py -- PC side of teensy/rf_survey (AD8318 power meter).

    python rf_survey_host.py COM9 survey [--csv walk.csv] [--seconds 600]
          GPS-tagged level once a second -> CSV (time, dBm mean/peak, uW/cm2, duty, lat, lon)
    python rf_survey_host.py COM9 spec fast 20        envelope spectrum 12 Hz-25 kHz, plotted, lines named
    python rf_survey_host.py COM9 spec slow 60        0.06-125 Hz (Wi-Fi beacon line at 9.77 Hz)
    python rf_survey_host.py COM9 pulses 30           pulse-interval histogram, named
    python rf_survey_host.py map walk.csv             the walk as a map coloured by uW/cm2 (or vs time without GPS)
"""
import argparse
import csv
import datetime as dt
import os
import sys
import time


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

    def readline(self):
        raw = self.s.readline()
        return raw.decode("ascii", "replace").strip() if raw else None


def parse_survey_line(line):
    """time,mean_dbm,peak_dbm,uw_cm2,duty[,lat,lon,fix] -> dict or None"""
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 5:
        return None
    try:
        d = dict(time=p[0], mean_dbm=float(p[1]), peak_dbm=float(p[2]), uw_cm2=float(p[3]), duty=float(p[4]))
        if len(p) >= 8:
            d.update(lat=float(p[5]), lon=float(p[6]), fix=int(p[7]))
        else:
            d.update(lat=0.0, lon=0.0, fix=0)
        return d
    except ValueError:
        return None


def cmd_survey(a):
    link = Link(a.port)
    out = a.csv or f"survey_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["utc", "board_time", "mean_dbm", "peak_dbm", "uw_cm2", "duty", "lat", "lon", "fix"])
        link.cmd("survey on")
        print(f"survey -> {out}  (Ctrl+C stops)")
        t0 = time.time()
        n = 0
        try:
            while True:
                line = link.readline()
                d = parse_survey_line(line) if line else None
                if not d:
                    if a.seconds and time.time() - t0 > a.seconds:
                        break
                    continue
                n += 1
                w.writerow([dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"), d["time"], d["mean_dbm"],
                            d["peak_dbm"], d["uw_cm2"], d["duty"], d["lat"], d["lon"], d["fix"]])
                fh.flush()
                print(f"{d['time']:>22s} {d['mean_dbm']:7.1f} dBm  peak {d['peak_dbm']:6.1f}  {d['uw_cm2']:9.4f} uW/cm2  "
                      f"duty {d['duty']:.2f}  {'fix' if d['fix'] else 'no GPS'} {d['lat']:.5f},{d['lon']:.5f}")
                if a.seconds and time.time() - t0 > a.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            link.cmd("survey off")
    print(f"{n} rows in {out}")
    return out


def collect_spectrum(link, header_tag, timeout=30):
    """read 'f,dB' lines after a '# ... spectrum' header until the '# strongest lines' block ends"""
    f, db, names = [], [], []
    t0 = time.time()
    state = 0
    while time.time() - t0 < timeout:
        line = link.readline()
        if line is None:
            continue
        if line.startswith("#"):
            if "spectrum" in line and state == 0:
                state = 1
            elif "strongest lines" in line:
                state = 2
            elif state == 2 and line.startswith("#   "):
                names.append(line[1:].strip())
            elif state == 2 and names:
                break
            continue
        if state == 1 and "," in line:
            p = line.split(",")
            try:
                f.append(float(p[0])); db.append(float(p[1]))
            except ValueError:
                pass
        if state == 2 and not line.startswith("#"):
            break
    return f, db, names


def cmd_spec(a):
    link = Link(a.port)
    link.cmd(f"spec {a.which} {a.seconds}")
    print(f"collecting the {a.which} spectrum for {a.seconds} s ...")
    f, db, names = collect_spectrum(link, "spectrum", timeout=a.seconds + 40)
    if not f:
        sys.exit("no spectrum received")
    print("strongest lines:")
    for n in names:
        print("  " + n)
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(f, db, lw=0.7)
        ax.set_xscale("log")
        ax.set_xlabel("modulation frequency, Hz"); ax.set_ylabel("dB rel. strongest line")
        ax.set_title(f"RF envelope spectrum ({a.which})"); ax.grid(alpha=0.3, which="both")
        fig.tight_layout()
        png = f"envelope_{a.which}_{time.strftime('%Y%m%d_%H%M%S')}.png"
        fig.savefig(png, dpi=120)
        print("saved", png)
        if not a.no_show:
            plt.show()
    except ImportError:
        print("(matplotlib missing: no plot)")


def cmd_pulses(a):
    link = Link(a.port)
    link.cmd(f"pulses {a.seconds}")
    t0 = time.time()
    while time.time() - t0 < a.seconds + 30:
        line = link.readline()
        if line is None:
            continue
        print(line)
        if line.startswith("#   ") and "," in line:
            continue
        if "likely source" in line:
            pass


def cmd_map(a):
    import matplotlib.pyplot as plt
    rows = []
    with open(a.csv) as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
    if not rows:
        sys.exit("empty CSV")
    uw = [float(r["uw_cm2"]) for r in rows]
    fixes = [r for r in rows if int(r.get("fix", 0))]
    fig, ax = plt.subplots(figsize=(9, 7))
    if len(fixes) >= 5:
        lat = [float(r["lat"]) for r in fixes]; lon = [float(r["lon"]) for r in fixes]
        c = [float(r["uw_cm2"]) for r in fixes]
        import math
        sc = ax.scatter(lon, lat, c=[math.log10(max(v, 1e-6)) for v in c], cmap="inferno", s=18)
        cb = fig.colorbar(sc); cb.set_label("log10 uW/cm2")
        ax.set_xlabel("longitude"); ax.set_ylabel("latitude"); ax.set_aspect(1.0 / max(1e-6, math.cos(math.radians(sum(lat) / len(lat)))))
        ax.set_title(f"{os.path.basename(a.csv)}: {len(fixes)} GPS points, max {max(c):.3f} uW/cm2")
    else:
        ax.semilogy(range(len(uw)), [max(v, 1e-6) for v in uw], lw=0.8)
        ax.set_xlabel("seconds"); ax.set_ylabel("uW/cm2")
        ax.set_title(f"{os.path.basename(a.csv)}: no GPS fix, level vs time")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    png = os.path.splitext(a.csv)[0] + "_map.png"
    fig.savefig(png, dpi=120)
    print("saved", png)
    if not a.no_show:
        plt.show()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "map":
        p = argparse.ArgumentParser(prog="rf_survey_host.py map")
        p.add_argument("cmd"); p.add_argument("csv"); p.add_argument("--no-show", action="store_true")
        cmd_map(p.parse_args()); return
    p = argparse.ArgumentParser(description="rf_survey host")
    p.add_argument("port")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("survey"); s.add_argument("--csv"); s.add_argument("--seconds", type=int)
    s = sub.add_parser("spec"); s.add_argument("which", choices=["fast", "slow"]); s.add_argument("seconds", type=int); s.add_argument("--no-show", action="store_true")
    s = sub.add_parser("pulses"); s.add_argument("seconds", type=int)
    a = p.parse_args()
    {"survey": cmd_survey, "spec": cmd_spec, "pulses": cmd_pulses}[a.cmd](a)


if __name__ == "__main__":
    main()
