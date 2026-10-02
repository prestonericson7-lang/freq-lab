#!/usr/bin/env python3
"""
sid_server.py -- central server + live dashboard for a network of freq-lab
space-weather stations (VLF SID receivers, lightning sferic stamps, riometers).

    python tools/sid_server.py                      # http://localhost:8750, data in sid_data.sqlite
    python tools/sid_server.py --port 8750 --goes   # also fetch the GOES X-ray flux every minute

Stations upload with  vlf_host.py COM7 log --station HOME --push http://server:8750
(and  sferics --push,  riometer_host.py run --push). Nothing but the Python
standard library is needed.

What it does
  * stores every row in SQLite (one file), per station and kind
  * every 60 s runs the flare detector (sid_detect.py) over the last 4 h of each
    VLF station, confirms with GOES when the flux reached C1, flags events seen by
    more than one station, and keeps an alert list
  * matches lightning stamps across stations (same stroke within 3 ms) for a
    time-of-arrival network
  * serves a self-contained dashboard (dashboard/index.html) and a JSON API:
      POST /api/ingest        {"station": "HOME", "kind": "vlf"|"sferic"|"riometer"|..., "rows": [...]}
      GET  /api/status
      GET  /api/series?station=HOME&from=<unix>&to=<unix>&step=10
      GET  /api/sferics?station=HOME&from=&to=&bin=60     (counts per bin; raw if span < 1 h)
      GET  /api/toa?from=&to=                              (strokes seen by >= 2 stations)
      GET  /api/goes?from=&to=
      GET  /api/alerts?from=&to=
      GET  /api/riometer?station=&from=&to=
      GET  /api/generic?station=&kind=schumann&from=&to=     (rows of any other kind, as uploaded)
"""
import argparse
import http.server
import json
import os
import sqlite3
import sys
import threading
import time
import urllib.parse
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sid_detect  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
GOES_URLS = [
    "https://services.swpc.noaa.gov/json/goes/primary/xrays-6-hour.json",
    "https://services.swpc.noaa.gov/json/goes/primary/xrays-1-day.json",
]

SCHEMA = """
CREATE TABLE IF NOT EXISTS vlf (station TEXT, t REAL, pps INTEGER, clip INTEGER, rms REAL,
    ch TEXT, spk TEXT, PRIMARY KEY (station, t));
CREATE TABLE IF NOT EXISTS sferic (station TEXT, t REAL, pps INTEGER, clk INTEGER, off REAL,
    peak INTEGER, pol INTEGER, seq INTEGER, snap TEXT, PRIMARY KEY (station, t, seq));
CREATE TABLE IF NOT EXISTS riometer (station TEXT, t REAL, freq_mhz REAL, power_db REAL,
    quiet_db REAL, absorption_db REAL, PRIMARY KEY (station, t, freq_mhz));
CREATE TABLE IF NOT EXISTS generic (station TEXT, kind TEXT, t REAL, data TEXT,
    PRIMARY KEY (station, kind, t));
CREATE TABLE IF NOT EXISTS goes (t REAL PRIMARY KEY, flux REAL);
CREATE TABLE IF NOT EXISTS alerts (id INTEGER PRIMARY KEY AUTOINCREMENT, station TEXT, t_onset REAL,
    t_peak REAL, t_end REAL, dev_db REAL, channels TEXT, n_ch INTEGER, sign INTEGER,
    goes_class TEXT, goes_confirmed INTEGER, multi_station INTEGER, other_stations TEXT, updated REAL);
CREATE TABLE IF NOT EXISTS stations (station TEXT PRIMARY KEY, last_seen REAL, rows INTEGER, kinds TEXT);
CREATE INDEX IF NOT EXISTS vlf_t ON vlf (t);
CREATE INDEX IF NOT EXISTS sferic_t ON sferic (t);
CREATE INDEX IF NOT EXISTS alerts_t ON alerts (t_onset);
"""


class Store:
    def __init__(self, path):
        self.path = path
        self.lock = threading.RLock()
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self.conn.executescript(SCHEMA)
        self.conn.commit()

    def ingest(self, station, kind, rows):
        n = 0
        with self.lock:
            c = self.conn
            for r in rows:
                try:
                    if kind == "vlf":
                        c.execute("INSERT OR REPLACE INTO vlf VALUES (?,?,?,?,?,?,?)",
                                  (station, float(r["t"]), int(r.get("pps", 0)), int(r.get("clip", 0)),
                                   float(r.get("rms", 0) or 0), json.dumps(r.get("ch", {})),
                                   json.dumps(r.get("spk", {}))))
                    elif kind == "sferic":
                        c.execute("INSERT OR REPLACE INTO sferic VALUES (?,?,?,?,?,?,?,?,?)",
                                  (station, float(r["t"]), int(r.get("pps", 0)), int(r.get("clk", 0)),
                                   float(r.get("off", 0)), int(r.get("peak", 0)), int(r.get("pol", 0)),
                                   int(r.get("seq", 0)), json.dumps(r.get("snap", []))))
                    elif kind == "riometer":
                        c.execute("INSERT OR REPLACE INTO riometer VALUES (?,?,?,?,?,?)",
                                  (station, float(r["t"]), float(r.get("freq_mhz", 0)), float(r.get("power_db", 0)),
                                   None if r.get("quiet_db") is None else float(r["quiet_db"]),
                                   None if r.get("absorption_db") is None else float(r["absorption_db"])))
                    else:
                        c.execute("INSERT OR REPLACE INTO generic VALUES (?,?,?,?)",
                                  (station, kind, float(r["t"]), json.dumps(r)))
                    n += 1
                except (KeyError, ValueError, TypeError):
                    continue
            row = c.execute("SELECT rows, kinds FROM stations WHERE station=?", (station,)).fetchone()
            kinds = set(json.loads(row[1])) if row else set()
            kinds.add(kind)
            c.execute("INSERT OR REPLACE INTO stations VALUES (?,?,?,?)",
                      (station, time.time(), (row[0] if row else 0) + n, json.dumps(sorted(kinds))))
            c.commit()
        return n

    def q(self, sql, args=()):
        with self.lock:
            return self.conn.execute(sql, args).fetchall()

    def status(self):
        st = [dict(station=s, last_seen=ls, rows=n, kinds=json.loads(k))
              for s, ls, n, k in self.q("SELECT * FROM stations ORDER BY station")]
        g = self.q("SELECT t, flux FROM goes ORDER BY t DESC LIMIT 1")
        na = self.q("SELECT COUNT(*) FROM alerts WHERE t_onset > ?", (time.time() - 86400,))[0][0]
        return dict(time=time.time(), stations=st,
                    goes=dict(t=g[0][0], flux=g[0][1], cls=sid_detect.flare_class(g[0][1])) if g else None,
                    alerts_24h=na)

    def series(self, station, t0, t1, step):
        rows = self.q("SELECT t, ch, clip, rms FROM vlf WHERE station=? AND t>=? AND t<=? ORDER BY t",
                      (station, t0, t1))
        step = max(1, int(step))
        out = dict(t=[], clip=[], rms=[], ch={})
        for i, (t, ch, clip, rms) in enumerate(rows):
            if i % step:
                continue
            out["t"].append(t)
            out["clip"].append(clip)
            out["rms"].append(rms)
            for call, v in json.loads(ch).items():
                out["ch"].setdefault(call, []).append(v)
        return out

    def vlf_rows(self, station, t0, t1):
        return [(t, json.loads(ch)) for t, ch in
                self.q("SELECT t, ch FROM vlf WHERE station=? AND t>=? AND t<=? ORDER BY t", (station, t0, t1))]

    def sferics(self, station, t0, t1, bin_s):
        where = "t>=? AND t<=?" + (" AND station=?" if station else "")
        args = (t0, t1) + ((station,) if station else ())
        rows = self.q(f"SELECT station, t, peak, pol FROM sferic WHERE {where} ORDER BY t", args)
        bins = {}
        for s, t, pk, pol in rows:
            b = int(t // bin_s) * bin_s
            bins[b] = bins.get(b, 0) + 1
        out = dict(bins=sorted(bins.items()), total=len(rows))
        if t1 - t0 <= 3600:
            out["events"] = [dict(station=s, t=t, peak=pk, pol=pol) for s, t, pk, pol in rows]
        return out

    def toa(self, t0, t1, window=0.003):
        """strokes seen by two or more stations within `window` seconds"""
        rows = self.q("SELECT station, t, peak, pol FROM sferic WHERE t>=? AND t<=? ORDER BY t", (t0, t1))
        groups, cur = [], []
        for r in rows:
            if cur and r[1] - cur[0][1] > window:
                if len({x[0] for x in cur}) >= 2:
                    groups.append(cur)
                cur = []
            cur.append(r)
        if cur and len({x[0] for x in cur}) >= 2:
            groups.append(cur)
        out = []
        for g in groups:
            first = min(g, key=lambda x: x[1])
            out.append(dict(t=first[1], n=len(g),
                            arrivals={s: dict(dt_us=round((t - first[1]) * 1e6, 2), peak=pk, pol=pol)
                                      for s, t, pk, pol in g}))
        return out

    def goes(self, t0, t1):
        return self.q("SELECT t, flux FROM goes WHERE t>=? AND t<=? ORDER BY t", (t0, t1))

    def riometer(self, station, t0, t1):
        return self.q("SELECT t, freq_mhz, power_db, quiet_db, absorption_db FROM riometer "
                      "WHERE station=? AND t>=? AND t<=? ORDER BY t", (station, t0, t1))

    def generic(self, station, kind, t0, t1):
        """rows of any other kind (schumann, ...) as the dicts that were uploaded"""
        return [json.loads(d) for (d,) in
                self.q("SELECT data FROM generic WHERE station=? AND kind=? AND t>=? AND t<=? ORDER BY t",
                       (station, kind, t0, t1))]

    def alerts(self, t0, t1):
        cols = ["id", "station", "t_onset", "t_peak", "t_end", "dev_db", "channels", "n_ch", "sign",
                "goes_class", "goes_confirmed", "multi_station", "other_stations", "updated"]
        rows = self.q("SELECT * FROM alerts WHERE t_onset>=? AND t_onset<=? ORDER BY t_onset DESC", (t0, t1))
        out = []
        for r in rows:
            d = dict(zip(cols, r))
            d["channels"] = json.loads(d["channels"])
            d["other_stations"] = json.loads(d["other_stations"] or "[]")
            out.append(d)
        return out

    def upsert_alert(self, station, e):
        with self.lock:
            row = self.conn.execute("SELECT id FROM alerts WHERE station=? AND ABS(t_onset-?) < 600",
                                    (station, e["t_onset"])).fetchone()
            vals = (e["t_onset"], e["t_peak"], e["t_end"], e["dev_db"], json.dumps(e["channels"]), e["n_ch"],
                    e["sign"], e.get("goes_class", "no data"), int(bool(e.get("goes_confirmed"))),
                    int(bool(e.get("multi_station"))), json.dumps(e.get("other_stations", [])), time.time())
            if row:
                self.conn.execute("UPDATE alerts SET t_onset=?, t_peak=?, t_end=?, dev_db=?, channels=?, n_ch=?, "
                                  "sign=?, goes_class=?, goes_confirmed=?, multi_station=?, other_stations=?, "
                                  "updated=? WHERE id=?", vals + (row[0],))
            else:
                self.conn.execute("INSERT INTO alerts (station, t_onset, t_peak, t_end, dev_db, channels, n_ch, "
                                  "sign, goes_class, goes_confirmed, multi_station, other_stations, updated) "
                                  "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)", (station,) + vals)
            self.conn.commit()

    def add_goes(self, rows):
        with self.lock:
            self.conn.executemany("INSERT OR REPLACE INTO goes VALUES (?,?)", rows)
            self.conn.commit()


# ---------------------------------------------------------------------- background work
def run_detector(store, span_s=4 * 3600):
    """detector over the last span_s of every VLF station; returns the number of events"""
    now = time.time()
    found = {}
    for st in store.q("SELECT station FROM stations"):
        station = st[0]
        rows = store.vlf_rows(station, now - span_s, now)
        if len(rows) < 600:
            continue
        ev = sid_detect.detect(rows)
        goes = store.goes(now - span_s - 3600, now)
        if goes:
            sid_detect.goes_confirm(ev, goes)
        found[station] = ev
    sid_detect.cross_station(found)
    n = 0
    for station, evs in found.items():
        for e in evs:
            store.upsert_alert(station, e)
            n += 1
    return n


def detector_loop(store, interval=60):
    while True:
        try:
            run_detector(store)
        except Exception as e:                       # keep serving whatever happens
            print("detector:", e)
        time.sleep(interval)


def goes_loop(store, interval=60):
    import datetime as dt
    while True:
        for url in GOES_URLS:
            try:
                with urllib.request.urlopen(url, timeout=15) as r:
                    data = json.load(r)
                rows = []
                for d in data:
                    if d.get("energy") == "0.1-0.8nm" and d.get("flux"):
                        t = dt.datetime.strptime(d["time_tag"], "%Y-%m-%dT%H:%M:%SZ").replace(
                            tzinfo=dt.timezone.utc).timestamp()
                        rows.append((t, float(d["flux"])))
                store.add_goes(rows)
                break
            except Exception:
                continue
        time.sleep(interval)


# ---------------------------------------------------------------------- HTTP
class Handler(http.server.BaseHTTPRequestHandler):
    store = None
    dash = ""

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        qs = {k: v[0] for k, v in urllib.parse.parse_qs(u.query).items()}
        now = time.time()
        t0 = float(qs.get("from", now - 86400))
        t1 = float(qs.get("to", now))
        p = u.path
        try:
            if p in ("/", "/index.html"):
                body = self.dash.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif p == "/api/status":
                self._json(self.store.status())
            elif p == "/api/series":
                self._json(self.store.series(qs.get("station", ""), t0, t1, qs.get("step", 1)))
            elif p == "/api/sferics":
                self._json(self.store.sferics(qs.get("station"), t0, t1, float(qs.get("bin", 60))))
            elif p == "/api/toa":
                self._json(self.store.toa(t0, t1, float(qs.get("window", 0.003))))
            elif p == "/api/goes":
                self._json(self.store.goes(t0, t1))
            elif p == "/api/alerts":
                self._json(self.store.alerts(t0, t1))
            elif p == "/api/riometer":
                self._json(self.store.riometer(qs.get("station", ""), t0, t1))
            elif p == "/api/generic":
                self._json(self.store.generic(qs.get("station", ""), qs.get("kind", "schumann"), t0, t1))
            else:
                self._json({"error": "unknown path"}, 404)
        except Exception as e:
            self._json({"error": str(e)}, 500)

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/api/ingest":
            return self._json({"error": "unknown path"}, 404)
        n = int(self.headers.get("Content-Length", 0))
        try:
            msg = json.loads(self.rfile.read(n))
            station = str(msg["station"])[:32]
            kind = str(msg.get("kind", "vlf"))[:16]
            rows = msg["rows"]
            if not isinstance(rows, list):
                raise ValueError("rows must be a list")
        except Exception as e:
            return self._json({"error": f"bad request: {e}"}, 400)
        stored = self.store.ingest(station, kind, rows)
        self._json({"ok": True, "stored": stored})

    def log_message(self, fmt, *a):
        line = fmt % a
        if "/api/ingest" not in line:                   # one upload per second per station: too chatty
            sys.stderr.write("%s - %s\n" % (self.address_string(), line))

    def do_HEAD(self):
        self.send_response(200)
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()


def serve(port, db, dash_path, goes, bind="0.0.0.0", detector=True):
    store = Store(db)
    Handler.store = store
    Handler.dash = open(dash_path, encoding="utf-8").read() if os.path.exists(dash_path) else \
        "<p>dashboard/index.html not found next to this server</p>"
    if goes:
        threading.Thread(target=goes_loop, args=(store,), daemon=True).start()
    if detector:
        threading.Thread(target=detector_loop, args=(store,), daemon=True).start()
    srv = http.server.ThreadingHTTPServer((bind, port), Handler)
    return srv, store


def main():
    ap = argparse.ArgumentParser(description="freq-lab space-weather station server")
    ap.add_argument("--port", type=int, default=8750)
    ap.add_argument("--db", default="sid_data.sqlite")
    ap.add_argument("--dash", default=os.path.join(HERE, "..", "dashboard", "index.html"))
    ap.add_argument("--goes", action="store_true", help="fetch the GOES X-ray flux every minute")
    ap.add_argument("--bind", default="0.0.0.0")
    a = ap.parse_args()
    srv, store = serve(a.port, a.db, a.dash, a.goes, a.bind)
    print(f"sid_server on http://localhost:{a.port}/  (data: {a.db}, GOES {'on' if a.goes else 'off'})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
