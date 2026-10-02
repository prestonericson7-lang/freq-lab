#!/usr/bin/env python3
"""
sid_detect.py -- find sudden ionospheric disturbances (solar flares) in VLF
station logs. Used by sid_server.py live, and standalone over a CSV:

    python sid_detect.py vlf_2026-10-02.csv [--goes goes_2026-10-02.csv]

How a flare looks in the data: a station's level steps up or down by 1-6 dB
over 1-5 minutes, then recovers over 20-90 minutes. Sunrise and sunset swing
the levels by more, but over an hour. Lightning makes single-second spikes.

Detector (per station):
  * only channels that are real signals take part: median level at least
    MIN_SNR_DB (10 dB) above the weakest channel (the empty reference frequency)
  * baseline per channel = the trend of the recent past: medians of two
    half-windows (default 15 min ending 2 min before the sample), extrapolated
    linearly to now, so sunrise/sunset ramps are predicted to within about
    0.6 dB even where they bend, not flagged
  * a channel enters an event when
      |level - baseline| >= DEV_DB (1.5 dB)                         and
      |level(t) - level(t - SLOPE_S)| >= SLOPE_DB (0.8 dB over 3 min: sunrise
      and sunset move about 0.4 dB per 3 min, a flare 1.5-6 dB)
    and stays in it (voting every second) while |deviation| >= DEV_DB. While a
    channel is in an event its samples are NOT learned into the baseline, so
    the slow recovery is measured against the pre-flare level instead of
    reading as a second, negative event. After MAX_ACTIVE_S (2 h) learning
    resumes (a transmitter power change is not a flare).
  * an event needs >= MIN_CH channels (default 2) voting within 120 s of each
    other and the votes sustained for >= SUSTAIN_S (180 s): lightning spikes
    and single-channel glitches cannot pass
  * events less than MERGE_S apart are one event; the peak deviation and the
    channels involved are reported

The GOES 0.1-0.8 nm X-ray flux, when available, confirms an event if the flux
reaches C1 (1e-6 W/m^2) within -5/+15 minutes of the onset. A second station
reporting an onset within 5 minutes makes it multi-station.
"""
import argparse
import bisect
import collections
import csv
import datetime as dt
import math
import statistics
import sys

DEFAULTS = dict(base_win_s=900, lag_s=120, dev_db=1.5, slope_s=180, slope_db=0.8, min_snr_db=10.0,
                onset_s=20, min_ch=2, sustain_s=180, vote_gap_s=120, merge_s=900, max_active_s=7200)
EXCLUDE = {"REF"}


def flare_class(flux):
    if not flux or flux <= 0:
        return "?"
    for letter, lo in (("X", 1e-4), ("M", 1e-5), ("C", 1e-6), ("B", 1e-7), ("A", 1e-8)):
        if flux >= lo:
            return f"{letter}{flux / lo:.1f}"
    return f"A{flux / 1e-8:.2f}"


class RollingMedian:
    """median of the values with t in [t_now - lag - win, t_now - lag]; samples
    arrive in time order. Incremental: O(log n) per sample."""

    def __init__(self, win_s, lag_s):
        self.win, self.lag = win_s, lag_s
        self.pending = collections.deque()      # (t, v) newer than the lag
        self.wt = collections.deque()           # (t, v) inside the window, time order
        self.sv = []                            # the window's values, sorted

    def push(self, t, v):
        self.pending.append((t, v))

    def median(self, t_now):
        cut = t_now - self.lag
        while self.pending and self.pending[0][0] <= cut:
            t, v = self.pending.popleft()
            self.wt.append((t, v))
            bisect.insort(self.sv, v)
        old = cut - self.win
        while self.wt and self.wt[0][0] < old:
            t, v = self.wt.popleft()
            del self.sv[bisect.bisect_left(self.sv, v)]
        if len(self.sv) < 20:
            return None
        return self.sv[len(self.sv) // 2]


class TrendBaseline:
    """Expected level now, from the trend of the recent past: the medians of
    two half-windows (older half A, newer half B, both ending before a lag)
    give a level and a slope, extrapolated to now. A sunrise or sunset ramp
    (about 0.2 dB/min, slowly curving) is predicted to within about 0.7 dB; a
    flare's 1-5 dB step in a few minutes is not, and shows as the deviation."""

    def __init__(self, win_s, lag_s):
        half = win_s / 2.0
        self.half, self.lag = half, lag_s
        self.a = RollingMedian(half, lag_s + half)       # [t-lag-win, t-lag-win/2]
        self.b = RollingMedian(half, lag_s)              # [t-lag-win/2, t-lag]

    def push(self, t, v):
        self.a.push(t, v)
        self.b.push(t, v)

    def value(self, t):
        ma, mb = self.a.median(t), self.b.median(t)
        if mb is None:
            return None
        if ma is None:
            return mb
        slope = (mb - ma) / self.half                     # dB per second, centre A to centre B
        return mb + slope * (self.lag + self.half / 2.0)  # from B's centre to now


def detect(rows, channels=None, **kw):
    """rows: list of (t_unix, {call: level_db or None}) in time order, one station.
    Returns a list of events: dict(t_onset, t_peak, t_end, dev_db, channels, n_ch, sign)."""
    p = dict(DEFAULTS)
    p.update(kw)
    if not rows:
        return []
    all_ch = [c for c in rows[0][1].keys()]
    chans = channels or [c for c in all_ch if c not in EXCLUDE]
    # A channel takes part only if it is a real signal: its median level must be
    # MIN_SNR_DB above the weakest channel's median (the empty reference
    # frequency, or whatever is nearest the noise).
    medians = {}
    for c in all_ch:
        vals = sorted(v[1][c] for v in rows if v[1].get(c) is not None and not math.isnan(v[1][c]))
        if vals:
            medians[c] = vals[len(vals) // 2]
    if not medians:
        return []
    floor_db = min(medians.values())
    chans = [c for c in chans if c in medians and medians[c] >= floor_db + p["min_snr_db"]]
    # sample cadence (for the vote count that "sustained" requires)
    gaps = sorted(rows[i + 1][0] - rows[i][0] for i in range(min(len(rows) - 1, 5000)))
    dt_s = gaps[len(gaps) // 2] if gaps else 1.0
    need_votes = max(2, int(0.8 * p["sustain_s"] / max(dt_s, 1e-3)))
    onset_n = max(3, int(round(p["onset_s"] / max(dt_s, 1e-3))))
    med = {c: TrendBaseline(p["base_win_s"], p["lag_s"]) for c in chans}
    hist = {c: collections.deque() for c in chans}      # (t, level) of the last slope_s (+5 s)
    recent = {c: collections.deque(maxlen=onset_n) for c in chans}   # last onset_s deviations
    pending = {c: None for c in chans}                   # t_first of a possible onset
    active = {c: None for c in chans}                    # onset time while a channel is in an event
    frozen = {c: None for c in chans}                    # baseline value held during an event
    votes = []                                           # (t, channel, smoothed deviation, onset)
    for t, lv in rows:
        for c in chans:
            v = lv.get(c)
            if v is None or (isinstance(v, float) and math.isnan(v)):
                continue
            h = hist[c]
            h.append((t, v))
            while h and h[0][0] < t - p["slope_s"] - 5:
                h.popleft()
            base = frozen[c] if active[c] is not None else med[c].value(t)
            learn = True
            if base is not None:
                dev = v - base
                r = recent[c]
                r.append(dev)
                mean = sum(r) / len(r)                   # over the last onset_s: spikes shrink 20x
                if active[c] is None:
                    if pending[c] is None:
                        # a possible onset: a deviation AND a fast change (sunrise and
                        # sunset are predicted by the trend baseline; what is left of
                        # them moves well under 0.8 dB per 3 min)
                        enough_hist = h[0][0] <= t - 0.5 * p["slope_s"]
                        if abs(dev) >= p["dev_db"] and enough_hist and abs(v - h[0][1]) >= p["slope_db"]:
                            pending[c] = t
                            r.clear()
                            r.append(dev)
                    elif len(r) >= onset_n:
                        # confirmed when the mean deviation over onset_s is still past
                        # the threshold and most seconds agree in sign: a one-second
                        # lightning spike averages down to nothing
                        sgn = 1 if mean > 0 else -1
                        agree = sum(1 for d in r if (d > 0) == (sgn > 0)) / len(r)
                        if abs(mean) >= p["dev_db"] and agree >= 0.75:
                            active[c] = pending[c]
                            frozen[c] = base
                        pending[c] = None
                else:
                    # the event ends when the smoothed level is back near the baseline
                    # (hysteresis at 70 % of the threshold), or after max_active_s
                    # (a real level change: start learning again)
                    if abs(mean) < 0.7 * p["dev_db"] or t - active[c] > p["max_active_s"]:
                        active[c] = None
                        frozen[c] = None
                if active[c] is not None:
                    votes.append((t, c, mean, active[c]))
                    learn = False                        # keep the baseline where it was
            if learn:
                med[c].push(t, v)
    if not votes:
        return []
    # group votes into candidate events: consecutive votes within vote_gap_s
    events = []
    cur = None
    for t, c, dev, onset in votes:
        if cur is None or t - cur["t_last"] > p["vote_gap_s"]:
            if cur:
                events.append(cur)
            cur = dict(t_last=t, by_ch={}, onset_ch={}, t_peak=t, dev_db=dev)
        cur["t_last"] = t
        cur["by_ch"].setdefault(c, []).append((t, dev))
        cur["onset_ch"][c] = min(onset, cur["onset_ch"].get(c, onset))
        if abs(dev) > abs(cur["dev_db"]):
            cur["dev_db"], cur["t_peak"] = dev, t
    if cur:
        events.append(cur)
    # keep the ones with enough channels voting for long enough (a count of
    # samples, so a few spikes spread over minutes cannot pass)
    out = []
    for e in events:
        good = [c for c, lst in e["by_ch"].items() if len(lst) >= need_votes]
        if len(good) < p["min_ch"]:
            continue
        onset = min(e["onset_ch"][c] for c in good)
        out.append(dict(t_onset=onset, t_peak=e["t_peak"], t_end=e["t_last"], dev_db=round(e["dev_db"], 2),
                        channels=sorted(good), n_ch=len(good), sign=1 if e["dev_db"] > 0 else -1))
    # merge events closer than merge_s
    merged = []
    for e in sorted(out, key=lambda x: x["t_onset"]):
        if merged and e["t_onset"] - merged[-1]["t_end"] < p["merge_s"]:
            m = merged[-1]
            m["t_end"] = max(m["t_end"], e["t_end"])
            if abs(e["dev_db"]) > abs(m["dev_db"]):
                m["dev_db"], m["t_peak"] = e["dev_db"], e["t_peak"]
            m["channels"] = sorted(set(m["channels"]) | set(e["channels"]))
            m["n_ch"] = len(m["channels"])
        else:
            merged.append(e)
    return merged


def goes_confirm(events, goes, before_s=300, after_s=900):
    """goes: list of (t_unix, flux). Adds goes_max, goes_class, goes_confirmed to each event."""
    gt = [g[0] for g in goes]
    for e in events:
        i0 = bisect.bisect_left(gt, e["t_onset"] - before_s)
        i1 = bisect.bisect_right(gt, e["t_onset"] + after_s)
        fl = [goes[i][1] for i in range(i0, i1) if goes[i][1]]
        e["goes_max"] = max(fl) if fl else None
        e["goes_class"] = flare_class(e["goes_max"]) if fl else "no data"
        e["goes_confirmed"] = bool(fl) and e["goes_max"] >= 1e-6
    return events


def cross_station(events_by_station, window_s=300):
    """Mark events seen by more than one station within window_s of each other."""
    flat = [(e["t_onset"], st, e) for st, evs in events_by_station.items() for e in evs]
    for t, st, e in flat:
        others = sorted({s2 for t2, s2, _ in flat if s2 != st and abs(t2 - t) <= window_s})
        e["other_stations"] = others
        e["multi_station"] = bool(others)
    return events_by_station


# ---------------------------------------------------------------------- CSV front end
def read_vlf_csv(path):
    rows = []
    calls = None
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if calls is None:
                calls = [k[:-3] for k in r.keys() if k.endswith("_db")]
            lv = {}
            for c in calls:
                v = r.get(f"{c}_db", "")
                lv[c] = float(v) if v not in ("", "nan") else None
            rows.append((float(r["unix"]), lv))
    return rows


def read_goes_csv(path):
    out = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t = dt.datetime.strptime(r["utc"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=dt.timezone.utc).timestamp()
            out.append((t, float(r["flux_w_m2"])))
    return sorted(out)


def fmt(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


def main():
    ap = argparse.ArgumentParser(description="find solar-flare SIDs in a vlf_host.py log")
    ap.add_argument("csv")
    ap.add_argument("--goes", help="goes_YYYY-MM-DD.csv from  vlf_host.py log --goes")
    ap.add_argument("--dev", type=float, default=DEFAULTS["dev_db"], help="dB deviation to vote (1.5)")
    ap.add_argument("--min-ch", type=int, default=DEFAULTS["min_ch"], help="channels needed (2)")
    a = ap.parse_args()
    rows = read_vlf_csv(a.csv)
    print(f"{len(rows)} rows, {fmt(rows[0][0])} .. {fmt(rows[-1][0])} UTC")
    ev = detect(rows, dev_db=a.dev, min_ch=a.min_ch)
    if a.goes:
        goes_confirm(ev, read_goes_csv(a.goes))
    if not ev:
        print("no events")
        return
    print(f"{'onset (UTC)':20s} {'peak':9s} {'dev dB':>7s}  ch  channels                 GOES")
    for e in ev:
        print(f"{fmt(e['t_onset']):20s} {fmt(e['t_peak'])[11:]:9s} {e['dev_db']:+7.2f}  {e['n_ch']:2d}  "
              f"{','.join(e['channels']):24s} {e.get('goes_class', '')}"
              + ("  CONFIRMED" if e.get("goes_confirmed") else ""))


if __name__ == "__main__":
    main()
