#!/usr/bin/env python3
"""
torsion_host.py -- PC side of the 2-liter-bottle torsion pendulum (FOIA list #2).

The ESP32-S3 CAM on the lid measures the rotor angle every frame and prints it;
this program shows it, calibrates the pendulum, runs blind-scheduled sessions,
keeps a tamper-evident log of every session, and does the statistics.

    python torsion_host.py COM8 snap               picture of what the camera sees (aim + check)
    python torsion_host.py COM8 live               live angle, swing and period (Ctrl+C stops)
    python torsion_host.py COM8 calibrate          period, damping, torsion constant, which way is clockwise
    python torsion_host.py COM8 session --kind agent    one 22-minute session (8 PUSH + 8 REST minutes)
    python torsion_host.py COM8 session --kind empty    same schedule, nobody in the room (false-alarm check)
    python torsion_host.py COM8 session --kind warm     warm-water-bottle positive control
    python torsion_host.py report                  every session: chain check, results, combined score, plots
    python torsion_host.py analyze sessions/xxx.csv
    python torsion_host.py simulate                test the statistics on a simulated pendulum

Needs: pip install pyserial numpy pillow matplotlib
"""
import argparse
import base64
import csv
import datetime as dt
import hashlib
import itertools
import json
import math
import os
import secrets
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.getcwd(), "torsion_data")
SESS_DIR = os.path.join(DATA, "sessions")
CHAIN = os.path.join(DATA, "sessions.jsonl")
CAL = os.path.join(DATA, "calibration.json")

ROTOR_I = 1.015e-5         # kg m^2: printed rotor (solid PLA) + two US quarters, computed from the CAD
SETTLE_S, PERIOD_S, N_PERIODS, TAIL_S = 300, 60, 16, 60


# ------------------------------------------------------------------------- serial
class Cam:
    """Reads the camera's lines in a background thread."""

    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is missing: pip install pyserial")
        try:
            self.s = serial.Serial(port, 921600, timeout=0.2)
        except Exception as e:
            sys.exit(f"cannot open {port}: {e}")
        self.lock = threading.Lock()
        self.rows = []            # parsed A-lines: (host_t, ms, frame, angle, rotor, ref, flags, cx, cy, area, thr, acq)
        self.other = []           # everything else (info, errors, snapshots)
        self.lost = 0             # frames where the camera could not find the rotor
        self.keep = True
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        buf = b""
        while self.keep:
            try:
                chunk = self.s.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                if text.startswith("A,"):
                    p = text.split(",")
                    if len(p) == 12:
                        try:
                            v = [time.time()] + [int(x) for x in p[1:]]
                        except ValueError:
                            continue
                        with self.lock:
                            if v[6] & 1:                  # rotor found in this frame
                                self.rows.append(v)
                            else:
                                self.lost += 1
                elif text:
                    with self.lock:
                        self.other.append(text)

    def send(self, cmd):
        self.s.write((cmd + "\n").encode())

    def take(self):
        with self.lock:
            r, self.rows = self.rows, []
        return r

    def take_other(self):
        with self.lock:
            o, self.other = self.other, []
        return o

    def close(self):
        self.keep = False
        time.sleep(0.3)
        self.s.close()


def angle_mrad(row):
    """Pick rotor-minus-reference when the reference squares were found, else the rotor angle."""
    flags = row[6]
    return (row[3] if flags & 2 else row[4]) / 1000.0


def load_cal():
    if os.path.exists(CAL):
        return json.load(open(CAL))
    return None


# ------------------------------------------------------------------------- snap
def cmd_snap(cam, args):
    cam.take_other()
    cam.send("snapfull" if args.full else "snap")
    t0 = time.time()
    header, lines = None, []
    while time.time() - t0 < 30:
        for o in cam.take_other():
            if o.startswith("SNAP "):
                header, lines = o.split(), []
            elif o == "SNAPEND" and header:
                return save_snap(header, lines, args.out)
            elif header:
                lines.append(o)
            elif o.startswith("E,"):
                print(o)
        time.sleep(0.05)
    sys.exit("no picture came back: is the camera running (try: live)?")


def save_snap(h, lines, out):
    from PIL import Image, ImageDraw
    w, hh, f = int(h[1]), int(h[2]), int(h[3])
    rx, ry, hl, hw = (float(v) for v in h[4:8])
    ang = int(h[8]) / 1e6
    nref = int(h[9])
    qx0, qy0, qx1, qy1, qh, px, py, pr = (float(v) for v in h[10:18])
    raw = base64.b64decode("".join(lines))
    img = Image.frombytes("L", (w, hh), raw[: w * hh]).convert("RGB")
    scale = max(1, 640 // w)
    img = img.resize((w * scale, hh * scale), Image.NEAREST)
    d = ImageDraw.Draw(img)
    k = scale / f
    if hl > 0:
        ca, sa = math.cos(ang), math.sin(ang)
        pts = []
        for t in np.linspace(-math.pi / 2, math.pi / 2, 20):        # stadium outline
            pts.append((rx + hl * ca + hw * (ca * math.cos(t) - sa * math.sin(t)),
                        ry + hl * sa + hw * (sa * math.cos(t) + ca * math.sin(t))))
        for t in np.linspace(math.pi / 2, 3 * math.pi / 2, 20):
            pts.append((rx - hl * ca + hw * (ca * math.cos(t) - sa * math.sin(t)),
                        ry - hl * sa + hw * (sa * math.cos(t) + ca * math.sin(t))))
        d.line([(x * k, y * k) for x, y in pts + pts[:1]], fill=(255, 60, 60), width=2)
        d.line([((rx - (hl + hw) * ca) * k, (ry - (hl + hw) * sa) * k), ((rx + (hl + hw) * ca) * k, (ry + (hl + hw) * sa) * k)],
               fill=(255, 200, 0), width=1)
    if nref == 2:
        for qx, qy in ((qx0, qy0), (qx1, qy1)):
            d.rectangle([(qx - qh) * k, (qy - qh) * k, (qx + qh) * k, (qy + qh) * k], outline=(60, 160, 255), width=2)
    if pr > 0:
        d.ellipse([(px - pr) * k, (py - pr) * k, (px + pr) * k, (py + pr) * k], outline=(80, 220, 80), width=1)
    d.text((6, 6), f"rotor region red, axis yellow, reference squares blue ({nref} found), lit paper green", fill=(255, 255, 0))
    out = out or time.strftime("snap_%Y%m%d_%H%M%S.png")
    img.save(out)
    print(f"saved {out}  ({w}x{hh} at 1/{f} resolution)")
    if nref != 2:
        print("reference squares NOT found: check the paper target is in view and the rotor lies across the arrow")
    return out


# ------------------------------------------------------------------------- live
def swing_stats(t, a):
    """amplitude (half peak-to-peak after removing the trend) and period from zero crossings"""
    if len(a) < 20:
        return float("nan"), float("nan")
    p = np.polyfit(t, a, 1)
    r = a - np.polyval(p, t)
    amp = 0.5 * (np.percentile(r, 98) - np.percentile(r, 2))
    s = np.sign(r)
    zc = t[1:][(s[1:] > 0) & (s[:-1] <= 0)]
    per = float(np.median(np.diff(zc))) if len(zc) >= 3 else float("nan")
    return amp, per


def cmd_live(cam, args):
    print("time      angle mrad   swing mrad  period s   fps  refs  (Ctrl+C stops)")
    hist = []
    try:
        while True:
            time.sleep(1.0)
            rows = cam.take()
            for o in cam.take_other():
                if o.startswith(("E,", "I,", "K,")):
                    print("   camera:", o)
            if not rows:
                print(f"   (no rotor in view: {cam.lost} frames without it - run snap)" if cam.lost else "   (no data from the camera)")
                cam.lost = 0
                continue
            hist += [(r[1] / 1000.0, angle_mrad(r), r[6]) for r in rows]
            hist = hist[-1200:]
            t = np.array([h[0] for h in hist]); a = np.array([h[1] for h in hist])
            amp, per = swing_stats(t[-600:], a[-600:])
            if t[-1] - t[0] < 20 or not amp > 1.0:
                per = float("nan")                                   # too little data / swing for a period
            fps = len(rows) / 1.0
            refs = "yes" if rows[-1][6] & 2 else "NO"
            per_s = f"{per:7.2f}" if np.isfinite(per) else "      -"
            print(f"{time.strftime('%H:%M:%S')}  {a[-1]:10.3f}   {amp:9.3f}   {per_s}  {fps:5.1f}  {refs}")
    except KeyboardInterrupt:
        print()


# ------------------------------------------------------------------------- calibrate
def fit_damped(t, a):
    """least squares fit of A exp(-t/tau) cos(w t + phi) + c + d t; returns w, tau, residual rms"""
    from scipy.optimize import least_squares
    t = t - t[0]
    amp, per = swing_stats(t, a)
    if not np.isfinite(per):
        per = 8.0
    p0 = [amp, 400.0, 2 * math.pi / per, 0.0, float(np.mean(a)), 0.0]
    def res(p):
        A, tau, w, ph, c, d = p
        return A * np.exp(-t / max(tau, 1e-3)) * np.cos(w * t + ph) + c + d * t - a
    best = None
    for ph in np.linspace(0, 2 * math.pi, 8, endpoint=False):
        p0[3] = ph
        r = least_squares(res, p0, x_scale="jac", max_nfev=4000)
        if best is None or r.cost < best.cost:
            best = r
    A, tau, w, ph, c, d = best.x
    return abs(w), tau, float(np.sqrt(np.mean(best.fun ** 2))), abs(A), c + d * t.mean()


def beep(kind):
    try:
        import winsound
        if kind == "push":
            winsound.Beep(880, 600)
        elif kind == "rest":
            winsound.Beep(440, 150); time.sleep(0.1); winsound.Beep(440, 150)
        else:
            winsound.Beep(660, 200)
    except Exception:
        sys.stdout.write("\a"); sys.stdout.flush()


def collect(cam, seconds, label=""):
    rows = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(0.5)
        rows += cam.take()
        left = int(t_end - time.time())
        sys.stdout.write(f"\r  {label} {left:4d} s left, {len(rows)} frames   ")
        sys.stdout.flush()
    print()
    return rows


def cmd_calibrate(cam, args):
    os.makedirs(DATA, exist_ok=True)
    cam.take()
    print("Step 1 of 2 - which way is clockwise. Keep away from the bottle.")
    base = collect(cam, 20, "baseline")
    beep("push")
    print(">>> NOW turn the knob ONE big tick CLOCKWISE (10 degrees, seen from above), then step back.")
    turned = collect(cam, 45, "after the turn")
    if len(base) < 20 or len(turned) < 20:
        sys.exit("not enough frames: check `live` first")
    a0 = np.mean([angle_mrad(r) for r in base])
    tt = np.array([r[1] / 1000.0 for r in turned]); aa = np.array([angle_mrad(r) for r in turned])
    late = tt > tt[0] + 10                                       # skip the turn itself; fit the swing around its new centre
    a1 = fit_damped(tt[late], aa[late])[4]
    step = a1 - a0
    cw_sign = 1 if step > 0 else -1
    print(f"  equilibrium moved {step:+.1f} mrad for +10 deg (174.5 mrad) of knob: "
          f"clockwise = {'+' if cw_sign > 0 else '-'} on the camera's scale, coupling {abs(step) / 174.5:.2f}")
    beep("rest")
    print(">>> Turn the knob back ONE tick counterclockwise, then step back. Recording the free swing for "
          f"{args.seconds} s.")
    time.sleep(3)
    swing = collect(cam, args.seconds, "free swing")
    t = np.array([r[1] / 1000.0 for r in swing]); a = np.array([angle_mrad(r) for r in swing])
    keep = t > t[0] + 5
    w, tau, rms, A, _ = fit_damped(t[keep], a[keep])
    T = 2 * math.pi / w
    Q = w * tau / 2
    kappa = ROTOR_I * (w ** 2 + 1 / tau ** 2)
    cal = {"utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"), "cw_sign": cw_sign,
           "step_mrad_per_10deg": step, "period_s": T, "tau_s": tau, "Q": Q, "kappa_Nm_per_rad": kappa,
           "rotor_I_kg_m2": ROTOR_I, "fit_rms_mrad": rms, "swing_mrad": A,
           "noise_mrad_per_frame": float(np.std(np.diff(a)) / math.sqrt(2))}
    json.dump(cal, open(CAL, "w"), indent=1)
    print(f"  period {T:.2f} s, decay time {tau:.0f} s (Q {Q:.1f}), torsion constant {kappa:.3e} N m/rad")
    print(f"  1 mrad of steady deflection = {kappa * 1e-3:.2e} N m of torque; fit residual {rms:.3f} mrad")
    print(f"saved {CAL}")


# ------------------------------------------------------------------------- sessions
def chain_records():
    if not os.path.exists(CHAIN):
        return []
    return [json.loads(l) for l in open(CHAIN) if l.strip()]


def record_hash(rec):
    body = {k: v for k, v in rec.items() if k != "hash"}
    return hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()


def file_sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


PROMPTS = {
    "agent": {"push": "PUSH: will the rotor to turn CLOCKWISE (seen from above). Stay seated.",
              "rest": "REST: relax, think about something else. Stay seated."},
    "warm": {"push": "PUT the warm water bottle on the marked spot (1 m away), then sit back down.",
             "rest": "TAKE the warm bottle away to the far side of the room, then sit back down."},
    "empty": {"push": "", "rest": ""},
}


def cmd_session(cam, args):
    os.makedirs(SESS_DIR, exist_ok=True)
    settle, period, n, tail = (5, 4, 16, 3) if args.fast else (SETTLE_S, PERIOD_S, N_PERIODS, TAIL_S)
    order = ["P"] * (n // 2) + ["R"] * (n // 2)
    for i in range(len(order) - 1, 0, -1):                     # Fisher-Yates with a crypto RNG
        j = secrets.randbelow(i + 1)
        order[i], order[j] = order[j], order[i]
    schedule = "".join(order)
    salt = secrets.token_hex(16)
    commit = hashlib.sha256(f"{schedule}:{salt}".encode()).hexdigest()
    sid = time.strftime("%Y%m%d_%H%M%S") + f"_{args.kind}"
    csv_path = os.path.join(SESS_DIR, sid + ".csv")
    with open(os.path.join(DATA, "commitments.txt"), "a") as f:
        f.write(f"{dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds')} {sid} {commit}\n")
    print(f"session {sid}: {args.kind}, {n} periods of {period} s after {settle} s to settle")
    print(f"schedule fingerprint (SHA-256): {commit}")
    print("  (written to torsion_data/commitments.txt; text it to someone if you want an outside record)")
    if args.kind == "empty":
        print("Leave the room now. The session starts in 120 s and needs nobody nearby until it ends.")
        if not args.fast:
            time.sleep(120)
    cam.take()
    rows_out = []
    phases = [("settle", -1, settle)] + [("push" if c == "P" else "rest", i, period) for i, c in enumerate(schedule)] + \
             [("tail", -1, tail)]
    try:
        for ph, idx, dur in phases:
            if ph in ("push", "rest") and args.kind != "empty":
                beep(ph)
                print(f"\n[{idx + 1:2d}/{n}] {PROMPTS[args.kind][ph]}")
            elif ph == "settle":
                print("\nSettling: sit where you will stay, about 1 m from the bottle. Keep still.")
            elif ph == "tail":
                if args.kind != "empty":
                    beep("end")
                print("\nLast minute: stay put.")
            t_end = time.time() + dur
            while time.time() < t_end:
                time.sleep(0.25)
                for r in cam.take():
                    rows_out.append(r + [ph, idx])
    except KeyboardInterrupt:
        print("\nstopped early: the session is still logged (an aborted session counts in the report)")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_t", "ms", "frame", "angle_urad", "rotor_urad", "ref_urad", "flags", "cx10", "cy10",
                    "area", "thr", "acq", "phase", "period"])
        w.writerows(rows_out)
    prev = chain_records()
    rec = {"id": sid, "kind": args.kind, "utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
           "distance_m": args.distance, "schedule": schedule, "salt": salt, "commit": commit,
           "period_s": period, "settle_s": settle, "csv": os.path.basename(csv_path), "csv_sha256": file_sha(csv_path),
           "frames": len(rows_out), "note": args.note or "", "prev": prev[-1]["hash"] if prev else "genesis"}
    res = analyze_rows(rows_out, schedule, period, load_cal())
    rec["result"] = res
    rec["hash"] = record_hash(rec)
    with open(CHAIN, "a") as f:
        f.write(json.dumps(rec, sort_keys=True) + "\n")
    print_result(sid, args.kind, res)


# ------------------------------------------------------------------------- statistics
_ALL_SCHEDULES = {}


def all_balanced(n):
    if n not in _ALL_SCHEDULES:
        idx = np.array(list(itertools.combinations(range(n), n // 2)))
        m = np.zeros((len(idx), n), bool)
        m[np.arange(len(idx))[:, None], idx] = True
        _ALL_SCHEDULES[n] = m
    return _ALL_SCHEDULES[n]


def analyze_rows(rows, schedule, period_s, cal):
    """Primary: D = mean angle in PUSH minutes - mean in REST minutes, after removing a quadratic
    drift fitted to the whole run, clockwise positive. Exact randomization p over all balanced
    schedules. Secondary: swing amplitude ratio PUSH/REST (two-sided)."""
    n = len(schedule)
    rows = [r for r in rows if r[12] in ("push", "rest") and r[6] & 1]
    if len(rows) < n * 10:
        return {"valid": False, "why": f"only {len(rows)} frames in the periods"}
    ref_frac = float(np.mean([1.0 if r[6] & 2 else 0.0 for r in rows]))
    use_ref = ref_frac >= 0.95
    t = np.array([r[1] / 1000.0 for r in rows])
    a = np.array([(r[3] if use_ref else r[4]) / 1000.0 for r in rows])
    per = np.array([r[13] for r in rows])
    cw = (cal or {}).get("cw_sign", 1)
    a = a * cw
    tt = (t - t.mean()) / (t.std() + 1e-9)
    a_d = a - np.polyval(np.polyfit(tt, a, 2), tt)
    means = np.array([a_d[per == i].mean() if np.any(per == i) else np.nan for i in range(n)])
    if np.any(~np.isfinite(means)):
        return {"valid": False, "why": "a period has no frames"}
    obs = np.array([c == "P" for c in schedule])
    S = all_balanced(n)
    D_all = (S @ means) / (n / 2) - ((~S) @ means) / (n / 2)
    D = float(means[obs].mean() - means[~obs].mean())
    p = float(np.mean(D_all >= D - 1e-12))
    # secondary: swing amplitude per period (detrended within the period)
    amp = []
    for i in range(n):
        m = per == i
        ti, ai = t[m], a_d[m]
        r = ai - np.polyval(np.polyfit(ti - ti.mean(), ai, 1), ti - ti.mean())
        amp.append(np.sqrt(np.mean(r ** 2)))
    amp = np.log(np.array(amp))
    L_all = (S @ amp) / (n / 2) - ((~S) @ amp) / (n / 2)
    L = float(amp[obs].mean() - amp[~obs].mean())
    p_amp = float(np.mean(np.abs(L_all) >= abs(L) - 1e-12))
    from statistics import NormalDist
    z = NormalDist().inv_cdf(min(max(1 - p, 1e-9), 1 - 1e-9))
    kappa = (cal or {}).get("kappa_Nm_per_rad")
    return {"valid": True, "D_mrad": D, "p": p, "z": z, "sd_null_mrad": float(D_all.std()),
            "swing_ratio": float(math.exp(L)), "p_swing": p_amp, "used_reference": use_ref, "ref_fraction": ref_frac,
            "torque_Nm": D * 1e-3 * kappa if kappa else None, "frames": len(rows)}


def print_result(sid, kind, r):
    if not r.get("valid"):
        print(f"{sid}: not analysable ({r.get('why')})")
        return
    tq = f", torque {r['torque_Nm']:+.2e} N m" if r.get("torque_Nm") is not None else ""
    print(f"\n{sid} [{kind}]  PUSH-minus-REST {r['D_mrad']:+.3f} mrad (chance spread {r['sd_null_mrad']:.3f}){tq}")
    print(f"    p = {r['p']:.4f} (one-sided, exact over 12,870 schedules), z = {r['z']:+.2f}")
    print(f"    swing PUSH/REST = {r['swing_ratio']:.2f} (p = {r['p_swing']:.3f}; SRI reported 2:1 to 5:1)")


def analyze_csv(path, schedule, period_s):
    rows = []
    with open(path) as f:
        rd = csv.reader(f)
        next(rd)
        for p in rd:
            rows.append([float(p[0])] + [int(x) for x in p[1:12]] + [p[12], int(p[13])])
    return analyze_rows(rows, schedule, period_s, load_cal())


def cmd_analyze(args):
    recs = {r["csv"]: r for r in chain_records()}
    rec = recs.get(os.path.basename(args.csv))
    if not rec:
        sys.exit("that CSV is not in sessions.jsonl")
    print_result(rec["id"], rec["kind"], analyze_csv(args.csv, rec["schedule"], rec["period_s"]))


def cmd_report(args):
    recs = chain_records()
    if not recs:
        sys.exit("no sessions yet")
    ok = True
    prev = "genesis"
    for r in recs:
        if r["prev"] != prev or record_hash(r) != r["hash"]:
            ok = False
            print(f"CHAIN BROKEN at {r['id']}")
        path = os.path.join(SESS_DIR, r["csv"])
        if not os.path.exists(path) or file_sha(path) != r["csv_sha256"]:
            ok = False
            print(f"DATA FILE CHANGED OR MISSING: {r['csv']}")
        if hashlib.sha256(f"{r['schedule']}:{r['salt']}".encode()).hexdigest() != r["commit"]:
            ok = False
            print(f"SCHEDULE DOES NOT MATCH ITS FINGERPRINT: {r['id']}")
        prev = r["hash"]
    print(f"{len(recs)} sessions; log check {'OK' if ok else 'FAILED (see above)'}: every session counts, "
          f"none can be dropped or edited without this check failing\n")
    from statistics import NormalDist
    print(f"{'session':28s} {'kind':6s} {'D mrad':>8s} {'p':>7s} {'z':>6s} {'swing':>6s}")
    by = {}
    for r in recs:
        res = r["result"]
        if not res.get("valid"):
            print(f"{r['id']:28s} {r['kind']:6s}  not analysable: {res.get('why')}")
            continue
        print(f"{r['id']:28s} {r['kind']:6s} {res['D_mrad']:+8.3f} {res['p']:7.4f} {res['z']:+6.2f} {res['swing_ratio']:6.2f}")
        by.setdefault(r["kind"], []).append(res)
    print()
    for kind, rs in by.items():
        zs = np.array([x["z"] for x in rs])
        Z = zs.sum() / math.sqrt(len(zs))
        p = 1 - NormalDist().cdf(Z)
        Ds = np.array([x["D_mrad"] for x in rs])
        se = Ds.std(ddof=1) / math.sqrt(len(Ds)) if len(Ds) > 1 else float("nan")
        print(f"{kind:6s}: {len(rs):2d} sessions, combined Z = {Z:+.2f} (p = {p:.4f}), mean D = {Ds.mean():+.3f} "
              f"+/- {se:.3f} mrad")
    if args.plot:
        plot_sessions(recs)


def plot_sessions(recs):
    import matplotlib.pyplot as plt
    kinds = sorted({r["kind"] for r in recs})
    fig, ax = plt.subplots(figsize=(9, 4))
    for k, kind in enumerate(kinds):
        Ds = [r["result"]["D_mrad"] for r in recs if r["kind"] == kind and r["result"].get("valid")]
        ax.scatter(np.full(len(Ds), k) + np.random.uniform(-0.12, 0.12, len(Ds)), Ds, s=25)
        if Ds:
            ax.errorbar(k + 0.3, np.mean(Ds), yerr=np.std(Ds, ddof=1) / math.sqrt(len(Ds)) if len(Ds) > 1 else 0,
                        fmt="o", color="k", capsize=4)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xticks(range(len(kinds)))
    ax.set_xticklabels(kinds)
    ax.set_ylabel("PUSH minus REST, mrad (clockwise +)")
    ax.set_title("Torsion pendulum sessions (dots) and mean +/- s.e. (black)")
    fig.tight_layout()
    out = os.path.join(DATA, "report.png")
    fig.savefig(out, dpi=120)
    print(f"saved {out}")


# ------------------------------------------------------------------------- simulation
def simulate_session(rng, schedule, period_s, settle_s, tail_s, effect_mrad=0.0, fps=10.0, T=8.0, Q=15.0,
                     drift_mrad_per_min=0.5, noise_mrad=0.2, torque_noise=0.08):
    """Pendulum driven by random torque + slow drift (+ an optional steady push during PUSH).
    Returns rows in the camera's format, plus phase labels."""
    w0 = 2 * math.pi / T
    dt_ = 1.0 / fps
    phases = [("settle", -1, settle_s)] + [("push" if c == "P" else "rest", i, period_s) for i, c in enumerate(schedule)] + \
             [("tail", -1, tail_s)]
    th, om = 0.0, 0.0
    rows = []
    ms = 0.0
    walk = 0.0
    k = 0
    for ph, idx, dur in phases:
        for _ in range(int(dur * fps)):
            walk += rng.normal(0, drift_mrad_per_min / math.sqrt(60 * fps))
            eq = walk + (effect_mrad if ph == "push" else 0.0)            # equilibrium (mrad)
            for _s in range(10):
                h = dt_ / 10
                acc = -w0 ** 2 * (th - eq) - (w0 / Q) * om + rng.normal(0, torque_noise) / math.sqrt(h)
                om += acc * h
                th += om * h
            ms += dt_ * 1000
            meas = th + rng.normal(0, noise_mrad)
            k += 1
            rows.append([time.time(), int(ms), k, int(meas * 1000), int(meas * 1000), 0, 3, 0, 0, 0, 0, 1, ph, idx])
    return rows


def cmd_simulate(args):
    rng = np.random.default_rng(args.seed)
    n = N_PERIODS
    for effect in (0.0, args.effect):
        ps = []
        for s in range(args.sessions):
            order = list("P" * (n // 2) + "R" * (n // 2))
            rng.shuffle(order)
            sched = "".join(order)
            rows = simulate_session(rng, sched, PERIOD_S, 30, 10, effect_mrad=effect)
            r = analyze_rows(rows, sched, PERIOD_S, {"cw_sign": 1, "kappa_Nm_per_rad": 7.7e-6})
            ps.append(r["p"])
        ps = np.array(ps)
        print(f"injected push effect {effect:4.2f} mrad: {args.sessions} simulated sessions, "
              f"p < 0.05 in {np.mean(ps < 0.05) * 100:.0f} % (expect ~5 % with no effect), "
              f"median p {np.median(ps):.3f}")
        if effect == args.effect:
            break


# ------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="torsion pendulum host")
    ap.add_argument("port", nargs="?", help="serial port of the ESP32-S3 CAM, e.g. COM8")
    ap.add_argument("cmd", choices=["snap", "live", "calibrate", "session", "report", "analyze", "simulate"])
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--full", action="store_true", help="snap: full resolution")
    ap.add_argument("--out")
    ap.add_argument("--seconds", type=int, default=300, help="calibrate: free-swing recording time")
    ap.add_argument("--kind", choices=["agent", "empty", "warm"], default="agent")
    ap.add_argument("--distance", type=float, default=1.0, help="metres from the bottle (agent / warm bottle)")
    ap.add_argument("--note")
    ap.add_argument("--fast", action="store_true", help="session: 4 s periods (for a quick test only)")
    ap.add_argument("--plot", action="store_true", help="report: also save report.png")
    ap.add_argument("--sessions", type=int, default=200, help="simulate: sessions per condition")
    ap.add_argument("--effect", type=float, default=0.3, help="simulate: injected push effect, mrad")
    ap.add_argument("--seed", type=int, default=1)
    # allow "report" / "simulate" / "analyze" without a port
    argv = sys.argv[1:]
    if argv and argv[0] in ("report", "analyze", "simulate"):
        argv = ["-"] + argv
    args = ap.parse_args(argv)
    if args.cmd == "report":
        return cmd_report(args)
    if args.cmd == "analyze":
        return cmd_analyze(args)
    if args.cmd == "simulate":
        return cmd_simulate(args)
    cam = Cam(args.port)
    try:
        {"snap": cmd_snap, "live": cmd_live, "calibrate": cmd_calibrate, "session": cmd_session}[args.cmd](cam, args)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
