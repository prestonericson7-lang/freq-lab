#!/usr/bin/env python3
"""
check_vlf.py -- verify the vlf_sid simulation against the reference models.

Reads sim_log.txt written by tb_vlf.v:
  S code dec close run close_pps close_to   one line per ADC sample (stage A)
  D <I bus hex> <Q bus hex>                  one line per decimated output
  W seq n ndec clip min max sq sum start_pps end_pps end_to   window latched
  C ch pow pmax                              channel results of that window
  R tag STATUS SEQ NSAMP NDEC FLAGS CLIP XMINMAX SQ_LO SQ_HI SUM_LO SUM_HI (UART reads)
  P tag ch POW_HI POW_LO PMAX                                              (UART reads)

Checks:
  1. every decimated I/Q of all 8 channels == bit-exact integer model
  2. every latched window (sample stats, NDEC, per-channel power and max)
     == values recomputed from the logged samples
  3. every register read over the UART == the latched window it reports
  4. the integer receiver agrees with an ideal floating-point receiver
  5. physics: tone amplitudes, adjacent-signal rejection, noise floor
"""
import math
import sys

import numpy as np

from vlf_model import (STATIONS, FS_NOM, R, K_AMP, ftw_for, channel_int,
                       channel_float, amp_counts)

LOG = sys.argv[1] if len(sys.argv) > 1 else "sim_log.txt"
NCH = 8
FTWS = [ftw_for(khz * 1000.0) for _, khz, _ in STATIONS]

fails = 0
passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


def s24(v):
    v &= 0xFFFFFF
    return v - (1 << 24) if v & 0x800000 else v


# ------------------------------------------------------------------ parse
samples, decs, wins, regs, pregs = [], [], [], {}, {}
gtimes, trigs, events = [], [], []          # sferic capture: PPS edges, triggers, events read
cur_w = None
ended = False
with open(LOG) as f:
    for line in f:
        p = line.split()
        if not p:
            continue
        if p[0] == "END":
            ended = True
            continue
        if p[0] == "G":
            gtimes.append(int(p[1]))
            continue
        if p[0] == "T":
            trigs.append((int(p[1]), int(p[2])))
            continue
        if p[0] == "E":
            events.append(dict(tag=int(p[1]), nev=int(p[2]),
                               pps=int(p[3], 16), clk=int(p[4], 16), info=int(p[5], 16), seq=int(p[6], 16),
                               words=[int(x, 16) for x in p[7:39]]))
            continue
        if ended:
            continue
        if p[0] == "S":
            samples.append(tuple(int(x) for x in p[1:7]))
        elif p[0] == "D":
            ib, qb = int(p[1], 16), int(p[2], 16)
            decs.append(([s24(ib >> (24 * k)) for k in range(NCH)],
                         [s24(qb >> (24 * k)) for k in range(NCH)]))
        elif p[0] == "W":
            v = [int(x) for x in p[1:]]
            cur_w = dict(seq=v[0], n=v[1], ndec=v[2], clip=v[3], min=v[4], max=v[5],
                         sq=v[6], sum=v[7], start_pps=v[8], end_pps=v[9], end_to=v[10],
                         pow=[None] * NCH, pmax=[None] * NCH)
            wins.append(cur_w)
        elif p[0] == "C":
            k, pw, pm = int(p[1]), int(p[2]), int(p[3])
            cur_w["pow"][k] = pw
            cur_w["pmax"][k] = pm
        elif p[0] == "R":
            regs[int(p[1])] = [int(x, 16) for x in p[2:]]
        elif p[0] == "P":
            pregs.setdefault(int(p[1]), {})[int(p[2])] = tuple(int(x, 16) for x in p[3:6])

codes = [s[0] for s in samples]
print(f"log: {len(samples)} samples, {len(decs)} decimated outputs, {len(wins)} windows, "
      f"{len(regs)} UART snapshots")

# ------------------------------------------------------------------ 1. bit-exact DDC
print("\n[1] decimated I/Q vs bit-exact model")
model = [channel_int(codes, FTWS[k]) for k in range(NCH)]
dec_n = [n for (n, _, _) in model[0]]
check(all(samples[n][1] == 1 for n in dec_n) and sum(s[1] for s in samples) == len(dec_n),
      f"decimation flags on samples n = 1023 mod 1024 ({len(dec_n)} of them)")
nd = min(len(decs), len(dec_n))
check(len(decs) in (len(dec_n), len(dec_n) - 1),
      f"decimated output count {len(decs)} (model {len(dec_n)})")
bad = 0
for k in range(NCH):
    for j in range(nd):
        _, mi, mq = model[k][j]
        if decs[j][0][k] != mi or decs[j][1][k] != mq:
            bad += 1
            if bad <= 5:
                print(f"    mismatch ch{k} dec#{j}: sim ({decs[j][0][k]}, {decs[j][1][k]}) "
                      f"model ({mi}, {mq})")
check(bad == 0, f"all {nd * NCH} I/Q values identical (8 channels x {nd})")

# ------------------------------------------------------------------ 2. window sums
print("\n[2] latched windows vs recomputed sums")
pw_of_dec = {n: [mi * mi + mq * mq for (mi, mq) in [(model[k][j][1], model[k][j][2]) for k in range(NCH)]]
             for j, n in enumerate(dec_n)}
windows = []          # recomputed: list of dicts in close order
cur = None


def new_acc():
    return dict(n=0, clip=0, ndec=0, sq=0, sum=0, min=4095, max=0, start_pps=0,
                pow=[0] * NCH, pmax=[0] * NCH, first=None)


acc = new_acc()
for i, (code, dec, close, run, cpps, cto) in enumerate(samples):
    xs = code - 2048
    if not run:
        acc = new_acc()
        continue
    if close:
        done = acc
        done.update(end_pps=cpps, end_to=cto, last=i - 1)
        windows.append(done)
        acc = new_acc()
        acc["start_pps"] = cpps
    if acc["first"] is None:
        acc["first"] = i
    acc["n"] += 1
    acc["clip"] += code in (0, 4095)
    acc["sq"] += xs * xs
    acc["sum"] += xs
    acc["min"] = min(acc["min"], code)
    acc["max"] = max(acc["max"], code)
    if dec:
        acc["ndec"] += 1
        if i in pw_of_dec:
            for k in range(NCH):
                pk = pw_of_dec[i][k]
                acc["pow"][k] += pk
                acc["pmax"][k] = max(acc["pmax"][k], pk)

check(len(windows) == len(wins), f"{len(wins)} windows latched, {len(windows)} expected")
ok = True
for w_sim, w_ref in zip(wins, windows):
    for key in ("n", "ndec", "clip", "min", "max", "sq", "sum", "start_pps", "end_pps", "end_to"):
        if w_sim[key] != w_ref[key]:
            ok = False
            print(f"    window {w_sim['seq']}: {key} sim {w_sim[key]} ref {w_ref[key]}")
    for k in range(NCH):
        if w_sim["pow"][k] != w_ref["pow"][k] or w_sim["pmax"][k] != w_ref["pmax"][k]:
            ok = False
            print(f"    window {w_sim['seq']} ch{k}: pow {w_sim['pow'][k]} / {w_ref['pow'][k]}, "
                  f"pmax {w_sim['pmax'][k]} / {w_ref['pmax'][k]}")
check(ok, "every window: N, NDEC, CLIP, min/max, sum x^2, sum x, flags, 8x power, 8x max")
for w in wins:
    print(f"    window {w['seq']}: N={w['n']:6d} NDEC={w['ndec']:3d} CLIP={w['clip']} "
          f"pps start/end={w['start_pps']}/{w['end_pps']} timeout={w['end_to']}")

# ------------------------------------------------------------------ 3. UART registers
print("\n[3] UART snapshot registers vs latched windows")
by_seq = {w["seq"]: w for w in wins}
ok = True
for tag, r in sorted(regs.items()):
    status, seq, nsamp, ndec, flags, clip, xmm, sql, sqh, suml, sumh = r
    w = by_seq.get(seq)
    if w is None:
        ok = False
        print(f"    snapshot {tag}: SEQ {seq} not in log")
        continue
    sumv = (sumh << 32) | suml
    if sumv >> 63:
        sumv -= 1 << 64
    exp_flags = w["end_pps"] | (w["start_pps"] << 1) | (w["end_to"] << 2)
    got = dict(n=nsamp, ndec=ndec, clip=clip, sq=(sqh << 32) | sql, sum=sumv,
               min=xmm & 0xFFF, max=(xmm >> 16) & 0xFFF, flags=flags, status_seq=status >> 16)
    exp = dict(n=w["n"], ndec=w["ndec"], clip=w["clip"], sq=w["sq"], sum=w["sum"],
               min=w["min"], max=w["max"], flags=exp_flags, status_seq=seq & 0xFFFF)
    for key in exp:
        if got[key] != exp[key]:
            ok = False
            print(f"    snapshot {tag}: {key} reg {got[key]} window {exp[key]}")
    for k in range(NCH):
        hi, lo, pm = pregs[tag][k]
        if ((hi << 32) | lo) != w["pow"][k] or pm != (w["pmax"][k] >> 16):
            ok = False
            print(f"    snapshot {tag} ch{k}: POW/PMAX registers differ from window {seq}")
check(ok and len(regs) == 3, f"{len(regs)} snapshots x 35 registers match the windows they name")

# ------------------------------------------------------------------ 4/5. physics
print("\n[4] fixed-point receiver vs ideal floating-point receiver")
fl = [channel_float(codes, None, ftw=FTWS[k]) for k in range(NCH)]
fl_p = [dict(zip(fl[k][0].tolist(), (fl[k][1] ** 2 + fl[k][2] ** 2).tolist())) for k in range(NCH)]


def win_amp_float(w_ref, k):
    """float-model amplitude over the same window membership as the hardware"""
    tot, cnt = 0.0, 0
    for n, p in fl_p[k].items():
        if w_ref["first"] <= n <= w_ref["last"] and samples[n][3]:
            tot += p
            cnt += 1
    return math.sqrt(tot / cnt) / K_AMP if cnt else 0.0


hdr = "    window  " + "".join(f"{c:>9s}" for c, _, _ in STATIONS)
print(hdr + "     (amplitude in ADC counts: hardware model / ideal receiver)")
worst_db = 0.0
for w_sim, w_ref in zip(wins, windows):
    if w_sim["seq"] < 3:
        continue                      # CIC still filling in windows 1-2
    hw = [amp_counts(w_sim["pow"][k], w_sim["ndec"]) for k in range(NCH)]
    idl = [win_amp_float(w_ref, k) for k in range(NCH)]
    print(f"    {w_sim['seq']:4d} hw " + "".join(f"{a:9.3f}" for a in hw))
    print(f"         id " + "".join(f"{a:9.3f}" for a in idl))
    for k in range(NCH):
        if idl[k] > 1.0:              # strong channels: compare in dB
            worst_db = max(worst_db, abs(20 * math.log10(hw[k] / idl[k])))
check(worst_db < 0.01, f"channels above 1 count agree with the ideal receiver within "
      f"{worst_db:.4f} dB (< 0.01 dB)")

print("\n[5] physics, clean PPS window 3 (55-105 ms)")
w3 = next(w for w in wins if w["seq"] == 3)
w3r = windows[wins.index(w3)]
amp = [amp_counts(w3["pow"][k], w3["ndec"]) for k in range(NCH)]
db = [20 * math.log10(a) if a > 0 else -999 for a in amp]
for k, (call, khz, _) in enumerate(STATIONS):
    print(f"    ch{k} {call} {khz:6.2f} kHz  {amp[k]:9.4f} counts  {db[k]:7.2f} dB re 1 count")

import gen_files
stim = gen_files.make_stimulus()
check(codes == stim[:len(codes)], "the design consumed exactly the stimulus samples, in order")


def part_amp(k, **parts):
    """ideal receiver, channel k, window 3, on selected stimulus components only"""
    x = gen_files.make_stimulus(quantize=False, clip=False, **parts)[:len(codes)]
    n_, i_, q_ = channel_float(x, None, ftw=FTWS[k])
    sel = (n_ >= w3r["first"]) & (n_ <= w3r["last"])
    return math.sqrt(np.mean(i_[sel] ** 2 + q_[sel] ** 2)) / K_AMP


check(abs(amp[1] - 20.0) < 0.05, f"21.4 kHz tone of 20 counts reads {amp[1]:.4f} counts (+/-0.05)")
msk_db = 20 * math.log10(amp[0] / 100.0)
check(-1.2 < msk_db < 0.0, f"24.8 kHz MSK of 100 counts reads {amp[0]:.2f} = {msk_db:.2f} dB "
      f"(long-run average -0.60 dB: the CIC passes most of the +/-150 Hz MSK spectrum)")
t_leak = part_amp(2, msk=False, tone1=False, tone2=True, noise=False)
m_leak = part_amp(2, msk=True, tone1=False, tone2=False, noise=False)
n_floor = part_amp(2, msk=False, tone1=False, tone2=False, noise=True)
print(f"    24.0 kHz channel, separate contributions: 23.6 kHz tone {t_leak:.4f}, "
      f"24.8 kHz MSK sidebands {m_leak:.4f}, noise {n_floor:.4f} counts")
check(20 * math.log10(300.0 / t_leak) > 75.0,
      f"300-count tone 400 Hz off-channel rejected by {20 * math.log10(300.0 / t_leak):.1f} dB (> 75)")
check(abs(math.sqrt(t_leak ** 2 + m_leak ** 2 + n_floor ** 2) - amp[2]) < 0.05,
      f"24.0 kHz reading {amp[2]:.4f} = tone + MSK sideband + noise contributions "
      f"({math.sqrt(t_leak ** 2 + m_leak ** 2 + n_floor ** 2):.4f})")
check(amp[7] < 0.15 and amp[4] < 0.15, f"empty channels (30.0, 40.75 kHz) at the noise floor: "
      f"{amp[7]:.4f}, {amp[4]:.4f} counts (expected about 0.09)")
check(db[0] - db[3] > 30.0, f"25.2 kHz channel {db[0] - db[3]:.1f} dB below the 24.8 kHz MSK "
      f"400 Hz away (long-run MSK sideband energy there: -33.5 dB)")
rms = math.sqrt(w3["sq"] / w3["n"])
exp_rms = math.sqrt(100 ** 2 / 2 + 20 ** 2 / 2 + 300 ** 2 / 2 + 4.0 + 1.0 / 12)
check(abs(rms - exp_rms) / exp_rms < 0.01, f"input RMS {rms:.2f} counts (expected {exp_rms:.2f})")

print("\n[6] channel frequency response (bit-exact model = the hardware arithmetic)")
fs = float(FS_NOM)
f_ch = STATIONS[0][1] * 1000.0
print("    offset Hz   measured dB   CIC formula dB")
worst = 0.0
for off in (0.0, 50.0, 100.0, 200.0, 300.0, 400.0, 545.0, 800.0, 1200.0):
    nn = R * 48
    x = [int(round(2048 + 1000.0 * math.cos(2 * math.pi * (f_ch + off) * n / fs))) for n in range(nn)]
    out = channel_int(x, FTWS[0])[8:]
    a = math.sqrt(sum(i * i + q * q for _, i, q in out) / len(out)) / K_AMP / 1000.0
    u = math.pi * off / fs
    h = 1.0 if off == 0 else abs(math.sin(u * R) / (R * math.sin(u))) ** 3
    m_db, f_db = 20 * math.log10(max(a, 1e-12)), 20 * math.log10(h)
    print(f"    {off:9.1f}   {m_db:10.2f}   {f_db:12.2f}")
    if f_db > -70:
        worst = max(worst, abs(m_db - f_db))
check(worst < 0.05, f"response matches the 3-stage CIC formula within {worst:.3f} dB down to -70 dB")

# ------------------------------------------------------------------ 7. sferic capture
print("\n[7] sferic (lightning) capture: triggers, 20 ns timestamps, snapshots")
THRESH, HOLDOFF, BUSY, PRE, LEN = 600, 1000, 48, 16, 64        # as the testbench programmed it
exp_trigs = []
hold = busy = 0
for n, c in enumerate(stim[:len(codes)]):
    armed = (hold == 0 and busy == 0)
    if hold:
        hold -= 1
    if busy:
        busy -= 1
    if armed and abs(c - 2048) >= THRESH:
        exp_trigs.append(n)
        hold, busy = HOLDOFF, BUSY
print(f"    triggers: hardware {[t for t, _ in trigs]}, expected from the stimulus {exp_trigs}")
check([t for t, _ in trigs] == exp_trigs,
      f"{len(trigs)} triggers at exactly the samples the threshold/hold-off rule predicts")
check(len(events) == len(trigs), f"{len(events)} events read over the UART, one per trigger")
ok = True
for ev, (n, t_trig) in zip(events, trigs):
    g_before = [g for g in gtimes if g <= t_trig]
    exp_pps = len(g_before)
    exp_clk = (t_trig - g_before[-1]) // 20 if g_before else None
    seg = stim[n:n + LEN - PRE]
    peak, pol = 0, 0
    for c in seg:
        a = abs(c - 2048)
        if a > peak:
            peak, pol = a, int(c < 2048)
    exp_info = (PRE << 24) | (LEN << 16) | (pol << 12) | peak
    snap = []
    for w in ev["words"]:
        snap += [w & 0xFFF, (w >> 16) & 0xFFF]
    exp_snap = stim[n - PRE:n + LEN - PRE]
    got = dict(pps=ev["pps"], clk=ev["clk"] & 0x3FFFFFF, seen=ev["clk"] >> 31, info=ev["info"], seq=ev["seq"])
    exp = dict(pps=exp_pps, clk=exp_clk, seen=1, info=exp_info, seq=ev["tag"])
    for key in exp:
        if got[key] != exp[key]:
            ok = False
            print(f"    event {ev['tag']} (sample {n}): {key} register {got[key]} expected {exp[key]}")
    if snap != exp_snap:
        ok = False
        bad = [i for i in range(LEN) if snap[i] != exp_snap[i]]
        print(f"    event {ev['tag']}: snapshot differs at {len(bad)} of {LEN} samples, first {bad[:5]}")
    print(f"    event {ev['tag']}: sample {n}, GPS second {ev['pps']}, {got['clk']} clocks = "
          f"{got['clk'] * 20e-6:.3f} ms after its PPS, |peak| {peak} {'neg' if pol else 'pos'}, "
          f"snapshot {'matches' if snap == exp_snap else 'DIFFERS'} the stimulus sample for sample")
check(ok, "every event: GPS second, 20 ns clock count, PPS-seen flag, PRE/LEN, polarity, |peak|, "
          "SEQ and all 64 snapshot samples match the stimulus and the logged PPS/trigger times")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
