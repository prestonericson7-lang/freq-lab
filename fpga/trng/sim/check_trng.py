#!/usr/bin/env python3
"""
check_trng.py -- verify the TRNG simulation logs against the bit-exact model.

  python3 check_trng.py stim_balanced.txt log_balanced.txt [...more pairs...]

For each (stim, log): load the raw bits + config from stim, run trng_model,
and compare the HDL's logged output words (W), register counters (R) and
FIFO DATA reads (D) to the model.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import trng_model as m

passes = fails = 0


def check(cond, what):
    global passes, fails
    if cond:
        passes += 1
        print(f"  PASS  {what}")
    else:
        fails += 1
        print(f"  FAIL  {what}")


def load_stim(path):
    with open(path) as f:
        cfg = f.readline().split()
        rct_cut, apt_win, apt_cut = int(cfg[0]), int(cfg[1]), int(cfg[2])
        bits = [1 if c == "1" else 0 for c in f.read() if c in "01"]
    return bits, rct_cut, apt_win, apt_cut


def load_log(path):
    words, datas, regs = [], [], None
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == "W":
                words.append(int(p[1], 16))
            elif p[0] == "D":
                datas.append(int(p[1], 16))
            elif p[0] == "R":
                regs = dict(nbits=int(p[1]), raw=int(p[2], 16), vn=int(p[3], 16), rct_f=int(p[4], 16),
                            apt_f=int(p[5], 16), words=int(p[6], 16), status=int(p[7], 16),
                            rct_cut=int(p[8]), apt_win=int(p[9]), apt_cut=int(p[10]))
    return words, datas, regs


def run(stim, log):
    name = os.path.basename(stim)
    print(f"\n[{name}]")
    bits, rct_cut, apt_win, apt_cut = load_stim(stim)
    words, datas, regs = load_log(log)
    md = m.pipeline(bits, rct_cut, apt_win, apt_cut)

    check(regs is not None and regs["raw"] == md["raw_count"],
          f"raw_count {regs['raw'] if regs else '?'} == model {md['raw_count']}")
    check(regs["vn"] == md["vn_count"], f"vn_count {regs['vn']} == model {md['vn_count']}")
    check(regs["words"] == len(md["words"]), f"word_count {regs['words']} == model {len(md['words'])}")
    check(words == md["words"], f"all {len(words)} output words bit-exact vs the model ({len(md['words'])})")
    check(regs["rct_f"] == md["rct_fail_count"], f"RCT fail count {regs['rct_f']} == model {md['rct_fail_count']}")
    check(regs["apt_f"] == md["apt_fail_count"], f"APT fail count {regs['apt_f']} == model {md['apt_fail_count']}")
    rct_alarm = bool(regs["status"] & 0x2)
    apt_alarm = bool(regs["status"] & 0x4)
    check(rct_alarm == md["rct_fail"], f"RCT alarm bit {rct_alarm} == model {md['rct_fail']}")
    check(apt_alarm == md["apt_fail"], f"APT alarm bit {apt_alarm} == model {md['apt_fail']}")
    # the FIFO holds up to 64 words; DATA reads must equal the first words produced
    nfifo = min(64, len(md["words"]))
    check(datas == md["words"][:nfifo], f"{len(datas)} DATA reads == the first {nfifo} words from the FIFO")

    # scenario expectations
    if "balanced" in name:
        check(not md["rct_fail"] and not md["apt_fail"], "balanced input trips no health test")
        check(len(md["words"]) >= 10, f"balanced input produces usable words ({len(md['words'])})")
    if "stuck" in name:
        check(md["rct_fail"] and md["rct_fail_count"] >= 1, "all-ones input trips the Repetition Count Test")
        check(len(md["words"]) == 0, "no words pass while the source is stuck")
    if "biased" in name:
        check(md["apt_fail"], "biased input trips the Adaptive Proportion Test")
        check(not md["rct_fail"], "the bias-isolated stream does NOT trip RCT (runs capped below the cutoff)")


def main():
    args = sys.argv[1:]
    if len(args) < 2 or len(args) % 2:
        sys.exit("usage: check_trng.py stim1 log1 [stim2 log2 ...]")
    for i in range(0, len(args), 2):
        run(args[i], args[i + 1])
    print(f"\n=== {passes} PASS, {fails} FAIL ===")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
