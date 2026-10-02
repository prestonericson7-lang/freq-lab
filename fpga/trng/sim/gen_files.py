#!/usr/bin/env python3
"""
gen_files.py -- generate the raw-bit stimulus files for the TRNG testbench.
Each file: first line "rct_cut apt_win apt_cut", then one character ('0'/'1')
per raw noise-source bit.

  balanced : a good RNG, trips nothing, exercises the full datapath to words
  stuck    : all ones, trips the Repetition Count Test
  biased   : 72 % ones with no run longer than 15 (so RCT cannot trip at cutoff
             20): isolates the Adaptive Proportion Test
"""
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
RCT_CUT, APT_WIN, APT_CUT = 20, 1024, 660


def write(name, bits):
    with open(os.path.join(HERE, name), "w", newline="\n") as f:
        f.write(f"{RCT_CUT} {APT_WIN} {APT_CUT}\n")
        f.write("".join("1" if b else "0" for b in bits))
        f.write("\n")
    return bits


def balanced(n, seed):
    rng = random.Random(seed)
    return [rng.getrandbits(1) for _ in range(n)]


def biased_no_long_run(n, p_one, max_run, seed):
    rng = random.Random(seed)
    out = []
    run = 0
    last = -1
    for _ in range(n):
        b = 1 if rng.random() < p_one else 0
        if b == last and run >= max_run:      # force a flip to cap the run length
            b = 1 - b
        if b == last:
            run += 1
        else:
            run, last = 1, b
        out.append(b)
    return out


if __name__ == "__main__":
    b = write("stim_balanced.txt", balanced(4000, 73))
    print(f"balanced: {len(b)} bits, {sum(b)} ones ({100*sum(b)/len(b):.1f} %)")
    s = write("stim_stuck.txt", [1] * 100)
    print(f"stuck: {len(s)} bits all ones")
    z = biased_no_long_run(1500, 0.72, 15, 7)
    # verify no run >= 20
    mx = run = 1
    for i in range(1, len(z)):
        run = run + 1 if z[i] == z[i - 1] else 1
        mx = max(mx, run)
    write("stim_biased.txt", z)
    print(f"biased: {len(z)} bits, {sum(z)} ones ({100*sum(z)/len(z):.1f} %), longest run {mx} (< 20, so RCT cannot trip)")
