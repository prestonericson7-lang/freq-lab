#!/usr/bin/env python3
"""
sleep_cue_score.py -- blind scoring of a sleep_cue word-pair night.

    python sleep_cue_score.py recall.csv --key 0110...1:3fa9c0... --commit <sha256 printed before the night>
                              [--night-key stim:9ab1... --night-commit <sha256>] [--perm 20000]

recall.csv (one row per pair, in the order the pairs were numbered; pair 1 = CUE001.WAV):
    pair,evening,morning          1/0 for recalled; or fractions 0..1 if you score partial credit
      1,1,1
      2,0,1
      ...

Checks first, then scores:
  1. SHA-256(mask:salt) equals the commitment printed BEFORE the night, so the
     split could not have been chosen after looking at the morning results.
  2. (optional) the same for the night's stim/sham condition.
Then: for cued and uncued pairs separately, the change in recall from evening to
morning (forgetting is usually negative; TMR's effect is less forgetting for cued
pairs). The statistic is the difference of means (cued - uncued), with an exact-
style permutation p-value: the split is re-drawn at random --perm times among
the same pairs, as the hardware drew it (exactly half cued).
"""
import argparse
import csv
import hashlib
import random
import statistics
import sys


def verify(text, commit):
    return hashlib.sha256(text.encode()).hexdigest() == commit.strip().lower()


def load_recall(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            rows.append((int(r["pair"]), float(r["evening"]), float(r["morning"])))
    rows.sort()
    if [p for p, _, _ in rows] != list(range(1, len(rows) + 1)):
        raise ValueError("pairs must be numbered 1..n with none missing")
    return rows


def score(rows, mask, n_perm=20000, seed=1):
    if len(mask) != len(rows):
        raise ValueError(f"key has {len(mask)} pairs, recall file has {len(rows)}")
    delta = [m - e for _, e, m in rows]
    cued = [d for d, k in zip(delta, mask) if k]
    unc = [d for d, k in zip(delta, mask) if not k]
    obs = statistics.mean(cued) - statistics.mean(unc)
    n, k = len(delta), sum(mask)
    rng = random.Random(seed)
    ge = 0
    for _ in range(n_perm):
        idx = set(rng.sample(range(n), k))
        a = [delta[i] for i in range(n) if i in idx]
        b = [delta[i] for i in range(n) if i not in idx]
        if statistics.mean(a) - statistics.mean(b) >= obs - 1e-12:
            ge += 1
    p_one = (ge + 1) / (n_perm + 1)
    sd = statistics.pstdev(delta) or 1.0
    return dict(n=n, n_cued=k, cued_change=statistics.mean(cued), uncued_change=statistics.mean(unc),
                diff=obs, p_one_sided=p_one, d=obs / sd,
                evening_cued=statistics.mean(e for (_, e, _), m in zip(rows, mask) if m),
                evening_uncued=statistics.mean(e for (_, e, _), m in zip(rows, mask) if not m))


def main():
    ap = argparse.ArgumentParser(description="blind scoring of a sleep_cue night")
    ap.add_argument("recall")
    ap.add_argument("--key", required=True, help="the pair key from `reveal`: mask:salt")
    ap.add_argument("--commit", required=True, help="the pair-split commitment printed before the night")
    ap.add_argument("--night-key", help="night key from `reveal`: stim:salt or sham:salt")
    ap.add_argument("--night-commit")
    ap.add_argument("--perm", type=int, default=20000)
    a = ap.parse_args()
    if not verify(a.key, a.commit):
        sys.exit("KEY DOES NOT MATCH THE COMMITMENT: the split was changed, or the wrong key/commitment was given. Not scoring.")
    print("pair key matches its commitment: the split was fixed before the night")
    cond = None
    if a.night_key:
        if not a.night_commit or not verify(a.night_key, a.night_commit):
            sys.exit("NIGHT KEY DOES NOT MATCH ITS COMMITMENT. Not scoring.")
        cond = a.night_key.split(":")[0]
        print(f"night key matches its commitment: this night was {cond.upper()}")
    mask = [c == "1" for c in a.key.split(":")[0]]
    r = score(load_recall(a.recall), mask, a.perm)
    print(f"\n{r['n']} pairs, {r['n_cued']} cued")
    print(f"evening recall: cued {r['evening_cued']:.3f}, uncued {r['evening_uncued']:.3f}  (should be similar: the split is random)")
    print(f"change overnight: cued {r['cued_change']:+.3f}, uncued {r['uncued_change']:+.3f}")
    print(f"cued - uncued = {r['diff']:+.3f}  (standardised {r['d']:+.2f}),  one-sided permutation p = {r['p_one_sided']:.4f}")
    if cond == "sham":
        print("this was a SHAM night: no sounds were played, so any difference here is chance by construction --"
              " that is what the sham nights are for.")


if __name__ == "__main__":
    main()
