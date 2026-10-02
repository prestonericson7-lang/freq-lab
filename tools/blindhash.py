#!/usr/bin/env python3
"""
blindhash.py -- the blind randomization + pre-posted commitment harness
(list item #158). Build it once, use it for every psi/bioeffect experiment in
the stack so a result cannot be gamed after the fact.

The problem the FOIA files keep failing on: targets or condition labels chosen
(or boundaries moved) after the data is seen (the Ryzl post-hoc boundary case).
The fix: draw the schedule with a cryptographic RNG, hash it with a secret salt,
POST THE HASH before the run, reveal the schedule and salt after, and score only
against the sealed schedule. Anyone can then check the hash matches.

    # before the run -- draw and seal:
    python blindhash.py commit --conditions A,B --n 40 --out run1
       writes run1.schedule (secret), run1.commit (post this now)

    # ... run the experiment blind, record results keyed by trial index ...

    # after -- verify and score:
    python blindhash.py verify run1.commit run1.schedule
    python blindhash.py score run1.schedule results.csv
       results.csv: trial,outcome   (outcome numeric; higher = the hypothesised direction)

Also importable:
    sched, salt, commit = make_schedule(["A","B"], 40)
    assert verify_commit(commit, sched, salt)
    stats = score(sched, outcomes)     # permutation test, condition A vs B
"""
import argparse
import csv
import hashlib
import json
import os
import secrets
import statistics
import sys


def make_schedule(conditions, n, balanced=True, seed_bytes=None):
    """Return (schedule, salt, commitment). schedule is a list of condition labels.
    balanced: equal counts per condition (n rounded down to a multiple)."""
    if len(conditions) < 2:
        raise ValueError("need at least two conditions")
    rng = secrets.SystemRandom() if seed_bytes is None else _SeededRandom(seed_bytes)
    if balanced:
        per = n // len(conditions)
        sched = [c for c in conditions for _ in range(per)]
    else:
        sched = [rng.choice(conditions) for _ in range(n)]
    for i in range(len(sched) - 1, 0, -1):            # Fisher-Yates
        j = rng.randrange(i + 1)
        sched[i], sched[j] = sched[j], sched[i]
    salt = secrets.token_hex(16)
    return sched, salt, commitment(sched, salt)


class _SeededRandom:
    """deterministic RNG for tests only (reproducible schedules from a seed)."""
    def __init__(self, seed_bytes):
        import random
        self.r = random.Random(seed_bytes)

    def choice(self, seq):
        return self.r.choice(seq)

    def randrange(self, n):
        return self.r.randrange(n)


def commitment(schedule, salt):
    body = json.dumps(schedule, separators=(",", ":"))
    return hashlib.sha256((body + ":" + salt).encode()).hexdigest()


def verify_commit(commit, schedule, salt):
    return commitment(schedule, salt) == commit.strip().lower()


def score(schedule, outcomes, n_perm=20000, seed=1, higher=None):
    """Two-condition permutation test. outcomes aligned to schedule (numbers).
    The statistic is mean(a) - mean(b) for the two condition labels in sorted
    order (a < b), so it never depends on the order trials happened to be drawn.

    By default the p-value is TWO-SIDED: P(|permuted diff| >= |observed|). A
    one-sided test needs a PRE-REGISTERED direction -- pass higher=<label> (the
    condition hypothesised to score higher); the p-value is then
    P(permuted(higher - other) >= observed). Choosing the direction after seeing
    the data is exactly the post-hoc move this harness exists to prevent, so
    one-sided is never the default and never inferred from the data."""
    conds = sorted(set(schedule))
    if len(conds) != 2:
        raise ValueError("score handles exactly two conditions; got " + str(conds))
    if len(outcomes) != len(schedule):
        raise ValueError(f"{len(outcomes)} outcomes for {len(schedule)} trials")
    if higher is not None and higher not in conds:
        raise ValueError(f"higher={higher!r} is not one of {conds}")
    a_label, b_label = conds
    a = [o for o, c in zip(outcomes, schedule) if c == a_label]
    b = [o for o, c in zip(outcomes, schedule) if c == b_label]
    obs = statistics.mean(a) - statistics.mean(b)        # a - b, a<b by sort
    # direction-corrected observed statistic for the one-sided case
    obs_dir = obs if (higher is None or higher == a_label) else -obs
    import random
    rng = random.Random(seed)
    pool = list(outcomes)
    n_a = len(a)
    ge_abs = ge_dir = 0
    for _ in range(n_perm):
        rng.shuffle(pool)
        d = statistics.mean(pool[:n_a]) - statistics.mean(pool[n_a:])   # permuted (a - b)
        if abs(d) >= abs(obs) - 1e-12:
            ge_abs += 1
        d_dir = d if (higher is None or higher == a_label) else -d
        if d_dir >= obs_dir - 1e-12:
            ge_dir += 1
    p_two = (ge_abs + 1) / (n_perm + 1)
    p_one = (ge_dir + 1) / (n_perm + 1)
    sd = statistics.pstdev(outcomes) or 1.0
    return dict(a=a_label, b=b_label, n=len(schedule), mean_a=statistics.mean(a), mean_b=statistics.mean(b),
                diff=obs, d=obs / sd, p_two_sided=p_two,
                p_one_sided=(p_one if higher is not None else None), higher=higher)


# ---------------------------------------------------------------------- CLI
def cmd_commit(a):
    conds = a.conditions.split(",")
    sched, salt, commit = make_schedule(conds, a.n, not a.unbalanced)
    with open(a.out + ".schedule", "w") as f:
        json.dump({"schedule": sched, "salt": salt}, f)
    with open(a.out + ".commit", "w") as f:
        f.write(commit + "\n")
    print(f"{len(sched)} trials, conditions {conds}")
    print(f"commitment (SHA-256): {commit}")
    print(f"  -> {a.out}.commit   POST THIS NOW (text it, email it, timestamp it)")
    print(f"  -> {a.out}.schedule SECRET until the run is over")


def cmd_verify(a):
    commit = open(a.commit).read().strip()
    d = json.load(open(a.schedule))
    ok = verify_commit(commit, d["schedule"], d["salt"])
    print("MATCH: the schedule is the one that was sealed" if ok else
          "MISMATCH: the schedule does NOT match the posted commitment")
    sys.exit(0 if ok else 1)


def cmd_score(a):
    d = json.load(open(a.schedule))
    sched = d["schedule"]
    out = {}
    with open(a.results) as fh:
        for r in csv.DictReader(fh):
            out[int(r["trial"])] = float(r["outcome"])
    outcomes = [out[i] for i in range(len(sched))] if set(out) == set(range(len(sched))) else None
    if outcomes is None:
        # allow 1-based trial numbering
        outcomes = [out[i + 1] for i in range(len(sched))] if set(out) == set(range(1, len(sched) + 1)) else None
    if outcomes is None:
        sys.exit(f"results must have one row per trial, numbered 0..{len(sched)-1} or 1..{len(sched)}")
    s = score(sched, outcomes, a.perm, higher=a.higher)
    print(f"{s['n']} trials: {s['a']} mean {s['mean_a']:.3f}, {s['b']} mean {s['mean_b']:.3f}")
    line = f"{s['a']} - {s['b']} = {s['diff']:+.3f} (standardised {s['d']:+.2f}), two-sided permutation p = {s['p_two_sided']:.4f}"
    if s["p_one_sided"] is not None:
        line += f"; one-sided (pre-registered {s['higher']} higher) p = {s['p_one_sided']:.4f}"
    print(line)


def main():
    ap = argparse.ArgumentParser(description="blind randomization + pre-posted commitment (#158)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("commit"); c.add_argument("--conditions", required=True, help="comma list, e.g. A,B")
    c.add_argument("--n", type=int, required=True); c.add_argument("--out", required=True)
    c.add_argument("--unbalanced", action="store_true")
    v = sub.add_parser("verify"); v.add_argument("commit"); v.add_argument("schedule")
    s = sub.add_parser("score"); s.add_argument("schedule"); s.add_argument("results"); s.add_argument("--perm", type=int, default=20000)
    s.add_argument("--higher", help="pre-registered condition hypothesised higher (enables the one-sided p)")
    a = ap.parse_args()
    {"commit": cmd_commit, "verify": cmd_verify, "score": cmd_score}[a.cmd](a)


if __name__ == "__main__":
    main()
