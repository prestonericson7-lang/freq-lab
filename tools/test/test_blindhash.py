#!/usr/bin/env python3
"""test_blindhash.py -- the blind-commit harness: sealing, tamper detection, balance, stats, CLI."""
import csv
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import blindhash as bh  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


print("[1] commitment seals the schedule")
sched, salt, commit = bh.make_schedule(["A", "B"], 40)
check(len(sched) == 40 and sched.count("A") == 20 and sched.count("B") == 20, "balanced 40-trial schedule, 20/20")
check(bh.verify_commit(commit, sched, salt), "the real schedule + salt verify against the commitment")
tampered = sched[:]
tampered[0], tampered[1] = tampered[1], tampered[0]
check(not bh.verify_commit(commit, tampered, salt) if tampered != sched else True, "a swapped pair fails the commitment")
check(not bh.verify_commit(commit, sched, "deadbeef" * 4), "a wrong salt fails the commitment")

print("[2] schedules are unpredictable")
commits = set()
for _ in range(50):
    s, sa, cm = bh.make_schedule(["A", "B"], 20)
    commits.add(cm)
check(len(commits) == 50, "50 draws give 50 distinct commitments (crypto RNG)")

print("[3] stats: null is calibrated, real effect is found")
import random
rng = random.Random(0)
fp = fp1 = 0
for t in range(200):
    s, sa, cm = bh.make_schedule(["A", "B"], 40, seed_bytes=f"n{t}")
    outcomes = [rng.gauss(0, 1) for _ in s]          # no difference between conditions
    r = bh.score(s, outcomes, 500, seed=t, higher="A")
    if r["p_two_sided"] < 0.05:
        fp += 1
    if r["p_one_sided"] < 0.05:
        fp1 += 1
print(f"    null: two-sided {fp}/200, one-sided {fp1}/200 with p < 0.05")
check(fp <= 18, "two-sided false-positive rate near 5 % under the null")
check(fp1 <= 18, "one-sided false-positive rate near 5 % under the null")
pw = pw1 = 0
for t in range(100):
    s, sa, cm = bh.make_schedule(["A", "B"], 60, seed_bytes=f"p{t}")
    outcomes = [rng.gauss(0.7 if c == "A" else 0.0, 1.0) for c in s]   # A is 0.7 SD higher
    r = bh.score(s, outcomes, 500, seed=t, higher="A")
    if r["p_two_sided"] < 0.05:
        pw += 1
    if r["p_one_sided"] < 0.05:
        pw1 += 1
print(f"    0.7 SD effect over 60 trials: two-sided {pw}/100, one-sided {pw1}/100 significant")
check(pw >= 65, "two-sided test detects a real 0.7 SD difference in most sessions (order-independent, no post-hoc direction)")
check(pw1 >= 75, "the pre-registered one-sided test has more power still")
# the p-value must NOT depend on which label appears first in the schedule
s_a_first = ["A", "B"] * 20
s_b_first = ["B", "A"] * 20
out = [1.0 if c == "A" else 0.0 for c in s_a_first]
out_b = [1.0 if c == "A" else 0.0 for c in s_b_first]
check(abs(bh.score(s_a_first, out, 2000)["p_two_sided"] - bh.score(s_b_first, out_b, 2000)["p_two_sided"]) < 0.02,
      "the two-sided p is the same whichever condition was drawn first (the bug the red-team pass caught)")

print("[4] unbalanced and >2 conditions")
s3, _, c3 = bh.make_schedule(["X", "Y", "Z"], 30)
check(len(s3) == 30 and all(s3.count(c) == 10 for c in "XYZ"), "three conditions, 10 each")
try:
    bh.score(s3, [0] * 30)
    ok = False
except ValueError:
    ok = True
check(ok, "score refuses a 3-condition schedule (it is a 2-condition test)")

print("[5] CLI round trip")
tmp = tempfile.mkdtemp()
pre = os.path.join(tmp, "run1")
tool = os.path.join(HERE, "..", "blindhash.py")
r = subprocess.run([sys.executable, tool, "commit", "--conditions", "A,B", "--n", "20", "--out", pre], capture_output=True, text=True)
check(r.returncode == 0 and os.path.exists(pre + ".commit") and os.path.exists(pre + ".schedule"), "commit writes .commit and .schedule")
v = subprocess.run([sys.executable, tool, "verify", pre + ".commit", pre + ".schedule"], capture_output=True, text=True)
check(v.returncode == 0 and "MATCH" in v.stdout, "verify passes on the sealed pair")
# tamper with the schedule file: flip the first A to B (a real change to the schedule)
d = json.load(open(pre + ".schedule"))
orig = list(d["schedule"])
ia = d["schedule"].index("A")
d["schedule"][ia] = "B"
json.dump(d, open(pre + ".schedule", "w"))
v2 = subprocess.run([sys.executable, tool, "verify", pre + ".commit", pre + ".schedule"], capture_output=True, text=True)
check(v2.returncode != 0 and "MISMATCH" in v2.stdout, "verify fails after the schedule is edited (tamper-evident)")
# restore the real schedule and score it
json.dump({"schedule": orig, "salt": d["salt"]}, open(pre + ".schedule", "w"))
res = os.path.join(tmp, "results.csv")
with open(res, "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["trial", "outcome"])
    for i, c in enumerate(orig):
        w.writerow([i, 1.0 if c == "A" else 0.0])
sc = subprocess.run([sys.executable, tool, "score", pre + ".schedule", res, "--perm", "2000"], capture_output=True, text=True)
check(sc.returncode == 0 and "two-sided permutation p" in sc.stdout, "score runs against the sealed schedule (two-sided by default)")
sc1 = subprocess.run([sys.executable, tool, "score", pre + ".schedule", res, "--perm", "2000", "--higher", "A"], capture_output=True, text=True)
check(sc1.returncode == 0 and "one-sided" in sc1.stdout, "--higher enables the pre-registered one-sided p")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
