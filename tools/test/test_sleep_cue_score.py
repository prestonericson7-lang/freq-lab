#!/usr/bin/env python3
"""test_sleep_cue_score.py -- the blind scorer: commitment checks, statistics, false-positive rate."""
import hashlib
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import sleep_cue_score as sc  # noqa: E402

fails = passes = 0


def check(cond, what):
    global fails, passes
    if cond:
        passes += 1; print(f"  PASS  {what}")
    else:
        fails += 1; print(f"  FAIL  {what}")


def make(n, effect, seed):
    rng = random.Random(seed)
    mask = [1] * (n // 2) + [0] * (n - n // 2)
    rng.shuffle(mask)
    rows = []
    for i in range(n):
        e = 1 if rng.random() < 0.7 else 0
        p_keep = 0.75 + (effect if mask[i] else 0.0)
        m = (1 if rng.random() < p_keep else 0) if e else (1 if rng.random() < 0.05 else 0)
        rows.append((i + 1, float(e), float(m)))
    return mask, rows


print("[1] commitments")
salt = "a1b2c3d4"
mask_s = "0110100101"
commit = hashlib.sha256(f"{mask_s}:{salt}".encode()).hexdigest()
check(sc.verify(f"{mask_s}:{salt}", commit), "the right key verifies")
check(not sc.verify(f"0110100110:{salt}", commit), "a key with two pairs swapped fails")
check(not sc.verify(f"{mask_s}:a1b2c3d5", commit), "a different salt fails")

print("[2] statistics")
mask, rows = make(60, 0.20, 3)
r = sc.score(rows, mask, 5000)
print(f"    true effect +0.20 retention: diff {r['diff']:+.3f}, p {r['p_one_sided']:.4f}")
check(r["diff"] > 0.05, "a planted retention benefit for cued pairs gives a positive difference")
fp = 0
for s in range(200):
    m0, r0 = make(40, 0.0, 1000 + s)
    if sc.score(r0, m0, 400, seed=s)["p_one_sided"] < 0.05:
        fp += 1
print(f"    null data: {fp} of 200 sessions with p < 0.05")
check(fp <= 18, "false-positive rate under the null near 5 % (<= 9 % of 200)")
pw = sum(1 for s in range(100) if sc.score(*reversed(make(100, 0.20, 5000 + s)), 400, seed=s)["p_one_sided"] < 0.05)
print(f"    power with 100 pairs and +0.20 retention: {pw} of 100 sessions significant")
check(pw >= 40, "with 100 pairs a +0.20 retention benefit is detected in a useful fraction of sessions")

print("[3] command line")
tmp = tempfile.mkdtemp()
path = os.path.join(tmp, "recall.csv")
with open(path, "w") as f:
    f.write("pair,evening,morning\n")
    for p, e, m in rows:
        f.write(f"{p},{int(e)},{int(m)}\n")
key = "".join("1" if k else "0" for k in mask) + ":deadbeef"
cm = hashlib.sha256(key.encode()).hexdigest()
nk = "stim:00ff"
ncm = hashlib.sha256(nk.encode()).hexdigest()
tool = os.path.join(HERE, "..", "sleep_cue_score.py")
out = subprocess.run([sys.executable, tool, path, "--key", key, "--commit", cm, "--night-key", nk, "--night-commit", ncm, "--perm", "2000"],
                     capture_output=True, text=True)
check(out.returncode == 0 and "matches its commitment" in out.stdout and "STIM" in out.stdout and "permutation p" in out.stdout,
      "valid keys: scored, night condition revealed")
bad = subprocess.run([sys.executable, tool, path, "--key", key, "--commit", "0" * 64], capture_output=True, text=True)
check(bad.returncode != 0 and "DOES NOT MATCH" in (bad.stderr + bad.stdout), "a wrong commitment refuses to score")

print(f"\n=== {passes} PASS, {fails} FAIL ===")
sys.exit(1 if fails else 0)
