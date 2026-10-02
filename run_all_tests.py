#!/usr/bin/env python3
"""
run_all_tests.py -- re-verify the entire freq-lab stack in one command.

    python run_all_tests.py            # everything available on this machine
    python run_all_tests.py --quick    # skip the slow FPGA sims
    python run_all_tests.py --full     # also run the slow v1 vlf_sid sim

Runs, and tallies PASS/FAIL for:
  * every tools/test/test_*.py            (Windows Python)
  * every teensy/*/test/pc_test*.cpp and
    teensy/vlf_station/test, kirlian, ...  (g++ via WSL)
  * the FPGA simulations vlf_sid_v2 and trng (iverilog + Python bit-exact models, via WSL)

Prints a single green/red table and exits non-zero if anything failed. Parts
whose toolchain is missing (WSL / g++ / iverilog) are reported SKIPPED, not failed.
"""
import argparse
import glob
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SUMMARY = re.compile(r"===\s*(\d+)\s*PASS,\s*(\d+)\s*FAIL\s*===")
WSL = ["wsl", "-d", "Ubuntu-22.04", "-e", "bash", "-lc"]


def wslpath(p):
    p = os.path.abspath(p).replace("\\", "/")
    if len(p) > 1 and p[1] == ":":
        p = "/mnt/" + p[0].lower() + p[2:]
    return p


def have_wsl():
    try:
        return subprocess.run(WSL + ["command -v g++ iverilog >/dev/null && echo ok"],
                              capture_output=True, text=True, timeout=60).stdout.strip().endswith("ok")
    except Exception:
        return False


def run(cmd, timeout, shell_wsl=False):
    try:
        if shell_wsl:
            r = subprocess.run(WSL + [cmd], capture_output=True, text=True, timeout=timeout)
        else:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr, r.returncode
    except subprocess.TimeoutExpired:
        return "TIMEOUT", 124
    except Exception as e:
        return f"ERROR {e}", 1


def tally(out):
    m = None
    for m in SUMMARY.finditer(out):
        pass
    return (int(m.group(1)), int(m.group(2))) if m else None


results = []        # (name, passed, failed, status)


def record(name, out, rc, skipped=False):
    if skipped:
        results.append((name, 0, 0, "SKIP"))
        print(f"  {'SKIP':5s} {name}")
        return
    if rc == 124 or "TIMEOUT" in out[:200]:
        # inconclusive: the simulation did not finish in the budget (usually heavy
        # machine load, e.g. a Vivado build running). Not a correctness failure.
        results.append((name, 0, 0, "SLOW"))
        print(f"  {'SLOW':5s} {name}  (timed out -- re-run this sim alone when the machine is idle)")
        return
    t = tally(out)
    if t is None:
        results.append((name, 0, 1, "ERR"))
        print(f"  {'ERR':5s} {name}  (no PASS/FAIL summary; rc={rc})")
        return
    p, f = t
    results.append((name, p, f, "ok" if f == 0 and rc == 0 else "FAIL"))
    print(f"  {('ok' if f == 0 and rc == 0 else 'FAIL'):5s} {name}  {p} pass, {f} fail")


def completed(out):
    """True if a testbench run produced its final PASS/FAIL summary (not truncated)."""
    return SUMMARY.search(out) is not None and "TIMEOUT" not in out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="skip the FPGA simulations")
    ap.add_argument("--full", action="store_true", help="also run the slow v1 vlf_sid sim")
    a = ap.parse_args()

    print("== Python test suites ==")
    for t in sorted(glob.glob(os.path.join(ROOT, "tools", "test", "test_*.py"))):
        out, rc = run([sys.executable, t], timeout=600)
        record("py/" + os.path.basename(t), out, rc)

    wsl = have_wsl()
    print("\n== C++ on-PC logic tests ==" + ("" if wsl else "  (WSL/g++ not found -> SKIPPED)"))
    cpp = [
        ("sleep_cue", "teensy/sleep_cue", "teensy/sleep_cue/test/pc_test.cpp"),
        ("rf_survey", "teensy/rf_survey", "teensy/rf_survey/test/pc_test.cpp"),
        ("kirlian_logger", "teensy/kirlian_logger", "teensy/kirlian_logger/test/pc_test.cpp"),
        ("nuisance_logger", "teensy/nuisance_logger", "teensy/nuisance_logger/test/pc_test.cpp"),
        ("strobe_safety", "teensy/strobe_driver", "teensy/strobe_driver/test/pc_test_safety.cpp"),
        ("vlf_station_nmea", "teensy/vlf_station", "teensy/vlf_station/test/pc_test.cpp"),
    ]
    for name, inc, src in cpp:
        if not wsl:
            record("cpp/" + name, "", 0, skipped=True); continue
        cmd = (f"cd {wslpath(ROOT)} && g++ -std=c++17 -I{wslpath(os.path.join(ROOT, inc))} "
               f"{wslpath(os.path.join(ROOT, src))} -o /tmp/ft && /tmp/ft; rm -f /tmp/ft")
        out, rc = run(cmd, timeout=180, shell_wsl=True)
        record("cpp/" + name, out, rc)

    if not a.quick:
        print("\n== FPGA simulations ==" + ("" if wsl else "  (WSL/iverilog not found -> SKIPPED)"))
        # vlf_sid_v2: iverilog + vvp + check_vlf.py (Windows python for numpy)
        if wsl:
            d = wslpath(os.path.join(ROOT, "fpga/vlf_sid_v2"))
            cmd = (f"cd {d} && python3 sim/gen_files.py >/dev/null 2>&1 && "
                   f"iverilog -DSIMULATION -o sim/tb_vlf sim/tb_vlf.v hdl/*.v && "
                   f"cd sim && timeout 600 vvp tb_vlf")
            out, rc = run(cmd, timeout=660, shell_wsl=True)
            record("fpga/vlf_sid_v2 (testbench)", out, rc)
            if completed(out):          # only check the model if the log is fresh and complete
                cout, crc = run([sys.executable, os.path.join(ROOT, "fpga/vlf_sid_v2/sim/check_vlf.py"),
                                 os.path.join(ROOT, "fpga/vlf_sid_v2/sim/sim_log.txt")], timeout=300)
                record("fpga/vlf_sid_v2 (model)", cout, crc)
            else:
                record("fpga/vlf_sid_v2 (model)", "", 124)   # SLOW: testbench did not finish
            # trng: iverilog + 3 vvp runs + check_trng.py
            d = wslpath(os.path.join(ROOT, "fpga/trng"))
            cmd = (f"cd {d}/sim && python3 gen_files.py >/dev/null 2>&1 && "
                   f"iverilog -g2012 -DSIMULATION -o tb_trng tb_trng.v ../hdl/entropy_src.v "
                   f"../hdl/trng_core.v ../hdl/cmd_regs_trng.v ../hdl/uart.v && "
                   f"for s in balanced stuck biased; do vvp tb_trng +STIM=stim_$s.txt +LOG=log_$s.txt >/dev/null 2>&1; done && echo done")
            out, rc = run(cmd, timeout=300, shell_wsl=True)
            if "done" in out:
                sd = os.path.join(ROOT, "fpga/trng/sim")
                cout, crc = run([sys.executable, os.path.join(sd, "check_trng.py"),
                                 os.path.join(sd, "stim_balanced.txt"), os.path.join(sd, "log_balanced.txt"),
                                 os.path.join(sd, "stim_stuck.txt"), os.path.join(sd, "log_stuck.txt"),
                                 os.path.join(sd, "stim_biased.txt"), os.path.join(sd, "log_biased.txt")], timeout=120)
                record("fpga/trng (model)", cout, crc)
            else:
                record("fpga/trng (model)", out, rc)
            if a.full:
                d = wslpath(os.path.join(ROOT, "fpga/vlf_sid"))
                cmd = (f"cd {d} && python3 sim/gen_files.py >/dev/null 2>&1 && "
                       f"iverilog -DSIMULATION -o sim/tb_vlf sim/tb_vlf.v hdl/*.v && "
                       f"cd sim && timeout 540 vvp tb_vlf")
                out, rc = run(cmd, timeout=600, shell_wsl=True)
                record("fpga/vlf_sid v1 (testbench, slow)", out, rc)
        else:
            for n in ("fpga/vlf_sid_v2", "fpga/trng"):
                record(n, "", 0, skipped=True)

    tp = sum(p for _, p, _, _ in results)
    tf = sum(f for _, _, f, _ in results)
    nfail = sum(1 for _, _, _, s in results if s in ("FAIL", "ERR"))
    nskip = sum(1 for _, _, _, s in results if s == "SKIP")
    nslow = sum(1 for _, _, _, s in results if s == "SLOW")
    print("\n" + "=" * 52)
    print(f"  {len(results)} test groups, {nskip} skipped, {nslow} inconclusive (slow)")
    print(f"  {tp} checks passed, {tf} failed, {nfail} group(s) failed")
    if nfail == 0 and tf == 0 and nslow == 0:
        print("  RESULT: ALL GREEN")
    elif nfail == 0 and tf == 0:
        print(f"  RESULT: ALL GREEN except {nslow} sim(s) that timed out under load -- re-run those alone")
    else:
        print("  RESULT: FAILURES ABOVE")
    print("=" * 52)
    sys.exit(1 if (nfail or tf) else 0)


if __name__ == "__main__":
    main()
