# trng — ring-oscillator TRNG with SP 800-90B health tests (experiment #73)

A true random number generator on the PZ7020-StarLite. SRI certified their
1980 generator with 500,000-sample control runs and logged the diode
temperature to 0.2 °C; this does the modern equivalent: a ring-oscillator
noise source, the NIST SP 800-90B health tests in hardware, von Neumann
debiasing, LFSR conditioning, and the on-chip temperature logged alongside.
It gives every psi/bioeffect experiment in the stack an auditable, tamper-
logged target picker.

## Pipeline

```
16 ring oscillators --XOR, metastability-hardened, decimated--> raw bits
   |
   +--> RCT (Repetition Count Test)   \  SP 800-90B health tests on the RAW stream;
   +--> APT (Adaptive Proportion Test) /  output is withheld while either is tripped
   |
   +--> von Neumann debiaser (01->0, 10->1, 00/11->drop)
   +--> LFSR conditioning (x^32+x^22+x^2+x+1)
   +--> 32-bit packer --> 64-word FIFO --> UART
```

* **RCT**: if a raw value repeats `RCT_CUT` times in a row (default 20), alarm.
* **APT**: in a window of `APT_WIN` raw bits (default 1024), if the count equal
  to the window's first bit reaches `APT_CUT` (default 660), alarm.
* Both latch a sticky alarm and a fail counter; a CTRL write clears them. While
  an alarm is latched, von Neumann stops, so biased or stuck entropy never
  reaches the host.

The ring oscillators are deliberate combinational loops (odd inverter chains),
kept with `ALLOW_COMBINATORIAL_LOOPS`; the entropy is their jitter against the
50 MHz sampling clock. Nothing in this design transmits.

## Verification

Ring oscillators can't be simulated (combinational loops), so `entropy_src.v`
has a simulation backdoor: the testbench feeds a chosen raw-bit stream and the
**whole deterministic pipeline** — health tests, debiaser, conditioner, packer,
FIFO, UART register file — is checked bit-for-bit against `sim/trng_model.py`.

```
cd fpga/trng/sim
python3 gen_files.py
iverilog -g2012 -DSIMULATION -o tb_trng tb_trng.v ../hdl/entropy_src.v \
         ../hdl/trng_core.v ../hdl/cmd_regs_trng.v ../hdl/uart.v
for s in balanced stuck biased; do vvp tb_trng +STIM=stim_$s.txt +LOG=log_$s.txt; done
python3 check_trng.py stim_balanced.txt log_balanced.txt \
                      stim_stuck.txt    log_stuck.txt \
                      stim_biased.txt   log_biased.txt
```

Results (9 testbench + 33 model checks, all pass):

| Stream | What it shows |
|---|---|
| balanced (4000 bits, 49.4 % ones) | full datapath: raw/VN counts, 31 output words and all 31 FIFO reads bit-exact, no health alarm |
| stuck (100 ones) | RCT trips (5 counts, re-arming every 20), 0 words pass |
| biased (72 % ones, runs capped at 15) | APT trips (112 counts), RCT does not — the test isolated |

Every output word, the von Neumann and raw counters, both health flags and
fail counts, and the FIFO DATA reads match the Python model exactly.

Vivado 2026.1 (xc7z020clg400-2, Oct 1 2026): builds, exit 0, **0 critical
warnings**. Timing met at 50 MHz (WNS +12.59 ns, hold +0.115 ns, 0 failing of
2289 endpoints). 570 LUTs (1.1 %), 705 flip-flops (0.7 %), 0.5 BRAM, 1 DSP, 8
IOBs. The 16 ring-oscillator loops are reported as `LUTLP-2 Combinatorial Loop
Allowed` (intentional, downgraded to a warning). 0.110 W, junction 26.3 °C.
Bitstream `build/trng.bit`. Not yet run on hardware.

## Build and load

```
fpga\build_trng.bat                       Vivado build (~2.5 min)
   Vivado Tcl shell:  cd P:/Downloads/freq-lab/fpga/trng ; source program.tcl
   (loads RAM only; a power cycle restores the board)
```

## Use

```
python tools/trng_host.py COM7 info          status, health, die temperature
python tools/trng_host.py COM7 get 16        16 random 32-bit words
python tools/trng_host.py COM7 bytes 1000000 r.bin
python tools/trng_host.py COM7 selftest      125 kB + monobit/runs/chi-square + die-temp log
python tools/trng_host.py COM7 config --rct 20 --apt-win 1024 --apt-cut 660
python tools/trng_host.py COM7 clear         clear a latched health alarm
```

## Register map (UART, 115200 8N1, `R aa` / `W aa dddddddd`)

| Addr | Name | | |
|---|---|---|---|
| 00 | ID | RO | 0x54524E31 ("TRN1") |
| 01 | CTRL | RW | bit0 RUN; writing bit8=1 also clears the health alarms (default 0x1) |
| 02 | STATUS | RO | bit0 RUN, bit1 RCT, bit2 APT, bit3 FIFO empty, bit4 full, bit5 underflow, [15:8] words available |
| 03 | DATA | RO | a 32-bit random word; **reading it pops the FIFO** (0 if empty) |
| 04 | RAW_CNT | RO | raw noise-source bits |
| 05 | VN_CNT | RO | von Neumann output bits |
| 06 | RCT_FAILS | RO | Repetition Count Test alarms |
| 07 | APT_FAILS | RO | Adaptive Proportion Test alarms |
| 08 | WORD_CNT | RO | 32-bit words produced |
| 09 | RCT_CUT | RW | RCT cutoff (default 20) |
| 0A | APT_WIN | RW | APT window (default 1024) |
| 0B | APT_CUT | RW | APT cutoff (default 660) |
| 0C | DIE_TEMP | RO | XADC temp code (T_C = code·503.975/4096 − 273.15) |
| 0D | VCCINT | RO | XADC VCCINT code (V = code·3.0/4096) |
| 18 | FAN_DUTY | RW | % (default 60) |
| 19 | FAN_RPM | RO | |

## Honest note on the conditioning

The LFSR conditioning whitens the output; it is not itself entropy. The
entropy is the ring-oscillator jitter, and the health tests run on the raw
bits before any conditioning (as SP 800-90B requires) so a failing noise
source is caught even though the LFSR would make its output look random. For a
certified entropy claim you would run the full SP 800-90B estimators on the raw
stream; `trng_host.py selftest` runs the cheaper SP 800-22-style checks on the
conditioned output as a sanity gate.
