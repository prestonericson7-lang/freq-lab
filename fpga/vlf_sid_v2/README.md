# vlf_sid_v2 — VLF solar-flare receiver + lightning time-of-arrival capture

Version 2 of [`../vlf_sid`](../vlf_sid/README.md). Everything in v1 is unchanged
(same 8 station channels, same windows, same register map, same wiring); v2 adds
a **sferic capture** block so a network of these receivers can locate lightning
by time of arrival, and the host software can upload both to `tools/sid_server.py`.

v1 stays in its own folder with its own verified bitstream. Build and load v2
from here; `vlf_host.py` recognises both (ID 0x564C0001 = v1, 0x564C0002 = v2).

## What the sferic capture does

A lightning stroke arrives at the antenna as a broadband impulse much larger
than the steady station carriers. `hdl/sferic_capture.v` watches every raw ADC
sample (390,625 S/s). When |code − 2048| reaches `SF_THRESH` it records:

* **which GPS second** (the PPS count) and **how many 50 MHz clocks** had
  passed since that second's PPS edge: a 20 ns stamp;
* a **64-sample snapshot**: 16 samples before the trigger, the trigger sample,
  47 after (2.56 µs per sample), so the host can interpolate the arrival to a
  fraction of a sample and cross-correlate stations;
* the largest |x| in the snapshot and its sign (stroke polarity).

Then it ignores triggers for `SF_HOLDOFF` samples (default 5 ms: one stroke =
one event). Up to 32 events wait in a block-RAM FIFO; triggers that arrive while
it is full are counted in `SF_LOST`. The host reads the oldest event's header
and snapshot (36 registers, one USB round trip) and pops it.

Two stations stamping the same stroke give the arrival-time difference
directly (1 µs = 300 m). The stamp is the clock at which the sample was
registered; the XADC's conversion delay is the same at every station and
cancels in the difference.

## New registers (UART, same protocol as v1)

| Addr | Name | | |
|---|---|---|---|
| 40 | SF_THRESH | RW | |code−2048| at or above this triggers; 0 = off (default 0) |
| 41 | SF_HOLDOFF | RW | samples ignored after a trigger (default 1953 = 5 ms) |
| 42 | SF_COUNT | RO | events captured since power-up |
| 43 | SF_NEV | RO | events waiting (0..32) |
| 44 | SF_POP | RO | reading it drops the oldest event; returns SF_NEV before the drop |
| 45 | EV_PPS | RO | oldest event: PPS count at the trigger |
| 46 | EV_CLK | RO | bit 31 = a PPS had been seen; [25:0] clocks since that PPS (20 ns each) |
| 47 | EV_INFO | RO | {pre-trigger samples = 16, length = 64, 3'b0, polarity (1 = negative), |peak| (12 bit)} |
| 48 | EV_SEQ | RO | event number |
| 49 | SF_LOST | RO | triggers dropped because the FIFO was full |
| 60–7F | EV_SNAP | RO | 64 raw codes, two per word {4'b0, s[2k+1], 4'b0, s[2k]}; s[16] is the trigger sample |

ID reads 0x564C0002. Registers 00–3F are exactly v1's.

## Use

```
fpga\build_vlf_sid_v2.bat                         Vivado build (about 6 minutes)
   Vivado Tcl shell:  cd P:/Downloads/freq-lab/fpga/vlf_sid_v2 ; source program.tcl

python tools/vlf_host.py COM7 info                shows v2 and the capture state
python tools/vlf_host.py COM7 level               set the preamp gain as before
python tools/vlf_host.py COM7 sferics --thresh 600
       one line per stroke: UTC time to the microsecond, 20 ns clock, polarity, |peak|;
       CSV with the 64-sample snapshot; add --station NAME --push http://server:8750
       to feed the network server (tools/sid_server.py)
python tools/vlf_host.py COM7 log --station NAME --push http://server:8750
       the usual 1-s station log, uploaded as well
```

Choosing the threshold: run `level` and note the input RMS; the station carriers
sum to a few hundred counts peak. Set `--thresh` 2–3× the peak of the quiet
signal (600 is right for an RMS around 120). Near strokes exceed 1000 counts;
the clipping LED tells you the gain is too high.

Absolute time: the host maps the board's PPS count to UTC using the PC clock
(must be within ±0.5 s, e.g. NTP). Inside the second the stamp is the GPS PPS
plus the 20 ns clock count, independent of the PC.

## Verification

Simulation (`sim/`, Icarus Verilog; Python checker needs numpy):

```
cd fpga/vlf_sid_v2 && python3 sim/gen_files.py
iverilog -DSIMULATION -o sim/tb_vlf sim/tb_vlf.v hdl/*.v && cd sim && vvp tb_vlf && python3 check_vlf.py
```

The stimulus is v1's (MSK station, tones, noise, clip burst) plus two synthetic
strokes: damped 8 kHz oscillations of 1400 counts, positive-first at sample
45000 and negative-first at 75000. The clipping burst at sample 50000 is a
third, legitimate trigger (|peak| = 2048, negative).

* testbench: **59/59** checks — all 33 of v1's, the new register defaults and
  write/readback, three events read over the UART with the right GPS second,
  PPS-seen flag, PRE/LEN, polarity, the 2048 clip peak, sequence numbers, FIFO
  counts before and after each pop, pop on an empty FIFO;
* `check_vlf.py`: **19/19** — all 16 of v1's (bit-exact DDC, windows, registers,
  physics) plus: triggers at exactly the samples the threshold/hold-off rule
  predicts (45007, 50000, 75002), and for every event the GPS second, the
  20 ns clock count (exact), polarity, |peak|, SEQ and all 64 snapshot samples
  equal to the stimulus and the logged PPS/trigger times.

Vivado 2026.1 (xc7z020clg400-2, Oct 1 2026): exit code 0, **0 critical
warnings**, all timing constraints met at 50 MHz (setup slack +4.46 ns, hold
+0.031 ns, 0 failing of 46,642 endpoints). 9,535 LUTs (17.9 %), 14,561
flip-flops (13.7 %), 50 DSPs, 7.5 block-RAM tiles (5.4 %). All 11 pins placed
as constrained (A20 uart_rx, B19 uart_tx, C20 gps_pps, E17/D18 antenna, G14 key,
H16/H17 fan, R19/V13 LEDs, U18 clock). 0.155 W, junction 26.8 °C. Not yet run
on hardware.

Host software: `tools/test/test_vlf_host.py` runs `vlf_host.py` against a
software model of this register file (`tools/test/fake_vlf_board.py`) and a
stub server: **16/16** — both IDs accepted, `log --push` rows delivered with
the right station and levels, three injected strokes logged with raw stamps
within 20 ns of the injected times, polarity, peak, snapshot and sub-sample
refinement, uploads, and queued delivery through a server outage.

## Files

| File | |
|---|---|
| `hdl/sferic_capture.v` | the new block (trigger, 20 ns stamps, snapshot RAM, FIFO, read port) |
| `hdl/vlf_top.v`, `hdl/cmd_regs_vlf.v` | v1's with the block instantiated and the registers above |
| `hdl/*` others, `constraints/vlf_jm1.xdc` | unchanged from v1 |
| `sim/gen_files.py` | v1's stimulus + the two strokes |
| `sim/tb_vlf.v`, `sim/check_vlf.py` | v1's checks + the event checks |
| `build.tcl`, `program.tcl`, `../build_vlf_sid_v2.bat` | build and load; output `build/vlf_sid_v2.bit` |
