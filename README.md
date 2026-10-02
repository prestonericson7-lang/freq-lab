# freq-lab

**A GPS-disciplined space-weather monitoring stack, built on an FPGA.**

An 8-station VLF solar-flare receiver that watches U.S. Navy transmitters for the
ionospheric signature of a solar flare, lightning geolocation by time-of-arrival across
stations, a 30 MHz riometer for cosmic-noise absorption, and a live multi-station
dashboard — all timed to GPS and verified to the bit before it touches hardware.

Verilog on a Zynq-7020 (PZ7020-StarLite), firmware on Teensy 4.1 / ESP32 / STM32, Python
host tools and a browser dashboard, plus the supporting instrumentation: a NIST
SP 800-90B-tested hardware random number generator, a digital lock-in amplifier, a
coincidence photon counter and a two-channel correlator.

**Every design is verified before it touches hardware:**

- **343 automated checks, all green**, in one command: `python run_all_tests.py`
- Two FPGA bitstreams that **close timing in Vivado 2026.1 with zero critical warnings**
- FPGA simulations checked **bit-exact against independent Python reference models**
- Firmware signal processing lifted into headers and **tested on a PC** (no hand-waving)

I don't overclaim. **Nothing here has run on real hardware yet** — everything is compile-,
simulation- or model-verified, and [STATUS.md](STATUS.md) says exactly what's proven and what
still needs a bench. When I say it works, there's a logged test behind it.

**Start here:** [STATUS.md](STATUS.md) (what's verified) · [WIRING.md](WIRING.md) (every pin,
every board) · [docs/station-setup.md](docs/station-setup.md) (build the VLF receiver station).

Toolchain to build/run it yourself: Python 3 (numpy, pyserial, matplotlib for plots) for the host
tools; the Arduino IDE or `arduino-cli` with the Teensy, ESP32 and STM32 cores for the sketches;
Xilinx Vivado (tested 2026.1) for the FPGA bitstreams, or just load the pre-built `.bit` files in
`fpga/*/build/`; Icarus Verilog + g++ to re-run the simulations and PC tests.

| Folder | Runs on | What it does | Checked |
|---|---|---|---|
| `teensy/resonance_sweep` | Teensy 4.1 + MPU-6050 + any exciter | Swept-sine lock-in, 0.05–400 Hz: resonance curve, Q, phase-locked tracking, ring-down | Compiles. Against a simulated resonator: f0 37.281 vs 37.271 Hz, Q 18.0 vs 18.0, tracker followed a drift from 37.3 to 38.0 Hz, ring-down time exact |
| `teensy/body_resonance` | Teensy 4.1 + up to six MPU-6050 | Vibration spectra, transfer function between two points on a body or structure, heartbeat recoil (ballistocardiogram) | Compiles. Simulated: 7 Hz line read 4997 µg of 5000; transfer peak 3.95 at 4.88 Hz vs 4.03 at 4.92; heart rate 72.0 of 72 |
| `teensy/hemisync_gen` | Teensy 4.1 + headphones | Six binaural tone pairs, exact beat frequencies, pink/surf noise, blind binaural / monaural / sham sessions | Compiles. Synth output measured: 100.000 / 104.000 Hz, beat 4.000 Hz, noise −10.0 dB |
| `teensy/elf_logger` | Teensy 4.1 + amplifier + coil or electrodes, GPS optional | 0.25–45 Hz spectrum logger with mains rejection, SRI's 19 bands, the first three Earth-ionosphere modes, GPS-stamped raw waveform | Compiles. Simulated front end with 1 V of mains on it: a 2 µV line read 1.4135 µV rms of 1.4144; noise floor within 3 %; 240 Hz mains harmonic did not fold into the band; 204 of 204 GPS second marks placed on the right sample |
| `teensy/strobe_driver` | Teensy 4.1 + LED(s) | Flashing-light driver, 0.1–100 Hz, randomized trial blocks (SRI's 0 / 6 / 16 Hz layout), glide, flash and trial markers, light-sensor check | Compiles. Every 50 µs tick simulated: 16 Hz period and width exact, 36 trials of exactly 10.00000 s with 0 / 60 / 160 flashes, blind order kept off the screen and matching the SD key |
| `teensy/cc1101_scanner` | Teensy 4.1 + CC1101 | Sub-GHz spectrum scanner, 300–928 MHz, variable RBW, peak hold, CSV sweep export | Compiles (Teensyduino 1.62, RadioLib 7.7.1) |
| `fpga/lockin` | PZ7020-StarLite (Vivado) | DDS + sigma-delta drive + on-chip 12-bit ADC + digital lock-in + resonance tracker, UART-controlled, up to about 100 kHz | Simulates end to end (iverilog): amplitude within 0.1 %, lag within 0.4°, tracker pulled 19.0 kHz onto a 20.0 kHz resonance. Synthesizes for xc7 (yosys). Vivado 2026.1: builds, timing met (WNS +11.9 ns at 50 MHz), 3.6 % LUTs, 0 critical warnings, ~0.13 W. Not yet run on hardware |
| `fpga/photon_counter` | PZ7020-StarLite (Vivado) | Coincidence photon counter: dual detectors, programmable coincidence window, singles + coincidence counts, elapsed clock, fan PWM, UART register protocol | Simulates end to end (iverilog): 12 checks pass — ID read, window write/readback, start/stop, singles and coincidence counts checked exactly (same-clock pairs counted, each pulse in at most one pair, none outside the window), error handling. Vivado 2026.1: builds, timing met (WNS +11.6 ns), 1.1 % LUTs, 0 critical warnings, ~0.11 W. Not yet run on hardware |
| `fpga/correlator` | PZ7020-StarLite (Vivado) | Two-channel cross-correlator: dual XADC (VAUX1 + VAUX9, ~195 kS/s each), DC-removed accumulation, circular-buffer lag queries, GPS PPS timestamping, fan PWM, UART register protocol | Simulates end to end (iverilog): 7 tests pass — ID read, live ADC reads, 100-sample acquisition with sin/cos test signals, accumulator readback (N=100 exact), lag query, error handling, GPS PPS counter. Vivado 2026.1: builds, timing met (WNS +8.1 ns), 2.4 % LUTs, 0 critical warnings, ~0.12 W; channel B is JM1-9/11 (E18/E19). Known limitation: nonzero-lag queries wrap around the 256-sample buffer (lag-0 coherence is exact). Not yet run on hardware |
| `fpga/vlf_sid` | PZ7020-StarLite (Vivado) | 8-station VLF solar-flare (SID) receiver: XADC at 390.6 kS/s, eight NCO + CIC channels (NLK, NPM, NAA, NML, NAU, NWC, JJI + a noise reference), power per GPS second, lightning-spike and clipping monitors, UART register protocol | Simulates end to end (iverilog): 33 testbench checks pass, and every decimated I/Q value of all 8 channels matches a bit-exact Python model; agrees with an ideal floating-point receiver within 0.003 dB, 80 dB rejection 400 Hz off-channel. Vivado 2026.1: builds, timing met (WNS +6.3 ns), 17 % LUTs, 50 DSPs, 0 critical warnings, ~0.15 W. Not yet run on hardware |
| `fpga/vlf_sid_v2` | PZ7020-StarLite (Vivado) | vlf_sid plus lightning (sferic) capture: threshold trigger, GPS-second + 20 ns clock stamps, 64-sample pre/post-trigger snapshots, 32-event FIFO, UART registers 40–49 and 60–7F; for a time-of-arrival lightning network | Simulates end to end (iverilog): 59/59 testbench checks (all of v1's + events over the UART), 19/19 Python checks (all of v1's + triggers at the predicted samples, every event's GPS second, exact 20 ns count, polarity, peak and all 64 snapshot samples vs the stimulus). Vivado 2026.1: builds, timing met (WNS +4.5 ns), 17.9 % LUTs, 7.5 BRAM tiles, 50 DSPs, 0 critical warnings, ~0.16 W. Not yet run on hardware. See `fpga/vlf_sid_v2/README.md` |
| `stm32/rf_resonance` | STM32 + SX1262 (DX-LR20 kit) | RF resonance probe: CW + swept-frequency transmission from one radio to another, measures insertion loss vs frequency to find cavity or tissue resonances | Compiles with arduino-cli (STM32 core 3.0.0 + RadioLib 7.7.1) for F103C8 (63.8 KB, 97 % flash) and F411CE: core 3.x needs RadioLib's pin casts, passed by `stm32/rf_resonance/build.bat` (the plain arduino-cli build fails in RadioLib's HAL). MAX_SWEEP 500 to fit the F103C8's 20 KB RAM; raise to 2000 on boards with ≥64 KB |
| `tools/lockin_host.py` | PC (Python + pyserial) | Sweeps, Q, tracking and delay calibration for the FPGA lock-in | Run against a software model of the FPGA: f0 439.93 of 440, Q 39.97 of 40 |
| `tools/photon_host.py` | PC (Python + pyserial) | Live counts, run management, accidental subtraction, CSV export for the photon counter | Not yet tested against hardware |
| `tools/correlator_host.py` | PC (Python + pyserial + numpy) | Acquisition, lag sweep, coherence computation, CSV/plot export for the correlator | Not yet tested against hardware |
| `tools/vlf_host.py` | PC (Python + pyserial, matplotlib for plots) | VLF receiver: info, input level, 10–50 kHz station scan, 1-second CSV logging with GOES X-ray flux, plots; `--station/--push` upload to the server; `sferics` (v2) logs strokes with UTC to the microsecond + 20 ns clock + snapshot | Run against a simulated board: scan found and named the stations, GPS sample-clock correction worked (+29.7 ppm of +30), GOES feed fetched and plotted. `tools/test/test_vlf_host.py` (fake v2 board + stub server): 16/16 — uploads, three strokes stamped within 20 ns of the injected times, queued delivery through an outage. Not yet tested against hardware |
| `tools/sid_detect.py` | PC (Python) | Solar-flare (SID) detector: trend-following baseline, 20-s mean onset confirmation, frozen baseline during events, 2-channel sustain, GOES confirmation, multi-station flag; also runs standalone over a CSV | `tools/test/test_sid_detect.py`, a synthetic day with 9 dB sunrise/sunset ramps, 200 lightning spikes, a one-channel step and two flares (+3.2, −2.0 dB): 14/14 — both flares found within 90 s of onset at +3.55 / −2.03 dB, 0 false events, GOES class C4.2 confirmed on the right one, multi-station flag right |
| `tools/sid_server.py` + `dashboard/index.html` | PC (Python stdlib only) | Station network server: SQLite store, JSON API, background flare detector + GOES fetcher, cross-station stroke matching (3 ms), and a dependency-free live dashboard (levels, GOES, strokes/min, alerts, time-of-arrival table) | `tools/test/test_sid_server.py`: two stations × 3 h ingested, strokes matched across stations to 1200 µs, detector creates GOES-confirmed multi-station alerts without duplicates, every endpoint and the dashboard served, bad input refused |
| `tools/riometer_host.py` | PC + RTL-SDR (`rtl_power`) | 30 MHz riometer / 245 MHz solar-burst logger: band power with interferer rejection, sidereal quiet-day curve (upper quartile over 7 days), absorption or enhancement in dB, CSV, upload | `tools/test/test_riometer.py`, 3 sidereal days synthetic with a +20 dB interferer bin and a 2.5 dB absorption event: 8/8 — band power within 0.23 dB of the true sky with the interferer dropped, the event read 2.56 dB, nothing outside it above 0.25 dB |
| `torsion_pendulum` | 2-liter bottle + 3D-printed parts + ESP32-S3 CAM (GC2145) + PC | SRI's 1974 bell-jar torsion pendulum test: sealed, static-shielded, backlit rotor tracked by the camera (rotor angle against two reference squares), blind random PUSH/REST schedule, hash-chained session log, exact randomization statistics, empty-room and warm-bottle controls | Parts watertight and fit-checked; tracker on synthetic frames 0.1-0.2 mrad/frame, camera movement cancelled; firmware compiles (Arduino-ESP32 3.3.12 and 2.0.17); statistics give 5 % false alarms on 200 simulated null sessions. Not yet run on hardware. See `torsion_pendulum/README.md` |
| `teensy/fpga_uart_bridge` | Teensy 4.1 | USB to FPGA serial pass-through | Compiles |
| `teensy/vlf_station` | Teensy 4.1 + vlf_sid FPGA + GPS | GPS-aware station bridge: passes the FPGA UART to the PC AND reads GPS NMEA (Serial2, pin 7) + 1PPS (pin 2) so `vlf_host.py --gps-bridge` stamps windows with true GPS UTC; `@gps`/`@pps`/`@id` bridge commands. For the full-rig station (FPGA + Teensy + ESP32 + SDR + GPS) — see `docs/station-setup.md` | Compiles (50 KB). NMEA parser PC-tested (`teensy/vlf_station/test/pc_test.cpp`, 11/11). Not yet run on hardware |
| `teensy/schumann_monitor` | Teensy 4.1 + ADS1256 (24-bit) + coil preamp + ball antenna | Two-channel ELF monitor: H and E multiplexed at 1000 SPS each, mains notches + 45 Hz low-pass, 4096-point Welch spectra, the five Schumann modes (centre Hz, pT/√Hz, floor ratio), E/H wave impedance with coherence at modes 1–2, GPS stamps, SD, JSON reports | Compiles (Teensyduino 1.62): 105 KB code, 145 KB RAM1 free. Filters and mode finder are the ones verified in `elf_logger` on a simulated front end. Not yet run on hardware |
| `esp32/schumann_gen` | ESP32 (DAC) or ESP32-S3 (PWM) + DC-coupled coil driver | Calibrated ELF field generator: 7.83 Hz + 4 harmonics (8 kS/s synthesis), current-sense field calibration in µT (coil geometry), closed-loop `target`, blinded sham sessions with a SHA-256 commitment, magnetophosphene sweep/ramp mode with a threshold button, phone web page (AP) | Compiles for esp32 (74 % flash, 15 % RAM) and esp32s3. Not yet run on hardware |
| `tools/schumann_host.py` | PC | Monitor host: reports to CSV, one-line summaries, upload to the server as kind `schumann`, live spectrum, 24 h+ history plots | `tools/test/test_schumann_host.py` (fake monitor + stub server): 6/6 |
| `teensy/rf_survey` | Teensy 4.1 + AD8318 log detector (1 MHz–8 GHz) | RF survey meter: 50 kS/s detector sampling, power-mean/peak dBm and µW/cm² per second (antenna aperture), envelope spectra 0.06–125 Hz and 12 Hz–25 kHz with named modulation lines (Wi-Fi beacon 9.77 Hz, LTE, DECT, Bluetooth, mains), pulse-interval histogram with a hysteresis detector, two-point calibration, GPS-tagged survey rows | Compiles (102 KB). Arithmetic PC-tested (`teensy/rf_survey/test/pc_test.cpp`, 18/18): calibration fit, aperture maths, power-mean level, pulse histogram on a synthetic Wi-Fi+LTE waveform (10 ms frame on top at 6 dB, 97 beacon intervals at 25 dB), peak finder and naming. Not yet run on hardware |
| `tools/rf_survey_host.py` | PC | Survey CSV, map coloured by µW/cm² (or level vs time without GPS), envelope-spectrum capture and plot, pulse listing | `tools/test/test_rf_survey_host.py` (fake meter): 7/7 |
| `teensy/sleep_cue` | Teensy 4.1 + EEG front end (+ EMG), earbuds | Closed-loop slow-oscillation cueing (Ngo 2013) + targeted memory reactivation: 30-s deep-sleep gate, SO trough detector (filter-delay compensated), cue 0.5 s after the trough, 2.5 s refractory + 6/min cap + arousal pause, SHA-256-sealed word-pair split, blinded stim/sham nights | Compiles (115 KB). Signal processing PC-tested (`teensy/sleep_cue/test/pc_test.cpp`, 22/22) on a synthetic night: gate, SO detection (≥95 µV caught, ≤65 µV rejected), cue timing (worst 8 ms), rate cap, arousal pause, SHA-256 vectors. Not yet run on hardware |
| `tools/sleep_cue_score.py` | PC | Blind scoring of a word-pair night: checks the SHA-256 commitments before scoring, permutation test of cued vs uncued recall | `tools/test/test_sleep_cue_score.py`: 8/8 (5 % null, power, commitment refusal) |
| `teensy/kirlian_logger` | Teensy 4.1 + GSR/FSR/photodiode front ends (HV rig external) | Controlled Kirlian: logs skin conductance, contact pressure and corona brightness per exposure, fits corona = moisture + pressure and reports R² and the residual; interlocks the HV (enable only during an armed exposure) but never generates it | Compiles (88 KB). Fit PC-tested (`teensy/kirlian_logger/test/pc_test.cpp`, 8/8): recovers coefficients, drops R² and recovers a hidden component, refuses singular/collinear inputs. Not yet run on hardware |
| `teensy/nuisance_logger` | Teensy 4.1 + MPU-6050 + fluxgate + mic + mains pickup | Logs a signal beside vibration, B-field, sound and mains and vetoes the ordinary cause: per-sample windowed correlation flags when an excursion is explained by a nuisance (the 8 Hz mount resonance, #133; 60 Hz everywhere) | Compiles (92 KB). Veto core PC-tested (`teensy/nuisance_logger/test/pc_test.cpp`, 10/10): genuine signal not vetoed, vibration-as-signal and continuous/impulsive mains vetoed, no false blame. Not yet run on hardware |
| `fpga/trng` | PZ7020-StarLite (Vivado) | Ring-oscillator TRNG (#73): 16 ring oscillators → SP 800-90B health tests (Repetition Count + Adaptive Proportion) → von Neumann debiaser → LFSR conditioning → 32-bit words → FIFO; on-chip temperature and VCCINT logged; output withheld while a health test is tripped | Simulates end to end (iverilog, entropy backdoor): 9 testbench + 33 Python checks, every output word, counter, health flag and FIFO read bit-exact vs a model across balanced/stuck/biased streams. Vivado 2026.1: builds, timing met (WNS +12.6 ns), 570 LUTs (1 %), 0 critical warnings, the 16 ring-oscillator loops flagged allowed, ~0.11 W. Not yet run on hardware. See `fpga/trng/` |
| `tools/trng_host.py` | PC | TRNG host: info + die temp, word/byte streaming, live health, config, `selftest` (125 kB sample, monobit/runs/chi-square + die-temp log, the SRI 1980-style certification run) | `tools/test/test_trng_host.py` (fake board): 14/14 (health-alarm stop, config, streaming, stats pass good / fail biased) |
| `tools/blindhash.py` | PC | Blind randomization + pre-posted SHA-256 commitment (#158): draw a schedule, seal it, verify it, score it (two-sided permutation by default; one-sided only with a pre-registered direction) | `tools/test/test_blindhash.py`: 17/17 (tamper-evident, calibrated null, order-independent p — a real statistical bug the red-team pass caught) |
| `tools/shield_audit.py` | PC | Shielding attenuation vs frequency (#134, #160) from lock-in / CC1101 / VLF sweep CSVs, with skin-depth theory: shows the ELF hole in an RF-tight cage | `tools/test/test_shield_audit.py`: 10/10 (attenuation maths, format reading, copper/mu-metal skin depth) |
| `tools/cavity_readout_host.py` | PC + SX1262 pair / RTL-SDR | Passive-cavity resonator test-tone readout (#190, #191, the Great Seal demo): recover a known test tone off a tuned cavity, measure recoverable SNR vs illuminator power | `tools/test/test_cavity_readout.py`: 11/11 (carrier finding, AM/PM demod, SNR-vs-depth, power-step threshold) |

## Teensy sketches: first run

Arduino IDE, board Teensy 4.1, upload, open the serial monitor, type `help`.

- **resonance_sweep**: fix the MPU-6050 and the exciter to the object, then
  `amp 0.2`, `sweep 5 200 100 log`, `ringdown <peak Hz> 5 5`, `track 60`.
- **body_resonance**: `scan`, then `spec 60` for spectra, `tf 0 240` for the transfer
  function from sensor 0 to the others, `bcg 2 120` for the heartbeat recoil at sensor 2.
- **hemisync_gen**: `preset patent4`, `vol 0.15`, `run 10`. For a test that means something:
  `blind patent4 10`, write down what you noticed, then `reveal`.
- **elf_logger**: `gain 1000`, `mains 60`, `raw on`, `start 300`. `live` prints the full spectrum.
- **strobe_driver**: read the warning at the top of the file first. `arm`, `arm yes`,
  `rate 10`, `on`, `check 4`, `off`. SRI's layout: `blind on`, `sri`.
- **cc1101_scanner**: `sweep 433 434 12` scans 433–434 MHz with 58 kHz RBW.
  `peak`, `hold on`, `csv` dumps the last sweep.
- **schumann_monitor**: `gain h 1000`, `pga h 8`, `coil 30000 0.0158`, `ecal 1.0`,
  `start 60`. `live` prints the full H/E spectrum; `json on` for the host tool.
- **rf_survey**: `cal -20` then `cal -40` with known levels, `ant 2.44 2.0`,
  then `level`, `spec slow 60` (Wi-Fi beacon at 9.77 Hz), `pulses 30`, `survey on`.
- **sleep_cue**: battery power only with electrodes on. `gain 1000`, `vol 0.05`,
  `test` to set the volume, `pairs 60`, then `night` (blind) or `mode sham` + `start`.
- **nuisance_logger**: `window 64` (for 60 Hz) or `256` (for 8 Hz), `start 300`;
  `report` shows which nuisance explained the signal excursions.
- **kirlian_logger**: build the interlocked HV rig first. `gsrcal`, `vary moist 8`,
  `expose` each, then `fit` for R² and the residual. HV safety is on you.
- **vlf_station**: the GPS-aware bridge for the VLF station — no serial commands of
  its own; flash it, then talk to the FPGA through it with `vlf_host.py`. See below.

## STM32 + SX1262: RF resonance probe

Arduino IDE with STM32 board package + RadioLib. Wire the DX-LR20 kit's SPI/BUSY/DIO1
pins to the STM32 (pin map at the top of `rf_resonance.ino`). Power both radios, connect
one to each STM32, serial monitor on each.

On the transmitter: `mode tx`, `cw 915.0`, then on the receiver: `mode rx`, `measure`.
For a swept measurement: transmitter `sweep 900 928 0.1`, receiver `sweep 900 928 0.1`.

The idea is to put a sample (tissue, water, crystal, cavity) between the two antennas and
look for absorption dips or resonance peaks in the insertion-loss-vs-frequency curve.

## FPGA lock-in

### Wiring (JM1, even-numbered row)

```
JM1-2  3.3 V        JM1-4  GND
JM1-6  ADC +   0 .. 1.0 V ONLY          JM1-8  ADC -   -> GND at the sensor
JM1-10 drive   -> 1 kΩ -> node -> 10 nF -> GND      (node = analog sine, 0..3.3 V)
JM1-12 sync    square wave at the drive frequency
JM1-14 FPGA TX -> adapter RX (or Teensy pin 0)
JM1-16 FPGA RX <- adapter TX (or Teensy pin 1)
```

Fan pins JM1-5 / JM1-7 behave exactly as in `fan_top.v`, so the fan keeps running.

Loop-back for a first test and for `cal`: node -> 33 kΩ -> JM1-6, and 10 kΩ from JM1-6 to GND
(divides by 4.3, so even 3.3 V at the node is 0.77 V at the pin). JM1-8 to JM1-4.

A sensor goes in the same way: bias JM1-6 to about 0.5 V (56 kΩ to 3.3 V, 10 kΩ to GND),
couple the sensor through 100 nF, and keep its swing under ±0.4 V.

### Build and load

```
cd fpga/lockin
vivado -mode batch -source build.tcl      # writes lockin_top.bit
vivado -mode batch -source program.tcl    # boot jumper on JTAG, JTAG Type-C connected
```

LED1 blinks once a second when the fabric is running. LED2 toggles on every finished
lock-in window.

### Use

```
pip install pyserial
python tools/lockin_host.py COM7 id
python tools/lockin_host.py COM7 cal
python tools/lockin_host.py COM7 sweep 50 5000 200 --log --amp 0.2 --out curve.csv
python tools/lockin_host.py COM7 track 440 --step 0.05
python tools/lockin_host.py COM7 off
```

Drive range: micro-hertz steps up to about 100 kHz (the RC filter's corner is 16 kHz;
change the capacitor for higher drive frequencies). ADC rate 390.6 kS/s.

### Re-run the simulation

```
cd fpga/lockin
iverilog -g2005 -DSIMULATION -o tb sim/tb_lockin.v hdl/*.v && vvp tb
```

## FPGA photon counter

Coincidence counter for biophoton detection experiments (Gurwitsch mitogenetic radiation,
SRI-style photomultiplier setups). Two detector inputs, programmable coincidence window,
64-bit singles and coincidence counters, elapsed clock for rate computation.

### Wiring (JM1, even-numbered row)

```
JM1-2  3.3 V             JM1-4  GND
JM1-6  det_a  detector A pulse input (3.3 V logic, rising edge = photon)
JM1-8  det_b  detector B pulse input
JM1-10 gate   counting enable (active high, tie to 3.3 V for always-on)
JM1-14 FPGA TX -> adapter RX
JM1-16 FPGA RX <- adapter TX
```

Fan on JM1-5 / JM1-7.

### Build and load

```
cd fpga/photon_counter
vivado -mode batch -source build.tcl
vivado -mode batch -source program.tcl
```

### Use

```
python tools/photon_host.py COM7     # connects, prints the ID, then gives a photon> prompt
photon> window 10                    # coincidence = A and B within ±10 ticks (±200 ns)
photon> run 10                       # count for 10 s, then print singles, coincidences, elapsed
photon> rate                         # counts per second
photon> accidental                   # chance overlaps vs real coincidences
photon> csv run1.csv 60 5            # five 60 s runs saved to run1.csv
photon> quit
```

### Re-run the simulation

```
cd fpga/photon_counter
iverilog -o tb sim/tb_photon.v hdl/*.v && vvp tb
```

## FPGA two-channel correlator

Cross-correlator for ELF two-station experiments. Two on-chip XADC channels
(VAUX1 + VAUX9, ~195 kS/s each), DC-removed accumulation via the identity
`sum((a-ma)(b-mb)) = sum(ab) - sum(a)*sum(b)/N`, circular buffer for lag queries
up to ±MAX_LAG samples, GPS PPS timestamping for multi-station synchronization.

### Wiring (JM1, even-numbered row)

```
JM1-2  3.3 V             JM1-4  GND
JM1-6  VAUX1+  channel A analog input (0..1.0 V)
JM1-8  VAUX1-  -> GND at sensor
JM1-9  VAUX9+  channel B analog input (0..1.0 V)   (odd row, ball E18)
JM1-11 VAUX9-  -> GND at sensor                    (odd row, ball E19)
JM1-18 gps_pps GPS 1PPS input (3.3 V logic, optional)
JM1-14 FPGA TX -> adapter RX
JM1-16 FPGA RX <- adapter TX
```

Fan on JM1-5 / JM1-7.

### Build and load

```
cd fpga/correlator
vivado -mode batch -source build.tcl
vivado -mode batch -source program.tcl
```

### Use

```
python tools/correlator_host.py COM7  # connects, prints the ID, then gives a corr> prompt
corr> adc               # live reading of both inputs (about 2048 = 0.5 V on the bias network)
corr> window 100000     # samples per window: 100000 = 0.5 s, up to about 1900000 = 10 s
corr> stream 5          # 5 windows back to back, coherence (-1 .. +1) for each
corr> lagplot 16        # last window's correlation at offsets -16..+16 samples (5.12 us each)
corr> csv out.csv 50    # 50 windows saved to out.csv
corr> quit
```

### Re-run the simulation

```
cd fpga/correlator
iverilog -DSIMULATION -o tb sim/tb_correlator.v hdl/*.v && vvp tb
```

## FPGA VLF solar-flare receiver (vlf_sid)

Listens to 8 Navy VLF transmitters at once and logs their strength every second, so
a solar flare shows up as a step in the daytime stations at the moment the GOES X-ray
curve rises. Same JM1 wiring as the lock-in (antenna preamp on JM1-6/8, UART on 14/16,
optional GPS PPS on 18). Full details, front end and register map:
[fpga/vlf_sid/README.md](fpga/vlf_sid/README.md).

```
fpga\build_vlf_sid.bat                          (double-click: Vivado build)
python tools/vlf_host.py COM7 level             set the preamp gain
python tools/vlf_host.py COM7 scan              10-50 kHz spectrum, names the stations
python tools/vlf_host.py COM7 log --goes        1 line per second + GOES X-rays to CSV
python tools/vlf_host.py plot vlf_2026-10-02.csv --goes goes_2026-10-02.csv
```

**Version 2 (`fpga/vlf_sid_v2`)** adds lightning time-of-arrival capture (20 ns
GPS-relative stamps) on top of everything above; build it with
`fpga\build_vlf_sid_v2.bat`. **For a full receiver station** (FPGA + a GPS-aware
Teensy bridge + an RTL-SDR riometer complement + GPS/PPS), including the exact
wiring, the GPS convention and `vlf_host.py --gps-bridge` for true GPS time, see
[docs/station-setup.md](docs/station-setup.md). Product-line kit docs for the
space-weather, Schumann, RF-survey and biofeedback lines are in [docs/](docs/).
