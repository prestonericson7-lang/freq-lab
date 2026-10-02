# Preston E
prestonericson7@gmail.com · github.com/prestonericson7-lang

**Embedded Systems / FPGA / Instrumentation Engineer** — self-taught builder who ships verified hardware-software systems end to end: Verilog on Zynq-7020 FPGAs, real-time firmware on Teensy 4.1 / ESP32 / STM32, Python data pipelines and servers, and the test harnesses that prove each one works before it touches hardware.

## Core Skills
FPGA: Verilog-2005, Xilinx Vivado 2026.1 (synthesis, place-and-route, timing closure, XADC, DSP48, block RAM), Icarus Verilog testbenches, bit-exact Python reference models · Embedded: C/C++ on Teensy 4.1 (ARM Cortex-M7, CMSIS-DSP), ESP32 / ESP32-S3, STM32F1/F4/H7, Arduino-CLI and PlatformIO, SPI/I2C/UART, GPS PPS timing, RadioLib (SX1262, CC1101) · DSP: digital down-conversion, NCO + CIC decimation, lock-in detection, cross-correlation, FFT/Welch spectra, IIR notch/Butterworth design, sub-sample time-of-arrival · Software: Python 3 (numpy, scipy, pyserial, matplotlib, sqlite3, http.server), JavaScript/HTML5 canvas, Git · Domains: VLF/ELF radio, space-weather instrumentation, SDR (RTL-SDR, rtl_power), sensor fusion (MPU-6050, fluxgate, EEG front ends), game engines (Unreal Engine 5.7 C++, Unity 6)

## Projects

### freq-lab — Open Instrumentation Stack for Space-Weather and Low-Frequency Measurement (2026)
Zynq-7020 FPGA + Teensy 4.1 + Python. Receivers, loggers, a station network server and kit documentation, built as a product line.
- Designed an 8-channel VLF solar-flare (SID) receiver in Verilog: on-chip 12-bit ADC at 390,625 S/s, eight 32-bit NCO + 3-stage CIC digital down-converters (1024× decimation, −80 dB rejection 400 Hz off-channel), GPS-second windowing; Vivado build meets 50 MHz timing (WNS +6.3 ns), 17% LUTs, 50 DSPs, 0 critical warnings.
- Verified it to the bit: 33/33 Verilog testbench checks and every decimated I/Q sample of all 8 channels identical to an independent Python integer model; fixed-point receiver within 0.003 dB of an ideal floating-point receiver.
- Added lightning time-of-arrival capture (v2): threshold trigger, 20 ns GPS-relative timestamps, 32-event FIFO with 64-sample pre/post-trigger snapshots in block RAM; 59/59 testbench and 19/19 model checks, timestamps exact to the clock, timing met (WNS +4.5 ns) in a single build.
- Found and fixed a latent testbench bug (hex replies beginning with "E" misread as errors) by logging the UART byte stream; re-verified the baseline design before touching it.
- Wrote the station host software: batched register reads (one USB round trip per second), GPS-corrected sample clock (±50 ppm crystal trimmed against PPS), NOAA GOES X-ray feed integration, sub-sample arrival refinement, resilient HTTP uploader with queue-and-retry (verified through a simulated server outage); 16/16 tests against a software model of the board.
- Built a solar-flare detector with a trend-following baseline (two half-window medians, linear extrapolation) and 20-second mean onset confirmation: on a synthetic day with 9 dB sunrise/sunset ramps, 200 lightning spikes and two flares (+3.2 dB, −2.0 dB), detects both within 90 s of onset with 0 false alarms; 14/14 checks including GOES confirmation and multi-station correlation.
- Built a stdlib-only central server (SQLite, JSON API, background detector, GOES fetcher, cross-station stroke matching to 3 ms) and a dependency-free live dashboard (HTML5 canvas, auto-refresh, 1 h–3 d ranges, alert and time-of-arrival tables); 20/20 end-to-end tests: 21,600 rows from two stations, a stroke matched across stations to 1,200 µs, GOES-confirmed multi-station alerts with no duplicates, malformed input refused.
- Wrote an RTL-SDR riometer / solar-burst logger (band power with narrowband-interferer rejection, sidereal-day quiet curve as a 7-day upper quartile): on 3 synthetic sidereal days a 2.5 dB absorption event read 2.56 dB and the quiet baseline held within 0.25 dB; 8/8 tests.
- Documented the product line as sellable kits (antenna + preamp design and BOM, acceptance test, network setup, pricing tiers) with the limits stated plainly.
- RF survey meter on Teensy 4.1 + AD8318 log detector: 50 kS/s sampling, power-mean dBm and µW/cm² from antenna aperture, two envelope spectra (0.06–125 Hz, 12 Hz–25 kHz) with a modulation-line identifier (Wi-Fi beacon 9.77 Hz, LTE, DECT, Bluetooth), a hysteresis pulse detector with a 120-bin log interval histogram, two-point calibration and GPS-tagged survey rows; analysis kept Arduino-free and PC-tested (18/18 on a synthetic Wi-Fi + LTE waveform: 97 beacon intervals recovered for 98 beacons), host tool 7/7, compiles at 102 KB.
- Schumann-resonance product pair: a Teensy 4.1 + ADS1256 24-bit two-channel ELF monitor (H and E multiplexed at 1,000 SPS each, mains notches, 4,096-point Welch spectra, five cavity modes tracked with parabolic centre-frequency interpolation, E/H wave impedance with coherence, GPS stamps, JSON reports; compiles at 105 KB) and an ESP32 calibrated field generator (8 kS/s five-tone synthesis, coil current sensed and converted to microtesla from coil geometry, closed-loop field target, SHA-256-committed blinded sham sessions from the hardware RNG, magnetophosphene threshold mode, phone web page; compiles for ESP32 and ESP32-S3), plus a host tool (6/6 tests) and a kit document with a noise budget (0.25 pT/√Hz floor for a 30,000-turn rod coil).
- Earlier freq-lab designs, all Vivado-built and simulated end to end: digital lock-in amplifier with resonance tracker (amplitude within 0.1%, phase within 0.4°), two-detector photon coincidence counter (12/12 checks, ±20 ns window), two-channel cross-correlator with GPS PPS stamping (7/7 checks); Teensy firmware for swept-sine resonance measurement (f0 to 0.03%, Q exact on a simulated resonator), six-sensor vibration/transfer-function logger, ELF 0.25–45 Hz spectrum logger with mains rejection (2 µV line read 1.4135 µV rms of 1.4144 under 1 V of mains), binaural-beat generator with blinded sessions, randomized-trial strobe driver with hardware RNG and safety interlock, sub-GHz CC1101 spectrum scanner, and an STM32 + SX1262 swept RF insertion-loss probe.

- Designed a ring-oscillator true random number generator in Verilog with the NIST SP 800-90B health tests (Repetition Count and Adaptive Proportion) in hardware, von Neumann debiasing and LFSR conditioning: output is withheld while a health test is tripped, and on-chip temperature is logged. Verified the entire deterministic pipeline bit-exactly against an independent Python model across balanced, stuck and biased streams (42 checks) using a simulation backdoor for the un-simulatable oscillators; Vivado build meets timing (WNS +12.6 ns), 570 LUTs, 0 critical warnings; host tool with a statistical self-test passes 14/14.
- Built a closed-loop slow-oscillation sleep-cueing system (Teensy 4.1 + EEG): a deep-sleep stage gate, a slow-oscillation trough detector with filter-delay compensation (cues land within 8 ms of target), refractory/rate/arousal gating, a SHA-256-sealed word-pair split and blinded stim/sham nights with a permutation-test scorer; signal processing PC-tested 22/22 on synthetic EEG.
- Wrote reusable experiment-integrity tooling: a blind-randomization harness with pre-posted SHA-256 commitments and an order-independent permutation test, a nuisance-channel veto that flags when a signal is explained by vibration/mains/field/sound via windowed correlation, and a seizure-safety gate for flicker stimulation (ramp-in, brightness cap, frequency-band screen, hardware abort) — each PC-tested to zero failures.
- Red-team pass over all of the above found and fixed real defects: a permutation p-value inverted by condition-label order, a continuous-mains contamination the veto missed, an EEG high-pass that cut a 100 µV slow wave to 39 µV, and a UART test reader that misparsed hex values starting with "E".

### Large-Scale Document Mining (2026)
- Built a parallel PDF-to-text pipeline over 12,301 declassified documents (6.6 GB) and extracted 2,206 frequency- and field-measurement statements with a unit-aware regex ranker — the source material for the VLF, ELF and RF instrument designs above.

### NeuroRuins — Unreal Engine 5.7 C++ Survival Game with a Shared Neural AI (2026)
- Architected "one shared brain, individual minds": a single learned model serving every AI character with per-agent memory, personality and needs; project documented (vision, roadmap, class map) and compiling against UE 5.7.4.

### Coldreach — UE 5.7 C++ Survival-Extraction Framework (2026)
- Compile-verified gameplay framework (inventory, extraction loop, damage/medical, networking scaffolding) built as a reusable base for DayZ/Tarkov-style projects.

### Vehicle Diagnostics and In-Car Monitoring (2026)
- merc-diag: OBD/CAN scanner plus camshaft/crankshaft oscilloscope for a 2010 Mercedes W212 on STM32H743 + Teensy 4.1 + ESP32; all four firmware targets compile-verified.
- Car-Sentinel: in-car monitor/logger/controller for a 2017 BMW 330e (Orange Pi + ESP32-P4 + Teensy + ESP32-S3 + FPGA option); foundation built and verified.

### Autonomous Agents and Local AI (2026)
- OMEGA_AI: multi-agent "council" system on Python 3.14 with Groq inference, brought from broken venv to live operation.
- All Things Human: offline survival-knowledge assistant with portable SQLite + embeddings RAG over local Ollama models and a 35-role reasoning council (v0.9 online).
- StonksAi: Kalshi whale-flow trading agent with verified credentials and paper-capital risk limits.

### Other Builds
- Torsion pendulum test rig: 3D-printed sealed rotor, ESP32-S3 camera tracker (0.1–0.2 mrad/frame), hash-chained blinded session log, 5% false-alarm rate on 200 simulated null sessions.
- Living Garden (Unity 6 artificial-life sim with shareable creature genomes); Mining Around (Unity 6 / UE 5.7 tycoon); PZ7020 FPGA as a GPU for Orange Pi 4 Pro; ESP32 ESP-NOW sensor-node network (Kodi); Teensy/ESP32 vehicle and lab instruments.

## Tools and Environments
Vivado 2026.1 · Icarus Verilog 11 · Arduino-CLI (Teensy 1.62, ESP32 3.3, STM32 3.0 cores) · PlatformIO · Python 3.14 · WSL Ubuntu 22.04 · Git · Unreal Engine 5.7.4 · Unity 6 · KiCad-level schematic reading, datasheet-driven pin mapping

## Education and Self-Directed Training
Self-taught through shipped projects (DayZ modding → engine C++ → embedded → FPGA); [add any formal education, certifications, or coursework]

