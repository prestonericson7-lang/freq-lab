# freq-lab — verified status

Re-verify everything in one command:

```
python run_all_tests.py          # Python + C++ + FPGA sims
python run_all_tests.py --quick  # skip the FPGA sims (faster)
python run_all_tests.py --full   # also the slow v1 vlf_sid sim
```

Last full run: **21 test groups, 343 checks, ALL GREEN.** A sim that times out
under heavy machine load (e.g. a Vivado build running) is reported `SLOW`
(inconclusive), not a failure — re-run it alone when the machine is idle.

## What is verified

| Area | How | Result |
|---|---|---|
| 12 Python host/server/analysis tools | `tools/test/test_*.py` | 137 checks, 0 fail |
| End-to-end station chain (real clients → real server → detector → API) | `tools/test/test_station_e2e.py` | 10, 0 fail |
| 6 Arduino signal-processing / safety cores | `teensy/*/test/pc_test*.cpp` (g++) | 85 checks, 0 fail |
| `fpga/vlf_sid_v2` (VLF + lightning TOA) | iverilog testbench + Python bit-exact model | 59 + 19, 0 fail |
| `fpga/trng` (ring-osc + SP 800-90B) | iverilog + bit-exact model (balanced/stuck/biased) | 9 + 33, 0 fail |
| `fpga/vlf_sid` v1, lockin, photon, correlator | iverilog (verified earlier; v1 re-confirmed 33+16) | 0 fail |
| All Teensy / ESP32 / STM32 sketches | arduino-cli compile | all compile |
| `fpga/vlf_sid_v2`, `fpga/trng` bitstreams | Vivado 2026.1 batch build | built, timing met, 0 critical warnings |

## Build artifacts (ready to load)

| Bitstream | Timing | Resources | Notes |
|---|---|---|---|
| `fpga/vlf_sid_v2/build/vlf_sid_v2.bit` | WNS +4.5 ns @ 50 MHz | 17.9 % LUTs, 50 DSP | `program.tcl` loads it (RAM only) |
| `fpga/trng/build/trng.bit` | WNS +12.6 ns @ 50 MHz | 1.1 % LUTs | 16 ring-osc loops flagged allowed |

Earlier builds (lockin, photon_counter, correlator, vlf_sid v1) are in their
own `build/` folders.

## The VLF SID station

FPGA (PZ7020) + Teensy 4.1 GPS-aware bridge + ESP32-32E (spare) + RTL-SDR +
GPS/PPS. Full wiring and run sequence: **`docs/station-setup.md`**.
- Receiver: `fpga/vlf_sid_v2` (built). Load with its `program.tcl` (RAM only, so
  it does not disturb the GPU matrix-math config — power-cycle restores it).
- Bridge: `teensy/vlf_station/vlf_station.ino` (compiles; NMEA parser 11/11).
- Host: `tools/vlf_host.py --gps-bridge` stamps windows with true GPS time (21/21).
- SDR complement: `tools/riometer_host.py` (30 MHz absorption; the SDR can't do VLF).
- Dashboard + alerts: `tools/sid_server.py` + `dashboard/index.html` (20/20, e2e 10/10).

## What is NOT done (needs real hardware)

1. **Nothing has run on hardware yet.** Everything above is compile-, simulation-
   or model-verified. The first power-on follows the run sequence in
   `docs/station-setup.md`, now with the Teensy GPS bridge in the loop.
2. **The VLF loop antenna + preamp** (1 m loop, ~60 dB preamp, 10–50 kHz) feeding
   the FPGA JM1-6/8 is not built — the receiver can't hear the Navy stations
   without it. Design: `docs/analog-front-ends.md`.
3. The analog front ends in `docs/analog-front-ends.md` (VLF loop, Schumann coil,
   RF detector, photon SiPM) are designed from a noise budget but not yet built.

## Documentation

- `README.md` — overview and first-run.
- `WIRING.md` — every pin, every board, with diagrams.
- `docs/station-setup.md` — the full VLF SID receiver station.
- `docs/analog-front-ends.md` — the analog front-end designs.
- `fpga/*/README.md` — each FPGA design, its register map and its verification.
- `RESUME.md` — background.
