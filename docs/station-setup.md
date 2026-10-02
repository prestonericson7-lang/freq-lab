# VLF SID station on your rig (FPGA + Teensy + ESP32 + SDR + GPS)

Your hardware: PZ7020 FPGA + Teensy 4.1 (wired together, currently running the
GPU matrix-math job), ESP32-32E, an RTL-SDR, and a GPS with PPS — the same GPS
and the same NMEA/PPS convention as your past .ino builds (NMEA 9600 on a
hardware serial, PPS on an interrupt pin).

This is the space-weather station built from item #180. Each part has one job.

## Who does what

| Part | Job | Software |
|---|---|---|
| **FPGA (PZ7020)** | the VLF receiver: 8 Navy stations at once, 390 kS/s, lightning time-of-arrival (v2) | `fpga/vlf_sid_v2/build/vlf_sid_v2.bit` |
| **Teensy 4.1** | GPS-aware bridge: passes the FPGA's UART to the PC AND reads the GPS so windows get true GPS time | `teensy/vlf_station/vlf_station.ino` |
| **GPS + PPS** | the clock: NMEA to the Teensy, 1PPS to BOTH the Teensy and the FPGA | — |
| **RTL-SDR** | the complement the SDR can actually receive: 30 MHz riometer (flare absorption) + 245 MHz solar bursts | `tools/riometer_host.py` |
| **ESP32-32E** | not required for the station — the PC does the uploading. Spare: run the Schumann generator on it, or use it later as a standalone WiFi upload node | `esp32/schumann_gen` (no station-uploader firmware is written for it; `vlf_host.py --push` uploads from the PC) |
| **PC** | runs `vlf_host.py` and `riometer_host.py`, and `sid_server.py` for the dashboard | — |

The RTL-SDR tunes 24 MHz and up, so it cannot receive VLF (24 kHz) directly —
that is exactly why the FPGA does the VLF at 390 kS/s and the SDR does the HF
riometer. The two together cover the flare from two independent angles.

## Your GPU rig stays put

Loading `vlf_sid_v2.bit` over JTAG (`program.tcl`) writes the FPGA's RAM
configuration only. It temporarily replaces whatever is running (your GPU
matrix-math bitstream); a power cycle brings the board back exactly as it was.
So: switch to VLF when you want to run the station, power-cycle to return to the
GPU job. Nothing is written to flash, nothing about the GPU wiring changes.

## Wiring for VLF mode

The FPGA↔Teensy link you already have for the GPU job carries the UART in VLF
mode too. Add the antenna, the GPS, and (if using the Teensy bridge) confirm the
UART pins:

```
Antenna/preamp (1 m loop + ~60 dB preamp, 10-50 kHz) -> FPGA JM1-6 (ANTENNA+, 0..1.0 V)
                                                        FPGA JM1-8 (ANTENNA-, preamp ground)
FPGA UART: JM1-14 (TX) -> Teensy pin 0 (RX1)
           JM1-16 (RX) <- Teensy pin 1 (TX1)
GPS NMEA (9600):        GPS TX -> Teensy pin 7 (RX2)
GPS 1PPS:               GPS PPS -> Teensy pin 2  AND  -> FPGA JM1-18 (C20)     (one line, two inputs)
GND common:             Teensy GND -- FPGA JM1-4 -- GPS GND
RTL-SDR:                USB to the PC, antenna = a 30 MHz dipole (2.4 m legs) or a 2-element Yagi
```

The GPS wiring is the same idea as elf_logger / schumann_monitor (NMEA on a
hardware serial, PPS on an interrupt pin); here NMEA is on Serial2 (pin 7) so
Serial1 (pins 0/1) is free for the FPGA. The one 1PPS line fans out to both the
Teensy (for the timestamp) and the FPGA JM1-18 (for GPS-second windows).

## Run it

1. **Load the receiver** (Vivado Tcl shell):
   ```
   cd P:/Downloads/freq-lab/fpga/vlf_sid_v2
   source program.tcl
   ```
   LED R19 blinks at 1 Hz when it is running.
2. **Flash the Teensy** with `teensy/vlf_station/vlf_station.ino`.
3. **Set the preamp gain** (the Teensy is now the serial port):
   ```
   python tools/vlf_host.py COM_TEENSY level
   ```
   aim for 30-300 counts RMS, zero clipping.
4. **Confirm the stations and the GPS:**
   ```
   python tools/vlf_host.py COM_TEENSY scan       # names the Navy stations you hear
   python tools/vlf_host.py COM_TEENSY info       # PPS alive, windows advancing
   ```
   `info` reads the FPGA; the Teensy also answers `@gps` with the GPS time.
5. **Log, GPS-stamped, uploading to the dashboard:**
   ```
   python tools/sid_server.py --goes             # on any always-on machine
   python tools/vlf_host.py COM_TEENSY log --goes --gps-bridge --station HOME --push http://SERVER:8750
   ```
   `--gps-bridge` stamps every window with the GPS UTC from the Teensy instead
   of the PC clock. Add `sferics --gps-bridge --push ...` on a second run for the
   lightning time-of-arrival network.
6. **The SDR complement** (another terminal):
   ```
   python tools/riometer_host.py run --freq 30.0 --station HOME --push http://SERVER:8750
   ```
   30 MHz cosmic-noise absorption dips line up with the same flares the VLF
   stations step on, from an independent receiver.

Open `http://SERVER:8750/` for the live dashboard: station levels, GOES overlay,
flare alerts, riometer absorption, and (with sferics on two sites) the
time-of-arrival strokes.

## What's verified vs. what needs the hardware run

- `vlf_sid_v2` bitstream: Vivado-built, timing met, 0 critical warnings;
  simulation 59 + 19 checks bit-exact. `program.tcl` fixed to load it.
- `vlf_station.ino`: compiles (50 KB); its NMEA parser is PC-tested 11/11.
- `vlf_host.py --gps-bridge`: tested 21/21 against a software board + bridge.
- `riometer_host.py`: tested 8/8.
- Not yet run on real hardware. The first run is the acceptance test in
  the run sequence above, now with the Teensy GPS bridge in the loop.
