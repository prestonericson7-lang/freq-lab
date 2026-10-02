# Analog front-end designs

The firmware and FPGA designs in this repo assume a signal already conditioned
to the converter's input range. This note gives the analog front ends that do
that conditioning — the part that lives between the physical sensor and the pin.
All are designed from first principles (noise budget, bandwidth, dynamic range);
none has been built and measured yet.

- [VLF loop antenna + preamp (for the SID receiver)](#vlf-loop-antenna--preamp)
- [Schumann / ELF induction coil, preamp and ball antenna](#schumann--elf-front-end)
- [Broadband RF power detector (AD8318)](#broadband-rf-power-detector)
- [Photon detector front end (SiPM)](#photon-detector-front-end)

---

## VLF loop antenna + preamp

Feeds `fpga/vlf_sid` / `vlf_sid_v2` on JM1-6/8. The on-chip ADC wants 0–1.0 V
centred at 0.5 V, 10–50 kHz.

```
loop (1 m square, 10 turns)  ->  preamp  ->  0.5 V bias + 100 Ω  ->  JM1-6
                                            2.2 nF to JM1-8 (ground)
```

**Loop.** 1 m × 1 m square frame, **10 turns** of 22–26 AWG magnet wire (~40 m).
Inductance ~0.4 mH, self-resonant well above 100 kHz. A loop responds to the
magnetic field, so it rejects most of a building's electric-field noise; it is
directional (edge-on to the station is loudest, face-on is a null). Site it
outdoors or in an attic, away from switching supplies and house wiring. Strong
Navy stations (NAA, NLK at ~1000 km) induce ~0.1–1 mV across 10 turns; weak ones
~10 µV. The preamp must bring 10 µV–1 mV to 30–300 ADC counts (7–75 mV) without
clipping on a nearby lightning stroke.

**Preamp (two stages, single 3.3 V from JM1-2).**
1. *Input stage* — low-noise op-amp (OPA1612 / LT6202) as an instrumentation
   input across the loop, gain 20; 10 kΩ across the loop damps its resonance.
   Input-referred noise under 2 nV/√Hz.
2. *Band-pass* — 2nd-order high-pass at 8 kHz (rejects 60 Hz harmonics and
   supply hash) and 2nd-order low-pass at **60 kHz**. The low-pass is mandatory:
   the ADC samples at 390.6 kS/s, so AM broadcast (530 kHz+) would alias into
   the band without it.
3. *Output stage* — gain 10–50 (trimmer; `vlf_host.py level` sets it), biased to
   0.5 V, 100 Ω series + 2.2 nF to ground at the pin. Running it from the FPGA's
   3.3 V rail means it physically cannot exceed 3.3 V; the ADC reads full scale
   above 1.0 V and the clip LED shows it.
4. Total gain ~60 dB, adjustable 40–70 dB.

**GPS.** Any 3.3 V module with a PPS output (u-blox NEO-6M/M8N). PPS → JM1-18;
the FPGA needs only the PPS. With the `vlf_station` Teensy bridge the NMEA stream
gives true UTC as well.

---

## Schumann / ELF front end

Feeds `teensy/schumann_monitor` (ADS1256, 24-bit). Two channels — magnetic (H)
and electric (E) — so the monitor can report the Earth–ionosphere cavity's wave
impedance, not just amplitude.

### Induction coil (H channel)

Signal: the Schumann fundamental is ~1 pT/√Hz. An air/ferrite coil gives
V = 2π f N A B. To get 1 pT at 7.8 Hz above a ~1 nV/√Hz preamp, the coil needs
N·A ≳ 50e-9 / (2π·7.8·1e-12) ≈ 1000 turn·m².

| Design | Turns | Core | N·A | V per pT @ 7.8 Hz | R | Notes |
|---|---|---|---|---|---|---|
| Rod coil | 30,000 × 36 AWG | 60 cm × 10 mm ferrite/mu-metal rod, µ_eff ≈ 200 (A_eff ≈ 1.6e-2 m²) | 470 | 23 nV | ~2 kΩ | observatory style, ~70 cm, ~1.5 kg |
| Air loop | 400 × 24 AWG | 2 m × 2 m frame, A = 4 m² | 1600 | 78 nV | ~60 Ω | needs a yard; cheap |

Noise budget (rod coil): thermal √(4kTR) = 5.8 nV/√Hz at 2 kΩ dominates the
1 nV preamp, so the floor is 5.8 / 23 = **0.25 pT/√Hz**. The 1 pT fundamental
stands 12 dB above it; modes 2–3 clear it; 4–5 are marginal. Lower resistance
(thicker wire / bigger rod) buys floor. The `floor30_pT` column reports the
measured floor so it can be checked against this budget.

Winding: split bobbin (two halves) to halve self-capacitance; keep the
self-resonance above 100 Hz (30 k turns on a rod resonate ~300–600 Hz). A slit
copper-foil electrostatic shield tied to the preamp ground (a closed wrap is a
shorted turn).

### Preamp (H)

Low-noise instrumentation front end: AD8421 (3 nV/√Hz) or a paralleled chopper
pair OPA2188/ADA4522 (5.8 nV each → 4 nV paired, no 1/f). Gain 1000 (32 × 32),
1st-order high-pass at 0.3 Hz, 2nd-order low-pass at 100 Hz, output biased at
2.5 V for the ADS1256. 1 pT → 23 nV → 23 µV at the ADC; at PGA 8 that is ~300
codes, noise ~70 codes rms over the band. Battery supply; preamp at the coil,
ADC board at the Teensy, shielded twisted pair between.

### Ball antenna (E channel)

Schumann E-field ~0.3 mV/m at the fundamental. A 20 cm sphere on a 2 m insulated
mast picks up E × h_eff (h_eff ≈ 1 m) → ~300 µV. An electrometer buffer (LMC6001
or ADA4530-1, 10¹⁴ Ω input, unity gain) with a 10 GΩ bias resistor; the sphere
capacitance (~10 pF) and 10 GΩ give a 1.6 Hz corner — fine for 7.8 Hz, calibrate
the roll-off with `ecal`. Dry insulator (PTFE); rain kills the E channel.

The two channels together give E/H at each resonance (the cavity's wave
impedance), and the coherence tells whether both sensors see the real field or
local noise (coherence > 0.5 = real).

---

## Broadband RF power detector

Feeds `teensy/rf_survey` on A0. An AD8318 logarithmic detector (1 MHz–8 GHz)
outputs a DC voltage proportional to input power in dB (~−24.4 mV/dB, −60 to
0 dBm).

```
AD8318 module: VCC 5 V, GND, VOUT -> 10 kΩ -> Teensy A0, 10 nF A0->GND
antenna / band filter -> AD8318 RF input (SMA)
```

Calibrate with two known levels ≥ 20 dB apart (`cal <dBm>` twice); the firmware
fits slope and intercept. `ant <GHz> <dBi>` sets the antenna aperture so the
reading converts to µW/cm² (A_eff = G·λ²/4π). A band-select filter or horn in
front chooses what the detector integrates — it cannot separate frequencies by
itself.

---

## Photon detector front end

Feeds `fpga/photon_counter` on JM1-6/8 (two channels). Each channel is a
single-photon avalanche detector (SiPM, e.g. MicroFC-10035) plus a fast
comparator producing a 3.3 V rising edge per photon. In a light-tight, cooled
enclosure the dark-count rate falls; the FPGA's coincidence window and
accidental-rate subtraction then separate correlated photons from independent
dark counts. With 1000 dark cps per channel and a 100 ns window the accidental
coincidence rate is 2·100e-9·1000·1000 = 0.0002 cps, so any coincidence rate far
above that is a real correlated source.
