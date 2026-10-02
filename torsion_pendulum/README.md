# Torsion pendulum in a bottle (FOIA list #2)

SRI reported in 1974 that a person could disturb a torsion pendulum sealed in a bell
jar. The pendulum was three 100 g balls on a metal fiber, read out by a laser 1.5 m
away. The change from baseline was "often 5:1" from 1 m and "typically 2:1" from
12 m. The runs were randomized, but no statistics were done.

This rig repeats that test with the known culprits handled:

- **Static electricity:** a grounded foil shield blocks it.
- **Body heat:** a warm-water-bottle control session measures how much it moves the rotor.
- **Drafts:** the chamber is sealed.
- **Camera movement:** reference squares cancel it out.
- **Wishful analysis:** the random schedule is fixed before each session, every session is logged, and the statistics are exact.

How to read the result:

- **Grade C claim.** No one has shown it works under conditions like these. A clean
  null result is the expected outcome, and it is a real result.
- **An effect would show in one session.** SRI's claimed 2:1 to 5:1 effect is huge
  next to this rig's noise.

```
              knob (sets the zero)      ESP32-S3 CAM, lens down
                    |                       |
   lid  ===========[O]====[ sled  window ]=====      <- 132 mm plate
          \         |                        /       <- 45 deg cone: the top rim of the sleeve wedges on it
           \________|_______________________/           (any rim 85-120 mm across); tape over the joint
            |       |                       |
            |     fiber (one strand of      |        the bottle's straight middle, cut off top and bottom
            |     jumper wire)              |        ("the sleeve", 150-230 mm tall), foil-wrapped outside
            |       |                       |
            |  [==O===o===O==]  rotor: 2 quarters sealed inside, 20 mm above the floor
            |  []  --paper--  []            |        paper target glued to the floor, squares at +/-30
           /________________________________\        <- 45 deg cone: the bottom rim slides over it
   base   |          |  LED  |               |       <- 30 mm base; 94 mm backlit window in the floor,
          |__________|_______|_______________|          open underneath; the LED sits on a puck on the table
```

A real 2-liter bottle is 100-120 mm across and 300-330 mm tall depending on the brand,
with a five-footed base that is domed inside and a long curved shoulder. So the bottle
is used only for its clear wall: it is cut twice, and the printed lid and base each
carry a 45-degree cone that any rim between 85 and 120 mm wedges onto. The floor is
printed and flat, and nothing in the design depends on the bottle's diameter.

## Parts

| Item | Notes |
|---|---|
| `torsion_pendulum.3mf` | Bambu Studio project, 4 plates: lid, base, rotor, knob + camera sled + LED puck. PLA, white for the base (the floor's window is the light diffuser). The rotor is set to 100 % infill and needs a **pause at 3.40 mm** to drop one US quarter into each pocket. |
| 2-liter soda bottle | Empty, clean, label off. Any brand: only its straight middle is used. |
| ESP32-S3 CAM | Your Meshnology GC2145 board, plus a USB-C cable. |
| 2 US quarters | The rotor's weights. |
| Fiber | One strand pulled out of a stranded jumper wire, about 25 cm long. A strand from a usual 26 AWG Dupont jumper is 0.16 mm and gives a swing period of about 5 s; a 28 AWG strand (0.127 mm) gives about 8 s and 2.5x the sensitivity. Thinner is better; `calibrate` measures whatever you use. |
| 5 mm white LED + 470 ohm resistor | Backlight, powered from the camera board's 5V and GND pins with female jumpers. A *diffused* (cloudy) LED lights the floor evenly; if yours is water-clear, scuff its dome with sandpaper. Use 220 ohm if the picture is too dark. |
| Aluminium foil + 1 jumper wire | Static shield around the sleeve, wired to the board's GND. |
| 2x M3x10 screws + 2x M3 nuts | Hold the camera sled to the lid. M3x12 also fits. |
| Hot glue or superglue, a glue stick | Fix the fiber ends and the camera board; the glue stick holds the paper target to the floor. |
| Clear tape | Seals the camera window and the two sleeve joints. |
| Plain printer paper | For `paper_target.pdf`, printed at 100 %. |

## Build

1. **Cut the bottle twice.** Stand it on a table and mark the lines with a marker held
   on a stack of books while you turn the bottle.
   - *Bottom cut:* about 40 mm up, just above the feet, where the body has become round.
     If your bottle has a grip or waist low down, cut above it.
   - *Top cut:* in the shoulder, where the bottle has narrowed to about 90 mm across
     (anywhere between 85 and 120 mm is fine; the lid's cone takes any of it). Cutting
     there keeps the most height.
   - Both rims must be round: not through a grip dent or a rib. Aim for a sleeve
     150-230 mm tall; 180-210 is typical. Trim the rims with scissors so they are even.
2. **Print the parts.** The plates are already oriented: lid top-face-down (cone up),
   base floor-down (its well opens upward), rotor flat, knob face-down. For the rotor
   (plate 3), drag the layer slider to 3.40 mm, right-click, and choose *Add pause*.
   When the printer pauses, drop one quarter flat into each pocket and resume.
3. **Paper target.** Print `paper_target.pdf` at 100 % ("Actual size"). Check that the
   scale bar measures 50 mm, cut out one 96 mm disc, and glue-stick it onto the base's
   floor with the two squares lined up with the small notch on the floor's edge.
4. **LED puck.** Push the LED up through the puck's centre hole from below until its
   flange seats in the recess; bend the legs flat into the groove, with the resistor
   on one leg, and run two wires out along the groove. (The two outer holes are for
   extra LEDs if one is not bright enough.) Put the puck on the table, LED up, and set
   the base over it with the base's wire channel over the wires.
5. **Lid.**
   - Put clear tape over the camera window from the *underside*.
   - Push 2 M3 nuts into the hex pockets under the lid.
   - Bolt the camera sled on top with the 2 M3x10 screws.
6. **Knob and fiber.** Glue one end of the strand about 6 mm into the hole at the tip
   of the knob's pin. Thread the fiber down through the lid's centre hole and seat the
   knob. Friction holds it; turning it sets the rotor's resting angle (ticks every 10°).
7. **Dry fit and measure.** Slide the sleeve's bottom rim over the base's cone and press
   it down until it wedges. Drop the lid's cone into the top rim. Measure from the table
   to the lid's top face: call it H. Take the lid off again.
8. **Hang the rotor.** The free fiber length, from the pin's tip to the top of the
   rotor's hub, is **H minus 74 mm**. Cut the strand 8 mm longer than that, push the
   extra all the way down the hub's hole and glue it. The rotor then hangs 20 mm above
   the floor; anywhere from 10 to 30 mm works.
   Let it hang overnight: a fresh copper strand creeps for a few hours, then settles.
9. **Shield.** Wrap the outside of the sleeve in one layer of foil and tape it. Tape the
   bare end of a jumper wire to the foil, and plug the other end into the board's GND.
10. **Close it up.** Lower the rotor into the sleeve and seat the lid's cone in the top
    rim. Turn the lid so its engraved arrow points the same way as the floor's notch
    (the squares then lie along the arrow). Run clear tape round both joints, from the
    printed part onto the bottle.
11. **Camera.** Lay the board camera-side down on the sled's rails, lens over the window,
    with the rest of the board pointing outward, away from the knob. Tack it with hot
    glue, or use zip ties through the slots. Wire the LED's two leads to the board's
    5V and GND.
12. **Flash the board.** In the Arduino IDE, open `firmware/torsion_cam/torsion_cam.ino`
    and pick these settings, then upload through the USB-C port marked COM/UART:
    - Board: "ESP32S3 Dev Module"
    - PSRAM: "OPI PSRAM"
    - Flash Size: "16MB"
    - USB CDC On Boot: "Disabled"

    The firmware finds the camera's pin layout by itself.

## Set up and check (PC: `pip install pyserial numpy scipy pillow matplotlib`)

```
python host/torsion_host.py COM8 snap        # saves a PNG of what the camera sees, with what it measures
python host/torsion_host.py COM8 live        # live angle, swing and period
python host/torsion_host.py COM8 calibrate   # follow the prompts: one knob tick clockwise, then back
```

**Snap:** the PNG should show:

- the rotor outlined in red, whole, inside the lit window;
- both squares in blue boxes;
- the lit window circled in green.

Turn the knob until the rotor lies *across* the line of the squares, not along it.

**Calibrate** measures:

- the swing period (about 5 s with a 0.16 mm strand, 8 s with 0.127 mm) and how fast
  it dies out;
- the torsion constant, which turns angle into torque;
- which way "clockwise" reads on the camera.

## The experiment

Decide the plan before you collect any data, and stick to it. This is the default:

- 2 warm-bottle sessions (`--kind warm`);
- 10 empty-room sessions (`--kind empty`);
- 10 agent sessions (`--kind agent`), alternated with the empty ones.

Each session lasts 22 minutes:

- 5 minutes settling;
- 16 one-minute periods, 8 PUSH and 8 REST in random order;
- 1 minute tail.

```
python host/torsion_host.py COM8 session --kind agent    # sit 1 m away; high beep = PUSH (will it clockwise),
                                                         # double low beep = REST. Stay seated the whole time.
python host/torsion_host.py COM8 session --kind empty    # start it, then leave the room within 2 minutes
python host/torsion_host.py COM8 session --kind warm     # 37 C water bottle onto the 1 m spot at PUSH,
                                                         # away to the far side at REST
python host/torsion_host.py report --plot                # checks the log, then every result and the combined score
```

**Where to set up:**

- A solid table, away from heating and AC vents, windows and direct sun, and the PC
  (it is warm).
- Mark the 1 m spot on the floor.

**What each session prints:**

- **D:** the PUSH-minus-REST deflection in mrad, with clockwise positive, and the
  torque it means.
- **p:** the exact chance probability, counted over all 12,870 possible schedules.
- **Swing ratio:** PUSH/REST, the number SRI reported as 2:1 to 5:1.

**Decision rule (fixed now):** call it an effect only if both of these hold:

- the agent sessions' combined Z is at least 2.33 (p < 0.01);
- the empty-room sessions' combined Z is under 1.64.

The warm-bottle sessions show what a real physical push looks like on this rig.

**Integrity checks.**

- Before each session runs, its schedule's fingerprint goes into
  `torsion_data/commitments.txt`. Text it to someone if you want an outside record.
- Every session is chained into `torsion_data/sessions.jsonl` with its data file's hash.
- `report` fails loudly if a session is dropped, a data file is edited, or a schedule
  doesn't match its fingerprint.

## Files

| Path | What |
|---|---|
| `cad/torsion_pendulum.3mf` | the print project (4 plates) |
| `cad/*.stl`, `cad/assembly.png` | single parts and a picture of the assembly |
| `cad/paper_target.pdf` | the floor target, two per page |
| `cad/parts.py`, `cad/build.py` | the part generator (Python + manifold3d), to change any dimension |
| `firmware/torsion_cam/` | ESP32-S3 sketch: camera, tracker, serial protocol |
| `firmware/platformio.ini` | same sketch for PlatformIO |
| `host/torsion_host.py` | snap, live, calibrate, session, report, analyze, simulate |
| `sim/` | tracker test (synthetic camera frames) and a fake camera for testing the host |

## How it was checked

- **Bottle facts:** 2-liter bottles run 100-120 mm across and 300-330 mm tall
  (Wikipedia, "Two-liter bottle"); the common US one is 110 mm by 315 mm
  (dimensions.com); the base is a petaloid with a convex centre, not a flat floor
  (EP2555984B1). The cones take rims from 85 to 121 mm (lid) and 99 to 125 mm (base).
- **Parts:** every part is watertight and a single solid; the rotor's two sealed
  pockets are 974 mm3 each. Fit checks (`build.py`):
  - where each rim size wedges on each cone (a 104 mm rim: 9 mm below the lid, 3 mm
    below the floor);
  - the rotor clears the wall of the narrowest bottle by 11.6 mm;
  - the rotor's tips and the squares' tracking boxes stay inside the backlit window as
    the camera sees them, for sleeves from 150 to 230 mm;
  - the lid's cone hides none of the window; the camera's view cone clears the lid
    window with 2.2 mm to spare.
- **Rotor:** 16.5 g; moment of inertia 1.02e-5 kg m^2 (the PLA part is computed
  solid, which is why it's printed at 100 % infill).
- **Tracker** (same C++ the board runs), on synthetic camera frames of this exact
  layout, with the camera 165, 205 and 240 mm above the floor:
  - 0.05-0.19 mrad noise per frame;
  - a 10 mrad step measured as 9.77-9.78 against a true 9.77;
  - a 2 mrad turn of the camera itself cancelled to under 0.2 mrad;
  - the 175 mrad knob turn of the calibrate step followed without losing lock;
  - the rotor hung anywhere from 8 to 32 mm above the floor, lens fields of view
    from 48° to 62°, and image noise up to 5 counts all handled (at 40 mm the rotor's
    tips reach past the lit window and it is not found: keep to the fiber length above).
- **Firmware:** compiles with no warnings for the ESP32-S3 with both Arduino-ESP32
  3.3.12 (current) and 2.0.17. Both cores' camera libraries include the GC2145 driver.
- **Statistics:** across 200 simulated sessions with no effect, p < 0.05 came up
  5 % of the time, exactly as it should. A steady 0.3 mrad push reached p < 0.05 in
  42 % of single sessions.
- **Host program:** run against a simulated camera:
  - calibration recovered the period (8.03 s against 8.0) and Q (15.0 against 15);
  - sessions logged;
  - the report caught an edited data file and a deleted session.

Not yet run on the real hardware.
