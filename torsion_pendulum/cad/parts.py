"""
parts.py -- printed parts for the 2-liter-bottle torsion pendulum (FOIA list #2).

The bottle only supplies the clear wall: it is cut twice and the straight middle
section (the "sleeve") is used. A real 2-liter bottle has a five-footed petaloid
base that is nowhere near flat inside, a long curved shoulder, and a body of
100-120 mm depending on the brand, so nothing here assumes a diameter:

- the lid carries a 45-degree cone that drops into the top of the sleeve until
  the rim wedges on it (any rim from 85 to 121 mm inside diameter);
- the base carries a 45-degree cone the bottom of the sleeve slides over (any
  rim from 99 to 125 mm); its flat top is the floor, with a thin backlit window
  in the middle for the paper target.

All parts are built in their USE orientation (z up, mm). build.py turns each one
to its print orientation, checks it and writes the Bambu Studio 3MF.

Lid frame: z = 0 is the lid's underside; the top face is z = +3; the cone and the
pendulum hang below. The fiber hangs on the axis (x = y = 0); the camera looks
down through a window at (+26, 0).
Base frame: z = 0 is the underside that stands on the table; the floor is z = 30.
"""
import math

import manifold3d as m3d

SEG = 128          # facets on large circles
SEG_S = 48         # facets on small circles

# ---------------------------------------------------------------- dimensions
# 2-liter PET bottle (Wikipedia: 100-120 mm across, 300-330 mm tall; wall ~0.3 mm)
BOTTLE_OD_MIN, BOTTLE_OD_MAX = 100.0, 120.0
BOTTLE_WALL = 0.35

LID_T = 3.0                    # lid plate
LID_R = 66.0                   # lid outer radius (132 mm)
CONE_H = 19.0                  # cone plug under the lid
CONE_R_TOP = 61.0              # cone radius at the lid underside (122 mm)
CONE_R_TIP = CONE_R_TOP - CONE_H   # 42 at the tip: 45 degrees
CONE_WALL = 2.4
BOSS_R = 7.0                   # bearing boss under the lid, 14 mm
BOSS_H = 10.0
PIN_HOLE_D = 6.5               # torsion pin bearing hole
PIN_D = 6.0                    # torsion pin (0.25 mm clearance per side)

CAM_X = 26.0                   # camera window centre (as close to the axis as the knob allows)
WIN_D = 22.0                   # window diameter (lens field of view clears it)
SCREW_Y = 24.0                 # sled bolts at (CAM_X, +/-24)
M3_CLEAR = 3.4
M3_NUT_AF = 5.9                # 5.5 nut + 0.4
M3_NUT_T = 2.7                 # 2.4 nut + 0.3

KNOB_D = 24.0
KNOB_H = 5.0

# rotor
QUARTER_D, QUARTER_T, QUARTER_G = 24.26, 1.75, 5.670   # US quarter (US Mint spec)
POCKET_D = 24.9
DISC_D = 28.0
ARM = 24.0                     # disc centres at x = +/-24 (rotor 76 mm long)
BAR_W = 8.0
ROTOR_FLOOR = 1.2
POCKET_H = 2.0                 # quarter 1.75 + 0.25
ROTOR_ROOF = 1.0
ROTOR_T = ROTOR_FLOOR + POCKET_H + ROTOR_ROOF          # 4.2
HUB_D, HUB_H = 8.0, 5.8
FIBER_HOLE_D = 1.2
ROTOR_GAP = 20.0               # rotor hangs this far above the paper

# sled (camera mount)
SLED_X0, SLED_X1 = 14.0, 42.0
SLED_Y = 30.0
SLED_T = 3.0
RAIL_Y0, RAIL_Y1 = 10.0, 16.0
RAIL_H = 8.0

# base: plate + 45-degree cone the sleeve slides over; the cone's flat top is the floor
BASE_R = 66.0
PLATE_T = 16.0
FRUS_H = 14.0
FRUS_R_TOP = 48.5              # floor 97 mm across
FRUS_R_BOT = FRUS_R_TOP + FRUS_H   # 63
BASE_H = PLATE_T + FRUS_H      # 30: floor height
WELL_R = 46.5                  # backlit window 93 mm across (open underneath for the LED)
MEMBRANE_T = 0.8               # 4 layers of white PLA: the diffuser under the paper
CHANNEL_W, CHANNEL_D = 8.0, 4.0    # wire channel in the underside

# LED puck: lies on the table inside the base's well, LED pointing up
PUCK_R = 45.5                  # 1 mm clearance a side in the well
PUCK_T = 4.0
LED_D = 5.2                    # 5 mm LED body
LED_X = 18.0                   # two optional extra LED holes at +/-18

# paper target (make_target.py)
PAPER_D = 95.0
REF_X = 30.0                   # reference squares at +/-30 along the lid's arrow
REF_SIDE = 8.0


def cyl(r, h, z0=0.0, x=0.0, y=0.0, seg=SEG):
    return m3d.Manifold.cylinder(h, r, r, seg).translate([x, y, z0])


def cone(r_bottom, r_top, h, z0=0.0, seg=SEG):
    return m3d.Manifold.cylinder(h, r_bottom, r_top, seg).translate([0, 0, z0])


def box(x0, x1, y0, y1, z0, z1):
    return m3d.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def hexagon(af, h, z0=0.0, x=0.0, y=0.0, rot=30.0):
    r = af / math.sqrt(3.0)
    return m3d.Manifold.cylinder(h, r, r, 6).rotate([0, 0, rot]).translate([x, y, z0])


# ---------------------------------------------------------------- lid
def lid():
    body = cyl(LID_R, LID_T)                                          # plate z 0..3
    # 45-degree cone plug: outer r = 61 + z (z from -19 to 0), wall 2.4 measured across
    outer = cone(CONE_R_TIP, CONE_R_TOP, CONE_H, -CONE_H)
    inner = cone(CONE_R_TIP - 1.0 - CONE_WALL, CONE_R_TOP - CONE_WALL, CONE_H + 1.0, -CONE_H - 1.0)
    body += outer - inner
    body += cyl(BOSS_R, BOSS_H, -BOSS_H, seg=SEG_S)
    for sy in (1, -1):                                                # nut bosses under the sled bolts
        body += cyl(5.5, 4.0, -4.0, CAM_X, sy * SCREW_Y, SEG_S)
    # holes
    body -= cyl(PIN_HOLE_D / 2, LID_T + BOSS_H + 2, -BOSS_H - 1, seg=SEG_S)
    body -= cyl(WIN_D / 2, LID_T + 2, -1, CAM_X, 0, SEG_S)
    for sy in (1, -1):
        body -= cyl(M3_CLEAR / 2, LID_T + 6, -5, CAM_X, sy * SCREW_Y, SEG_S)
        body -= hexagon(M3_NUT_AF, M3_NUT_T + 1, -4.0 - 1, CAM_X, sy * SCREW_Y)   # nut trap opens downward
    # angle scale engraved in the top face around the knob: every 10 deg, long every 90
    for k in range(36):
        a = k * 10.0
        r0, r1 = (13.0, 19.0) if k % 9 == 0 else ((13.0, 17.0) if k % 3 == 0 else (13.0, 15.5))
        tick = box(r0, r1, -0.35, 0.35, LID_T - 0.4, LID_T + 1).rotate([0, 0, a])
        body -= tick
    # arrow showing which way the camera window is (+x): shaft and two barbs
    body -= box(46, 58, -0.6, 0.6, LID_T - 0.4, LID_T + 1)
    for s in (1, -1):
        body -= box(-5, 0.3, -0.6, 0.6, LID_T - 0.4, LID_T + 1).rotate([0, 0, s * 35]).translate([58, 0, 0])
    return body


def lid_cone_seat(rim_id):
    """z (below the lid underside, negative) where a rim of this inside diameter wedges on the cone."""
    return (rim_id / 2) - CONE_R_TOP


# ---------------------------------------------------------------- torsion knob + pin
def knob():
    pin_len = LID_T + BOSS_H + 1.0                                    # 14: tip 1 mm below the boss
    k = cyl(KNOB_D / 2, KNOB_H, 0, seg=SEG)                           # knob z 0..5 (sits on the lid top)
    for i in range(24):                                               # grip notches
        a = 2 * math.pi * i / 24
        k -= cyl(1.0, KNOB_H + 2, -1, (KNOB_D / 2 + 0.3) * math.cos(a), (KNOB_D / 2 + 0.3) * math.sin(a), 16)
    k -= box(KNOB_D / 2 - 4.0, KNOB_D / 2 + 1, -0.6, 0.6, KNOB_H - 0.8, KNOB_H + 1)   # pointer line on top
    k += cyl(PIN_D / 2, pin_len, -pin_len, seg=SEG_S)                 # pin z -14..0
    k -= cyl(FIBER_HOLE_D / 2, 8.0, -pin_len - 1, seg=16)             # fiber hole up the pin tip
    return k


# ---------------------------------------------------------------- rotor
def rotor():
    r = box(-ARM, ARM, -BAR_W / 2, BAR_W / 2, 0, ROTOR_T)
    for sx in (1, -1):
        r += cyl(DISC_D / 2, ROTOR_T, 0, sx * ARM, 0, SEG)
    r += cyl(HUB_D / 2, HUB_H, ROTOR_T, seg=SEG_S)                    # hub z 4.2..10
    for sx in (1, -1):                                                # sealed coin pockets
        r -= cyl(POCKET_D / 2, POCKET_H, ROTOR_FLOOR, sx * ARM, 0, SEG)
    r -= cyl(FIBER_HOLE_D / 2, ROTOR_T + HUB_H, 2.0, seg=16)          # fiber hole z 2..10
    return r


# ---------------------------------------------------------------- camera sled
def sled():
    s = box(SLED_X0, SLED_X1, -SLED_Y, SLED_Y, 0, SLED_T)
    for sy in (1, -1):
        s += box(SLED_X0, SLED_X1, sy * RAIL_Y0 if sy > 0 else -RAIL_Y1,
                 sy * RAIL_Y1 if sy > 0 else -RAIL_Y0, SLED_T, SLED_T + RAIL_H)
    s -= cyl(WIN_D / 2, SLED_T + 2, -1, CAM_X, 0, SEG_S)
    for sy in (1, -1):
        s -= cyl(M3_CLEAR / 2, SLED_T + 2, -1, CAM_X, sy * SCREW_Y, SEG_S)
        for x0 in (19.0, 33.0):                                       # zip-tie slots beside each rail
            s -= box(x0, x0 + 4.0, sy * 17.5 if sy > 0 else -19.5,
                     sy * 19.5 if sy > 0 else -17.5, -1, SLED_T + 1)
    return s


# ---------------------------------------------------------------- base: cone seat + backlit floor
def base():
    b = cyl(BASE_R, PLATE_T, 0, seg=SEG)
    b += cone(FRUS_R_BOT, FRUS_R_TOP, FRUS_H, PLATE_T)                # the sleeve slides over this
    b -= cyl(WELL_R, BASE_H - MEMBRANE_T + 1, -1, seg=SEG)            # open well under the window
    b -= box(0, BASE_R + 1, -CHANNEL_W / 2, CHANNEL_W / 2, -0.01, CHANNEL_D)   # wire channel to the edge
    # notch in the floor's edge at +x (line the paper's squares up with it)
    b -= box(FRUS_R_TOP - 4.0, FRUS_R_TOP + 1, -0.6, 0.6, BASE_H - 0.6, BASE_H + 1)
    return b


def base_cone_seat(rim_id):
    """floor height minus where a rim of this inside diameter wedges on the base cone (mm below the floor)."""
    return rim_id / 2 - FRUS_R_TOP


# ---------------------------------------------------------------- LED puck
def puck():
    p = cyl(PUCK_R, PUCK_T, 0, seg=SEG)
    for x in (0.0, LED_X, -LED_X):
        p -= cyl(LED_D / 2, PUCK_T + 2, -1, x, 0, SEG_S)
    p -= box(-PUCK_R - 1, PUCK_R + 1, -1.6, 1.6, -0.01, 2.0)          # leg/wire groove underneath, along x
    for x in (0.0, LED_X, -LED_X):                                    # room for the LED's flange
        p -= cyl(3.1, 1.2, -0.01, x, 0, SEG_S)
    return p


def rotor_inertia():
    """Moment of inertia about the fiber axis (kg m^2) and mass (g): PLA part + two quarters."""
    rho = 1.24e-3            # g/mm^3, PLA, printed solid (thin part: walls + top/bottom dominate)
    r = rotor()
    mg = r.to_mesh()
    import numpy as np
    v = np.asarray(mg.vert_properties)[:, :3]
    f = np.asarray(mg.tri_verts)
    # volume integrals over tetrahedra from the origin
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6.0
    def sec(i):  # integral of x_i^2 over the solid
        return (det * (a[:, i] ** 2 + b[:, i] ** 2 + c[:, i] ** 2 + a[:, i] * b[:, i] + b[:, i] * c[:, i] + a[:, i] * c[:, i])).sum() / 60.0
    izz_pla = rho * (sec(0) + sec(1))                       # g mm^2
    m_pla = rho * vol
    m_q = QUARTER_G
    rq = QUARTER_D / 2
    izz_q = 2 * (m_q * ARM ** 2 + 0.5 * m_q * rq ** 2)       # g mm^2
    return (izz_pla + izz_q) * 1e-9, m_pla + 2 * m_q, m_pla
