#!/usr/bin/env python3
"""
build.py -- check the torsion pendulum parts and write one Bambu Studio project:
  torsion_pendulum.3mf   plate 1 lid, plate 2 base, plate 3 rotor (coins, pause), plate 4 knob + camera sled
plus STLs and check renders in out/.
"""
import io
import json
import math
import os
import uuid
import zipfile

import numpy as np
import trimesh
from PIL import Image

import parts as P
from render import render, label

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
TEMPLATE = os.path.join(HERE, "template_bambu.3mf")
os.makedirs(OUT, exist_ok=True)


def tm(man):
    mg = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mg.vert_properties)[:, :3], np.asarray(mg.tri_verts), process=True)


def check(name, man, voids=0):
    """one solid; `voids` sealed internal cavities allowed (the rotor's coin pockets)"""
    t = tm(man)
    shells = man.decompose()
    vols = sorted((s.volume() for s in shells), reverse=True)
    print(f"{name:12s} watertight={t.is_watertight} shells={len(shells)} faces={len(t.faces)} "
          f"extents={np.round(t.extents, 2).tolist()} volume={t.volume / 1000:.2f} cm3")
    assert t.is_watertight and len(shells) == 1 + voids, name
    if voids:
        # a sealed cavity's shell is a closed surface facing inward: its signed volume is the cavity's
        print(f"{'':12s} sealed cavities: {[round(abs(v), 1) for v in vols[1:]]} mm3")
    return t


# ------------------------------------------------------------------ build + check (use frame)
lid, knob, rotor, sled, base, puck = P.lid(), P.knob(), P.rotor(), P.sled(), P.base(), P.puck()
t_lid, t_knob, t_rotor, t_sled, t_base, t_puck = [check(n, m, v) for n, m, v in
    [("lid", lid, 0), ("knob+pin", knob, 0), ("rotor", rotor, 2), ("camera sled", sled, 0), ("base", base, 0),
     ("LED puck", puck, 0)]]

I, m_tot, m_pla = P.rotor_inertia()
print(f"rotor: PLA {m_pla:.2f} g (100% infill) + 2 quarters = {m_tot:.2f} g; I = {I:.3e} kg m^2")

# ---- fit checks
# the sleeve's rims wedge on the two 45-degree cones: where, for the range of real bottles
print("lid cone: rim inside diameter -> rim sits this far below the lid underside")
for rim_id in (85.0, 99.3, 104.0, 110.0, 119.3):
    z = P.lid_cone_seat(rim_id)
    print(f"   {rim_id:6.1f} mm -> {-z:5.1f} mm")
    assert -P.CONE_H + 0.5 <= z <= -0.5, rim_id
print("base cone: rim inside diameter -> rim sits this far below the floor")
for rim_id in (99.3, 104.0, 110.0, 119.3, 124.0):
    d = P.base_cone_seat(rim_id)
    print(f"   {rim_id:6.1f} mm -> {d:5.1f} mm")
    assert 0.1 <= d <= P.FRUS_H - 0.5, rim_id
# the cone's inner cavity clears the nut bosses and the camera window
r_inner_at_boss = P.CONE_R_TOP - P.CONE_WALL - 4.0
print(f"lid cone inner radius at the nut bosses {r_inner_at_boss:.1f} vs boss reach {math.hypot(P.CAM_X, P.SCREW_Y) + 5.5:.1f}")
assert r_inner_at_boss > math.hypot(P.CAM_X, P.SCREW_Y) + 5.5 + 1.0
rotor_r = P.ARM + P.DISC_D / 2
wall_gap = (P.BOTTLE_OD_MIN - 2 * P.BOTTLE_WALL) / 2 - rotor_r
print(f"rotor radius {rotor_r:.1f}; gap rotor-to-wall in the smallest (100 mm) bottle {wall_gap:.1f} mm")
assert wall_gap > 6
# rotor and reference squares must sit inside the backlit window as the camera sees them (parallax: the
# rotor is 20 mm nearer the camera than the paper), for sleeves from 150 to 230 mm
qh = 0.85 * P.REF_SIDE + 4.0                                   # tracker's reference box half-size (mm)
ref_corner = math.hypot(P.REF_X + qh, qh)
print(f"reference boxes reach r = {ref_corner:.1f} mm; backlit window r = {P.WELL_R:.1f}")
assert ref_corner <= P.WELL_R - 3
for z_paper in (165.0, 205.0, 240.0):
    tip = rotor_r * z_paper / (z_paper - P.ROTOR_GAP)
    print(f"camera {z_paper:.0f} mm above the paper: rotor tips seen at r = {tip:.1f} mm")
    assert tip <= P.WELL_R - 3, z_paper
knob_gap = P.SLED_X0 - P.KNOB_D / 2
print(f"knob edge to camera sled: {knob_gap:.1f} mm")
assert knob_gap >= 1.5
# camera field of view through the windows (lens ~3 mm above the sled plate, 6 mm of plastic below it)
half_fov = math.radians(33.0)
need_r = 3.0 + (3.0 + P.SLED_T + P.LID_T) * math.tan(half_fov)
print(f"window radius {P.WIN_D / 2:.1f} vs cone radius needed {need_r:.1f} (+-3 mm lens offset tolerated: "
      f"{P.WIN_D / 2 - need_r:.1f} mm spare)")
assert P.WIN_D / 2 > need_r
# the lid's cone must not hide any of the backlit window: trace the ray from the lens (3 mm above the sled)
# to the window's far edge and see where it passes the cone's tip
lens_above = P.LID_T + P.SLED_T + 3.0
tip_depth = lens_above + P.CONE_H
for z_paper in (150.0, 165.0, 205.0):
    far_x = P.CAM_X + (P.WELL_R - P.CAM_X) * tip_depth / z_paper          # ray to (+47, 0) on the paper
    far_y = P.WELL_R * tip_depth / z_paper                                 # ray to (26, +47)
    print(f"lens {z_paper:.0f} mm above the paper: rays to the window edge pass the cone tip at x = {far_x:.1f}, "
          f"|y| = {far_y:.1f} (cone inner radius there {P.CONE_R_TIP - P.CONE_WALL:.1f})")
    assert math.hypot(far_x, far_y) < P.CONE_R_TIP - P.CONE_WALL - 2, z_paper
# screw length: sled 3 + lid 3 + boss above nut 1.3 + nut 2.4
print(f"sled bolts: M3 x {3 + 3 + 1.3 + 2.4:.1f} minimum -> M3x10 (M3x12 also fine)")
print(f"LED puck {2 * P.PUCK_R:.0f} mm in a {2 * P.WELL_R:.0f} mm well; membrane {P.MEMBRANE_T} mm = {P.MEMBRANE_T / 0.2:.0f} layers at 0.2")
for n, t in [("lid", t_lid), ("base", t_base)]:
    assert max(t.extents[:2]) <= 175, n

# ------------------------------------------------------------------ print orientation
def to_print(man, flip=False):
    t = tm(man)
    if flip:
        t.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))
    t.apply_translation([-(t.bounds[0][0] + t.bounds[1][0]) / 2, -(t.bounds[0][1] + t.bounds[1][1]) / 2, -t.bounds[0][2]])
    return t


pr_lid = to_print(lid, flip=True)          # top face on the bed: cone, boss and nut traps print upward
pr_base = to_print(base, flip=True)        # floor on the bed: the window membrane is the first layers, well open above
pr_rotor = to_print(rotor)                 # flat; pause before the pocket roof
pr_knob = to_print(knob, flip=True)        # knob face on the bed, pin up (fiber hole open at the top)
pr_sled = to_print(sled)
pr_puck = to_print(puck)                   # groove down
for n, t in [("lid", pr_lid), ("base", pr_base), ("rotor", pr_rotor), ("knob", pr_knob), ("sled", pr_sled), ("puck", pr_puck)]:
    t.export(os.path.join(OUT, f"{n}.stl"))
    print(f"print {n:6s} bed footprint {t.extents[0]:.1f} x {t.extents[1]:.1f}, height {t.extents[2]:.1f}")
pause_z = P.ROTOR_FLOOR + P.POCKET_H                     # pocket top = first roof layer starts here
print(f"rotor pause: insert one quarter per pocket when layer {pause_z + 0.2:.2f} mm is about to print "
      f"(coin top {P.ROTOR_FLOOR + P.QUARTER_T:.2f} mm, roof starts {pause_z:.2f} mm)")

# ------------------------------------------------------------------ assembly picture (use frame)
SLEEVE_H = 200.0                                          # a typical cut: label panel + lower shoulder
RIM_ID = 104.0                                            # a 105 mm bottle
z_top_rim = P.lid_cone_seat(RIM_ID)                       # rim below the lid underside
z_floor = z_top_rim - SLEEVE_H + P.base_cone_seat(RIM_ID) # floor height in the lid frame
z_base0 = z_floor - P.BASE_H
HANG_Z = z_floor + P.ROTOR_GAP                            # rotor bottom
asm = []
def add(t, dz=0.0, dx=0.0, color=(90, 130, 200)):
    t = t.copy(); t.apply_translation([dx, 0, dz]); asm.append(((np.asarray(t.vertices), np.asarray(t.faces)), color))

add(t_lid, color=(70, 70, 75))
add(t_knob, P.LID_T, color=(200, 120, 40))
add(t_sled, P.LID_T, color=(60, 110, 200))
add(t_rotor, HANG_Z, color=(30, 30, 30))
bottle = trimesh.creation.annulus(RIM_ID / 2, RIM_ID / 2 + P.BOTTLE_WALL, SLEEVE_H, sections=96)
bottle.apply_translation([0, 0, z_top_rim - SLEEVE_H / 2])
add(bottle, color=(170, 210, 230))
add(t_base, z_base0, color=(120, 120, 120))
add(t_puck, z_base0, color=(200, 200, 200))
fiber_top, fiber_bot = -(P.LID_T + P.BOSS_H + 1.0) + 8.0, HANG_Z + P.ROTOR_T + P.HUB_H - 8.0
fiber = trimesh.creation.cylinder(0.35, fiber_top - fiber_bot, sections=8)
fiber.apply_translation([0, 0, (fiber_top + fiber_bot) / 2])
add(fiber, color=(200, 80, 40))
cam = trimesh.creation.box([67, 29, 1.6]); cam.apply_translation([P.CAM_X + 20, 0, P.LID_T + P.SLED_T + P.RAIL_H + 0.8])
add(cam, color=(30, 120, 60))
print(f"assembly picture: {SLEEVE_H:.0f} mm sleeve from a {RIM_ID + 2 * P.BOTTLE_WALL:.0f} mm bottle: lid underside to floor "
      f"{-z_floor:.1f} mm, fiber {fiber_top - fiber_bot:.0f} mm, camera lens about {-z_floor + P.LID_T + P.SLED_T + 3:.0f} mm above the paper")

# cut-away: drop the bottle's near half for the front view
def cutaway(item):
    (V, F), c = item
    keep = ~(V[F][:, :, 1].min(axis=1) < -1.0)
    return (V, F[keep]), c

imgs = []
for view, items in [("front", [cutaway(a) if a[1] == (170, 210, 230) else a for a in asm]), ("iso", asm[:3] + asm[8:9])]:
    im, _ = render([a[0] for a in items], view=view, size=760, colors=[a[1] for a in items])
    imgs.append(label(im, f"{view} view" + (" (bottle sleeve cut away; 200 mm sleeve shown)" if view == "front" else " (lid, knob, sled, camera)")))
sheet = Image.new("RGB", (1520, 760), "white")
sheet.paste(imgs[0], (0, 0)); sheet.paste(imgs[1], (760, 0))
sheet.save(os.path.join(OUT, "assembly.png"))

plate_imgs = []
for n, t, c in [("lid", pr_lid, (70, 70, 75)), ("base", pr_base, (120, 120, 120)), ("rotor", pr_rotor, (30, 30, 30)),
                ("knob", pr_knob, (200, 120, 40)), ("sled", pr_sled, (60, 110, 200)), ("puck", pr_puck, (200, 200, 200))]:
    im, _ = render([(np.asarray(t.vertices), np.asarray(t.faces))], view="iso", size=380, colors=[c])
    plate_imgs.append(label(im, n))
sheet2 = Image.new("RGB", (380 * 6, 380), "white")
for i, im in enumerate(plate_imgs):
    sheet2.paste(im, (380 * i, 0))
sheet2.save(os.path.join(OUT, "parts_print_orientation.png"))

# ------------------------------------------------------------------ Bambu Studio project
PLATES = [  # (plate name, [(object name, mesh, dx, dy)], per-object settings)
    ("Lid (cone up)", [("Lid (prints top-down, cone up)", pr_lid, 0, 0, {})]),
    ("Base (floor down)", [("Base (prints floor-down, well open above)", pr_base, 0, 0, {})]),
    ("Rotor - PAUSE for coins", [("Rotor (2 quarters, pause at 3.4 mm)", pr_rotor, 0, 0, {"sparse_infill_density": "100%"})]),
    ("Knob + sled + LED puck", [("Torsion knob + pin", pr_knob, -62, 30, {}), ("Camera sled", pr_sled, -62, -30, {}),
                               ("LED puck", pr_puck, 22, 0, {})]),
]
NCOL = math.ceil(math.sqrt(len(PLATES)))


def mesh_model_xml(obj_id, t):
    vs = "\n".join(f'     <vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>' for x, y, z in t.vertices)
    fs = "\n".join(f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t.faces)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <resources>
  <object id="{obj_id}" p:UUID="{uuid.uuid4()}" type="model">
   <mesh>
    <vertices>
{vs}
    </vertices>
    <triangles>
{fs}
    </triangles>
   </mesh>
  </object>
 </resources>
 <build/>
</model>
"""


tpl = zipfile.ZipFile(TEMPLATE)
objects = []        # (top_id, mesh_id, name, mesh, tx, ty, settings, plate_index)
nid = 1
for pi, (pname, items) in enumerate(PLATES):
    ox = 90 + 216 * (pi % NCOL)
    oy = 90 - 216 * (pi // NCOL)
    for oname, mesh, dx, dy, settings in items:
        objects.append((nid + 1, nid, oname, mesh, ox + dx, oy + dy, settings, pi))
        nid += 2

res_xml, build_xml, rels = [], [], []
for top_id, mesh_id, oname, mesh, tx, ty, settings, pi in objects:
    res_xml.append(f"""  <object id="{top_id}" p:UUID="{uuid.uuid4()}" type="model">
   <components>
    <component p:path="/3D/Objects/object_{mesh_id}.model" objectid="{mesh_id}" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
   </components>
  </object>""")
    build_xml.append(f'  <item objectid="{top_id}" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 {tx:.6f} {ty:.6f} 0.000000" printable="1"/>')
    rels.append(f' <Relationship Target="/3D/Objects/object_{mesh_id}.model" Id="rel-{mesh_id}" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>')

desc = ("2-liter-bottle torsion pendulum (FOIA list #2, SRI 1974 bell-jar test). The bottle is cut twice and its "
        "straight middle wedges on the lid's and base's 45-degree cones (fits 100-120 mm bottles). Plate 3 (rotor): "
        "print solid, and add a pause at 3.40 mm to drop one US quarter in each pocket. Lid prints top face down, "
        "base prints floor down. Hardware: 2x M3x10 + 2x M3 nuts (camera sled), a 5 mm diffused white LED + 470 ohm.")
model_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
 <metadata name="Application">BambuStudio-02.08.03.66</metadata>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <metadata name="CreationDate">2026-10-01</metadata>
 <metadata name="ModificationDate">2026-10-01</metadata>
 <metadata name="Title">Torsion pendulum (2-liter bottle)</metadata>
 <metadata name="Designer">P</metadata>
 <metadata name="Description">{desc}</metadata>
 <metadata name="Origin">original</metadata>
 <metadata name="Thumbnail_Middle">/Auxiliaries/.thumbnails/thumbnail_middle.png</metadata>
 <metadata name="Thumbnail_Small">/Auxiliaries/.thumbnails/thumbnail_small.png</metadata>
 <resources>
{chr(10).join(res_xml)}
 </resources>
 <build p:UUID="{uuid.uuid4()}">
{chr(10).join(build_xml)}
 </build>
</model>
"""
rels_xml = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
            + "\n".join(rels) + "\n</Relationships>\n")

ms = ['<?xml version="1.0" encoding="UTF-8"?>', "<config>"]
for top_id, mesh_id, oname, mesh, tx, ty, settings, pi in objects:
    extra = "".join(f'\n    <metadata key="{k}" value="{v}"/>' for k, v in settings.items())
    ms.append(f"""  <object id="{top_id}">
    <metadata key="name" value="{oname}"/>
    <metadata key="extruder" value="1"/>{extra}
    <metadata face_count="{len(mesh.faces)}"/>
    <part id="{mesh_id}" subtype="normal_part">
      <metadata key="name" value="{oname}"/>
      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>
      <metadata key="source_file" value="{oname}.stl"/>
      <metadata key="source_object_id" value="0"/>
      <metadata key="source_volume_id" value="0"/>
      <mesh_stat face_count="{len(mesh.faces)}" edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>
    </part>
  </object>""")
for pi, (pname, items) in enumerate(PLATES):
    inst = "".join(f"""
    <model_instance>
      <metadata key="object_id" value="{o[0]}"/>
      <metadata key="instance_id" value="0"/>
      <metadata key="identify_id" value="{1001 + k}"/>
    </model_instance>""" for k, o in enumerate(objects) if o[7] == pi)
    n = pi + 1
    ms.append(f"""  <plate>
    <metadata key="plater_id" value="{n}"/>
    <metadata key="plater_name" value="{pname}"/>
    <metadata key="locked" value="false"/>
    <metadata key="filament_map_mode" value="Auto For Flush"/>
    <metadata key="thumbnail_file" value="Metadata/plate_{n}.png"/>
    <metadata key="thumbnail_no_light_file" value="Metadata/plate_no_light_{n}.png"/>
    <metadata key="top_file" value="Metadata/top_{n}.png"/>
    <metadata key="pick_file" value="Metadata/pick_{n}.png"/>{inst}
  </plate>""")
ms.append("  <assemble>")
for top_id, mesh_id, oname, mesh, tx, ty, settings, pi in objects:
    ms.append(f'   <assemble_item object_id="{top_id}" instance_id="0" transform="1 0 0 0 1 0 0 0 1 {tx:.6f} {ty:.6f} 0.000000" offset="0 0 0" />')
ms.append("  </assemble>")
ms.append("</config>")
model_settings = "\n".join(ms) + "\n"

cut_info = '<?xml version="1.0" encoding="utf-8"?>\n<objects>\n' + "".join(
    f' <object id="{o[0]}">\n  <cut_id id="0" check_sum="1" connectors_cnt="0"/>\n </object>\n' for o in objects) + "</objects>\n"
fil_seq = json.dumps({f"plate_{i + 1}": {"nozzle_sequence": [], "optimal_assignment": [], "sequence": []} for i in range(len(PLATES))})


def png_bytes(im, size):
    b = io.BytesIO()
    im.resize((size, size)).save(b, "PNG")
    return b.getvalue()


plate_png = {}
for pi, (pname, items) in enumerate(PLATES):
    meshes = []
    for oname, mesh, dx, dy, s in items:
        v = np.asarray(mesh.vertices) + [dx, dy, 0]
        meshes.append((v, np.asarray(mesh.faces)))
    im, _ = render(meshes, view="iso", size=512, colors=[(80, 80, 85)] * len(meshes), bg=(235, 235, 235))
    plate_png[pi + 1] = im

out_path = os.path.join(OUT, "torsion_pendulum.3mf")
with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as z:
    for name in ["[Content_Types].xml", "_rels/.rels", "Metadata/project_settings.config", "Metadata/slice_info.config",
                 "Metadata/filament_settings_1.config", "Metadata/filament_settings_2.config", "Metadata/filament_settings_3.config"]:
        z.writestr(name, tpl.read(name))
    z.writestr("3D/3dmodel.model", model_xml)
    z.writestr("3D/_rels/3dmodel.model.rels", rels_xml)
    for top_id, mesh_id, oname, mesh, tx, ty, settings, pi in objects:
        z.writestr(f"3D/Objects/object_{mesh_id}.model", mesh_model_xml(mesh_id, mesh))
    z.writestr("Metadata/model_settings.config", model_settings)
    z.writestr("Metadata/cut_information.xml", cut_info)
    z.writestr("Metadata/filament_sequence.json", fil_seq)
    for n, im in plate_png.items():
        big, small = png_bytes(im, 512), png_bytes(im, 128)
        for f in (f"plate_{n}.png", f"plate_no_light_{n}.png", f"top_{n}.png", f"pick_{n}.png"):
            z.writestr(f"Metadata/{f}", big)
        z.writestr(f"Metadata/plate_{n}_small.png", small)
    cover = Image.open(os.path.join(OUT, "assembly.png")).crop((0, 0, 760, 760))
    z.writestr("Auxiliaries/.thumbnails/thumbnail_3mf.png", png_bytes(cover, 512))
    z.writestr("Auxiliaries/.thumbnails/thumbnail_middle.png", png_bytes(cover, 512))
    z.writestr("Auxiliaries/.thumbnails/thumbnail_small.png", png_bytes(cover, 128))
print("wrote", out_path, os.path.getsize(out_path), "bytes;", len(objects), "objects on", len(PLATES), "plates")
json.dump({"rotor_I_kg_m2": float(I), "rotor_mass_g": float(m_tot), "rotor_pla_g": float(m_pla), "quarters": 2,
           "arm_mm": P.ARM, "pause_layer_top_mm": P.ROTOR_FLOOR + P.POCKET_H + 0.2, "rotor_gap_mm": P.ROTOR_GAP,
           "paper_d_mm": P.PAPER_D, "ref_x_mm": P.REF_X, "floor_d_mm": 2 * P.FRUS_R_TOP, "window_d_mm": 2 * P.WELL_R},
          open(os.path.join(OUT, "rotor.json"), "w"), indent=1)
