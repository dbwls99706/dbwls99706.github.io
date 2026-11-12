"""Parametric quadruped robot for the portfolio hero scene.

An original design: a narrow trunk with a sensor head and a spinning LiDAR,
four identical three-joint legs (abduction, thigh, knee). Each rigid link is
modelled in its own joint frame so the web scene can articulate it directly.

Frame: X forward, Y left, Z up. Modelled in millimetres, exported in metres.

    python quadruped.py <output-dir>          # tessellated OBJ per link and colour + manifest.json
    python quadruped.py <output-dir> --step   # also write a STEP assembly in the standing pose
"""
import json
import sys
from pathlib import Path

import cadquery as cq

# ============================================================
# PARAMETERS
# ============================================================
# kinematics (keep in sync with KIN in js/robot.js)
HIP_X = 205.0        # trunk centre -> abduction joints, fore/aft
HIP_Y = 58.0         # trunk centre -> abduction joints, lateral
THIGH_Y = 94.0       # abduction joint -> thigh joint, lateral
LINK = 220.0         # thigh length = calf length
FOOT_R = 24.0
STAND = 310.0        # thigh joint height above the foot centre when standing

# trunk
TRUNK_L, TRUNK_W, TRUNK_H = 430.0, 132.0, 88.0
COVER_L, COVER_W, COVER_H = 300.0, 150.0, 34.0
MOTOR_R = 45.0       # joint actuator housings

# tessellation
TOL, ANG_TOL = 0.35, 0.3

COLORS = {
    "shell": [0.80, 0.82, 0.85],
    "frame": [0.16, 0.18, 0.21],
    "rubber": [0.07, 0.075, 0.085],
    "accent": [1.0, 0.698, 0.141],
    "sensor": [0.455, 0.925, 0.875],
}


# ============================================================
# HELPERS
# ============================================================
def rbox(lx, ly, lz, center=(0, 0, 0), rv=0.0, rh=0.0):
    """Box with rounded vertical edges (rv), then rounded top/bottom edges (rh)."""
    w = cq.Workplane("XY").box(lx, ly, lz)
    if rv:
        w = w.edges("|Z").fillet(rv)
    if rh:
        w = w.edges("#Z").fillet(rh)
    return w.translate(center)


def cyl(axis, r, a0, a1, at=(0, 0), chamfer=0.0):
    """Cylinder along X, Y or Z spanning a0..a1 on that axis; `at` is its position on the other two."""
    w = cq.Workplane("XY").circle(r).extrude(a1 - a0)
    if chamfer:
        w = w.edges().chamfer(chamfer)
    if axis == "x":
        return w.rotate((0, 0, 0), (0, 1, 0), 90).translate((a0, at[0], at[1]))
    if axis == "y":
        return w.rotate((0, 0, 0), (1, 0, 0), -90).translate((at[0], a0, at[1]))
    return w.translate((at[0], at[1], a0))


def side_profile(points, y0, y1, r=0.0):
    """Extrude an X-Z outline across y0..y1, optionally rounding the edges that run across."""
    w = cq.Workplane("XZ").polyline(points).close().extrude(y1 - y0)
    if r:
        w = w.edges("|Y").fillet(r)
    bb = w.val().BoundingBox()
    return w.translate((0, y0 - bb.ymin, 0))


# ============================================================
# MODEL  (each link in its own joint frame, left side; right side is mirrored on export)
# ============================================================
def trunk():
    top = TRUNK_H / 2 + 6
    parts = {"frame": [], "shell": [], "accent": [], "sensor": []}

    parts["frame"].append(rbox(TRUNK_L, TRUNK_W, TRUNK_H, (0, 0, 6), rv=24, rh=12))
    # abduction actuators at the four corners
    for sx in (1, -1):
        for sy in (1, -1):
            parts["frame"].append(cyl("x", MOTOR_R, sx * HIP_X - 42, sx * HIP_X + 42, (sy * HIP_Y, 0), chamfer=6))
    # top cover and side skid plates
    parts["shell"].append(rbox(COVER_L, COVER_W, COVER_H, (-12, 0, top + COVER_H / 2 - 12), rv=30, rh=10))
    for sy in (1, -1):
        parts["shell"].append(rbox(210, 10, 50, (-6, sy * (TRUNK_W / 2 + 2), 4), rv=4))
        parts["accent"].append(rbox(150, 4, 8, (-6, sy * (TRUNK_W / 2 + 7), 22)))
    # sensor head: a wedge with a wide camera bar
    head = side_profile([(0, -40), (60, -34), (92, -8), (92, 26), (62, 50), (0, 50)], -58, 58, r=9)
    parts["frame"].append(head.translate((TRUNK_L / 2 - 14, 0, 0)))
    parts["sensor"].append(rbox(8, 92, 16, (TRUNK_L / 2 + 77, 0, 10), rv=3))
    # LiDAR pedestal, rear handle and tail light
    parts["frame"].append(cyl("z", 30, top + COVER_H - 14, top + COVER_H + 4, (118, 0)))
    parts["frame"].append(rbox(46, 96, 20, (-178, 0, top + 8), rv=8, rh=6))
    parts["accent"].append(rbox(6, 84, 9, (-TRUNK_L / 2 - 1, 0, 14)))
    return parts


def lidar():
    """Spinning head, origin on its rotation axis at the top of the pedestal."""
    return {
        "frame": [cyl("z", 40, 0, 30, chamfer=4)],
        "shell": [cyl("z", 34, 30, 36, chamfer=2)],
        "sensor": [cyl("z", 41.5, 11, 18), rbox(12, 26, 18, (38, 0, 15), rv=3)],
    }


def hip():
    """Abduction link: a neck from the abduction joint (origin) out to the thigh actuator."""
    return {
        "frame": [rbox(64, 44, 62, (0, 20, 0), rv=10, rh=8)],
        "shell": [cyl("y", MOTOR_R, 30, THIGH_Y - 10, (0, 0), chamfer=6)],
    }


def thigh():
    """Upper leg, origin at the thigh joint, knee at z = -LINK."""
    beam = side_profile([(-34, 10), (34, 10), (30, -90), (22, -LINK - 6), (-22, -LINK - 6), (-36, -80)], 0, 30, r=12)
    return {
        "frame": [cyl("y", MOTOR_R + 3, -8, 32, (0, 0), chamfer=5), cyl("y", 27, -3, 33, (0, -LINK), chamfer=4)],
        "shell": [beam],
        "accent": [cyl("y", 29, 32, 37, (0, 0), chamfer=2)],
    }


def calf():
    """Lower leg, origin at the knee, foot centre at z = -LINK."""
    beam = side_profile([(-17, 6), (17, 6), (15, -70), (8, -LINK + 12), (-8, -LINK + 12), (-13, -110), (-22, -50)], 4, 26, r=5)
    foot = cq.Workplane("XY").sphere(FOOT_R).translate((0, 15, -LINK))
    return {"frame": [beam], "rubber": [foot, cyl("z", 13, -LINK + 4, -LINK + 26, (0, 15))]}


LINKS = {"trunk": trunk, "lidar": lidar, "hip": hip, "thigh": thigh, "calf": calf}


# ============================================================
# EXPORT
# ============================================================
def write_obj(path, solids):
    verts, faces = [], []
    for wp in solids:
        for solid in wp.vals():
            v, t = solid.tessellate(TOL, ANG_TOL)
            base = len(verts)
            verts += [(p.x / 1000, p.y / 1000, p.z / 1000) for p in v]
            faces += [(a + base + 1, b + base + 1, c + base + 1) for a, b, c in t]
    with open(path, "w") as f:
        f.writelines("v %.6f %.6f %.6f\n" % p for p in verts)
        f.writelines("f %d %d %d\n" % t for t in faces)
    return len(faces)


def write_step(path):
    """Whole robot in the standing pose, for inspection in a CAD viewer."""
    import math

    q = math.acos((STAND / 2) / LINK)        # thigh swings back, calf forward, foot under the joint
    asm = cq.Assembly()

    def add(name, parts, loc, mirror=False):
        for color, solids in parts.items():
            for i, wp in enumerate(solids):
                shape = wp.mirror("XZ") if mirror else wp
                asm.add(shape, name="%s_%s_%d" % (name, color, i), loc=loc, color=cq.Color(*COLORS[color]))

    add("trunk", trunk(), cq.Location((0, 0, 0)))
    add("lidar", lidar(), cq.Location((118, 0, TRUNK_H / 2 + 6 + COVER_H + 4)))
    for name, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)):
        hip_loc = cq.Location((sx * HIP_X, sy * HIP_Y, 0))
        thigh_loc = hip_loc * cq.Location((0, sy * THIGH_Y, 0), (0, 1, 0), math.degrees(q))
        calf_loc = thigh_loc * cq.Location((0, 0, -LINK), (0, 1, 0), math.degrees(-2 * q))
        add(name + "_hip", hip(), hip_loc, sy < 0)
        add(name + "_thigh", thigh(), thigh_loc, sy < 0)
        add(name + "_calf", calf(), calf_loc, sy < 0)
    asm.save(str(path))


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "build")
    out.mkdir(parents=True, exist_ok=True)
    manifest = {
        "colors": COLORS,
        "kinematics": {
            "hipX": HIP_X / 1000, "hipY": HIP_Y / 1000, "thighY": THIGH_Y / 1000, "link": LINK / 1000,
            "footRadius": FOOT_R / 1000, "stand": STAND / 1000,
            "lidar": [0.118, 0, (TRUNK_H / 2 + 6 + COVER_H + 4) / 1000],
        },
        "links": {},
    }
    for name, build in LINKS.items():
        manifest["links"][name] = []
        for color, solids in build().items():
            file = "%s_%s.obj" % (name, color)
            triangles = write_obj(out / file, solids)
            manifest["links"][name].append({"file": file, "color": color})
            print("%-18s %6d triangles" % (file, triangles))
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    if "--step" in sys.argv:
        write_step(out / "quadruped.step")
        print("wrote quadruped.step")


if __name__ == "__main__":
    main()
