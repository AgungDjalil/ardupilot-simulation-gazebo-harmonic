#!/usr/bin/env python3
"""The N°COMMAND pickup, deployed, as the team's renders draw it (mobil_ncommand_semua_sudut, 2026-11-10): a
low-poly double-cab pickup in sage green, a box canopy over the bed a little above the cab roof with N°COMMAND
on its sides, the side panels folded down into light grey decks (two sections a side, four drones a side), black
bumper and grille with orange indicators, a snorkel on the right A-pillar, black running boards, dark alloy wheels.
And ArUco landing pads, visual only: nothing in the simulation detects them yet.

    symphony/.venv/bin/python symphony/make_ncommand.py [OUT_MODEL_DIR]    (default: ardupilot_sim/models/ncommand_r23)

Same frame and the same pads as models/r23_hilux, so make_world.py's spawns and the station's rows are unchanged:
metres, CAD frame of Symphony's PanggungR23.tsx (Z up, Y towards the rear, X to the vehicle's left, origin on the
ground under the front bumper's centre); deck tops at 1.095 m; pad centres (+-1.42, 4.115) (+-1.42, 4.905) on the
main decks and (+-2.30, ...) on the outer ones. The canopy roof (top 2.02 m) carries the hexa, H1, in the middle.

Pads: the fleet's own board, rendered by Symphony's gcs/companion/landing_marker_aruco.py (imported, not copied):
ArUco DICT_ARUCO_MIP_36H12, six markers of three sizes, the 718 x 526 mm banner ("dock6", 20 cm markers, read
from 8 m), each pad its own id block. G1..G5 on pads 1..5 (ids 1-6 ... 41-46) and H1 on pad 6 (51-56, the
hexacopter's pad in the fleet), as the Pis are told; G6..G8 on pads 7..9 (61-66, 71-76, 81-86), which the fleet
does not print. Textured as quads with explicit UVs, so a marker is never mirrored.

The body is one OBJ of flat-shaded prisms (the renders' faceted look) with colours in an .mtl; it collides as
boxes (an OBJ collision crashes DART's Bullet detector).
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SYMPHONY = os.path.expanduser(os.environ.get("SYMPHONY_DIR", "~/Documents/symphony"))
sys.path.insert(0, os.path.join(SYMPHONY, "gcs", "companion"))
import landing_marker_aruco as lma  # noqa: E402  (Symphony's pad renderer, read only)

DECK_TOP = 1.095
DECK_Y = (3.66, 5.36)                      # along the canopy
MAIN_X = (0.97, 1.86)                      # main deck (the cassette), outer edge of the canopy to the split
OUTER_X = (1.86, 2.74)                     # the extension
PADS = [((1.42, 4.115), 1), ((-1.42, 4.115), 2), ((1.42, 4.905), 3), ((-1.42, 4.905), 4),
        ((2.30, 4.115), 5), ((-2.30, 4.115), 7), ((2.30, 4.905), 8), ((-2.30, 4.905), 9)]     # G1..G8
ROOF_TOP = 2.02
ROOF_PAD = ((0.0, 4.51), 6)                # H1, the hexa
BOARD = "dock6"

MATERIALS = {   # Kd, Ks, Ns, texture
    "body": ((0.36, 0.41, 0.36), 0.12, 20, None),
    "canopy": ((0.33, 0.38, 0.33), 0.10, 18, None),
    "trim": ((0.055, 0.06, 0.055), 0.15, 20, None),
    "glass": ((0.025, 0.03, 0.035), 0.65, 90, None),
    "deck": ((0.86, 0.87, 0.86), 0.10, 15, None),
    "seam": ((0.16, 0.18, 0.16), 0.05, 10, None),
    "orange": ((1.0, 0.5, 0.08), 0.3, 40, None),
    "white": ((0.93, 0.93, 0.93), 0.3, 40, None),
    "red": ((0.62, 0.04, 0.04), 0.3, 40, None),
    "tyre": ((0.04, 0.04, 0.042), 0.05, 8, None),
    "rim": ((0.16, 0.165, 0.17), 0.4, 50, None),
    "silver": ((0.66, 0.67, 0.69), 0.6, 70, None),
    "label": ((1, 1, 1), 0.1, 15, "label.png"),
}


class Mesh:
    def __init__(self):
        self.v, self.vt, self.vn, self.faces = [], [], [], []      # faces: (material, [(vi, ti, ni), ...])

    def face(self, pts, mat, uv=None):
        """A flat polygon (counter-clockwise seen from outside)."""
        p = np.asarray(pts, float)
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-12:
            n = np.cross(p[2] - p[0], p[3] - p[0])
        n /= np.linalg.norm(n) + 1e-12
        base = len(self.v)
        self.v += [tuple(q) for q in p]
        self.vn.append(tuple(n))
        ni = len(self.vn)
        if uv is not None:
            tb = len(self.vt)
            self.vt += [tuple(t) for t in uv]
            idx = [(base + k + 1, tb + k + 1, ni) for k in range(len(p))]
        else:
            idx = [(base + k + 1, None, ni) for k in range(len(p))]
        for k in range(1, len(p) - 1):
            self.faces.append((mat, [idx[0], idx[k], idx[k + 1]]))

    def prism(self, bot, top, mat, skip=()):
        """bot, top: 4 corners each, counter-clockwise seen from above, top[k] above bot[k]."""
        b, t = [np.asarray(c, float) for c in bot], [np.asarray(c, float) for c in top]
        if "bottom" not in skip:
            self.face([b[0], b[3], b[2], b[1]], mat)
        if "top" not in skip:
            self.face(t, mat)
        for k in range(4):
            j = (k + 1) % 4
            self.face([b[k], b[j], t[j], t[k]], mat)

    def box(self, x, y, z, mat, top_inset=0.0):
        x0, x1 = x; y0, y1 = y; z0, z1 = z
        i = top_inset
        self.prism([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
                   [(x0 + i, y0 + i, z1), (x1 - i, y0 + i, z1), (x1 - i, y1 - i, z1), (x0 + i, y1 - i, z1)], mat)

    def cylinder_x(self, cx, cy, cz, r, length, mat, sides=16, phase=0.0):
        """Axis along X (a wheel), centred."""
        a = np.linspace(0, 2 * math.pi, sides, endpoint=False) + phase
        ring = lambda x: [(x, cy + r * math.cos(t), cz + r * math.sin(t)) for t in a]
        r0, r1 = ring(cx - length / 2), ring(cx + length / 2)
        self.face(r1, mat)
        self.face(r0[::-1], mat)
        for k in range(sides):
            j = (k + 1) % sides
            self.face([r0[k], r0[j], r1[j], r1[k]], mat)

    def plate(self, corners, mat, t=0.004, uv=None):
        """A thin plate on a surface: corners counter-clockwise seen from outside; the textured face is the outer
        one, offset t outwards."""
        c = np.asarray(corners, float)
        n = np.cross(c[1] - c[0], c[2] - c[0]); n /= np.linalg.norm(n)
        outer, inner = c + n * t, c + n * 0.0005
        self.face(outer, mat, uv)
        self.face(inner[::-1], "trim")
        for k in range(4):
            j = (k + 1) % 4
            self.face([inner[k], inner[j], outer[j], outer[k]], "trim")

    def write(self, path, mtl):
        groups = {}
        for mat, idx in self.faces:
            groups.setdefault(mat, []).append(idx)
        with open(path, "w") as f:
            f.write(f"# N°COMMAND pickup, by symphony/make_ncommand.py\nmtllib {mtl}\n")
            f.write("".join(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in self.v))
            f.write("".join(f"vt {u:.4f} {w:.4f}\n" for u, w in self.vt))
            f.write("".join(f"vn {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in self.vn))
            for mat, faces in groups.items():
                f.write(f"g {mat}\nusemtl {mat}\n")
                for idx in faces:
                    f.write("f " + " ".join(f"{v}/{t}/{n}" if t else f"{v}//{n}" for v, t, n in idx) + "\n")


def mirror(fn):
    """Build a part on the left (+X) and its mirror on the right."""
    for s in (1, -1):
        fn(s)


def sx(s, a, b):
    """An x interval on side s (1 left, -1 right), ordered."""
    return tuple(sorted((s * a, s * b)))


def build():
    m = Mesh()
    W = 0.95
    # ── lower body, with the wheel arches cut (front wheel y 1.15, rear 4.12, radius 0.36) ──
    for y0, y1 in ((0.10, 0.70), (1.60, 3.67), (4.57, 5.37)):
        m.box((-W, W), (y0, y1), (0.40, 1.00), "body")
    for y0, y1 in ((0.70, 1.60), (3.67, 4.57)):
        m.box((-W, W), (y0, y1), (0.80, 1.00), "body")
        m.box((-0.70, 0.70), (y0, y1), (0.40, 0.80), "trim")                       # under the arch, between wheels
    # front face: bumper, grille, lights
    m.box((-0.94, 0.94), (0.02, 0.17), (0.34, 0.62), "trim")
    m.box((-0.56, 0.56), (0.075, 0.10), (0.64, 0.95), "trim")
    for z in (0.69, 0.76, 0.83, 0.90):
        m.box((-0.52, 0.52), (0.068, 0.076), (z, z + 0.025), "seam")
    m.box((-0.13, 0.13), (0.066, 0.074), (0.865, 0.895), "silver")
    mirror(lambda s: m.box(sx(s, 0.70, 0.86), (0.012, 0.022), (0.56, 0.60), "orange"))
    mirror(lambda s: m.box(sx(s, 0.58, 0.80), (0.012, 0.020), (0.45, 0.475), "white"))
    # hood, sloping up to the windscreen, its top narrower (the faceted shoulders)
    m.prism([(-W, 0.08, 1.00), (W, 0.08, 1.00), (W, 1.75, 1.00), (-W, 1.75, 1.00)],
            [(-0.86, 0.16, 1.10), (0.86, 0.16, 1.10), (0.88, 1.72, 1.18), (-0.88, 1.72, 1.18)], "body")
    # cab: beltline, then the greenhouse with a raked windscreen
    m.prism([(-W, 1.75, 1.00), (W, 1.75, 1.00), (W, 3.62, 1.00), (-W, 3.62, 1.00)],
            [(-0.93, 1.75, 1.20), (0.93, 1.75, 1.20), (0.93, 3.62, 1.20), (-0.93, 3.62, 1.20)], "body")
    gh_b, gh_t = (0.93, 1.75, 3.60, 1.20), (0.82, 2.08, 3.52, 1.95)
    m.prism([(-gh_b[0], gh_b[1], gh_b[3]), (gh_b[0], gh_b[1], gh_b[3]), (gh_b[0], gh_b[2], gh_b[3]), (-gh_b[0], gh_b[2], gh_b[3])],
            [(-gh_t[0], gh_t[1], gh_t[3]), (gh_t[0], gh_t[1], gh_t[3]), (gh_t[0], gh_t[2], gh_t[3]), (-gh_t[0], gh_t[2], gh_t[3])], "body")
    # windscreen on the front face of the greenhouse
    lerp = lambda a, b, f: tuple(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * f)
    fl, fr = (-0.93, 1.75, 1.20), (0.93, 1.75, 1.20)
    tl, tr = (-gh_t[0], gh_t[1], gh_t[3]), (gh_t[0], gh_t[1], gh_t[3])
    ws = [lerp(lerp(fr, fl, 0.05), lerp(tr, tl, 0.05), 0.10), lerp(lerp(fr, fl, 0.95), lerp(tr, tl, 0.95), 0.10),
          lerp(lerp(fr, fl, 0.95), lerp(tr, tl, 0.95), 0.93), lerp(lerp(fr, fl, 0.05), lerp(tr, tl, 0.05), 0.93)]
    m.plate(ws, "glass")

    def side_windows(s):
        xz = lambda z: s * (gh_b[0] - (z - gh_b[3]) * ((gh_b[0] - gh_t[0]) / (gh_t[3] - gh_b[3])))
        for y0, y1 in ((2.36, 2.96), (3.03, 3.50)):
            z0, z1 = 1.28, 1.86
            c = [(xz(z0), y0, z0), (xz(z0), y1, z0), (xz(z1), y1, z1), (xz(z1), y0, z1)]
            m.plate(c if s > 0 else c[::-1], "glass")          # normal +x on the left, -x on the right
    mirror(side_windows)

    def doors(s):
        for y in (1.80, 2.99, 3.58):
            m.box(sx(s, W - 0.002, W + 0.004), (y - 0.006, y + 0.006), (0.46, 1.19), "seam")
        for y in (2.78, 3.40):
            m.box(sx(s, W, W + 0.018), (y, y + 0.12), (1.04, 1.075), "trim")
        m.box(sx(s, 0.95, 1.08), (1.95, 2.06), (1.27, 1.40), "trim")                # mirror
        m.box(sx(s, 0.95, 1.12), (1.75, 3.62), (0.42, 0.47), "trim")                # running board
        for k in range(8):
            y = 1.82 + k * 0.225
            m.box(sx(s, 0.99, 1.09), (y, y + 0.12), (0.471, 0.474), "seam")
    mirror(doors)
    # snorkel up the right A-pillar
    m.box((-1.06, -0.96), (1.62, 1.72), (0.95, 1.90), "trim")
    m.box((-1.06, -0.96), (1.52, 1.72), (1.86, 1.96), "trim")

    # ── canopy over the bed: vertical sides, bevelled roof edges ──
    m.box((-0.97, 0.97), (3.64, 5.38), (1.00, 1.94), "canopy")
    m.prism([(-0.97, 3.64, 1.94), (0.97, 3.64, 1.94), (0.97, 5.38, 1.94), (-0.97, 5.38, 1.94)],
            [(-0.90, 3.70, ROOF_TOP), (0.90, 3.70, ROOF_TOP), (0.90, 5.32, ROOF_TOP), (-0.90, 5.32, ROOF_TOP)], "canopy",
            skip=("bottom",))

    def canopy_side(s):
        m.box(sx(s, 0.968, 0.975), (3.64, 5.38), (1.055, 1.065), "seam")              # the hinge of the deck
        m.box(sx(s, 0.968, 0.975), (3.64, 5.38), (1.60, 1.607), "seam")               # panel line
        m.box(sx(s, 0.97, 0.985), (5.10, 5.24), (1.40, 1.44), "silver")               # latch
        # N°COMMAND near the top front, reading front to rear on the left, rear to front on the right
        y0, y1, z0, z1 = 3.76, 4.30, 1.76, 1.84
        x = s * 0.97
        c = [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]
        uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
        if s < 0:      # outward -x, and the text still reads left to right from outside
            c = [c[1], c[0], c[3], c[2]]
        m.plate(c, "label", t=0.002, uv=uv)
    mirror(canopy_side)
    # rear: handle, plate, hitch, tail lights
    m.box((-0.18, 0.18), (5.38, 5.395), (1.50, 1.56), "silver")
    m.box((-0.17, 0.17), (5.37, 5.40), (0.47, 0.58), "white")
    m.box((-0.94, 0.94), (5.30, 5.44), (0.36, 0.46), "trim")
    m.box((-0.05, 0.05), (5.44, 5.60), (0.38, 0.44), "trim")
    mirror(lambda s: m.box(sx(s, 0.78, 0.92), (5.37, 5.39), (0.80, 0.92), "red"))

    # ── decks: the side panels folded down, main section and extension ──
    def decks(s):
        for x in (MAIN_X, OUTER_X):
            m.box(sx(s, *x), DECK_Y, (DECK_TOP - 0.035, DECK_TOP), "deck")
        m.box(sx(s, OUTER_X[0] - 0.004, OUTER_X[0] + 0.004), DECK_Y, (DECK_TOP, DECK_TOP + 0.002), "seam")
        m.box(sx(s, OUTER_X[1] - 0.03, OUTER_X[1]), DECK_Y, (DECK_TOP - 0.06, DECK_TOP - 0.035), "trim")   # edge rail
    mirror(decks)

    # ── wheels: tyre, dark rim, silver spokes ──
    for y in (1.15, 4.12):
        for s in (1, -1):
            cx = s * 0.83
            m.cylinder_x(cx, y, 0.36, 0.36, 0.27, "tyre", sides=18)
            m.cylinder_x(cx + s * 0.137, y, 0.36, 0.23, 0.006, "rim", sides=18)
            m.cylinder_x(cx + s * 0.142, y, 0.36, 0.05, 0.008, "silver", sides=10)
            for k in range(6):
                a = k * math.pi / 3
                dy, dz = math.cos(a), math.sin(a)
                p0, p1 = np.array([y + 0.05 * dy, 0.36 + 0.05 * dz]), np.array([y + 0.21 * dy, 0.36 + 0.21 * dz])
                nrm = np.array([-dz, dy]) * 0.018
                xo = cx + s * 0.141
                quad = [(xo, *(p0 - nrm)), (xo, *(p1 - nrm)), (xo, *(p1 + nrm)), (xo, *(p0 + nrm))]
                m.face(quad if s > 0 else quad[::-1], "silver")
    return m


def pads(m, out_tex):
    """ArUco pads as textured plates, image right = +X, image up = +Y (never mirrored seen from above)."""
    mats = {}
    for (cx, cy), pad in PADS + [ROOF_PAD]:
        img, (w_mm, h_mm) = lma.render_pad(pad, BOARD, px_per_mm=1.5)
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        name = f"pad{pad}"
        cv2.imwrite(os.path.join(out_tex, name + ".png"), img)
        mats[name] = ((1, 1, 1), 0.05, 10, name + ".png")
        w, h = w_mm / 1000, h_mm / 1000
        z = ROOF_TOP if pad == ROOF_PAD[1] else DECK_TOP
        c = [(cx - w / 2, cy - h / 2, z), (cx + w / 2, cy - h / 2, z), (cx + w / 2, cy + h / 2, z), (cx - w / 2, cy + h / 2, z)]
        m.plate(c, name, t=0.002, uv=[(0, 0), (1, 0), (1, 1), (0, 1)])
    return mats


def label(path):
    """N°COMMAND, white on the canopy's green."""
    w, h = 1080, 160
    kd = MATERIALS["canopy"][0]
    img = np.full((h, w, 3), [int(255 * kd[2] * 1.05), int(255 * kd[1] * 1.05), int(255 * kd[0] * 1.05)], np.uint8)
    font, scale, th = cv2.FONT_HERSHEY_DUPLEX, 3.0, 6
    (tw, _), _ = cv2.getTextSize("NCOMMAND", font, scale, th)
    x = (w - tw - 40) // 2
    cv2.putText(img, "N", (x, 118), font, scale, (240, 240, 240), th, cv2.LINE_AA)
    nx = x + cv2.getTextSize("N", font, scale, th)[0][0]
    cv2.circle(img, (nx + 18, 52), 13, (240, 240, 240), 5, cv2.LINE_AA)          # the degree sign
    cv2.putText(img, "COMMAND", (nx + 40, 118), font, scale, (240, 240, 240), th, cv2.LINE_AA)
    cv2.imwrite(path, img)


SDF = """<?xml version="1.0" ?>
<!-- N°COMMAND pickup, deployed, from the team's renders (mobil_ncommand_semua_sudut, 2026-11-10); generated by
     symphony/make_ncommand.py. Frame: Symphony's PanggungR23.tsx CAD frame (Z up, Y rear, X left, origin on the
     ground under the front bumper). Decks at 1.095 m with the R23's eight pad centres; the canopy roof (2.02 m)
     carries H1. ArUco pads (DICT_ARUCO_MIP_36H12, the fleet's 718 x 526 mm banner) are visual only. -->
<sdf version="1.9">
  <model name="ncommand_r23">
    <static>true</static>
    <link name="body">
      <visual name="body"><geometry><mesh><uri>model://ncommand_r23/meshes/ncommand.obj</uri></mesh></geometry></visual>
{collisions}
    </link>
  </model>
</sdf>
"""


def collisions():
    boxes = [("chassis", (0, 2.725, 0.70), (1.90, 5.29, 0.60)),
             ("hood", (0, 0.915, 1.09), (1.90, 1.67, 0.18)),
             ("cab", (0, 2.685, 1.46), (1.86, 1.87, 0.92)),
             ("canopy", (0, 4.51, 1.51), (1.94, 1.74, 1.02))]
    for s, side in ((1, "left"), (-1, "right")):
        for name, (x0, x1) in (("main", MAIN_X), ("outer", OUTER_X)):
            boxes.append((f"deck_{name}_{side}", (s * (x0 + x1) / 2, sum(DECK_Y) / 2, DECK_TOP - 0.0175),
                          (x1 - x0, DECK_Y[1] - DECK_Y[0], 0.035)))
    return "\n".join(f'      <collision name="{n}"><pose>{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} 0 0 0</pose>'
                     f'<geometry><box><size>{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}</size></box></geometry></collision>'
                     for n, p, s in boxes)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(HERE), "models", "ncommand_r23")
    meshes = os.path.join(out, "meshes")
    tex = os.path.join(meshes, "textures")
    os.makedirs(tex, exist_ok=True)
    m = build()
    label(os.path.join(tex, "label.png"))
    mats = dict(MATERIALS)
    mats.update(pads(m, tex))
    m.write(os.path.join(meshes, "ncommand.obj"), "ncommand.mtl")
    with open(os.path.join(meshes, "ncommand.mtl"), "w") as f:
        for name, (kd, ks, ns, t) in mats.items():
            f.write(f"newmtl {name}\nKa {kd[0] * 0.6:.3f} {kd[1] * 0.6:.3f} {kd[2] * 0.6:.3f}\n"
                    f"Kd {kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}\nKs {ks:.3f} {ks:.3f} {ks:.3f}\nNs {ns}\nd 1.0\nillum 2\n"
                    + (f"map_Kd textures/{t}\n" if t else "") + "\n")
    with open(os.path.join(out, "model.sdf"), "w") as f:
        f.write(SDF.format(collisions=collisions()))
    with open(os.path.join(out, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>ncommand_r23</name>\n  <version>1.0</version>\n'
                '  <sdf version="1.9">model.sdf</sdf>\n  <description>The N°COMMAND pickup, deployed, with ArUco '
                "pads (visual only); symphony/make_ncommand.py.</description>\n</model>\n")
    print(f"{out}: {len(m.v)} vertices, {len(m.faces)} triangles, pads "
          + ", ".join(f"{p}:{lma.pad_ids(p, BOARD)[0]}-{lma.pad_ids(p, BOARD)[-1]}" for _, p in PADS + [ROOF_PAD]))


if __name__ == "__main__":
    main()
