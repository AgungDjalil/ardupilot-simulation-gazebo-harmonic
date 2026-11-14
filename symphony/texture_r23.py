#!/usr/bin/env python3
"""Colours and textures for the R23 meshes that convert_r23.mjs writes (position-only OBJ, one piece each).

    python3 symphony/texture_r23.py [MODELS_DIR]     (default: ardupilot_sim/models)

The web models carry no parts, materials or UVs, so the parts are found by where they are, in the frame
PanggungR23.tsx draws them in (metres, Z up; the Hilux with Y towards the rear):
  Hilux  tyres and rims (around the two axles, y 0.95 and 4.1, wheel radius 0.39), glass (cab, y 1.35..3.75,
         z 1.28..1.74, faces not facing up), head and tail lights, grille and bumpers, the rest painted body;
  quad   landing gear (z < 0.10), carbon frame (r < 0.21), light grey payload box on top and motors (r > 0.24),
         propellers and the rest;
  hexa   carbon frame, dark trim below.
Each part gets a material in an .mtl beside the mesh, with a procedural texture; UVs are a box projection
(each face along its dominant axis), tyres cylindrical around the axle so the tread runs round. Normals are
smoothed across edges under 40 deg, so the panels read as panels. Writes *_tex.obj, *.mtl and textures/*.png
next to the input meshes, and textures for the R23 boxes (canopy, N°COMMAND label, decks, pad markings) into
r23_hilux/materials. Colours follow the R23 render on Symphony's landing page (public/landing/r23-seq): olive
drab body, a darker olive canopy, light grey decks with black foot markers.
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(HERE), "models")
RNG = np.random.default_rng(23)
ORANGE = (16, 110, 240)   # BGR, Norden orange
OLIVE = (70, 84, 78)       # BGR, the R23's body
OLIVE_DARK = (54, 64, 59)  # BGR, its canopy
DECK = (178, 184, 182)     # BGR, the cassette decks


# ── textures ─────────────────────────────────────────────────────────────────

def noise(size, scale, amp):
    small = RNG.normal(0, 1, (max(2, size // scale), max(2, size // scale))).astype(np.float32)
    return cv2.resize(small, (size, size), interpolation=cv2.INTER_CUBIC) * amp


def tile(bgr, size=512, grain=6.0, blotch=10.0):
    img = np.empty((size, size, 3), np.float32)
    img[:] = bgr
    img += noise(size, 1, grain)[..., None] + noise(size, 64, blotch)[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


def paint():
    # Olive drab, satin: fine flake over a slow cloudiness.
    img = tile(OLIVE, grain=8, blotch=6).astype(np.float32)
    flake = (RNG.random((512, 512)) > 0.99) * 25.0
    return np.clip(img + flake[..., None], 0, 255).astype(np.uint8)


def tyre():
    img = tile((24, 24, 26), grain=5, blotch=3)
    for x in range(0, 512, 32):            # tread blocks across the width, repeated round the tyre
        cv2.rectangle(img, (x, 0), (x + 14, 511), (14, 14, 15), -1)
    for y in range(0, 512, 64):
        cv2.line(img, (0, y), (511, y + 20), (12, 12, 13), 6)
    return img


def carbon():
    img = np.zeros((512, 512, 3), np.float32)
    yy, xx = np.mgrid[0:512, 0:512]
    weave = ((xx // 16 + yy // 16) % 2).astype(np.float32)
    sheen = np.where(weave > 0, np.sin(xx / 16 * np.pi) ** 2, np.sin(yy / 16 * np.pi) ** 2)
    img[:] = (26, 25, 24)
    img += (sheen * 26)[..., None] + noise(512, 1, 3)[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


def diamond_plate():
    img = tile((92, 94, 98), grain=6, blotch=8)
    for y in range(0, 512, 32):
        for x in range(0, 512, 32):
            ox = 16 if (y // 32) % 2 else 0
            c = (x + ox + 8, y + 8)
            cv2.ellipse(img, c, (9, 3), 45 if (x // 32) % 2 else -45, 0, 360, (130, 133, 138), -1)
            cv2.ellipse(img, (c[0] + 1, c[1] + 2), (9, 3), 45 if (x // 32) % 2 else -45, 0, 360, (60, 62, 66), 1)
    return img


def deck():
    """Light grey composite, anti-slip grain, a thin seam round the edge."""
    img = tile(DECK, grain=7, blotch=5)
    cv2.rectangle(img, (4, 4), (507, 507), (140, 146, 144), 4)
    return img


def pad_marks():
    """One aircraft's place on a deck (0.6 m square): four black foot locators and a faint outline, as on the
    render; symmetric, so the box face's texture orientation does not matter."""
    img = tile(DECK, grain=7, blotch=5)
    cv2.rectangle(img, (10, 10), (501, 501), (150, 156, 154), 3)
    for cx in (110, 401):
        for cy in (110, 401):
            cv2.rectangle(img, (cx - 34, cy - 22), (cx + 34, cy + 22), (22, 22, 24), -1)
    cv2.rectangle(img, (226, 236), (286, 276), (22, 22, 24), -1)
    return img


def canopy():
    """Darker olive panels with seams."""
    img = tile(OLIVE_DARK, grain=6, blotch=6)
    for x in (0, 170, 341, 511):
        cv2.line(img, (x, 0), (x, 511), (40, 48, 44), 3)
    return img


def label():
    """The canopy's rear face: N°COMMAND, as on the render."""
    img = canopy()
    font = cv2.FONT_HERSHEY_DUPLEX
    cv2.putText(img, "N", (150, 270), font, 2.0, (225, 228, 226), 3, cv2.LINE_AA)
    cv2.circle(img, (205, 228), 9, (225, 228, 226), 3, cv2.LINE_AA)
    cv2.putText(img, "COMMAND", (222, 270), font, 2.0, (225, 228, 226), 3, cv2.LINE_AA)
    cv2.putText(img, "R23", (236, 320), font, 0.9, (190, 195, 192), 2, cv2.LINE_AA)
    return cv2.flip(img, 1)


# ── mesh ─────────────────────────────────────────────────────────────────────

def load_obj(path):
    v, f = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                v.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    return np.array(v), np.array(f)


def face_normals(v, f):
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def corner_normals(v, f, fn, crease_deg=40.0):
    """Per face-corner normals: the mean of the faces around that vertex within crease_deg of this face."""
    cos_c = np.cos(np.radians(crease_deg))
    around = [[] for _ in range(len(v))]
    for fi, tri in enumerate(f):
        for vi in tri:
            around[vi].append(fi)
    out = np.empty((len(f), 3, 3))
    for fi, tri in enumerate(f):
        for k, vi in enumerate(tri):
            adj = np.array(around[vi])
            nn = fn[adj]
            s = nn[nn @ fn[fi] > cos_c].sum(0)
            out[fi, k] = s / (np.linalg.norm(s) + 1e-12)
    return out


def box_uv(v, f, fn, metres_per_tile):
    """Each face projected along its dominant normal axis."""
    p = v[f] / metres_per_tile                     # (F, 3, 3)
    axis = np.abs(fn).argmax(1)
    uv = np.empty((len(f), 3, 2))
    uv[axis == 0] = p[axis == 0][:, :, [1, 2]]
    uv[axis == 1] = p[axis == 1][:, :, [0, 2]]
    uv[axis == 2] = p[axis == 2][:, :, [0, 1]]
    return uv


def write_obj(path, mtl_name, v, f, groups, uv, cn, note):
    """groups: material name per face. Faces are written grouped by material."""
    with open(path, "w") as fh:
        fh.write(f"# {note}\nmtllib {mtl_name}\n")
        for x, y, z in v:
            fh.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for a, b in uv.reshape(-1, 2):
            fh.write(f"vt {a:.4f} {b:.4f}\n")
        for a, b, c in cn.reshape(-1, 3):
            fh.write(f"vn {a:.4f} {b:.4f} {c:.4f}\n")
        for name in dict.fromkeys(groups):
            fh.write(f"g {name}\nusemtl {name}\n")
            for fi in np.nonzero(groups == name)[0]:
                corners = " ".join(f"{f[fi, k] + 1}/{3 * fi + k + 1}/{3 * fi + k + 1}" for k in range(3))
                fh.write(f"f {corners}\n")


def write_mtl(path, materials):
    with open(path, "w") as fh:
        for name, (kd, ks, ns, tex) in materials.items():
            fh.write(f"newmtl {name}\nKa {kd[0] * 0.6:.3f} {kd[1] * 0.6:.3f} {kd[2] * 0.6:.3f}\n"
                     f"Kd {kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}\nKs {ks:.3f} {ks:.3f} {ks:.3f}\nNs {ns}\nd 1.0\nillum 2\n")
            if tex:
                fh.write(f"map_Kd textures/{tex}\n")
            fh.write("\n")


def save_textures(d, textures):
    os.makedirs(os.path.join(d, "textures"), exist_ok=True)
    for name, img in textures.items():
        cv2.imwrite(os.path.join(d, "textures", name), img)


# ── parts ────────────────────────────────────────────────────────────────────

def hilux(d):
    v, f = load_obj(os.path.join(d, "hilux_body.obj"))
    fn = face_normals(v, f)
    c = v[f].mean(1)
    x, y, z = c[:, 0], c[:, 1], c[:, 2]
    g = np.full(len(f), "paint", dtype=object)
    g[(y < 0.40) & (z < 0.62)] = "trim"                                       # front bumper, lower
    g[(y > 5.00) & (z < 0.62)] = "trim"                                       # rear bumper
    g[(y < 0.20) & (z > 0.60) & (z < 1.00) & (np.abs(x) < 0.45)] = "trim"     # grille
    g[(y < 0.35) & (z > 0.80) & (z < 1.05) & (np.abs(x) > 0.45)] = "headlight"
    g[(y > 5.15) & (z > 0.80) & (z < 1.20) & (np.abs(x) > 0.72)] = "taillight"
    g[(y > 1.35) & (y < 3.75) & (z > 1.28) & (z < 1.74) & (np.abs(fn[:, 2]) < 0.75)] = "glass"
    uv = box_uv(v, f, fn, 0.5)
    for axle in (0.95, 4.10):
        dy, dz = y - axle, z - 0.39
        rr = np.hypot(dy, dz)
        on_tyre = (rr < 0.40) & (np.abs(x) > 0.62)
        rim = on_tyre & (rr < 0.24) & (np.abs(x) > 0.80)
        g[on_tyre] = "tyre"
        g[rim] = "rim"
        # Tyres: u round the axle, v across the tread.
        sel = np.nonzero(on_tyre & ~rim)[0]
        p = v[f[sel]]
        uv[sel, :, 0] = (np.arctan2(p[:, :, 2] - 0.39, p[:, :, 1] - axle) / (2 * np.pi)) * 6.0
        uv[sel, :, 1] = p[:, :, 0] * 2.0
    cn = corner_normals(v, f, fn)
    save_textures(d, {"paint.png": paint(), "tyre.png": tyre(), "trim.png": tile((30, 30, 32), grain=6, blotch=4),
                      "rim.png": tile((150, 150, 155), grain=8, blotch=6)})
    write_mtl(os.path.join(d, "hilux.mtl"), {
        "paint": ((1, 1, 1), 0.55, 60, "paint.png"),
        "trim": ((1, 1, 1), 0.10, 10, "trim.png"),
        "tyre": ((1, 1, 1), 0.05, 5, "tyre.png"),
        "rim": ((1, 1, 1), 0.70, 80, "rim.png"),
        "glass": ((0.06, 0.09, 0.12), 0.90, 120, None),
        "headlight": ((0.92, 0.94, 0.98), 0.90, 120, None),
        "taillight": ((0.70, 0.05, 0.05), 0.80, 100, None),
    })
    write_obj(os.path.join(d, "hilux_tex.obj"), "hilux.mtl", v, f, g, uv, cn,
              '"2026 Toyota Hilux" by missyubibi attage, CC-BY-4.0; parts and textures by texture_r23.py')
    return {k: int((g == k).sum()) for k in dict.fromkeys(g)}


def quad(d):
    v, f = load_obj(os.path.join(d, "quad.obj"))
    fn = face_normals(v, f)
    c = v[f].mean(1)
    r, z = np.hypot(c[:, 0], c[:, 1]), c[:, 2]
    g = np.full(len(f), "prop", dtype=object)
    g[(r < 0.21) & (z >= 0.10)] = "carbon"
    g[(r < 0.21) & (z > 0.19)] = "payload"
    g[(r > 0.24) & (r < 0.32) & (z >= 0.10) & (z < 0.18)] = "motor"
    g[z < 0.10] = "gear"
    uv = box_uv(v, f, fn, 0.15)
    cn = corner_normals(v, f, fn)
    save_textures(d, {"carbon.png": carbon(), "motor.png": tile((170, 172, 174), grain=8, blotch=5)})
    write_mtl(os.path.join(d, "quad.mtl"), {
        "carbon": ((1, 1, 1), 0.50, 70, "carbon.png"),
        "payload": ((0.62, 0.63, 0.60), 0.30, 30, None),
        "motor": ((1, 1, 1), 0.70, 80, "motor.png"),
        "gear": ((0.05, 0.05, 0.05), 0.20, 20, None),
        "prop": ((0.12, 0.12, 0.13), 0.30, 30, None),
    })
    write_obj(os.path.join(d, "quad_tex.obj"), "quad.mtl", v, f, g, uv, cn, "R23 quad; parts by texture_r23.py")
    return {k: int((g == k).sum()) for k in dict.fromkeys(g)}


def hexa(d):
    v, f = load_obj(os.path.join(d, "hexa.obj"))
    fn = face_normals(v, f)
    z = v[f].mean(1)[:, 2]
    g = np.where(z < 0.12, "gear", "carbon").astype(object)
    uv = box_uv(v, f, fn, 0.2)
    cn = corner_normals(v, f, fn)
    save_textures(d, {"carbon.png": carbon()})
    write_mtl(os.path.join(d, "hexa.mtl"), {
        "carbon": ((1, 1, 1), 0.50, 70, "carbon.png"),
        "gear": ((0.06, 0.06, 0.06), 0.20, 20, None),
    })
    write_obj(os.path.join(d, "hexa_tex.obj"), "hexa.mtl", v, f, g, uv, cn, "R23 hexa; parts by texture_r23.py")
    return {k: int((g == k).sum()) for k in dict.fromkeys(g)}


def main():
    hd = os.path.join(MODELS, "r23_hilux", "meshes")
    print("hilux", hilux(hd))
    print("hexa", hexa(hd))
    print("quad", quad(os.path.join(MODELS, "r23_quad", "meshes")))
    md = os.path.join(MODELS, "r23_hilux", "materials", "textures")
    os.makedirs(md, exist_ok=True)
    for name, img in (("deck.png", deck()), ("pad_marks.png", pad_marks()), ("canopy.png", canopy()),
                      ("label.png", label()), ("rail.png", diamond_plate())):
        cv2.imwrite(os.path.join(md, name), img)
    for old in ("deck_plate.png", "landing_pad.png", "panel.png"):
        if os.path.exists(os.path.join(md, old)):
            os.remove(os.path.join(md, old))
    print("box textures in", md)


if __name__ == "__main__":
    main()
