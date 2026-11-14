#!/usr/bin/env python3
"""Textures for H1's Typhoon H480 (Gambit's Fuel model): its STL meshes, which carry no parts, materials or UVs,
as textured OBJ.

    python3 symphony/texture_h480.py [FUEL_MODEL_DIR] [OUT_MODEL_DIR]
        default: ~/.gz/fuel/fuel.gazebosim.org/gambit/models/typhoon h480/<newest>  ->  ardupilot_sim/models/h480_tex

make_world.py uses OUT_MODEL_DIR/meshes/<stl name>.obj in place of the STL when it exists (run.sh writes it on
the first start). The parts are found by where they are, in the model's frame (metres after the STL's 0.001
scale; Z up; the rotors on a 0.243 m radius), as texture_r23.py does for the R23:
  main body  shell (r < 0.15, facing up), belly (r < 0.15, the rest: battery bay, underside), arms
             (0.15..0.21), motor pods (r >= 0.21);
  legs       one part, the landing gear;
  propellers blades, and the hub (within 18 mm of the rotor axis).
Colours follow the Typhoon H480 itself: a graphite body in grained plastic, black satin arms, dark brushed-metal
motor pods, dark grey gear, black propellers with a metal hub. UVs are a box projection (each face along its
dominant axis); normals are smoothed across edges under 40 deg, so the shell reads as one moulding.
"""
import glob
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FUEL = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/gambit/models/typhoon h480")
RNG = np.random.default_rng(480)
ROTOR_R = 0.2427         # the six rotor axes' distance from the body centre


# ── textures (BGR) ───────────────────────────────────────────────────────────

def noise(size, scale, amp):
    small = RNG.normal(0, 1, (max(2, size // scale), max(2, size // scale))).astype(np.float32)
    return cv2.resize(small, (size, size), interpolation=cv2.INTER_CUBIC) * amp


def plastic(bgr, grain=7.0, blotch=4.0, size=512):
    """Moulded plastic: the fine sparkle of a textured mould over a slow unevenness."""
    img = np.empty((size, size, 3), np.float32)
    img[:] = bgr
    img += noise(size, 1, grain)[..., None] + noise(size, 48, blotch)[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


def brushed(bgr, size=512):
    """Brushed metal: streaks along one direction."""
    streak = cv2.resize(RNG.normal(0, 1, (size, 8)).astype(np.float32), (size, size), interpolation=cv2.INTER_LINEAR)
    img = np.empty((size, size, 3), np.float32)
    img[:] = bgr
    img += (streak * 14 + noise(size, 1, 4))[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


# ── mesh ─────────────────────────────────────────────────────────────────────

def load_stl(path, scale=0.001):
    data = open(path, "rb").read()
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                        count=n, offset=84)
    tri = rec["v"].astype(np.float64) * scale                  # (F, 3, 3)
    v, f = np.unique(np.round(tri.reshape(-1, 3), 7), axis=0, return_inverse=True)   # weld shared corners
    return v, f.reshape(-1, 3)


def face_normals(v, f):
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def corner_normals(v, f, fn, crease_deg=40.0):
    """Area-weighted vertex normals, except across a crease, where the corner keeps its face's normal."""
    area_n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v)
    for k in range(3):
        np.add.at(vn, f[:, k], area_n)
    vn /= np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12
    cn = vn[f]                                                   # (F, 3, 3)
    sharp = np.einsum("fkc,fc->fk", cn, fn) < np.cos(np.radians(crease_deg))
    cn[sharp] = np.repeat(fn[:, None, :], 3, axis=1)[sharp]
    return cn


def box_uv(v, f, fn, metres_per_tile):
    p = v[f] / metres_per_tile
    axis = np.abs(fn).argmax(1)
    uv = np.empty((len(f), 3, 2))
    for a, cols in ((0, [1, 2]), (1, [0, 2]), (2, [0, 1])):
        uv[axis == a] = p[axis == a][:, :, cols]
    return uv


def write_obj(path, mtl, v, f, groups, uv, cn, note):
    with open(path, "w") as fh:
        fh.write(f"# {note}\nmtllib {mtl}\n")
        fh.write("".join(f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in v))
        fh.write("".join(f"vt {a:.4f} {b:.4f}\n" for a, b in uv.reshape(-1, 2)))
        fh.write("".join(f"vn {a:.4f} {b:.4f} {c:.4f}\n" for a, b, c in cn.reshape(-1, 3)))
        for name in dict.fromkeys(groups):
            fh.write(f"g {name}\nusemtl {name}\n")
            idx = np.nonzero(groups == name)[0]
            fh.write("".join("f " + " ".join(f"{f[fi, k] + 1}/{3 * fi + k + 1}/{3 * fi + k + 1}" for k in range(3))
                             + "\n" for fi in idx))


def write_mtl(path, materials):
    with open(path, "w") as fh:
        for name, (kd, ks, ns, tex) in materials.items():
            fh.write(f"newmtl {name}\nKa {kd[0] * 0.6:.3f} {kd[1] * 0.6:.3f} {kd[2] * 0.6:.3f}\n"
                     f"Kd {kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}\nKs {ks:.3f} {ks:.3f} {ks:.3f}\nNs {ns}\nd 1.0\nillum 2\n"
                     + (f"map_Kd textures/{tex}\n" if tex else "") + "\n")


# ── parts ────────────────────────────────────────────────────────────────────

def body(src, out):
    v, f = load_stl(os.path.join(src, "main_body_remeshed_v3.stl"))
    fn = face_normals(v, f)
    c = v[f].mean(1)
    r = np.hypot(c[:, 0], c[:, 1])
    g = np.full(len(f), "belly", dtype=object)
    g[(r < 0.15) & (fn[:, 2] > 0.25) & (c[:, 2] > -0.02)] = "shell"
    g[(r >= 0.15) & (r < 0.21)] = "arm"
    g[r >= 0.21] = "motor"
    write_obj(os.path.join(out, "main_body_remeshed_v3.obj"), "main_body.mtl", v, f, g, box_uv(v, f, fn, 0.12),
              corner_normals(v, f, fn), "Typhoon H480 body; parts by texture_h480.py")
    write_mtl(os.path.join(out, "main_body.mtl"), {
        "shell": ((1, 1, 1), 0.35, 40, "shell.png"),
        "belly": ((1, 1, 1), 0.20, 20, "belly.png"),
        "arm": ((1, 1, 1), 0.40, 50, "arm.png"),
        "motor": ((1, 1, 1), 0.75, 90, "motor.png"),
    })
    return {k: int((g == k).sum()) for k in dict.fromkeys(g)}


def legs(src, out):
    parts = {}
    for name in ("leg1_remeshed_v3", "leg2_remeshed_v3"):
        v, f = load_stl(os.path.join(src, name + ".stl"))
        fn = face_normals(v, f)
        g = np.full(len(f), "gear", dtype=object)
        write_obj(os.path.join(out, name + ".obj"), "legs.mtl", v, f, g, box_uv(v, f, fn, 0.12),
                  corner_normals(v, f, fn), "Typhoon H480 landing gear; texture_h480.py")
        parts[name] = len(f)
    write_mtl(os.path.join(out, "legs.mtl"), {"gear": ((1, 1, 1), 0.25, 30, "gear.png")})
    return parts


def props(src, out):
    parts = {}
    for name in ("prop_cw_assembly_remeshed_v3", "prop_ccw_assembly_remeshed_v3"):
        v, f = load_stl(os.path.join(src, name + ".stl"))
        fn = face_normals(v, f)
        c = v[f].mean(1)
        # The STL sits where rotor 3 is (its visual pose takes it back to the rotor link): find that axis.
        axis = np.array([np.median(c[:, 0]), np.median(c[:, 1])])
        rr = np.hypot(c[:, 0] - axis[0], c[:, 1] - axis[1])
        g = np.where(rr < 0.018, "hub", "blade").astype(object)
        write_obj(os.path.join(out, name + ".obj"), "props.mtl", v, f, g, box_uv(v, f, fn, 0.08),
                  corner_normals(v, f, fn), "Typhoon H480 propeller; texture_h480.py")
        parts[name] = {k: int((g == k).sum()) for k in dict.fromkeys(g)}
    write_mtl(os.path.join(out, "props.mtl"), {
        "blade": ((0.07, 0.07, 0.075), 0.55, 80, None),
        "hub": ((1, 1, 1), 0.8, 90, "hub.png"),
    })
    return parts


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.join(FUEL, "[0-9]*")), key=lambda p: int(
        os.path.basename(p)))
    src = os.path.join(src, "meshes")
    model = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(HERE), "models", "h480_tex")
    out = os.path.join(model, "meshes")
    os.makedirs(os.path.join(out, "textures"), exist_ok=True)
    for name, img in (("shell.png", plastic((60, 58, 56), grain=7, blotch=5)),     # graphite
                      ("belly.png", plastic((40, 39, 38), grain=6, blotch=4)),
                      ("arm.png", plastic((24, 24, 25), grain=4, blotch=3)),       # black satin
                      ("motor.png", brushed((70, 70, 72))),
                      ("gear.png", plastic((50, 50, 52), grain=6, blotch=4)),
                      ("hub.png", brushed((150, 152, 156)))):
        cv2.imwrite(os.path.join(out, "textures", name), img)
    print("body", body(src, out))
    print("legs", legs(src, out))
    print("props", props(src, out))
    with open(os.path.join(model, "model.config"), "w") as fh:
        fh.write('<?xml version="1.0"?>\n<model>\n  <name>h480_tex</name>\n  <version>1.0</version>\n'
                 "  <description>Mesh only: Gambit's Typhoon H480 (Gazebo Fuel) as textured OBJ, by "
                 "symphony/texture_h480.py.</description>\n</model>\n")
    print("wrote", out)


if __name__ == "__main__":
    main()
