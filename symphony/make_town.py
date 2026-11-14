#!/usr/bin/env python3
"""A busy neighbourhood around the R23 for symphony/run.sh: buildings and parked/road vehicles from Gazebo Fuel
(OpenRobotics' city models), placed on the Baylands terrain. Called by make_world.py (SYM_TOWN=0 leaves it out).

    make_town.py [OUT_DIR]      standalone: prints the include block and writes the asset models to OUT_DIR/models

How things are placed (deterministic, SYM_TOWN_SEED):
  - the ground: the Baylands meshes (Fuel "Baylands", the world's models/baylands_park, 1.3 m down) read once
    into a 1 m grid over the whole park: heights, the land the ground mesh actually covers (the map's edge, less
    4 m: nothing is placed past it), water (only where the ground is below the water planes), streets (paths,
    roads, sidewalks), trees and the park's own structures; cached in ~/.cache/symphony-gimbal-sim/town/
    terrain_v2.npz (seconds, the first time);
  - rows along the streets (SYM_TOWN_ROW_BUILDINGS, default 170): shophouse fronts, shops, houses or civic
    buildings, one family a row, side by side 1 m apart, their fronts (local -y) on a line SETBACK (6 m) from the
    street's edge and following its curve, a car parked along the kerb in front of about half of them;
  - the open land between (SYM_TOWN_BUILDINGS, default 70): anything, with a car or two in front;
  - traffic on the park's roads and parking lots: at least 40, along the road (SYM_TOWN_VEHICLES, default 150, is the
    total), and two support vehicles by the R23;
  - every footprint on land, off water, streets, trees and structures, on a slope of at most 2 m (0.6 m for a
    vehicle), clear of everything placed before it (a 0.5 m occupancy grid), and nothing within 25 m of the R23
    (take-offs, landings) or taller than 12 m within 90 m (the survey flies at 25 m, RTL at 15 m).
Every asset is wrapped in its own static model: the Fuel visuals as they are (meshes by their Fuel URL, read from the
cache), textures the Fuel model gave only through an Ogre material script (which Harmonic ignores) re-applied as PBR
maps, and one box collision from the mesh's bounds (mesh collisions and the world's Bullet detector do not mix).
"""
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

FUEL = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models")
CACHE = os.path.expanduser(os.environ.get("SYM_DIR", "~/.cache/symphony-gimbal-sim"))
SEED = int(os.environ.get("SYM_TOWN_SEED", "23"))
N_ROW = int(os.environ.get("SYM_TOWN_ROW_BUILDINGS", "170"))     # buildings in rows along the streets
N_BUILDINGS = int(os.environ.get("SYM_TOWN_BUILDINGS", "70"))     # more on the open land between
N_VEHICLES = int(os.environ.get("SYM_TOWN_VEHICLES", "150"))
EXTENT = 430.0            # m: the grid covers the whole park (its land spans x -411..371, y -394..159)
CELL = 1.0
BAYLANDS_Z = -1.3         # models/baylands_park's pose
CLEAR_R = 25.0            # nothing within this of the R23
TOWN_R = float(os.environ.get("SYM_TOWN_RADIUS", "220"))   # buildings within this of the R23
LOW_R = 90.0              # only low buildings within this
LOW_H = 12.0
R23_CENTRE = (0.0, 2.7)   # world x, y of the vehicle's middle (its frame: y back from the front bumper at 0)

BUILDINGS = ["house 1", "house 2", "house 3", "fast food", "fire station", "gas station", "grocery store",
             "law office", "office building", "police station", "post office", "salon", "school", "thrift shop",
             "apartment", "water tower"]
VEHICLES = ["hatchback", "hatchback blue", "hatchback red", "suv", "pickup", "ambulance", "bus", "fire truck"]
VEHICLE_WEIGHTS = {"hatchback": 3, "hatchback blue": 3, "hatchback red": 3, "suv": 3, "pickup": 2, "ambulance": 1,
                   "bus": 1, "fire truck": 1}

GROUND = {"Object001", "DirtPath", "Asphalt", "Road", "Grass", "Sand", "ParkingLotMedian", "ParkingMarkings",
          "ConcreteGround_02", "ConcretePebbled", "Sidewalk", "Concrete", "Checker", "Curb", "Curb_Red",
          "ConcreteGround_01", "Tree_Concrete", "MurkyWater"}
ROADS = {"Asphalt", "Road", "ParkingMarkings"}
NO_BUILD = {"DirtPath", "Asphalt", "Road", "ParkingMarkings", "Sidewalk", "Curb", "Curb_Red"}   # paths and roads
STREETS = {"DirtPath", "Asphalt", "Road", "Sidewalk"}   # what the rows of buildings line
OBSTACLES = {"Branch", "Leaves", "Bark", "Metal_Roof", "RustyMetal", "Metal", "PicnicTable", "Rubber"}


# ── meshes ───────────────────────────────────────────────────────────────────

def _tag(e):
    return e.tag.rsplit("}", 1)[-1]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def read_dae(path, want=None, triangles=True):
    """{geometry name: (vertices (N,3) world-of-file metres, Z up; triangles (M,3) or None)}.
    want: names to keep (None: all). Streams the file, so 100 MB scenes fit."""
    geoms, unit, up = {}, 1.0, "Z_UP"
    nodes = []
    for ev, e in ET.iterparse(path, events=("end",)):
        t = _tag(e)
        if t == "unit":
            unit = float(e.get("meter", "1"))
        elif t == "up_axis":
            up = (e.text or "Z_UP").strip()
        elif t == "geometry":
            gid, name = e.get("id"), (e.get("name") or e.get("id") or "").replace("Mesh", "")
            if want is None or name in want:
                mesh = next((c for c in e if _tag(c) == "mesh"), None)
                if mesh is not None:
                    srcs = {s.get("id"): s for s in mesh if _tag(s) == "source"}
                    pos_id = None
                    for v in mesh:
                        if _tag(v) == "vertices":
                            for i in v:
                                if i.get("semantic") == "POSITION":
                                    pos_id = i.get("source", "#")[1:]
                    if pos_id in srcs:
                        fa = next(c for c in srcs[pos_id].iter() if _tag(c) == "float_array")
                        verts = np.fromstring(fa.text or "", sep=" ").reshape(-1, 3)
                        tris = []
                        if triangles:
                            for prim in mesh:
                                pt = _tag(prim)
                                if pt not in ("triangles", "polylist"):
                                    continue
                                ins = [i for i in prim if _tag(i) == "input"]
                                stride = max(int(i.get("offset", 0)) for i in ins) + 1
                                voff = next(int(i.get("offset", 0)) for i in ins if i.get("semantic") == "VERTEX")
                                pel = next((c for c in prim if _tag(c) == "p"), None)
                                if pel is None or not pel.text:
                                    continue
                                idx = np.fromstring(pel.text, sep=" ", dtype=np.int64).reshape(-1, stride)[:, voff]
                                if pt == "polylist":
                                    vc = next((c for c in prim if _tag(c) == "vcount"), None)
                                    counts = np.fromstring(vc.text, sep=" ", dtype=np.int64) if vc is not None else None
                                    if counts is not None and (counts != 3).any():
                                        out, k = [], 0
                                        for n in counts:
                                            out += [(idx[k], idx[k + j], idx[k + j + 1]) for j in range(1, n - 1)]
                                            k += n
                                        tris.append(np.array(out, np.int64))
                                        continue
                                tris.append(idx.reshape(-1, 3))
                        geoms[gid] = (name, verts, np.concatenate(tris) if tris else None)
            e.clear()
        elif t == "visual_scene":
            def walk(n, m):
                local = np.eye(4)
                for c in n:
                    ct = _tag(c)
                    if ct == "matrix":
                        local = local @ np.fromstring(c.text, sep=" ").reshape(4, 4)
                    elif ct == "translate":
                        tt = np.eye(4); tt[:3, 3] = np.fromstring(c.text, sep=" "); local = local @ tt
                    elif ct == "scale":
                        tt = np.eye(4); tt[[0, 1, 2], [0, 1, 2]] = np.fromstring(c.text, sep=" "); local = local @ tt
                    elif ct == "rotate":
                        ax = np.fromstring(c.text, sep=" "); a = math.radians(ax[3]); u = ax[:3] / (np.linalg.norm(ax[:3]) or 1)
                        K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
                        tt = np.eye(4); tt[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; local = local @ tt
                mm = m @ local
                for c in n:
                    if _tag(c) == "instance_geometry":
                        nodes.append((c.get("url", "#")[1:], mm))
                    elif _tag(c) == "node":
                        walk(c, mm)
            for n in e:
                if _tag(n) == "node":
                    walk(n, np.eye(4))
            e.clear()
    fix = np.eye(3) if up == "Z_UP" else (np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) if up == "Y_UP"
                                           else np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
    out = {}
    used = set()
    for gid, m in nodes:
        if gid not in geoms:
            continue
        name, v, tr = geoms[gid]
        w = (v @ m[:3, :3].T + m[:3, 3]) * unit @ fix.T
        used.add(gid)
        key = name if name not in out else f"{name}#{len(out)}"
        out[key] = (w, tr)
    for gid, (name, v, tr) in geoms.items():       # geometry no node instances: as is
        if gid not in used and nodes == []:
            out[name] = (v * unit @ fix.T, tr)
    return out


def read_obj_vertices(path):
    v = []
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                v.append(line.split()[1:4])
    return np.array(v, float)


# ── terrain ──────────────────────────────────────────────────────────────────

def _raster(grid, pts2, tris, value=1):
    """Fill triangles (world x, y) into a mask grid."""
    import cv2
    n = grid.shape[0]
    px = ((pts2 + EXTENT) / CELL).astype(np.float32)
    inside = ((px >= -50) & (px <= n + 50)).all(1)
    for t in tris:
        if not inside[t].all():
            continue
        cv2.fillConvexPoly(grid, np.round(px[t] * 4).astype(np.int32), value, shift=2)


def terrain():
    path = os.path.join(CACHE, "town", "terrain_v2.npz")
    if os.path.exists(path):
        d = np.load(path)
        return {k: d[k] for k in d.files}
    import cv2  # noqa: F401
    from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
    print("town: reading the Baylands terrain once (about a minute)…", file=sys.stderr)
    base = max((os.path.join(FUEL, "baylands", v) for v in os.listdir(os.path.join(FUEL, "baylands")) if v.isdigit()),
               key=lambda p: int(os.path.basename(p)))
    n = int(2 * EXTENT / CELL) + 1
    masks = {k: np.zeros((n, n), np.uint8) for k in ("water", "road", "nobuild", "obstacle", "land", "path")}
    water_z = np.full((n, n), -1e9, np.float32)
    gp = []
    for f in ("baylands_01.DAE", "baylands_02.DAE", "baylands_03.DAE"):
        # baylands_01 is the trees: their vertices mark obstacles, their triangles are not needed
        for name, (v, tr) in read_dae(os.path.join(base, "media", f), triangles=f != "baylands_01.DAE").items():
            nm = name.split("#")[0]
            v = v + [0, 0, BAYLANDS_Z]
            near = (np.abs(v[:, 0]) < EXTENT + 40) & (np.abs(v[:, 1]) < EXTENT + 40)
            if nm in GROUND and nm != "MurkyWater":
                gp.append(v[near])
            if tr is None:
                continue
            if nm == "MurkyWater":
                # Water planes reach under the land too: a cell is water only where the ground is below them.
                for t3 in tr:
                    wz = np.full((n, n), -1e9, np.float32)
                    _raster(wz, v[:, :2], t3[None], value=float(v[t3, 2].mean()))
                    water_z[:] = np.maximum(water_z, wz)
            if nm in GROUND and nm != "MurkyWater":
                _raster(masks["land"], v[:, :2], tr)          # where the park's ground mesh actually is
            if nm in ROADS:
                _raster(masks["road"], v[:, :2], tr)
            if nm in STREETS:
                _raster(masks["path"], v[:, :2], tr)
            if nm in NO_BUILD:
                _raster(masks["nobuild"], v[:, :2], tr)
            if nm in OBSTACLES:
                c = ((v[near][:, :2] + EXTENT) / CELL).astype(int)
                c = c[((c >= 0) & (c < n)).all(1)]
                masks["obstacle"][c[:, 1], c[:, 0]] = 1
    gp = np.concatenate(gp)
    # one point per 0.5 m cell (the highest) keeps the triangulation small and the ground on top
    key = np.floor(gp[:, :2] * 2).astype(np.int64)
    order = np.lexsort((gp[:, 2], key[:, 1], key[:, 0]))
    gp, key = gp[order], key[order]
    last = np.r_[(np.diff(key, axis=0) != 0).any(1), True]
    gp = gp[last]
    xs = np.linspace(-EXTENT, EXTENT, n)
    gx, gy = np.meshgrid(xs, xs)
    h = LinearNDInterpolator(gp[:, :2], gp[:, 2])(gx, gy)
    holes = np.isnan(h)
    if holes.any():
        h[holes] = NearestNDInterpolator(gp[:, :2], gp[:, 2])(gx[holes], gy[holes])
    import cv2
    masks["obstacle"] = cv2.dilate(masks["obstacle"], np.ones((3, 3), np.uint8))
    masks["water"] = (h <= water_z + 0.05).astype(np.uint8)
    # The edge of the map: the ground mesh's own extent, less 4 m, so nothing hangs over the bay.
    masks["land"] = cv2.erode(cv2.morphologyEx(masks["land"], cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)),
                              np.ones((9, 9), np.uint8))
    out = {"height": h.astype(np.float32), **masks}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, **out)
    print(f"town: terrain cached in {path}", file=sys.stderr)
    return out


# ── assets ───────────────────────────────────────────────────────────────────

def _slug(name):
    return "town_" + re.sub(r"\W+", "_", name).strip("_")


def _fuel_dir(name):
    root = os.path.join(FUEL, name)
    v = max((d for d in os.listdir(root) if d.isdigit()), key=int)
    return os.path.join(root, v), v


def asset(name, out_models):
    """Write the wrapper model for one Fuel asset; return its slug, box (centre, size) in its own frame."""
    d, ver = _fuel_dir(name)
    root = ET.parse(os.path.join(d, "model.sdf")).getroot()
    model = root.find("model")
    url = f"https://fuel.gazebosim.org/1.0/openrobotics/models/{name}/{ver}/files/"
    slug = _slug(name)
    mdir = os.path.join(out_models, slug)
    os.makedirs(os.path.join(mdir, "textures"), exist_ok=True)
    texdir = os.path.join(d, "materials", "textures")
    diffuse = sorted(f for f in os.listdir(texdir) if re.search(r"diffuse", f, re.I)) if os.path.isdir(texdir) else []
    normal = sorted(f for f in os.listdir(texdir) if re.search(r"normal", f, re.I)) if os.path.isdir(texdir) else []
    lo, hi, visuals = np.full(3, np.inf), np.full(3, -np.inf), []
    mpose = np.fromstring((model.findtext("pose") or "0 0 0 0 0 0"), sep=" ")
    for link in model.findall("link"):
        lpose = np.fromstring((link.findtext("pose") or "0 0 0 0 0 0"), sep=" ")
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.findtext("uri").strip()
            rel = re.sub(r"^https://fuel\.gazebosim\.org/1\.0/openrobotics/models/[^/]+/\w+/files/", "", uri, flags=re.I)
            rel = re.sub(r"^model://[^/]+/", "", rel)
            local = os.path.join(d, rel)
            scale = np.fromstring(mesh.findtext("scale") or "1 1 1", sep=" ")
            vpose = np.fromstring((vis.findtext("pose") or "0 0 0 0 0 0"), sep=" ")
            if local.lower().endswith(".dae"):
                v = np.concatenate([w for w, _ in read_dae(local, triangles=False).values()])
            else:
                v = read_obj_vertices(local)
            v = v * scale
            for p in (vpose, lpose, mpose):
                v = v @ _rpy(*p[3:6]).T + p[:3]
            lo, hi = np.minimum(lo, v.min(0)), np.maximum(hi, v.max(0))
            mat = vis.find("material")
            mat_xml = ""
            if mat is not None and mat.find("script") is not None and diffuse:
                for f in diffuse + normal[:1]:
                    shutil.copyfile(os.path.join(texdir, f), os.path.join(mdir, "textures", f))
                mat_xml = (f"<material><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular><pbr><metal>"
                           f"<albedo_map>model://{slug}/textures/{diffuse[0]}</albedo_map>"
                           + (f"<normal_map>model://{slug}/textures/{normal[0]}</normal_map>" if normal else "")
                           + "<metalness>0.0</metalness><roughness>0.8</roughness></metal></pbr></material>")
            pose = " ".join(f"{x:.6g}" for x in vpose)
            visuals.append(f'<visual name="v{len(visuals)}"><pose>{pose}</pose><geometry><mesh><uri>{url}{rel}</uri>'
                           f'<scale>{" ".join(f"{x:.6g}" for x in scale)}</scale></mesh></geometry>{mat_xml}</visual>')
    centre, size = (lo + hi) / 2, np.maximum(hi - lo, 0.05)
    lp = " ".join(f"{x:.6g}" for x in mpose)        # the Fuel model's own pose, on the wrapper's link
    sdf = (f'<?xml version="1.0"?>\n<!-- {name} (Gazebo Fuel, OpenRobotics) wrapped by symphony/make_town.py: static, '
           f'box collision from the mesh bounds -->\n<sdf version="1.9"><model name="{slug}"><static>true</static>'
           f'<link name="link"><pose>{lp}</pose>'
           + "".join(visuals)
           + f'<collision name="box"><pose>{" ".join(f"{x:.4f}" for x in (centre - mpose[:3]))} 0 0 0</pose>'
           f'<geometry><box><size>{" ".join(f"{x:.4f}" for x in size)}</size></box></geometry></collision>'
           "</link></model></sdf>\n")
    with open(os.path.join(mdir, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(mdir, "model.config"), "w") as f:
        f.write(f'<?xml version="1.0"?><model><name>{slug}</name><version>1.0</version><sdf version="1.9">model.sdf'
                f"</sdf><description>{name} from Gazebo Fuel (OpenRobotics), wrapped by make_town.py</description></model>\n")
    return slug, centre, size


# ── layout ───────────────────────────────────────────────────────────────────

def _cells(x, y):
    n = int(2 * EXTENT / CELL) + 1
    i = np.clip(np.round((np.asarray(y) + EXTENT) / CELL).astype(int), 0, n - 1)
    j = np.clip(np.round((np.asarray(x) + EXTENT) / CELL).astype(int), 0, n - 1)
    return i, j


def _footprint(x, y, yaw, centre, size, margin, step=1.0):
    """Sample points over the rotated footprint (world x, y)."""
    hx, hy = size[0] / 2 + margin, size[1] / 2 + margin
    u, v = np.meshgrid(np.arange(-hx, hx + 1e-6, step), np.arange(-hy, hy + 1e-6, step))
    u, v = u.ravel() + centre[0], v.ravel() + centre[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * u - s * v, y + s * u + c * v


ROW_FAMILIES = [                     # what a row along a street is made of (one family a row, mixed within it)
    ("shophouses", ["salon", "thrift shop", "law office"]),           # three-storey downtown fronts, 6-7.5 m wide
    ("shops", ["post office", "salon", "thrift shop", "fast food", "grocery store"]),
    ("houses", ["house 1", "house 2", "house 3"]),
    ("civic", ["police station", "fire station", "post office", "school"]),
]
SETBACK = 6.0                        # m from the street's edge to a row's front: room for a parked car
ROW_GAP = 1.0                        # m between neighbours in a row
OCC = 0.5                            # m, occupancy grid


def layout(T, assets, rng):
    import cv2
    h, water, road, nobuild, obst, land, street = (T[k] for k in ("height", "water", "road", "nobuild", "obstacle",
                                                                    "land", "path"))
    n_occ = int(2 * EXTENT / OCC) + 1
    occ = np.zeros((n_occ, n_occ), np.uint8)
    dist = cv2.distanceTransform((street == 0).astype(np.uint8), cv2.DIST_L2, 5)      # m from the nearest street
    gy, gx = np.gradient(cv2.GaussianBlur(dist, (0, 0), 2.0))
    out = []
    counts = {"rows": 0, "row_buildings": 0, "scattered": 0, "parked": 0, "road": 0}

    def corners(x, y, yaw, c, s, margin):
        cs, sn = math.cos(yaw), math.sin(yaw)
        return [(x + cs * (c[0] + u * (s[0] / 2 + margin)) - sn * (c[1] + v * (s[1] / 2 + margin)),
                 y + sn * (c[0] + u * (s[0] / 2 + margin)) + cs * (c[1] + v * (s[1] / 2 + margin)))
                for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    def occ_free(x, y, yaw, c, s, margin):
        fx, fy = _footprint(x, y, yaw, c, s, margin, step=OCC)
        i = np.clip(np.round((fy + EXTENT) / OCC).astype(int), 0, n_occ - 1)
        j = np.clip(np.round((fx + EXTENT) / OCC).astype(int), 0, n_occ - 1)
        return not occ[i, j].any()

    def occ_mark(x, y, yaw, c, s, margin):
        p = (np.array(corners(x, y, yaw, c, s, margin)) + EXTENT) / OCC
        cv2.fillConvexPoly(occ, np.round(p * 4).astype(np.int32), 1, shift=2)

    def site(x, y, yaw, c, s, margin, kind):
        """None if the footprint is not buildable; else the ground under it."""
        if math.hypot(x - R23_CENTRE[0], y - R23_CENTRE[1]) < CLEAR_R + (0 if kind == "vehicle" else math.hypot(*s[:2]) / 2):
            return None
        if kind != "vehicle" and s[2] > LOW_H and math.hypot(x - R23_CENTRE[0], y - R23_CENTRE[1]) < LOW_R:
            return None
        fx, fy = _footprint(x, y, yaw, c, s, margin, step=0.5 if kind == "vehicle" else 1.0)
        i, j = _cells(fx, fy)
        if not land[i, j].all() or water[i, j].any() or obst[i, j].any():
            return None
        if kind == "building" and (street[i, j].any() or nobuild[i, j].any() or road[i, j].any()):
            return None
        hh = h[i, j]
        if hh.max() - hh.min() > (0.6 if kind == "vehicle" else 2.0):
            return None
        if not occ_free(x, y, yaw, c, s, margin):
            return None
        return hh

    def put_building(slug, x, y, yaw, c, s, hh):
        out.append((slug, x, y, float(hh.min()) - 0.05 - (c[2] - s[2] / 2), yaw))
        occ_mark(x, y, yaw, c, s, 0.3)

    def put_vehicle(slug, x, y, yaw, c, s, hh):
        out.append((slug, x, y, float(np.median(hh)) - 0.02 - (c[2] - s[2] / 2), yaw))
        occ_mark(x, y, yaw, c, s, 0.3)

    cars = [v for v in ("hatchback", "hatchback blue", "hatchback red", "suv", "pickup") if v in assets]

    # support vehicles by the R23 (inside its clear zone on purpose)
    for name, (x, y, yaw) in (("pickup", (9.0, -6.0, 1.57)), ("ambulance", (-10.0, -4.0, 1.75))):
        if name in assets:
            slug, c, s = assets[name]
            fx, fy = _footprint(x, y, yaw, c, s, 0.0)
            hh = h[_cells(fx, fy)]
            put_vehicle(slug, x, y, yaw, c, s, hh)

    def street_frame(px, py):
        """At a point near a street: the unit normal away from it and the tangent along it."""
        i, j = _cells(px, py)
        nx, ny = gx[i, j], gy[i, j]
        L = math.hypot(nx, ny)
        if L < 1e-3:
            return None
        nx, ny = nx / L, ny / L
        return (nx, ny), (-ny, nx)

    # 1. Rows along the streets: seeds on the line SETBACK from a street, then walk both ways along it.
    band = np.argwhere((dist >= SETBACK - 0.5) & (dist <= SETBACK + 0.5) & (land > 0) & (water == 0))
    rng.shuffle(band)
    for bi, bj in band:
        if counts["row_buildings"] >= N_ROW:
            break
        px, py = bj * CELL - EXTENT, bi * CELL - EXTENT
        if math.hypot(px - R23_CENTRE[0], py - R23_CENTRE[1]) < CLEAR_R + 10:
            continue
        fam = ROW_FAMILIES[rng.integers(len(ROW_FAMILIES))][1]
        fam = [f for f in fam if f in assets]
        if not fam:
            continue
        placed_row = 0
        first_w = None
        for direction in (1, -1):
            qx, qy = px, py                                  # back to the first building's front, for -1
            prev_w = None if direction == 1 else first_w
            if direction == -1 and first_w is None:
                break
            for k in range(14):
                fr = street_frame(qx, qy)
                if fr is None:
                    break
                (nx, ny), (tx, ty) = fr
                name = fam[rng.integers(len(fam))]
                slug, c, s = assets[name]
                # the front (local -y) faces the street: local +y along the normal away from it
                yaw = math.atan2(ny, nx) - math.pi / 2
                if prev_w is not None:
                    step = prev_w / 2 + ROW_GAP + s[0] / 2
                    qx, qy = qx + direction * tx * step, qy + direction * ty * step
                    i, j = _cells(qx, qy)
                    if not (SETBACK - 2.5 <= dist[i, j] <= SETBACK + 2.5):      # the street turned away
                        break
                    fr = street_frame(qx, qy)
                    if fr is None:
                        break
                    (nx, ny), (tx, ty) = fr
                    yaw = math.atan2(ny, nx) - math.pi / 2
                    # back onto the setback line
                    i, j = _cells(qx, qy)
                    adj = SETBACK - dist[i, j]
                    qx, qy = qx + nx * adj, qy + ny * adj
                # centre: the front edge on the setback line
                cx = qx + nx * (s[1] / 2) - (math.cos(yaw) * c[0] - math.sin(yaw) * c[1])
                cy = qy + ny * (s[1] / 2) - (math.sin(yaw) * c[0] + math.cos(yaw) * c[1])
                hh = site(cx, cy, yaw, c, s, 0.3, "building")
                if hh is None:
                    if prev_w is None:
                        break
                    prev_w = s[0]
                    continue
                put_building(slug, cx, cy, yaw, c, s, hh)
                counts["row_buildings"] += 1
                placed_row += 1
                prev_w = s[0]
                if first_w is None:
                    first_w = s[0]
                # a car parked along the kerb in front, half the time
                if cars and rng.uniform() < 0.55 and counts["parked"] < N_VEHICLES * 0.6:
                    vn = cars[rng.integers(len(cars))]
                    vs, vc, vsz = assets[vn]
                    vx, vy = qx - nx * (SETBACK * 0.45), qy - ny * (SETBACK * 0.45)
                    vyaw = math.atan2(ty, tx) + (0 if vsz[0] >= vsz[1] else math.pi / 2) + math.pi * rng.integers(2)
                    vh = site(vx, vy, vyaw, vc, vsz, 0.2, "vehicle")
                    if vh is not None:
                        put_vehicle(vs, vx, vy, vyaw, vc, vsz, vh)
                        counts["parked"] += 1
            if direction == 1 and placed_row == 0:
                break
        if placed_row:
            counts["rows"] += 1

    # 2. The open land left over: anything, denser near the R23.
    bnames = [b for b in BUILDINGS if b in assets]
    tries = 0
    while counts["scattered"] < N_BUILDINGS and tries < 60000:
        tries += 1
        name = bnames[rng.integers(len(bnames))]
        slug, c, s = assets[name]
        x, y = rng.uniform(-EXTENT + 10, EXTENT - 10), rng.uniform(-EXTENT + 10, EXTENT - 10)
        yaw = rng.integers(4) * math.pi / 2 + rng.normal(0, 0.05)
        hh = site(x, y, yaw, c, s, 3.0, "building")
        if hh is None:
            continue
        put_building(slug, x, y, yaw, c, s, hh)
        counts["scattered"] += 1
        # a couple of cars in its forecourt
        for k in range(rng.integers(0, 3)):
            vn = cars[rng.integers(len(cars))]
            vs, vc, vsz = assets[vn]
            side = rng.integers(4)
            nyaw = yaw + side * math.pi / 2
            half = (s[0] if side % 2 == 0 else s[1]) / 2
            dd = half + 2.0 + max(vsz[:2]) / 2
            off = (k - 0.5) * 3.2
            cx = x + c[0] * math.cos(yaw) - c[1] * math.sin(yaw)
            cy = y + c[0] * math.sin(yaw) + c[1] * math.cos(yaw)
            vx, vy = cx + dd * math.cos(nyaw) - off * math.sin(nyaw), cy + dd * math.sin(nyaw) + off * math.cos(nyaw)
            vyaw = nyaw + (0 if vsz[0] >= vsz[1] else math.pi / 2) + math.pi * rng.integers(2)
            vh = site(vx, vy, vyaw, vc, vsz, 0.2, "vehicle")
            if vh is not None:
                put_vehicle(vs, vx, vy, vyaw, vc, vsz, vh)
                counts["parked"] += 1

    # 3. Traffic on the park's roads and parking lots, along the road.
    vnames = [nm for nm in VEHICLES if nm in assets for _ in range(VEHICLE_WEIGHTS[nm])]
    ri, rj = np.nonzero(road & land)
    rx, ry = rj * CELL - EXTENT, ri * CELL - EXTENT
    tries = 0
    while counts["road"] < max(40, N_VEHICLES - counts["parked"]) and tries < 30000 and len(rx):
        tries += 1
        k = rng.integers(len(rx))
        x, y = rx[k] + rng.uniform(-0.4, 0.4), ry[k] + rng.uniform(-0.4, 0.4)
        vn = vnames[rng.integers(len(vnames))]
        slug, c, s = assets[vn]
        best = None
        for yaw in np.arange(0, math.pi, math.pi / 8):
            fx, fy = _footprint(x, y, yaw, c, s, 0.0, step=0.5)
            score = road[_cells(fx, fy)].mean()
            if best is None or score > best[0]:
                best = (score, yaw)
        if best[0] < 0.9:
            continue
        yaw = best[1] + math.pi * rng.integers(2)
        hh = site(x, y, yaw, c, s, 0.4, "vehicle")
        if hh is None:
            continue
        put_vehicle(slug, x, y, yaw, c, s, hh)
        counts["road"] += 1
    return out, counts


def build(out_dir):
    """Write the asset models into out_dir/models; return the world's include block."""
    T = terrain()
    models = os.path.join(out_dir, "models")
    assets = {}
    for name in BUILDINGS + VEHICLES:
        try:
            slug, c, s = asset(name, models)
            assets[name] = (slug, c, s)
        except (OSError, ValueError, StopIteration) as exc:
            print(f"town: {name} skipped ({exc.__class__.__name__}: {exc})", file=sys.stderr)
    rng = np.random.default_rng(SEED)
    items, k = layout(T, assets, rng)
    print(f"town: {k['row_buildings']} buildings in {k['rows']} rows along the streets, {k['scattered']} on open land, "
          f"{k['parked']} parked and {k['road']} road vehicles (+2 by the R23), seed {SEED}", file=sys.stderr)
    return "".join(f"""    <include>
      <uri>model://{slug}</uri>
      <name>{slug}_{k}</name>
      <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 {yaw:.4f}</pose>
    </include>
""" for k, (slug, x, y, z, yaw) in enumerate(items))


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(CACHE, "town", "preview")
    print(build(out))
