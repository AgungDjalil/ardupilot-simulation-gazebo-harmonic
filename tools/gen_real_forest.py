#!/usr/bin/env python3
"""Generate a natural-looking forest world for GPS-denied flight with RTAB-Map.

The textured pillar world (gen_textured_world.py) repeats itself: every pillar has the same
bark, the ground repeats every few tiles and the perimeter walls repeat every 4 m, which made
RTAB-Map take one place for another (wrong loop closures that fold survey passes together).
Here nothing repeats:
  - trees with random trunk radius, height and lean, each with its own bark texture
    (pine / oak / birch styles), and leafy crowns well above the flight altitude
  - bushes, rocks and fallen logs low enough to fly over at 1 m
  - a forest floor of 8 x 8 m tiles, each with its own leaf litter, grass and twigs
  - a dense belt of trees around the flight area instead of walls
Writes, in the package:
  models/ego_forest_real_textures/   procedural textures (JPEG)
  worlds/ego_forest_real.sdf         ros2 launch ego_ardupilot_sim ... world:=ego_forest_real.sdf
Run from anywhere:  python3 tools/gen_real_forest.py
Only trunks, rocks and logs have collisions (crowns and bushes are visual only).
"""
import math
import os
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from scipy.ndimage import zoom

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_NAME = 'ego_forest_real_textures'
TEX_MODEL = os.path.join(PKG, 'models', TEX_NAME)
TEX_DIR = os.path.join(TEX_MODEL, 'materials', 'textures')
PLAIN_WORLD = os.path.join(PKG, 'worlds', 'ego_forest_plain.sdf')
OUT_WORLD = os.path.join(PKG, 'worlds', 'ego_forest_real.sdf')

# Flight area of the survey (x 0..26, y -8..8) plus a margin; the tree belt surrounds it.
INNER = (-3.0, 29.0, -10.5, 10.5)
OUTER = (-10.0, 36.0, -17.0, 17.0)
# Kept free: the spawn point and the survey waypoints (survey.py defaults).
CLEAR = [(0.0, 0.0, 2.5)] + [(x, y, 1.6) for x in (0.0, 26.0) for y in (-8.0, -4.0, 0.0, 4.0, 8.0)]
TILE = 8.0
rng = np.random.default_rng(7)


# ------------------------------------------------------------------ textures
def smooth_noise(h, w, cells, r=None, aspect=1.0):
    """Smooth value noise in [0, 1] with about `cells` features across the width."""
    r = r or rng
    gw = max(2, int(cells))
    gh = max(2, int(cells * h / w / aspect))
    g = r.random((gh + 3, gw + 3))
    out = zoom(g, ((h + 3 * h / gh) / g.shape[0], (w + 3 * w / gw) / g.shape[1]), order=3)
    out = out[:h, :w]
    out -= out.min()
    return out / max(out.max(), 1e-6)


def fractal(h, w, base_cells, octaves=5, aspect=1.0, r=None):
    out = np.zeros((h, w))
    for o in range(octaves):
        out += smooth_noise(h, w, base_cells * 2 ** o, r, aspect) / 2 ** o
    out -= out.min()
    return out / max(out.max(), 1e-6)


def to_img(arr):
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def leaf(d, x, y, length, angle, colour):
    """A leaf-shaped polygon (pointed ellipse)."""
    c, s = math.cos(angle), math.sin(angle)
    pts = []
    for t in np.linspace(0, 2 * math.pi, 10, endpoint=False):
        lx, ly = length / 2 * math.cos(t), length / 5 * math.sin(t) * abs(math.cos(t / 2)) ** 0.3
        pts.append((x + lx * c - ly * s, y + lx * s + ly * c))
    d.polygon(pts, fill=colour)


def forest_floor(path, size=1024):
    r = np.random.default_rng(rng.integers(1 << 30))
    n = fractal(size, size, 3, 6, r=r)
    soil = np.stack([95 + 55 * n, 75 + 45 * n, 50 + 30 * n], axis=-1)
    # Grass/moss patches with soft edges.
    grass = np.clip((fractal(size, size, 2, 4, r=r) - 0.55) * 4, 0, 1)[..., None]
    green = np.stack([70 + 40 * n, 100 + 50 * n, 40 + 20 * n], axis=-1)
    base = soil * (1 - grass) + green * grass + r.normal(0, 12, (size, size, 1))
    img = to_img(base)
    d = ImageDraw.Draw(img)
    litter = [(150, 90, 40), (120, 70, 30), (180, 130, 60), (90, 60, 35), (160, 110, 50),
              (110, 100, 50), (70, 50, 30), (190, 160, 90)]
    density = fractal(64, 64, 2, 3, r=r)
    for _ in range(int(r.integers(4000, 9000))):
        x, y = r.integers(0, size, 2)
        if r.random() > density[y * 64 // size, x * 64 // size] * 1.3:
            continue
        c = np.array(litter[r.integers(len(litter))]) + r.integers(-20, 20, 3)
        leaf(d, x, y, r.uniform(8, 26), r.uniform(0, math.pi), tuple(int(v) for v in c))
    for _ in range(int(r.integers(40, 120))):  # twigs
        x, y = r.integers(0, size, 2)
        a, L = r.uniform(0, math.pi), r.uniform(20, 120)
        d.line([(x, y), (x + L * math.cos(a), y + L * math.sin(a))],
               fill=(int(r.integers(50, 90)), int(r.integers(35, 60)), 25), width=int(r.integers(2, 5)))
    for _ in range(int(r.integers(30, 150))):  # pebbles
        x, y, rr = *r.integers(0, size, 2), r.integers(3, 10)
        g = int(r.integers(110, 190))
        d.ellipse([x - rr, y - rr, x + rr, y + rr * 0.7], fill=(g, g - 5, g - 15))
    img.filter(ImageFilter.SMOOTH).save(path, quality=88)


def bark(path, kind, w=256, h=1024):
    r = np.random.default_rng(rng.integers(1 << 30))
    fur = fractal(h, w, 6, 4, aspect=8.0, r=r)       # long vertical furrows
    n = fractal(h, w, 4, 5, r=r)
    if kind == 'birch':
        base = np.stack([215 + 25 * n, 210 + 25 * n, 195 + 25 * n], axis=-1) - 25 * fur[..., None]
    elif kind == 'pine':
        plates = np.clip((fractal(h, w, 5, 3, aspect=2.0, r=r) - 0.45) * 5, 0, 1)
        base = np.stack([110 + 70 * plates + 20 * n, 60 + 40 * plates + 15 * n, 40 + 20 * plates], axis=-1)
        base -= 50 * (fur[..., None] < 0.3)
    else:  # oak
        base = np.stack([85 + 50 * n, 75 + 45 * n, 62 + 35 * n], axis=-1) * (0.45 + 0.75 * fur[..., None])
    tint = r.uniform(0.85, 1.15, 3)
    base = base * tint + r.normal(0, 10, (h, w, 1))
    # Moss, thicker near the base.
    moss = np.clip((fractal(h, w, 3, 4, r=r) - 0.5) * 3, 0, 1) * np.linspace(0.2, 1.0, h)[:, None] ** 2
    base = base * (1 - moss[..., None]) + np.array([70, 105, 40]) * moss[..., None]
    img = to_img(base)
    d = ImageDraw.Draw(img)
    for _ in range(int(r.integers(4, 14))):  # knots / branch scars
        x, y = r.integers(10, w - 10), r.integers(10, h - 10)
        a, b = r.integers(6, 18), r.integers(8, 26)
        d.ellipse([x - a, y - b, x + a, y + b], fill=(40, 30, 22))
        d.ellipse([x - a // 2, y - b // 2, x + a // 2, y + b // 2], fill=(90, 70, 50))
    if kind == 'birch':
        for _ in range(int(r.integers(60, 140))):  # dark lenticels
            x, y = r.integers(0, w), r.integers(0, h)
            L = r.integers(8, 40)
            d.rectangle([x, y, x + L, y + r.integers(2, 6)], fill=(30, 30, 30))
    img.save(path, quality=88)


def foliage(path, base_colour, size=512):
    r = np.random.default_rng(rng.integers(1 << 30))
    n = fractal(size, size, 4, 5, r=r)
    img = to_img(np.array(base_colour) * (0.55 + 0.6 * n[..., None]))
    d = ImageDraw.Draw(img)
    for _ in range(5000):
        x, y = r.integers(0, size, 2)
        c = np.array(base_colour) * r.uniform(0.5, 1.4) + r.integers(-15, 15, 3)
        leaf(d, x, y, r.uniform(6, 16), r.uniform(0, math.pi), tuple(int(v) for v in np.clip(c, 0, 255)))
    img.save(path, quality=88)


def rock(path, size=256):
    r = np.random.default_rng(rng.integers(1 << 30))
    n = fractal(size, size, 3, 6, r=r)
    g = r.uniform(100, 150)
    base = np.stack([g + 70 * n, g + 65 * n, g + 55 * n], axis=-1)
    lichen = np.clip((fractal(size, size, 4, 3, r=r) - 0.6) * 4, 0, 1)[..., None]
    base = base * (1 - lichen) + np.array([150, 160, 90]) * lichen
    to_img(base + r.normal(0, 10, (size, size, 1))).save(path, quality=88)


# --------------------------------------------------------------- SDF helpers
def material(texture):
    return (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>'
            f'<albedo_map>model://{TEX_NAME}/materials/textures/{texture}</albedo_map>'
            '<metalness>0</metalness><roughness>0.95</roughness></metal></pbr></material>')


def fmt(*v):
    return ' '.join(f'{x:.4f}' for x in v)


def rot_rp(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


def poisson(n_try, box, min_dist, existing, clear, existing_dist=None):
    """Random points in box, at least min_dist from each other and existing_dist (default
    min_dist) from the `existing` points."""
    x0, x1, y0, y1 = box
    ed = min_dist if existing_dist is None else existing_dist
    pts = []
    for _ in range(n_try):
        p = np.array([rng.uniform(x0, x1), rng.uniform(y0, y1)])
        if any(np.hypot(*(p - q[:2])) < ed for q in existing) or \
                any(np.hypot(*(p - q[:2])) < min_dist for q in pts):
            continue
        if any(math.hypot(p[0] - cx, p[1] - cy) < cr for cx, cy, cr in clear):
            continue
        pts.append((p[0], p[1], min_dist))
    return pts


def tree_model(name, x, y, radius, height, lean, lean_dir, bark_tex, leaf_texs):
    roll, pitch = lean * math.cos(lean_dir), lean * math.sin(lean_dir)
    R = rot_rp(roll, pitch)
    centre = R @ np.array([0, 0, height / 2])
    top = R @ np.array([0, 0, height])
    cyl = f'<cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder>'
    parts = [f'<collision name="trunk"><pose>{fmt(*centre, roll, pitch, 0)}</pose>'
             f'<geometry>{cyl}</geometry></collision>',
             f'<visual name="trunk"><pose>{fmt(*centre, roll, pitch, 0)}</pose>'
             f'<geometry>{cyl}</geometry>{material(bark_tex)}</visual>']
    # Crown: a few overlapping ellipsoids around the upper trunk, bottom at >= 3.2 m.
    for k in range(int(rng.integers(2, 5))):
        a, b = rng.uniform(1.0, 2.2), rng.uniform(1.0, 2.2)
        c = rng.uniform(1.0, 2.0)
        z = max(top[2] - rng.uniform(0.5, 2.5), 3.2 + c)
        px, py = top[0] + rng.uniform(-0.8, 0.8), top[1] + rng.uniform(-0.8, 0.8)
        parts.append(f'<visual name="crown_{k}"><pose>{fmt(px, py, z, 0, 0, rng.uniform(0, math.pi))}</pose>'
                     f'<geometry><ellipsoid><radii>{fmt(a, b, c)}</radii></ellipsoid></geometry>'
                     f'{material(leaf_texs[rng.integers(len(leaf_texs))])}</visual>')
    return (f'<model name="{name}"><static>true</static><pose>{fmt(x, y, 0, 0, 0, 0)}</pose>'
            f'<link name="link">{"".join(parts)}</link></model>')


def main():
    if os.path.isdir(TEX_DIR):
        shutil.rmtree(TEX_DIR)
    os.makedirs(TEX_DIR)
    with open(os.path.join(TEX_MODEL, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<model><name>{TEX_NAME}</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Procedural textures for '
                'ego_forest_real.sdf (generated by tools/gen_real_forest.py)</description></model>\n')
    with open(os.path.join(TEX_MODEL, 'model.sdf'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<sdf version="1.9"><model name="{TEX_NAME}">'
                '<static>true</static><link name="link"/></model></sdf>\n')

    leaf_texs = []
    for k, c in enumerate([(60, 110, 40), (45, 90, 35), (90, 130, 50), (70, 100, 30), (100, 120, 45)]):
        leaf_texs.append(f'leaves_{k}.jpg')
        foliage(os.path.join(TEX_DIR, leaf_texs[-1]), c)
    bush_texs = []
    for k, c in enumerate([(50, 95, 35), (75, 110, 40), (40, 80, 45)]):
        bush_texs.append(f'bush_{k}.jpg')
        foliage(os.path.join(TEX_DIR, bush_texs[-1]), c)
    rock_texs = []
    for k in range(4):
        rock_texs.append(f'rock_{k}.jpg')
        rock(os.path.join(TEX_DIR, rock_texs[-1]))
    belt_bark = []  # the belt is seen from afar only: a few shared barks
    for k, kind in enumerate(['pine', 'oak', 'birch', 'pine', 'oak', 'pine']):
        belt_bark.append(f'bark_belt_{k}.jpg')
        bark(os.path.join(TEX_DIR, belt_bark[-1]), kind, 128, 512)

    tree = ET.parse(PLAIN_WORLD)
    world = tree.getroot().find('world')
    world.set('name', 'ego_forest_real')
    for m in [m for m in world.findall('model') if m.get('name').startswith('pillar_')]:
        world.remove(m)
    scene = world.find('scene')
    scene.find('ambient').text = '0.6 0.6 0.6'
    scene.find('background').text = '0.62 0.75 0.88'

    # Forest floor: one unique texture per tile.
    ground = world.find("model[@name='ground_plane']/link")
    ground.remove(ground.find('visual'))
    n_tiles = 0
    for x in np.arange(OUTER[0] - 4, OUTER[1] + 4, TILE):
        for y in np.arange(OUTER[2] - 3, OUTER[3] + 3, TILE):
            tex = f'floor_{n_tiles}.jpg'
            forest_floor(os.path.join(TEX_DIR, tex))
            ground.append(ET.fromstring(
                f'<visual name="floor_{n_tiles}"><pose>{fmt(x + TILE / 2, y + TILE / 2, 0.002, 0, 0, 0)}</pose>'
                f'<geometry><box><size>{TILE} {TILE} 0.004</size></box></geometry>{material(tex)}</visual>'))
            n_tiles += 1

    insert_at = list(world).index(world.find('include'))
    models = []
    # Trees in the flight area: own bark each; gaps of >= ~2 m between trunks for the drone.
    inner = poisson(4000, INNER, 3.3, [], CLEAR)
    for i, (x, y, _) in enumerate(inner):
        kind = ['pine', 'oak', 'birch'][int(rng.integers(3))]
        tex = f'bark_{i}.jpg'
        bark(os.path.join(TEX_DIR, tex), kind)
        models.append(tree_model(f'tree_{i}', x, y, rng.uniform(0.12, 0.32), rng.uniform(6, 12),
                                 math.radians(rng.uniform(0, 4)), rng.uniform(0, 2 * math.pi), tex, leaf_texs))
    # Dense belt around it (the camera always sees trees, like walls did).
    ox0, ox1, oy0, oy1 = OUTER
    belt = [p for p in poisson(6000, OUTER, 1.8, inner, CLEAR)
            if not (INNER[0] + 1 < p[0] < INNER[1] - 1 and INNER[2] + 1 < p[1] < INNER[3] - 1)]
    for i, (x, y, _) in enumerate(belt):
        models.append(tree_model(f'belt_{i}', x, y, rng.uniform(0.15, 0.4), rng.uniform(7, 14),
                                 math.radians(rng.uniform(0, 5)), rng.uniform(0, 2 * math.pi),
                                 belt_bark[i % len(belt_bark)], leaf_texs))
    obstacles = inner + belt
    # Low clutter (top <= ~0.35 m: flown over at 1 m even with the planner's 0.5 m inflation).
    for i, (x, y, _) in enumerate(poisson(800, INNER, 2.0, obstacles, CLEAR, 1.2)[:45]):
        a, b, c = rng.uniform(0.25, 0.6), rng.uniform(0.25, 0.6), rng.uniform(0.12, 0.2)
        models.append(f'<model name="bush_{i}"><static>true</static><pose>{fmt(x, y, c * 0.6, 0, 0, rng.uniform(0, 3.14))}</pose>'
                      f'<link name="link"><visual name="v"><geometry><ellipsoid><radii>{fmt(a, b, c)}</radii>'
                      f'</ellipsoid></geometry>{material(bush_texs[i % 3])}</visual></link></model>')
    rocks = poisson(600, INNER, 2.0, obstacles, CLEAR, 1.0)[:25]
    for i, (x, y, _) in enumerate(rocks):
        a, b, c = rng.uniform(0.15, 0.45), rng.uniform(0.15, 0.4), rng.uniform(0.1, 0.18)
        geo = f'<ellipsoid><radii>{fmt(a, b, c)}</radii></ellipsoid>'
        models.append(f'<model name="rock_{i}"><static>true</static><pose>{fmt(x, y, c * 0.5, 0, 0, rng.uniform(0, 3.14))}</pose>'
                      f'<link name="link"><collision name="c"><geometry>{geo}</geometry></collision>'
                      f'<visual name="v"><geometry>{geo}</geometry>{material(rock_texs[i % 4])}</visual></link></model>')
    logs = poisson(400, INNER, 4.0, obstacles + rocks, CLEAR, 2.0)[:8]
    for i, (x, y, _) in enumerate(logs):
        rad, L, yaw = rng.uniform(0.1, 0.15), rng.uniform(1.8, 3.2), rng.uniform(0, math.pi)
        geo = f'<cylinder><radius>{rad:.3f}</radius><length>{L:.3f}</length></cylinder>'
        models.append(f'<model name="log_{i}"><static>true</static><pose>{fmt(x, y, rad, 0, math.pi / 2, yaw)}</pose>'
                      f'<link name="link"><collision name="c"><geometry>{geo}</geometry></collision>'
                      f'<visual name="v"><geometry>{geo}</geometry>{material(belt_bark[i % len(belt_bark)])}</visual>'
                      '</link></model>')
    for k, m in enumerate(models):
        world.insert(insert_at + k, ET.fromstring(m))

    ET.indent(tree, space='  ')
    header = ('<?xml version="1.0" ?>\n<!-- Natural forest for GPS-denied flight (RTAB-Map). Generated by '
              'tools/gen_real_forest.py; edit the script, not this file. -->\n')
    with open(OUT_WORLD, 'w') as f:
        f.write(header + ET.tostring(tree.getroot(), encoding='unicode') + '\n')
    size = sum(os.path.getsize(os.path.join(TEX_DIR, t)) for t in os.listdir(TEX_DIR)) / 1e6
    print(f'wrote {OUT_WORLD}: {len(inner)} trees in the flight area, {len(belt)} in the belt, '
          f'{len(models) - len(inner) - len(belt)} bushes/rocks/logs, {n_tiles} floor tiles; '
          f'{len(os.listdir(TEX_DIR))} textures ({size:.1f} MB) in {TEX_MODEL}')


if __name__ == '__main__':
    main()
