#!/usr/bin/env python3
"""Build the world Gazebo runs (called by run_gazebo.sh): a world of worlds/ (or ardupilot_gazebo's)
plus the drone, models/truf_d455 as model "iris", at the world's spawn point.

- Spawn point: <frame name="ardupilot_sim_spawn"> of the world; a world that includes an iris itself
  (ardupilot_gazebo's iris_runway.sdf ...) has it removed, and its pose is the spawn point.
  --pose x,y,z,yaw_deg overrides it.
- --wind speed[,dir_deg[,gust_pct]]: steady wind of `speed` m/s blowing towards dir_deg (0 = +x,
  90 = +y), gusts of +-gust_pct % (default 20) over 10 s; replaces the world's own wind
  (baylands.sdf has 5 m/s). --wind 0 turns it off.
- --infra: keeps the D455's infrared pair (rendering cost), else it is removed from the model copy.

Writes /tmp/ardupilot_sim/worlds/<world>.sdf, /tmp/ardupilot_sim/models/truf_d455/ (put first in
GZ_SIM_RESOURCE_PATH) and /tmp/ardupilot_sim/current.json, which run_ros_bridge.sh reads (world
name, spawn point, infrared on/off, model file). Prints the world file to run.
Text-based on purpose: some worlds have comments an XML parser rejects ("--"), Gazebo doesn't.
"""
import argparse
import json
import math
import os
import re
import shutil
import sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = '/tmp/ardupilot_sim'
MODEL = 'truf_d455'
DEFAULT_SPAWN = (0.0, 0.0, 0.195, 0.0)  # iris standing on flat ground at the origin

WIND_PLUGIN = '''    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <!-- Strength of the wind force, as in baylands.sdf (measured on the hovering iris: ~2.6 deg
           tilt at 5 m/s, ~5 deg at 10 m/s). -->
      <force_approximation_scaling_factor>0.3</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>10</time_for_rise>
          <sin>
            <amplitude_percent>{gust}</amplitude_percent>
            <period>10</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.3</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin>
            <amplitude>10</amplitude>
            <period>30</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>2</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.03</stddev>
        </noise>
      </vertical>
    </plugin>
'''


def find_world(name):
    for d in (os.path.join(DIR, 'worlds'), os.path.join(DIR, 'ardupilot_gazebo', 'worlds')):
        path = os.path.join(d, name)
        if os.path.isfile(path):
            return path
    if os.path.isfile(name):
        return os.path.abspath(name)
    sys.exit(f'World {name} not found in worlds/ or ardupilot_gazebo/worlds/')


def parse_pose(element):
    """x, y, z, yaw_deg of a <pose> element's text (degrees="true" or radians)."""
    m = re.search(r'<pose(\s+degrees="true")?\s*>([^<]+)</pose>', element)
    if not m:
        return DEFAULT_SPAWN
    v = [float(t) for t in m.group(2).split()] + [0.0] * 6
    yaw = v[5] if m.group(1) else math.degrees(v[5])
    return (v[0], v[1], v[2], yaw)


def insert_before_world_end(s, block):
    """block (whole lines) inserted before the line of </world>."""
    i = s.rindex('\n', 0, s.index('</world>')) + 1
    return s[:i] + block + s[i:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('--pose', help='x,y,z,yaw_deg')
    ap.add_argument('--wind', help='speed[,dir_deg[,gust_pct]]')
    ap.add_argument('--infra', action='store_true')
    a = ap.parse_args()

    src = find_world(a.world)
    s = open(src).read()
    name = re.search(r'<world\s+name="([^"]+)"', s).group(1)

    spawn = None
    m = re.search(r'<frame\s+name="ardupilot_sim_spawn"\s*>.*?</frame>', s, re.S)
    if m:
        spawn = parse_pose(m.group(0))
    # A drone the world brings itself (ardupilot_gazebo's worlds): removed, its pose kept.
    for inc in list(re.finditer(r'[ \t]*<include>(?:(?!</include>).)*?<uri>model://[^<]*iris[^<]*</uri>'
                                r'.*?</include>\s*?\n', s, re.S)):
        if spawn is None:
            spawn = parse_pose(inc.group(0))
        s = s.replace(inc.group(0), '')
    spawn = spawn or DEFAULT_SPAWN
    if a.pose:
        v = [float(t) for t in a.pose.split(',')]
        if len(v) != 4:
            sys.exit('--pose needs x,y,z,yaw_deg')
        spawn = tuple(v)

    wind = None
    if a.wind is not None:
        v = [float(t) for t in a.wind.split(',')]
        speed = v[0]
        direction = v[1] if len(v) > 1 else 0.0
        gust = v[2] if len(v) > 2 else 20.0
        wind = {'speed': speed, 'dir_deg': direction, 'gust_pct': gust}
        s = re.sub(r'[ \t]*<wind>.*?</wind>\s*?\n', '', s, flags=re.S)
        s = re.sub(r'[ \t]*<plugin[^>]*wind-effects-system.*?</plugin>\s*?\n', '', s, flags=re.S)
        if speed > 0:
            vx = speed * math.cos(math.radians(direction))
            vy = speed * math.sin(math.radians(direction))
            block = (f'    <!-- run_gazebo.sh --wind {a.wind} -->\n    <wind>\n'
                     f'      <linear_velocity>{vx:.3f} {vy:.3f} 0</linear_velocity>\n    </wind>\n'
                     + WIND_PLUGIN.format(gust=gust / 100.0))
            s = insert_before_world_end(s, block)

    x, y, z, yaw = spawn
    drone = (f'    <!-- added by run_gazebo.sh (tools/make_world.py) -->\n'
             f'    <include>\n      <uri>model://{MODEL}</uri>\n      <name>iris</name>\n'
             f'      <pose degrees="true">{x:g} {y:g} {z:g} 0 0 {yaw:g}</pose>\n    </include>\n')
    s = insert_before_world_end(s, drone)

    os.makedirs(os.path.join(OUT, 'worlds'), exist_ok=True)
    world_out = os.path.join(OUT, 'worlds', os.path.basename(src))
    with open(world_out, 'w') as f:
        f.write(s)

    model_dir = os.path.join(OUT, 'models', MODEL)
    shutil.rmtree(model_dir, ignore_errors=True)
    shutil.copytree(os.path.join(DIR, 'models', MODEL), model_dir)
    model_sdf = os.path.join(model_dir, 'model.sdf')
    if not a.infra:
        m_sdf = open(model_sdf).read()
        m_sdf, n = re.subn(r'[ \t]*<!-- infra:begin.*?<!-- infra:end -->\s*?\n', '', m_sdf, flags=re.S)
        assert n == 1, 'infra block markers missing in models/truf_d455/model.sdf'
        with open(model_sdf, 'w') as f:
            f.write(m_sdf)

    with open(os.path.join(OUT, 'current.json'), 'w') as f:
        json.dump({'world_name': name, 'world_source': src, 'world_file': world_out,
                   'spawn': {'x': x, 'y': y, 'z': z, 'yaw_deg': yaw}, 'infra': a.infra, 'wind': wind,
                   'model_sdf': model_sdf}, f, indent=2)
    print(world_out)


if __name__ == '__main__':
    main()
