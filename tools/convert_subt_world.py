"""Adapt a DARPA SubT Finals world from Gazebo Fuel for ArduPilot SITL (Gazebo Harmonic).

  python3 tools/convert_subt_world.py final_event_08.sdf worlds/cave_megacavern.sdf

Removes the competition-only parts (performer detectors, triggered debris walls, breadcrumbs,
dust/fog emitters, wind) and models no longer downloadable, points the old Fuel host at
fuel.gazebosim.org, and adds what ardupilot_gazebo needs: the physics/sensor/IMU systems,
spherical coordinates and an Iris with the ArduPilot plugin and a RealSense D435 in the
staging area.
"""
import sys
import xml.etree.ElementTree as ET

OLD_FUEL = 'https://fuel.ignitionrobotics.org/'
NEW_FUEL = 'https://fuel.gazebosim.org/'
# megacavern_shell is no longer public on Fuel (HTTP 401); the cave itself is the megacavern model.
# Vent and Climbing Helmet (artifacts) have a relative Ogre1 material URI that Gazebo Harmonic
# can't resolve, which fails the whole world. Gas needs SubT's GasEmitterDetectorPlugin.
# Fiducial: scoring markers (one is a cone right in front of the vehicle's start).
DROP_INCLUDES = ('Dust Emitter', 'Fog Emitter', 'megacavern_shell', 'Vent', 'Climbing Helmet',
                 'Gas Small', 'Fiducial')
SYSTEMS = [
    ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
    ('gz-sim-sensors-system', 'gz::sim::systems::Sensors'),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
    ('gz-sim-navsat-system', 'gz::sim::systems::NavSat'),
]
# Middle of the staging area (x -20.6..-8, flat floor at z 0, ceiling ~3.8 m), facing the cave
# entrance (+x). Outside it the cave floor is uneven and the Iris tips over on spawn.
# z = Iris' standing height, as in iris_runway.sdf.
VEHICLE_POSE = '-12 0 0.195 0 0 0'
# Iris + forward RealSense D435 (models/iris_with_d435).
VEHICLE_MODEL = 'model://iris_with_d435'
# Same as SITL's default home (CMAC), so the EKF origin and Gazebo agree.
HOME = {'latitude_deg': '-35.363262', 'longitude_deg': '149.165237', 'elevation': '584',
        'heading_deg': '0', 'surface_model': 'EARTH_WGS84'}


def sub(parent, tag, text=None, **attrib):
    el = ET.SubElement(parent, tag, attrib)
    if text is not None:
        el.text = text
    return el


def main(src, dst):
    tree = ET.parse(src)
    root = tree.getroot()
    root.set('version', '1.9')
    world = root.find('world')
    world.set('name', 'cave_megacavern')

    for el in list(world):
        uri = el.findtext('uri', '') if el.tag == 'include' else ''
        if el.tag in ('plugin', 'model', 'wind') or any(d in uri for d in DROP_INCLUDES):
            world.remove(el)
    for uri in world.iter('uri'):
        uri.text = uri.text.replace(OLD_FUEL, NEW_FUEL)

    # ArduPilot's plugin is tuned for 1 ms steps (also used by iris_runway.sdf).
    physics = world.find('physics')
    physics.set('type', 'dart')
    physics.find('max_step_size').text = '0.001'
    # DART's default collision detector makes the huge cave mesh the bottleneck (real-time factor
    # ~0.5 with SITL); with Bullet it runs at 3-6x real time.
    sub(sub(physics, 'dart'), 'collision_detector', 'bullet')
    # The cave is lit only by spot lights; a little more ambient light to see the drone.
    world.find('scene').find('ambient').text = '0.35 0.35 0.35 1.0'
    # Keep the lights but don't draw their 47 green frustums in the GUI.
    for light in world.iter('light'):
        if light.find('visualize') is None:
            sub(light, 'visualize', 'false')

    pos = list(world).index(world.find('scene'))
    for i, (filename, name) in enumerate(SYSTEMS):
        plugin = ET.Element('plugin', {'filename': filename, 'name': name})
        if name.endswith('Sensors'):
            sub(plugin, 'render_engine', 'ogre2')
        world.insert(pos + i, plugin)
    coords = sub(world, 'spherical_coordinates')
    for k, v in HOME.items():
        sub(coords, k, v)

    vehicle = sub(world, 'include')
    sub(vehicle, 'uri', VEHICLE_MODEL)
    sub(vehicle, 'name', 'iris')
    sub(vehicle, 'pose', VEHICLE_POSE)

    ET.indent(tree, '  ')
    header = ('<?xml version="1.0" ?>\n'
              '<!-- Based on "Finals Prize Round World 08" (DARPA SubT Finals, Louisville Mega Cavern)\n'
              '     by Open Robotics, CC BY 4.0:\n'
              '     https://app.gazebosim.org/OpenRobotics/fuel/worlds/Finals%20Prize%20Round%20World%2008\n'
              '     Adapted for ArduPilot SITL by tools/convert_subt_world.py -->\n')
    with open(dst, 'w') as f:
        f.write(header + ET.tostring(root, encoding='unicode') + '\n')


if __name__ == '__main__':
    main(*sys.argv[1:3])
