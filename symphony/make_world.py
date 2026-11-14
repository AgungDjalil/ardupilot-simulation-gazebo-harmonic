#!/usr/bin/env python3
"""World for `symphony/run.sh`: N ArduPilot quads, each with a 3-axis gimbal camera, added to a world.

    make_world.py N OUT_DIR AG_DIR BASE_WORLD

Writes OUT_DIR/world.sdf and OUT_DIR/models/, and prints the world path, the SITL home "lat,lon,alt", and then one
line "G<i> lat lon" per aircraft: where it starts (the station's base for that aircraft).

SYM_MODEL picks the airframe:
  x500 (default) PX4's x500_gimbal from Gazebo Fuel (https://app.gazebosim.org/PX4/fuel/models/x500_gimbal): the
                 Holybro X500 body, meshes, masses and collisions, with the CGO3 3-axis gimbal camera. Read from
                 the Fuel cache (SYM_PX4_FUEL, default ~/.gz/fuel/fuel.gazebosim.org/px4/models; run.sh downloads
                 x500_gimbal, x500 and gimbal there). PX4 flies it through MulticopterMotorModel on PX4's
                 actuator topic, which ArduPilot cannot drive, so those plugins are replaced: the ArduPilot plugin
                 spins the rotor joints and LiftDrag on each rotor makes the thrust, as on ardupilot_gazebo's
                 iris_with_gimbal (same gains, same motor order: PX4's quad X and ArduPilot's are the same). The
                 IMU is the Iris' (its own link, 1 kHz, the frame the plugin expects). The gimbal's three joints
                 take the servo mount's channels 9..11 (roll, pitch, yaw) on /G<i>/gimbal/cmd_*.
  iris           ardupilot_gazebo's iris_with_gimbal (gimbal_small_3d), the earlier airframe; SYM_LOOK applies.

SYM_HEXA=1 (default) adds H1, the hexa in the R23's middle bay (in place of the vehicle's static hexa visual; in the
ground scene, after the quads): the Typhoon H480 from Gazebo Fuel (https://app.gazebosim.org/Gambit/fuel/models/
Typhoon%20H480, cached in SYM_H480_FUEL) with its CGO3 gimbal replaced by a fixed Livox Avia LiDAR. ArduCopter
SITL instance -I 8 (HEXA_INDEX: after the eight pads, so its ports stay put whatever N is), FRAME_CLASS 2 / TYPE 1.
The Typhoon's nose is its -x (camera, IMU and propeller handedness all say so), so it is included turned 180 deg
and the ArduPilot plugin, the IMU and the LiDAR work in the outer model's frame, nose +x. Its tilted motor axes are
set upright (LiftDrag pushes along the rotor link's z) and its retractable legs fixed down. The Avia: 91 x 61.2 x
64.8 mm, 498 g, a gpu_lidar over its 70.4 x 77.2 deg field, 160 x 150 rays at 10 Hz (its 240 000 points/s, as a
grid: Gazebo cannot draw the non-repetitive rosette), 1..450 m, 2 cm noise, on /H1/lidar and /H1/lidar/points.
SYM_LIDAR_TILT_DEG: 90 (default) looks straight down, 0 straight ahead.

SYM_SCENE picks where the aircraft start:
  r23 (default)  on the landing pads of the N°COMMAND R23 command vehicle (models/r23_hilux, a Toyota Hilux with its
                 side cassettes and their extensions deployed), one aircraft per pad in this order, up to 8:
                 G1/G2 main front left/right, G3/G4 main rear, G5/G6 outer front, G7/G8 outer rear.
                 SYM_R23_POSE="x,y,z,yaw_deg" places the vehicle (default: at the world's spawn frame, on the
                 ground there; Baylands "0,0,-0.09,0": open grass, the front bumper at the origin facing south).
                 With SYM_MODEL=iris the aircraft look like the R23 quad (models/r23_quad), on the Iris airframe
                 and physics (SYM_LOOK=iris: the Iris' own look).
  ground         on the ground at the world's <frame name="ardupilot_sim_spawn">, each next one 25 m west and
                 SYM_DROP_M (2 m) higher so that on sloping ground it falls onto the terrain; SYM_SPAWNS="x,y,z;..."
                 places them exactly (heights for the Iris' origin; an x500 is lowered to match).

With SYM_MODEL=iris, every aircraft gets its own copy of ardupilot_gazebo's `iris_with_gimbal` and `gimbal_small_3d`, because those
models hard-code what must differ per aircraft:
  - the ArduPilot plugin's JSON port (`fdm_port_in`): 9002 + 10*i, the port SITL instance -I i talks to;
  - the gimbal joint topics (`/gimbal/cmd_*`, global in the original): /G<i+1>/gimbal/cmd_*, otherwise the
    servos of one aircraft would turn every gimbal in the world;
  - the camera topic: /G<i+1>/camera/image (read by camera_bridge.py), zoom on /G<i+1>/camera/zoom.
The GstCameraPlugin is dropped: it needs x264enc (gstreamer1.0-plugins-ugly), and camera_bridge.py serves the
frames instead. With SYM_MODEL=x500 the same per-aircraft parts are made from PX4's models: one shared x500_sym
(body, rotors, IMU) and per aircraft gimbal_px4_sym_<i> (topics) and x500_gimbal_<i> (port, servo channels).

The camera is the fleet's SIYI A8 mini as the station models it (gcs/payload.py): 82.56 deg horizontal at
1280x720, so the footprint the station draws on the map matches what the video shows. Override with
SYM_CAM_HFOV_DEG, SYM_CAM_W, SYM_CAM_H, SYM_CAM_FPS.

The home is the world's <spherical_coordinates> and every SITL shares it: with the JSON backend an aircraft's GPS
is that origin plus its pose in the world, so the GPS and the scenery agree. The world is edited as text: some
worlds carry comments an XML parser rejects.
"""
import math
import os
import re
import sys

SPACING_M = 25.0
FDM_PORT_BASE = 9002
EARTH_R = 6378137.0
SCENE = os.environ.get("SYM_SCENE", "r23")
MODEL = os.environ.get("SYM_MODEL", "x500")
PX4_FUEL = os.path.expanduser(os.environ.get("SYM_PX4_FUEL", "~/.gz/fuel/fuel.gazebosim.org/px4/models"))
LOOK =os.environ.get("SYM_LOOK", "r23" if SCENE == "r23" else "iris")   # r23: the R23 quad's mesh; iris
DROP_M = float(os.environ.get("SYM_DROP_M", "2.0"))
CAM_HFOV_DEG = float(os.environ.get("SYM_CAM_HFOV_DEG", "82.56"))
CAM_W = int(os.environ.get("SYM_CAM_W", "1280"))
CAM_H = int(os.environ.get("SYM_CAM_H", "720"))
CAM_FPS = float(os.environ.get("SYM_CAM_FPS", "15"))

# R23 pad centres in the vehicle's frame, in the order G1..G8 (PanggungR23.tsx: main cassette x = +-(dockX 0.42 +
# travel 1.0), deck top padZ 1.095; outer extension 0.88 further out, top deckZ 1.066 + lift 0.028; rows y
# 4.115/4.905). The 4 main pads carry the main quads, the 4 outer ones the reserves. The Iris stands 0.195 m above
# its feet; 2 cm of drop onto the deck.
R23_PADS = [(1.42, 4.115, 1.095), (-1.42, 4.115, 1.095), (1.42, 4.905, 1.095), (-1.42, 4.905, 1.095),
            (2.30, 4.115, 1.094), (-2.30, 4.115, 1.094), (2.30, 4.905, 1.094), (-2.30, 4.905, 1.094)]
IRIS_STAND_M = 0.195
# The x500's model origin is 13 mm BELOW its feet: base_link is 0.24 m above the origin and the bottom of the
# landing-gear bars (collision_3/4) 0.227 m below base_link.
X500_STAND_M = -0.013
STAND_M = X500_STAND_M if MODEL == "x500" else IRIS_STAND_M

HEXA = os.environ.get("SYM_HEXA", "1") == "1"
HEXA_INDEX = 8
H480_FUEL = os.path.expanduser(os.environ.get("SYM_H480_FUEL",
                                              "~/.gz/fuel/fuel.gazebosim.org/gambit/models/typhoon h480"))
H480_STAND_M = 0.2195            # base_link above the bottom of the Typhoon's leg bars
R23_BAY = (0.0, 4.58, 1.06)      # the middle bay's floor (r23_hilux bay_floor top), vehicle frame
# The vehicle on the R23 scene: "ncommand" (default) the team's N°COMMAND pickup (models/ncommand_r23 by
# make_ncommand.py: the same frame and pads, ArUco pads on them), H1 on its canopy roof; "hilux" the earlier R23
# Hilux (models/r23_hilux), H1 in its open middle bay.
VEHICLE = os.environ.get("SYM_VEHICLE", "ncommand")
NCOMMAND_ROOF = (0.0, 4.51, 2.02)  # the canopy roof's middle, on its ArUco pad (pad 6)
LIDAR_TILT_DEG = float(os.environ.get("SYM_LIDAR_TILT_DEG", "90"))
# Rotor -> ArduPilot hexa X motor, in the Typhoon's frame (nose -x): motor 1 right CW, 2 left CCW, 3 front-left CW,
# 4 rear-right CCW, 5 front-right CCW, 6 rear-left CW (AP_MotorsMatrix). (servo channel, rotor link, direction)
HEXA_MOTORS = [(0, 4, "cw"), (1, 5, "ccw"), (2, 2, "cw"), (3, 3, "ccw"), (4, 0, "ccw"), (5, 1, "cw")]


def gimbal_model(src: str, i: int) -> str:
    g = f"G{i + 1}"
    sdf = re.sub(r'\s*<plugin name="GstCameraPlugin".*?</plugin>', "", src, flags=re.S)
    # No collisions on the gimbal: its meshes hang 2 cm above the feet, inside Bullet's contact margin of
    # whatever the aircraft stands on, and testing them every 1 ms step against the R23's decks slowed the
    # world to a real-time factor of 0.04. The links keep their inertia.
    sdf = re.sub(r"\s*<collision name=[\"'][^\"']*[\"']>.*?</collision>", "", sdf, flags=re.S)
    sdf = re.sub(r'<model name="[^"]*"', f'<model name="gimbal_sym_{i}"', sdf, count=1)
    sdf = re.sub(r'(<sensor name="camera" type="camera">)', rf"\1\n        <topic>/{g}/camera/image</topic>", sdf)
    sdf = re.sub(r"<horizontal_fov>[^<]*</horizontal_fov>",
                 f"<horizontal_fov>{math.radians(CAM_HFOV_DEG):.6f}</horizontal_fov>", sdf)
    sdf = re.sub(r"<width>\d+</width>", f"<width>{CAM_W}</width>", sdf)
    sdf = re.sub(r"<height>\d+</height>", f"<height>{CAM_H}</height>", sdf)
    sdf = re.sub(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{CAM_FPS:g}</update_rate>", sdf)
    return re.sub(r'(<plugin filename="CameraZoomPlugin"\s+name="CameraZoomPlugin">)',
                  rf"\1\n          <topic>/{g}/camera/zoom</topic>", sdf)


def r23_airframe(src: str) -> str:
    """iris_with_standoffs with the R23 quad's look: every Iris visual (body, legs, propellers) replaced by the
    R23 quad mesh (quad_tex.obj, coloured by texture_r23.py), feet on the Iris' feet. Links, collisions and inertia stay the Iris'."""
    sdf = re.sub(r"\s*<visual name='[^']*'>.*?</visual>", "", src, flags=re.S)
    sdf = re.sub(r"<model name='[^']*'>", "<model name='r23_quad_standoffs'>", sdf, count=1)
    visual = f"""
      <visual name='r23_quad_visual'>
        <pose>0 0 {-IRIS_STAND_M} 0 0 0</pose>
        <geometry><mesh><uri>model://r23_quad/meshes/quad_tex.obj</uri></mesh></geometry>
      </visual>"""
    return re.sub(r"(<link name='base_link'>)", r"\1" + visual.replace("\\", "\\\\"), sdf, count=1)


def aircraft_model(src: str, i: int) -> str:
    sdf = re.sub(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{FDM_PORT_BASE + 10 * i}</fdm_port_in>", src)
    sdf = re.sub(r'<model name="[^"]*"', f'<model name="iris_gimbal_{i}"', sdf, count=1)
    sdf = sdf.replace("<uri>model://gimbal_small_3d</uri>", f"<uri>model://gimbal_sym_{i}</uri>")
    if LOOK == "r23":
        # Keep the include's name: the plugin finds its IMU and rotor joints as iris_with_standoffs::...
        sdf = sdf.replace("<uri>model://iris_with_standoffs</uri>",
                          "<uri>model://r23_quad_standoffs</uri>\n      <name>iris_with_standoffs</name>")
    return sdf.replace("/gimbal/cmd_", f"/G{i + 1}/gimbal/cmd_")


def fuel_dir(name: str) -> str:
    """The newest cached version of a PX4 Fuel model: PX4_FUEL/<name>/<version>."""
    root = os.path.join(PX4_FUEL, name)
    vers = sorted((v for v in os.listdir(root) if v.isdigit()), key=int) if os.path.isdir(root) else []
    if not vers:
        sys.exit(f"no {name} in {PX4_FUEL}: gz fuel download -u https://fuel.gazebosim.org/1.0/PX4/models/{name}")
    return os.path.join(root, vers[-1])


def local_meshes(sdf: str) -> str:
    """Fuel URLs -> the cached files, so the world opens with no network."""
    return re.sub(r"https://fuel\.gazebosim\.org/1\.0/px4/models/(\w+)/\d+/files/",
                  lambda m: "file://" + fuel_dir(m.group(1)) + "/", sdf, flags=re.I)


# The Iris' IMU (ardupilot_gazebo iris_with_standoffs): its own link, rolled 180 deg for the plugin's
# modelXYZToAirplaneXForwardZDown, at 1 kHz for lock-step. The x500's own imu_sensor (250 Hz, unrolled) goes.
IMU = """
        <link name="imu_link">
            <pose relative_to="base_link">0 0 0 0 0 0</pose>
            <inertial><mass>0.15</mass><inertia><ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00002</iyy>
                <iyz>0</iyz><izz>0.00002</izz></inertia></inertial>
            <sensor name="imu_sensor" type="imu">
                <gz_frame_id>imu_link</gz_frame_id>
                <pose degrees="true">0 0 0 180 0 0</pose>
                <always_on>1</always_on>
                <update_rate>1000.0</update_rate>
            </sensor>
        </link>
        <joint name="imu_joint" type="fixed">
            <parent>base_link</parent>
            <child>imu_link</child>
        </joint>
"""


def x500_base(src: str) -> str:
    """PX4's x500 without its PX4 motor plugins and with the Iris' IMU: body, rotors, meshes, collisions.

    The rotors get the Iris rotor's inertia and joint damping. They are driven by the Iris' velocity controller
    (p_gain 0.2, +-2.5 N m, 1 ms steps), and the x500's own rotor (izz 2.65e-5, no damping) is six times lighter
    than that controller was tuned for: dt*p/izz = 7.5, past the explicit step's limit of 2, so a rotor told 0 rad/s
    limit-cycled at the torque limit and the propellers turned (+-12.5 rad/s, one at 80) on the ground, disarmed.
    With the Iris' 1.676e-4 it is 1.2, stable.

    Joint damping sets the top rotor speed: the controller's torque is capped at 2.5 N m, so a rotor tops out near
    2.5 / damping. With the Iris' 0.004 that is ~625 rad/s of the plugin's 838, ~8.3 N a rotor: thrust/weight 1.3
    for this 2.6 kg airframe, enough to hover and too little to manoeuvre in the park's wind. 0.003 gives ~833 rad/s,
    ~15 N a rotor, thrust/weight ~2.3, an X500's real margin."""
    sdf = re.sub(r'\s*<plugin filename="gz-sim-multicopter-motor-model-system".*?</plugin>', "", src, flags=re.S)

    def iris_rotor(m: re.Match) -> str:
        s = re.sub(r"<ixx>[^<]*</ixx>", "<ixx>9.75e-06</ixx>", m.group(0))
        s = re.sub(r"<iyy>[^<]*</iyy>", "<iyy>0.000166704</iyy>", s)
        return re.sub(r"<izz>[^<]*</izz>", "<izz>0.000167604</izz>", s)
    sdf = re.sub(r'<link name="rotor_\d">.*?</inertial>', iris_rotor, sdf, flags=re.S)
    sdf = re.sub(r'(<joint name="rotor_\d_joint".*?<dynamics>)', r"\1\n                    <damping>0.003</damping>",
                 sdf, flags=re.S)
    sdf = re.sub(r'\s*<sensor name="imu_sensor" type="imu">.*?</sensor>', "", sdf, count=1, flags=re.S)
    sdf = re.sub(r'<model name="[^"]*">', '<model name="x500_sym">', sdf, count=1)
    k = sdf.rindex("</model>")
    return local_meshes(sdf[:k] + IMU + "    " + sdf[k:])


def x500_gimbal_px4(src: str, i: int) -> str:
    """PX4's CGO3 gimbal for aircraft i: camera as the fleet's SIYI A8 mini on /G<i>/camera/image with a zoom on
    /G<i>/camera/zoom, joints driven from /G<i>/gimbal/cmd_* (PX4's gains), no collisions (see gimbal_model)."""
    g = f"G{i + 1}"
    sdf = re.sub(r'<model name="[^"]*">', f'<model name="gimbal_px4_sym_{i}">', src, count=1)
    sdf = re.sub(r"\s*<collision name=[\"'][^\"']*[\"']>.*?</collision>", "", sdf, flags=re.S)
    for joint, axis in (("cgo3_horizontal_arm_joint", "roll"), ("cgo3_camera_joint", "pitch"),
                        ("cgo3_vertical_arm_joint", "yaw")):
        sdf = re.sub(rf"(<joint_name>{joint}</joint_name>\s*)<sub_topic>[^<]*</sub_topic>",
                     rf"\1<topic>/{g}/gimbal/cmd_{axis}</topic>", sdf)

    def camera(m: re.Match) -> str:
        s = m.group(0)
        s = re.sub(r"<horizontal_fov>[^<]*</horizontal_fov>",
                   f"<horizontal_fov>{math.radians(CAM_HFOV_DEG):.6f}</horizontal_fov>", s)
        s = re.sub(r"<width>\d+</width>", f"<width>{CAM_W}</width>", s)
        s = re.sub(r"<height>\d+</height>", f"<height>{CAM_H}</height>", s)
        s = re.sub(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{CAM_FPS:g}</update_rate>", s)
        s = re.sub(r"<visualize>[^<]*</visualize>", "<visualize>false</visualize>", s)
        s = re.sub(r"<topic>[^<]*</topic>", f"<topic>/{g}/camera/image</topic>", s)
        # ardupilot_gazebo's zoom, as on gimbal_small_3d (max 125x, its slew rate).
        return s.replace("</sensor>", f"""    <plugin filename="CameraZoomPlugin" name="CameraZoomPlugin">
                    <topic>/{g}/camera/zoom</topic>
                    <max_zoom>125.0</max_zoom>
                    <slew_rate>0.42514285714</slew_rate>
                </plugin>
            </sensor>""")
    sdf = re.sub(r'<sensor name="camera" type="camera">.*?</sensor>', camera, sdf, count=1, flags=re.S)
    return local_meshes(sdf)


def x500_aircraft(iris: str, i: int) -> str:
    """Aircraft i: x500_sym + gimbal_px4_sym_<i> merged as PX4's x500_gimbal does it, flown by the ArduPilot plugin
    with iris_with_gimbal's rotor dynamics (LiftDrag, ApplyJointForce, the plugin's rotor channels and gains)."""
    plugins = iris[iris.index("<!-- plugins -->"):iris.index("</plugin>", iris.index('name="ArduPilotPlugin"'))
                   + len("</plugin>")]
    plugins = re.sub(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{FDM_PORT_BASE + 10 * i}</fdm_port_in>",
                     plugins)
    plugins = plugins.replace("iris_with_standoffs::imu_link::imu_sensor", "imu_link::imu_sensor")
    plugins = plugins.replace("iris_with_standoffs::", "")
    # The CGO3's joints, their sense worked out from the axes after the gimbal's 180 deg yaw on the body:
    #   roll  cgo3_horizontal_arm_joint, body +x: + = right roll (as the Iris gimbal's roll joint);
    #   pitch cgo3_camera_joint,         body -y: + = nose up    (the Iris gimbal's pitch is the other way);
    #   yaw   cgo3_vertical_arm_joint,   body -z: + = to the right (the Iris gimbal's yaw is the other way).
    plugins = plugins.replace("gimbal::roll_joint", "cgo3_horizontal_arm_joint")
    plugins = plugins.replace("gimbal::pitch_joint", "cgo3_camera_joint")
    plugins = plugins.replace("gimbal::yaw_joint", "cgo3_vertical_arm_joint")
    plugins = plugins.replace("<multiplier>-3.14159265</multiplier>", "<multiplier>3.14159265</multiplier>")
    plugins = plugins.replace("<multiplier>-5.5850536</multiplier>", "<multiplier>5.5850536</multiplier>")
    plugins = plugins.replace("/gimbal/cmd_", f"/G{i + 1}/gimbal/cmd_")
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="x500_gimbal_{i}">
    <include merge="true">
      <uri>model://x500_sym</uri>
    </include>
    <include merge="true">
      <uri>model://gimbal_px4_sym_{i}</uri>
      <pose>0 0 0.26 0 0 3.14</pose>
    </include>
    <joint name="GimbalAttachJoint" type="fixed">
      <parent>base_link</parent>
      <child>cgo3_mount_link</child>
    </joint>

    {plugins}

  </model>
</sdf>
"""


GAZEBO_COLOURS = {"Black": "0.05 0.05 0.05", "Blue": "0.1 0.18 0.55", "DarkGrey": "0.175 0.175 0.175",
                  "Grey": "0.5 0.5 0.5", "White": "0.95 0.95 0.95"}


def h480_base(src: str, out: str) -> str:
    """The Typhoon H480 as a body for ArduPilot: no CGO3 gimbal and no PX4 IMU, legs fixed down, rotor axes upright,
    meshes copied next to it (the cache path has a space in it), Ogre material scripts as plain colours."""
    import shutil
    sdf = re.sub(r"(<model name=)'[^']*'>\s*(<!--[^>]*-->)?\s*<pose>[^<]*</pose>", r'\1"h480_sym">', src, count=1)
    sdf = re.sub(r"\s*<link name=[\"']cgo3_\w+[\"']>.*?</link>", "", sdf, flags=re.S)
    sdf = re.sub(r"\s*<joint name=[\"']cgo3_\w+[\"'].*?</joint>", "", sdf, flags=re.S)
    sdf = re.sub(r"\s*<link name=[\"']typhoon_h480/imu_link[\"']>.*?</link>", "", sdf, flags=re.S)
    sdf = re.sub(r"\s*<joint name=[\"']typhoon_h480/imu_joint[\"'].*?</joint>", "", sdf, flags=re.S)
    for leg in ("left_leg", "right_leg"):
        sdf = re.sub(rf"<joint name=[\"']{leg}_joint[\"'].*?</joint>",
                     f"<joint name='{leg}_joint' type='fixed'><parent>base_link</parent><child>{leg}</child></joint>",
                     sdf, flags=re.S)
    sdf = re.sub(r"(<joint name=[\"']rotor_\d_joint[\"'].*?<xyz>)[^<]*(</xyz>)", r"\g<1>0 0 1\2", sdf, flags=re.S)
    # Rotor damping 0.005 -> 0.0035. The controller's torque is capped at 2.5 N m, so a rotor tops out near
    # 2.5 / damping: 500 rad/s at 0.005, ~5.3 N a rotor, thrust/weight 1.1 for this 2.9 kg hexa. It hovered, and in
    # the first survey leg in the park's wind three motors sat at full output ("Potential Thrust Loss (3)") and it
    # rolled over. 0.0035: ~714 rad/s, ~10.9 N a rotor, thrust/weight ~2.3.
    sdf = re.sub(r"(<joint name=[\"']rotor_\d_joint[\"'].*?<damping>)[^<]*(</damping>)", r"\g<1>0.0035\2", sdf,
                 flags=re.S)
    # Textured meshes (texture_h480.py -> models/h480_tex) where they exist: the OBJ is in metres and carries its
    # own materials, so the visual loses its scale and its colour.
    tex = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "h480_tex", "meshes")

    def textured(m: re.Match) -> str:
        vis = m.group(0)
        u = re.search(r"<uri>([\w.]+)\.stl</uri>", vis)
        if not u or not os.path.exists(os.path.join(tex, u.group(1) + ".obj")):
            return vis
        vis = vis.replace(u.group(0), f"<uri>model://h480_tex/meshes/{u.group(1)}.obj</uri>")
        vis = re.sub(r"\s*<scale>[^<]*</scale>", "", vis)
        return re.sub(r"\s*<material>.*?</material>", "", vis, flags=re.S)
    sdf = re.sub(r"<visual name=[\"'][^\"']*[\"']>.*?</visual>", textured, sdf, flags=re.S)
    sdf = re.sub(r"<material>\s*<script>.*?<name>Gazebo/(\w+)</name>\s*</script>\s*</material>",
                 lambda m: (f"<material><ambient>{GAZEBO_COLOURS.get(m.group(1), '0.5 0.5 0.5')} 1</ambient>"
                            f"<diffuse>{GAZEBO_COLOURS.get(m.group(1), '0.5 0.5 0.5')} 1</diffuse></material>"),
                 sdf, flags=re.S)
    src_dir = os.path.join(fuel_dir_at(H480_FUEL), "meshes")
    dst_dir = os.path.join(out, "models", "h480_sym", "meshes")
    os.makedirs(dst_dir, exist_ok=True)
    for name in set(re.findall(r"<uri>([\w.]+\.stl)</uri>", sdf)):
        shutil.copyfile(os.path.join(src_dir, name), os.path.join(dst_dir, name))
    return re.sub(r"<uri>([\w.]+\.stl)</uri>", r"<uri>model://h480_sym/meshes/\1</uri>", sdf)


def fuel_dir_at(root: str) -> str:
    vers = sorted((v for v in os.listdir(root) if v.isdigit()), key=int) if os.path.isdir(root) else []
    if not vers:
        sys.exit(f"no Fuel model in {root}")
    return os.path.join(root, vers[-1])


def box_inertia(m: float, x: float, y: float, z: float) -> str:
    return (f"<inertial><mass>{m}</mass><inertia><ixx>{m * (y * y + z * z) / 12:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{m * (x * x + z * z) / 12:.6g}</iyy><iyz>0</iyz><izz>{m * (x * x + y * y) / 12:.6g}</izz>"
            f"</inertia></inertial>")


def livox_avia(name: str) -> str:
    """Livox Avia on a fixed mount under the body, its window (x of livox_link) tilted SYM_LIDAR_TILT_DEG down from
    the nose. Body 61.2 deep (x) x 91 wide (y) x 64.8 high (z) mm, aluminium with cooling fins on the sides, the
    teal window on the front face, the M12 connector at the back."""
    d, w, h = 0.0612, 0.091, 0.0648
    alu = ("<material><ambient>0.62 0.63 0.65 1</ambient><diffuse>0.78 0.79 0.81 1</diffuse>"
           "<specular>0.6 0.6 0.6 1</specular><pbr><metal><metalness>0.7</metalness><roughness>0.35</roughness>"
           "</metal></pbr></material>")
    fins = "".join(
        f"""
      <visual name="fin_{side}_{k}"><pose>-0.004 {s * (w / 2 + 0.002):.4f} {-h / 2 + 0.006 + k * 0.0088:.4f} 0 0 0</pose>
        <geometry><box><size>{d - 0.012:.4f} 0.004 0.0025</size></box></geometry>{alu}</visual>"""
        for side, s in (("l", 1), ("r", -1)) for k in range(7))
    return f"""
    <link name="livox_link">
      <pose degrees="true">0.05 0 -0.12 0 {LIDAR_TILT_DEG:g} 0</pose>
      {box_inertia(0.498, d, w, h)}
      <collision name="livox_collision"><geometry><box><size>{d} {w} {h}</size></box></geometry></collision>
      <visual name="livox_body"><geometry><box><size>{d} {w} {h}</size></box></geometry>{alu}</visual>
      <visual name="livox_window"><pose>{d / 2 + 0.0008:.4f} 0 0 0 0 0</pose>
        <geometry><box><size>0.0016 {w - 0.012:.4f} {h - 0.012:.4f}</size></box></geometry>
        <material><ambient>0.12 0.42 0.38 1</ambient><diffuse>0.25 0.7 0.62 1</diffuse><specular>0.9 0.9 0.9 1</specular>
          <pbr><metal><metalness>0.1</metalness><roughness>0.08</roughness></metal></pbr></material></visual>
      <visual name="livox_connector"><pose>{-d / 2 - 0.009:.4f} 0.02 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.0075</radius><length>0.018</length></cylinder></geometry>{alu}</visual>{fins}
      <sensor name="livox_avia" type="gpu_lidar">
        <topic>/{name}/lidar</topic>
        <gz_frame_id>livox_link</gz_frame_id>
        <update_rate>10</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal><samples>160</samples><resolution>1</resolution>
              <min_angle>{-math.radians(70.4 / 2):.5f}</min_angle><max_angle>{math.radians(70.4 / 2):.5f}</max_angle></horizontal>
            <vertical><samples>150</samples><resolution>1</resolution>
              <min_angle>{-math.radians(77.2 / 2):.5f}</min_angle><max_angle>{math.radians(77.2 / 2):.5f}</max_angle></vertical>
          </scan>
          <range><min>1.0</min><max>450</max><resolution>0.002</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.02</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="livox_joint" type="fixed"><parent>base_link</parent><child>livox_link</child></joint>"""


def h480_aircraft(iris: str, name: str) -> str:
    """H1: h480_sym turned to nose +x, the Iris' IMU, the Livox, and iris_with_gimbal's rotor dynamics on six motors
    (a CCW rotor gets the Iris' rotor_0 LiftDrag pair and channel 0's control, a CW rotor rotor_2's and channel 2's)."""
    lift = re.findall(r'<plugin filename="gz-sim-lift-drag-system".*?</plugin>', iris, flags=re.S)
    tpl_lift = {"ccw": "\n    ".join(lift[0:2]), "cw": "\n    ".join(lift[4:6])}
    ctl = {d: re.search(rf'<control channel="{c}">.*?</control>', iris, flags=re.S).group(0)
           for d, c in (("ccw", 0), ("cw", 2))}
    ap = iris[iris.index('<plugin name="ArduPilotPlugin"'):iris.index("<control channel=")]
    ap = re.sub(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{FDM_PORT_BASE + 10 * HEXA_INDEX}</fdm_port_in>", ap)
    ap = ap.replace("iris_with_standoffs::imu_link::imu_sensor", "imu_link::imu_sensor")
    lifts, forces, controls = [], [], []
    for ch, rotor, d in HEXA_MOTORS:
        src_rotor = "rotor_0" if d == "ccw" else "rotor_2"
        lifts.append(tpl_lift[d].replace(f"iris_with_standoffs::{src_rotor}", f"rotor_{rotor}"))
        forces.append(f"""<plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{rotor}_joint</joint_name>
    </plugin>""")
        c = ctl[d].replace(f"iris_with_standoffs::{src_rotor}_joint", f"rotor_{rotor}_joint")
        controls.append(re.sub(r'<control channel="\d+">', f'<control channel="{ch}">', c))
    imu = IMU.replace('<pose relative_to="base_link">0 0 0 0 0 0</pose>', "<pose>0 0 0 0 0 0</pose>")
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="h480_lidar">
    <include merge="true">
      <uri>model://h480_sym</uri>
      <pose>0 0 0 0 0 3.14159265</pose>
    </include>
{imu}{livox_avia(name)}

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    {chr(10).join("    " + x for x in lifts)}
    {chr(10).join("    " + x for x in forces)}
    {ap}{(chr(10) + "      ").join(controls)}
    </plugin>
  </model>
</sdf>
"""


def r23_bay_cleared(src: str) -> str:
    """The R23 without its static hexa visual: H1 flies from the bay instead."""
    sdf = re.sub(r"\s*<!--[^>]*Hexa parked[^>]*-->", "", src)
    sdf = re.sub(r'\s*<visual name="hexa">.*?</visual>', "", sdf, flags=re.S)
    return sdf.replace('<model name="r23_hilux">', '<model name="r23_hilux_bay">', 1)


def write_model(out: str, name: str, sdf: str) -> None:
    d = os.path.join(out, "models", name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write(f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
                f'<sdf version="1.9">model.sdf</sdf></model>\n')


def ground_poses(n: int, x: float, y: float, z: float) -> list:
    poses = [(x - SPACING_M * i, y, z + (DROP_M if i else 0.0)) for i in range(n)]
    for i, p in enumerate(v for v in os.environ.get("SYM_SPAWNS", "").split(";") if v.strip()):
        if i < n:
            poses[i] = tuple(float(c) for c in p.split(","))
    return poses


def r23_poses(n: int, vx: float, vy: float, vz: float, yaw_deg: float) -> list:
    if n > len(R23_PADS):
        sys.exit(f"the R23 has {len(R23_PADS)} pads, not {n}")
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    # Each aircraft faces outwards, away from the vehicle, so its camera looks out over the ground it will cover.
    return [(vx + c * px - s * py, vy + s * px + c * py, vz + pz + STAND_M + 0.02,
             yaw_deg + (0.0 if px > 0 else 180.0)) for px, py, pz in R23_PADS[:n]]


def includes(poses: list) -> str:
    """poses: (x, y, z, yaw in degrees, 0 = nose east)."""
    return "\n".join(
        f"""    <include>
      <uri>model://{"x500_gimbal" if MODEL == "x500" else "iris_gimbal"}_{i}</uri>
      <name>G{i + 1}</name>
      <pose degrees="true">{px:.3f} {py:.3f} {pz:.3f} 0 0 {yaw:g}</pose>
    </include>""" for i, (px, py, pz, yaw) in enumerate(poses))


def main() -> None:
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    n, out, ag, base_path = int(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
    with open(os.path.join(ag, "models", "iris_with_gimbal", "model.sdf")) as f:
        iris = f.read()
    with open(os.path.join(ag, "models", "gimbal_small_3d", "model.sdf")) as f:
        gimbal = f.read()
    if MODEL == "x500":
        with open(os.path.join(fuel_dir("x500"), "model.sdf")) as f:
            write_model(out, "x500_sym", x500_base(f.read()))
        with open(os.path.join(fuel_dir("gimbal"), "model.sdf")) as f:
            px4_gimbal = f.read()
        for i in range(n):
            write_model(out, f"gimbal_px4_sym_{i}", x500_gimbal_px4(px4_gimbal, i))
            write_model(out, f"x500_gimbal_{i}", x500_aircraft(iris, i))
    elif MODEL == "iris":
        if LOOK == "r23":
            with open(os.path.join(ag, "models", "iris_with_standoffs", "model.sdf")) as f:
                write_model(out, "r23_quad_standoffs", r23_airframe(f.read()))
        for i in range(n):
            write_model(out, f"gimbal_sym_{i}", gimbal_model(gimbal, i))
            write_model(out, f"iris_gimbal_{i}", aircraft_model(iris, i))
    else:
        sys.exit(f"SYM_MODEL must be x500 or iris, not {MODEL!r}")
    if HEXA:
        with open(os.path.join(fuel_dir_at(H480_FUEL), "model.sdf")) as f:
            write_model(out, "h480_sym", h480_base(f.read(), out))
        write_model(out, "h480_lidar", h480_aircraft(iris, "H1"))
    with open(base_path) as f:
        base = f.read()
    m = re.search(r'<frame name="ardupilot_sim_spawn">\s*<pose[^>]*>([^<]+)</pose>', base)
    x, y, z = (float(v) for v in m.group(1).split()[:3]) if m else (0.0, 0.0, 0.195)
    sph = base.split("<spherical_coordinates>", 1)[1]
    tag = lambda t: float(re.search(rf"<{t}>([^<]+)</{t}>", sph).group(1))
    lat0, lon0, alt0 = tag("latitude_deg"), tag("longitude_deg"), tag("elevation")

    extra = ""
    if SCENE == "r23":
        # The spawn frame is where a quad stands; the vehicle's origin is on the ground below it.
        vx, vy, vz, yaw = (float(v) for v in (
            os.environ.get("SYM_R23_POSE") or f"{x},{y},{z - IRIS_STAND_M},0").split(","))
        poses = r23_poses(n, vx, vy, vz, yaw)
        r23 = "r23_hilux"
        if VEHICLE == "ncommand":
            r23 = "ncommand_r23"
            if HEXA:
                c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                bx, by, bz = NCOMMAND_ROOF
                hexa_pose = (vx + c * bx - s * by, vy + s * bx + c * by, vz + bz + H480_STAND_M + 0.02, yaw)
        elif VEHICLE != "hilux":
            sys.exit(f"SYM_VEHICLE must be ncommand or hilux, not {VEHICLE!r}")
        elif HEXA:
            with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                   "models", "r23_hilux", "model.sdf")) as f:
                write_model(out, "r23_hilux_bay", r23_bay_cleared(f.read()))
            r23 = "r23_hilux_bay"
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            bx, by, bz = R23_BAY
            hexa_pose = (vx + c * bx - s * by, vy + s * bx + c * by, vz + bz + H480_STAND_M + 0.02, yaw)
        extra = f"""    <include>
      <uri>model://{r23}</uri>
      <name>R23</name>
      <pose degrees="true">{vx:.3f} {vy:.3f} {vz:.3f} 0 0 {yaw:g}</pose>
    </include>
"""
    elif SCENE == "ground":
        # Spawn heights are for the Iris' origin, 0.195 m above its feet; another airframe's feet go to the same place.
        poses = [(px, py, pz + STAND_M - IRIS_STAND_M, 90.0) for px, py, pz in ground_poses(n, x, y, z)]
        # The hexa next in the row, SYM_DROP_M up like every later aircraft.
        hx, hy, hz = ground_poses(n + 1, x, y, z)[n]
        hexa_pose = (hx, hy, hz + H480_STAND_M - IRIS_STAND_M, 90.0)
    else:
        sys.exit(f"SYM_SCENE must be r23 or ground, not {SCENE!r}")

    if os.environ.get("SYM_TOWN", "1") == "1":
        # Buildings and vehicles around the R23 (make_town.py): Fuel's city models on the Baylands terrain.
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))      # -I leaves this folder off the path
        import make_town
        extra += make_town.build(out)
    if HEXA:
        extra += f"""    <include>
      <uri>model://h480_lidar</uri>
      <name>H1</name>
      <pose degrees="true">{hexa_pose[0]:.3f} {hexa_pose[1]:.3f} {hexa_pose[2]:.3f} 0 0 {hexa_pose[3]:g}</pose>
    </include>
"""
    if os.environ.get("SYM_SHADOWS", "1" if os.environ.get("SYM_TOWN", "1") != "1" else "0") == "0":
        # The sun's shadows are rendered again for every camera and LiDAR frame, over every model in view: with the
        # neighbourhood's ~400 models that held lock-step to a real-time factor of ~0.5. Off by default with the
        # town (SYM_SHADOWS=1 keeps them).
        base = re.sub(r"(<light[^>]*name=\"sun\".*?<cast_shadows>)true(</cast_shadows>)", r"\1false\2", base,
                      count=1, flags=re.S)
    k = base.rindex("</world>")
    path = os.path.join(out, "world.sdf")
    with open(path, "w") as f:
        f.write(base[:k] + extra + includes(poses) + "\n  " + base[k:])
    print(path)
    print(f"{lat0},{lon0},{alt0}")
    # x east, y north (the plugin's ENU), small offsets on a sphere: enough for a pad position.
    named = [(f"G{i + 1}", p) for i, p in enumerate(poses)] + ([("H1", hexa_pose)] if HEXA else [])
    for label, (px, py, _, _) in named:
        lat = lat0 + math.degrees(py / EARTH_R)
        lon = lon0 + math.degrees(px / (EARTH_R * math.cos(math.radians(lat0))))
        print(f"{label} {lat:.7f} {lon:.7f}")


if __name__ == "__main__":
    main()
