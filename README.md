# ardupilot-simulation-gazebo-harmonic

A simulation-only project: ArduCopter SITL with Gazebo Harmonic, cave and forest worlds for
**GPS-denied** flight, a drone with an **Intel RealSense D435** camera, a camera bridge to
**ROS 2 Jazzy / RViz2**, and control over **MAVLink** (QGroundControl, Mission Planner, MAVProxy,
MAVROS, pymavlink, MCP).

It doesn't depend on any ROS workspace. The core simulation needs no ROS at all; ROS 2 is only
used by `run_ros_bridge.sh`.

---

## Contents

1. [Folder layout](#1-folder-layout)
2. [Requirements](#2-requirements)
3. [Running](#3-running)
4. [Available worlds](#4-available-worlds)
5. [Scripts and options](#5-scripts-and-options)
6. [MAVLink: ports and how to connect](#6-mavlink-ports-and-how-to-connect)
7. [Sensors on the drone](#7-sensors-on-the-drone)
8. [GPS-denied flight (`--gps-denied`)](#8-gps-denied-flight---gps-denied)
9. [Camera in ROS 2 / RViz2](#9-camera-in-ros-2--rviz2)
10. [Control from Claude (MCP server `mavlink`)](#10-control-from-claude-mcp-server-mavlink)
11. [Performance: CPU vs GPU](#11-performance-cpu-vs-gpu)
12. [Adding worlds and vehicles](#12-adding-worlds-and-vehicles)
13. [Troubleshooting](#13-troubleshooting)
14. [Sources and licenses](#14-sources-and-licenses)

---

## 1. Folder layout

```
ardupilot_sim/
├── run_gazebo.sh          Gazebo (argument: world name)
├── run_sitl.sh            ArduCopter SITL (--gazebo, --gps-denied)
├── run_ros_bridge.sh      D435 camera -> ROS 2 + RViz2 (--points, --no-rviz)
├── stop_sim.sh            stop everything started by the scripts in this folder
├── worlds/                this project's worlds
│   ├── cave_megacavern.sdf    cave (DARPA SubT Finals), GPS-denied
│   └── ego_forest_real.sdf    forest, GPS-denied
├── models/                this project's models
│   ├── iris_with_d435/        Iris + ArduPilot + RealSense D435 (main drone)
│   ├── realsense_d435/        RealSense D435 camera
│   └── ego_forest_real_textures/  forest textures
├── params/
│   ├── sitl_ports.parm    extra MAVLink ports + MAVROS stream rates (always loaded)
│   └── gps_denied.parm    ArduPilot parameters for GPS-denied flight
├── ros/
│   ├── bridge.yaml        ros_gz_bridge: RGB, depth, camera_info, clock
│   ├── bridge_points.yaml ros_gz_bridge: point cloud (optional)
│   └── d435.rviz          RViz2 configuration
├── tools/
│   ├── convert_subt_world.py  builds worlds/cave_megacavern.sdf from the Fuel world
│   └── set_ekf_origin.py      sets the EKF origin (run automatically by --gps-denied)
└── ardupilot_gazebo/      git submodule: ArduPilot/ardupilot_gazebo + plugin build
                           (don't put your own files here)
```

Gazebo searches `worlds/` and `models/` **before** `ardupilot_gazebo/worlds|models`.

## 2. Requirements

| Component | Location / version |
|---|---|
| ArduPilot (SITL) | `~/Projects/ardupilot` (change with the `ARDUPILOT_DIR` env variable), venv `~/venv-ardupilot` |
| Gazebo | Harmonic (`gz sim`, gz-sim 8) |
| ArduPilot Gazebo plugin | `ardupilot_gazebo/build/libArduPilotPlugin.so` (build it, see below) |
| ROS 2 (optional) | Jazzy in `/opt/ros/jazzy`, packages `ros_gz_bridge`, `rviz2` (and `mavros` for MAVROS) |
| GPU | NVIDIA through PRIME offload (automatic); `GZ_NVIDIA=0` for the integrated GPU |

### Clone and build

`ardupilot_gazebo` is a git submodule (the official ArduPilot repository, pinned to one commit):

```bash
git clone --recursive https://github.com/AgungDjalil/ardupilot-simulation-gazebo-harmonic.git ardupilot_sim
cd ardupilot_sim
# cloned without --recursive?  git submodule update --init
```

Build the plugin (once after cloning, and after updating the submodule):

```bash
cd ardupilot_gazebo
GZ_VERSION=harmonic cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
```

Models from Gazebo Fuel (the cave is ~1 GB) are downloaded to `~/.gz/fuel` the first time a world
is opened.

## 3. Running

Each script in its own terminal, in this order. To stop: `./stop_sim.sh`.

### a. Cave, GPS-denied, camera in RViz2

```bash
./run_gazebo.sh cave_megacavern.sdf      # terminal 1, wait for the Gazebo window
./run_sitl.sh --gazebo --gps-denied      # terminal 2
./run_ros_bridge.sh                      # terminal 3 (optional): RViz2
```

### b. Forest, GPS-denied, camera in RViz2

```bash
./run_gazebo.sh ego_forest_real.sdf
./run_sitl.sh --gazebo --gps-denied
./run_ros_bridge.sh
```

### c. Runway with GPS (ardupilot_gazebo's own world)

```bash
./run_gazebo.sh                          # = iris_runway.sdf
./run_sitl.sh --gazebo
```

### d. Without Gazebo (SITL's built-in physics, lightest)

```bash
./run_sitl.sh
```

Once SITL is running, **QGroundControl connects automatically** (UDP 14550). The drone is ready to
fly when the EKF status is green; with `--gps-denied` wait for the line
`set_ekf_origin: EKF origin set ...` in terminal 2.

## 4. Available worlds

| World | Drone | Camera | GPS | Notes |
|---|---|---|---|---|
| `cave_megacavern.sdf` | `iris_with_d435` | D435 | use `--gps-denied` | Louisville Mega Cavern (SubT Finals World 08). Starts in the staging area (x = -12); the cave entrance is ~8 m ahead (+x). Staging area ceiling ~3.8 m. |
| `ego_forest_real.sdf` | `iris_with_d435` | D435 | use `--gps-denied` | Natural forest. Starts at (0, 0). Flight area x 0..26, y -8..8; only the start point and the survey points x ∈ {0, 26}, y ∈ {-8, -4, 0, 4, 8} (radius 1.6 m) are guaranteed free of trees. |
| `iris_runway.sdf` | `iris_with_gimbal` | gimbal camera 640×480 | yes | From ardupilot_gazebo. |
| `iris_warehouse.sdf` | `iris_with_gimbal` | gimbal camera | yes | From ardupilot_gazebo. |
| `zephyr_runway.sdf` | `zephyr_with_ardupilot` (plane) | – | yes | `./run_sitl.sh --gazebo -v ArduPlane -f gazebo-zephyr` (not tested in this project). |

All worlds use SITL's default home location (CMAC, -35.363262, 149.165237, 584 m), the same as the
worlds' `<spherical_coordinates>`.

There is no automatic obstacle avoidance: in the cave and the forest, fly slowly and in short legs.

## 5. Scripts and options

### `run_gazebo.sh [world.sdf]`

| | |
|---|---|
| Default world | `iris_runway.sdf` |
| World/model search path | `worlds/`, `models/`, then `ardupilot_gazebo/worlds`, `ardupilot_gazebo/models` |
| GPU | NVIDIA PRIME offload; `GZ_NVIDIA=0 ./run_gazebo.sh ...` for the integrated GPU |

### `run_sitl.sh [options] [sim_vehicle.py arguments]`

| Option | Effect |
|---|---|
| (none) | ArduCopter with SITL's built-in physics (+ quad), no Gazebo |
| `--gazebo` | `-f gazebo-iris --model JSON`: physics from Gazebo over UDP 9002 |
| `--gps-denied` | GPS off, position from the simulated vision system (ExtNav) on SERIAL5, `params/gps_denied.parm`, EKF origin set automatically |
| `-A "..."` | extra arguments for the SITL binary (merged, they don't replace the script's own) |
| (always) | MAVLink ports as in [section 6](#6-mavlink-ports-and-how-to-connect), `params/sitl_ports.parm` |
| anything else | passed to `sim_vehicle.py`, e.g. `-v ArduPlane -f gazebo-zephyr`, `--add-param-file=...`, `-L location` |

Parameters always start clean (`-w`, eeprom wiped). SITL output: `tail -f /tmp/ArduCopter.log`.

### `run_ros_bridge.sh [--points] [--no-rviz]`

See [section 9](#9-camera-in-ros-2--rviz2).

### `stop_sim.sh`

Stops SITL, Gazebo, the bridge and RViz2 started by this folder's scripts. A Gazebo started by
another project's launch file is left alone.

## 6. MAVLink: ports and how to connect

Vehicle: **system ID 1, component ID 1**, MAVLink 2. Each application has its own port, so they can
all be connected **at the same time**:

| Port | ArduPilot serial | For | How to connect |
|---|---|---|---|
| **UDP 14550** | SERIAL0 (SITL sends to `127.0.0.1:14550`) | **QGroundControl** | automatic, just open QGC |
| **TCP 5762** | SERIAL1 | **MCP server `mavlink`** (Claude) | automatic (`MAVLINK_URL`) |
| **TCP 5763** | SERIAL2 | **MAVROS** | `fcu_url:=tcp://127.0.0.1:5763` |
| **TCP 5766** | SERIAL6 | **Mission Planner** / MAVProxy / scripts | TCP, host `127.0.0.1`, port `5766` |
| (internal) | SERIAL5 (`sim:vicon`) | simulated vision system for `--gps-denied` | – |

- The TCP ports are servers on `0.0.0.0`: they can also be reached from another PC on the network
  (`<this machine's IP>:5766`, e.g. Mission Planner on Windows), if the firewall allows it.
- One TCP port = one client. A second client on the same port has to wait until the first
  disconnects.
- SITL doesn't wait for any connection to start; start the clients (QGC, MAVROS, Mission Planner)
  after SITL is running.
- With `--gps-denied`, `tools/set_ekf_origin.py` uses TCP 5766 for a few seconds at start-up (until
  `EKF origin set`) and then releases it. Connect Mission Planner after that.
- SERIAL6 and the MAVROS stream rates are set in `params/sitl_ports.parm` (always loaded by
  `run_sitl.sh`).

Other internal ports (not MAVLink): UDP 9002 Gazebo ↔ SITL (JSON physics), UDP 5501 SITL RC input,
UDP 9005 IR-Lock simulation.

### QGroundControl

Open QGC; the drone shows up by itself (QGC listens on UDP 14550 by default). Don't add a UDP 5760
comm link — that is not a port of this simulation.

### MAVROS (ROS 2 Jazzy) — TCP 5763

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5763
```

- `/mavros/state` → `connected: true`. The data streams on this port are preset
  (`MAV3_*` in `params/sitl_ports.parm`): `/mavros/local_position/pose` and `/mavros/imu/data` at
  ~20 Hz, `/mavros/battery` at 2 Hz.
- Change the rate at runtime: `ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate
  "{stream_id: 0, message_rate: 20, on_off: true}"`.
- GPS-denied (`--gps-denied`): the local position is still in `/mavros/local_position/*`;
  `global_position` is derived from the EKF origin.
- Don't connect MAVROS to 5762 (used by MCP) or 14550 (used by QGC).

### Mission Planner — TCP 5766

On Linux Mission Planner runs with `mono`, e.g. from `~/MissionPlanner`:

```bash
cd ~/MissionPlanner && mono MissionPlanner.exe
```

In the top right corner pick **TCP** → **Connect** → host `127.0.0.1` → port `5766`.
From another Windows PC: host = this machine's IP (`hostname -I`), port `5766`.

### MAVProxy / pymavlink — TCP 5766

```bash
mavproxy.py --master tcp:127.0.0.1:5766          # from ~/venv-ardupilot
python3 -c "from pymavlink import mavutil; m = mavutil.mavlink_connection('tcp:127.0.0.1:5766'); print(m.wait_heartbeat())"
```

MAVProxy and Mission Planner share 5766; if you need both at once, forward from MAVProxy:
`mavproxy.py --master tcp:127.0.0.1:5766 --out udp:127.0.0.1:14551` and connect Mission Planner over
**UDP** port 14551.

### Adding MAVLink outputs

More ports can be added to SITL with `-A`, e.g. UDP to a companion computer on the network
(SERIAL7; its MAVLink protocol must be enabled too):

```bash
./run_sitl.sh --gazebo -A "--serial7=udpclient:192.168.1.50:14551"
# then set SERIAL7_PROTOCOL = 2 (QGC/MAVProxy/MCP set_param) and reboot,
# or add "SERIAL7_PROTOCOL 2" to params/sitl_ports.parm
```

## 7. Sensors on the drone

The main drone `models/iris_with_d435` = `iris_with_ardupilot` (ardupilot_gazebo) + RealSense D435.
With `--model JSON`, Gazebo sends SITL the IMU data, position, velocity and attitude at every
physics step (1 kHz). The other sensors are simulated by SITL from that physical state.

| Sensor | Simulated by | Details |
|---|---|---|
| **IMU** (gyro + accelerometer) | Gazebo (`imu_sensor`, 1000 Hz) | ArduPilot sees 2 IMUs (`INS_GYR_ID`, `INS_GYR2_ID`) from the same data + SITL noise |
| **Barometer** | SITL | 2 barometers (`BARO1_DEVID`, `BARO2_DEVID`) |
| **Compass** | SITL | 3 magnetometers (`COMPASS_DEV_ID..3`) |
| **GPS** | SITL | GPS1 at 5 Hz (`SIM_GPS1_HZ`). **Off** with `--gps-denied` (`GPS1_TYPE 0`) |
| **Downward rangefinder** | SITL | `RNGFND1_TYPE 1`, facing down, max 50 m (from `gazebo-iris.parm`, for precision landing). Computed from the height above home, **not** from the world geometry, so it is wrong over the cave's uneven floor |
| **Battery** | SITL | `BATT_MONITOR 4`; with Gazebo it stays at 100% |
| **Vision / ExtNav** | SITL (`sim:vicon`) | Only with `--gps-denied`: position, velocity, yaw at 50 Hz + noise. See section 8 |
| **RealSense D435 RGB-D camera** | Gazebo (`rgbd_camera`, rendered on the GPU) | On the drone's nose (x = 0.11 m), looking straight ahead. 640×480 @ 30 Hz, HFOV 60° (fx = fy = 554.25), depth 0.1–10 m, noise σ 0.007. Gazebo topics `d435/image`, `d435/depth_image`, `d435/points`, `d435/camera_info`; frame `d435_link` |

The runway/warehouse worlds use `iris_with_gimbal`: the same sensors except the camera, which is a
3-axis gimbal camera instead (640×480, HFOV 2.0 rad).

## 8. GPS-denied flight (`--gps-denied`)

`params/gps_denied.parm` (the same as `copter-vicon.parm` in ArduPilot's autotest, plus noise):

| Parameter | Value | Meaning |
|---|---|---|
| `GPS1_TYPE`, `SIM_GPS1_ENABLE` | 0 | no GPS |
| `VISO_TYPE` | 1 | visual odometry over MAVLink |
| `SERIAL5_PROTOCOL` | 2 | MAVLink on SERIAL5 (`sim:vicon`) |
| `EK3_SRC1_POSXY/VELXY/POSZ/VELZ/YAW` | 6 | EKF3: everything from External Navigation |
| `SIM_VICON_P_SD` / `SIM_VICON_V_SD` | 0.01 m / 0.02 m/s | vision measurement noise |

The simulated vision system takes the drone's pose from the Gazebo physics; it stands in for a real
VIO/SLAM system (T265, VOXL, RTAB-Map), without drift. Without GPS the EKF has no global origin, so
`tools/set_ekf_origin.py` sends `SET_GPS_GLOBAL_ORIGIN` (the simulator's start position) over
TCP 5766 until it is confirmed; after that QGC shows the drone's position and GUIDED/LOITER work.

Failure tests in flight (QGC → Parameters, MAVProxy `param set`, or MCP `set_param`):

| Parameter | Effect |
|---|---|
| `SIM_VICON_GLIT_X` / `_Y` / `_Z` | the vision position jumps (m) |
| `SIM_VICON_FAIL 1` | vision lost |
| `SIM_VICON_P_SD`, `SIM_VICON_V_SD` | more noise |

## 9. Camera in ROS 2 / RViz2

```bash
./run_ros_bridge.sh              # RGB + depth + RViz2
./run_ros_bridge.sh --points     # + point cloud (heavy, see section 11)
./run_ros_bridge.sh --no-rviz    # bridge only
```

| ROS 2 topic | Type | Notes |
|---|---|---|
| `/d435/image` | `sensor_msgs/Image` (rgb8) | colour 640×480 |
| `/d435/depth_image` | `sensor_msgs/Image` (32FC1) | distance in metres |
| `/d435/camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| `/d435/points` | `sensor_msgs/PointCloud2` | only with `--points` |
| `/clock` | `rosgraph_msgs/Clock` | simulation time (`use_sim_time`) |

Frame `d435_link`: x forward, y left, z up. RViz2 (`ros/d435.rviz`) shows RGB and depth panels;
tick "D435 points" when using `--points`. Only needs `/opt/ros/jazzy` and uses the default RMW.

Topics from another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 run rqt_image_view rqt_image_view /d435/image
```

## 10. Control from Claude (MCP server `mavlink`)

An MCP server (`~/Projects/mavlink_mcp/server.py`, a separate project registered in Claude Code at
user scope) connects to `tcp:127.0.0.1:5762` (env `MAVLINK_URL`). Tools: `status`, `takeoff`,
`goto_local`, `goto_global`, `land`, `rtl`, `set_mode`, `arm`, `disarm`, `get_param`, `set_param`.
Example request: "take off to 1.5 m, fly 8 m east and land".

## 11. Performance: CPU vs GPU

- **Gazebo physics always runs on the CPU** (DART/Bullet have no GPU version). The NVIDIA GPU is
  used for rendering the GUI, the D435 camera and RViz2.
- Cave: DART's default collision detector makes the cave mesh the bottleneck (~0.5× real time);
  `cave_megacavern.sdf` uses `<collision_detector>bullet</collision_detector>` (3–6× real time,
  capped at 1×).
- Point cloud (`--points`): ~300k points per frame are packed on the CPU and sent over DDS; the
  simulation may drop below real time. RGB + depth alone stays at ~1× real time.
- Check the real-time factor: `gz topic -e -t /stats -n 1 | grep real_time_factor`.

## 12. Adding worlds and vehicles

### New world

1. Copy a template, e.g. `ardupilot_gazebo/worlds/iris_runway.sdf` → `worlds/name.sdf`.
2. Change `<world name>` and the contents. Keep the Physics, Sensors (`ogre2`), Imu and NavSat
   plugins, `<spherical_coordinates>` and `max_step_size` 0.001.
3. Add the drone: `<include><uri>model://iris_with_d435</uri><pose>x y 0.195 0 0 yaw</pose></include>`
   on flat ground (otherwise it may tip over when it spawns).
4. `./run_gazebo.sh name.sdf`

Rebuilding the cave world from its source:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Finals Prize Round World 08"
python3 tools/convert_subt_world.py \
  "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/finals prize round world 08/1/final_event_08.sdf" \
  worlds/cave_megacavern.sdf
```

### New vehicle

1. Copy a template to `models/name/`: `ardupilot_gazebo/models/iris_with_ardupilot` (multicopter),
   `zephyr_with_ardupilot` (plane), or combine like `models/iris_with_d435`
   (`<include merge="true">` of the base model + sensor + `fixed` joint).
2. The `<plugin name="ArduPilotPlugin">` block in `model.sdf`:
   - `fdm_port_in` 9002 (a second vehicle 9012 + SITL `-I1`, etc.),
   - `imuName` = the model's IMU sensor,
   - one `<control channel="N">` per motor/servo, in the order of ArduPilot's SERVO(N+1) outputs.
3. SITL frame and parameters:

   ```bash
   ./run_sitl.sh --gazebo -v ArduPlane -f gazebo-zephyr
   ./run_sitl.sh --gazebo --add-param-file=$PWD/params/name.parm
   ```

## 13. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Arm: Gyros inconsistent` for a few seconds after start | normal right after SITL starts; wait ~15 s and arm again |
| `PrecLand: Failsafe Measures` when landing | precision landing is enabled (`gazebo-iris.parm`) but there is no IR-Lock target; safe to ignore |
| QGC doesn't connect | QGC must listen on UDP 14550 (its default). A UDP 5760 comm link is useless (5760 is not a port of this SITL) |
| `run_sitl.sh --gazebo` doesn't start / `No JSON sensor message received` | Gazebo isn't running, or the world has no drone with the ArduPilotPlugin (port 9002) |
| Port 9002 / 5762 already in use | another simulation is still running: `./stop_sim.sh` (or the other project's stop script) |
| `--gps-denied` can't arm in GUIDED | wait for `set_ekf_origin: EKF origin set` in the SITL terminal; TCP 5766 must be free at start-up (connect Mission Planner afterwards) |
| MAVROS `connected: true` but position/IMU topics are empty | the stream rates for port 5763 are set in `params/sitl_ports.parm`; make sure SITL was started with `run_sitl.sh`, or call `/mavros/set_stream_rate` |
| A world fails to load (`Failed to load a world`) | check `[Err]` in the Gazebo output; Fuel models can disappear or become private — see the list of removed models in `tools/convert_subt_world.py` |
| The simulation is slow | see section 11; drop `--points`, close the Gazebo GUI if not needed |

## 14. Sources and licenses

- `worlds/cave_megacavern.sdf`: "Finals Prize Round World 08", Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/OpenRobotics/fuel/worlds/Finals%20Prize%20Round%20World%2008).
  Changed: competition elements, models that can no longer be downloaded and the Fiducial markers
  removed; ArduPilot systems, the Bullet collision detector and the drone added.
- `models/realsense_d435`: "Intel RealSense D435", bmchale / Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/bmchale/fuel/models/Intel%20RealSense%20D435). Changed: mesh rotated
  90° (the original's lenses face -y), sensor pose and topic.
- `worlds/ego_forest_real.sdf`, `models/ego_forest_real_textures`: procedurally generated forest
  (by `gen_real_forest.py` in the author's EGO-Planner workspace); drone replaced with
  `iris_with_d435`.
- `ardupilot_gazebo`: https://github.com/ArduPilot/ardupilot_gazebo (LGPL-3.0).
