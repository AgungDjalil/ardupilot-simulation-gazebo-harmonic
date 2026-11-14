# ardupilot-simulation-gazebo-harmonic

One simulation for every algorithm, as close to the real drone as possible: ArduCopter SITL with
Gazebo Harmonic, the **real drone's replica** (Iris + **Intel RealSense D455** mounted as on the
drone), worlds for **GPS-denied** flight, the D455 published in **ROS 2 Jazzy** exactly as the
RealSense driver publishes it on the drone, and control over **MAVLink** (MAVROS, QGroundControl,
Mission Planner, MAVProxy, pymavlink, MCP).

**Principle: the simulation is the drone's hardware.** An algorithm sees what it sees on the drone
(RealSense topics, frames and wall-clock stamps, with the D455's noise; MAVROS to an FC with the real
FC's settings), so the same code runs on the drone unchanged afterwards. Testing a new algorithm =
run this simulation and the algorithm; no simulation of its own.

The core needs no ROS workspace (ROS 2 from `/opt/ros/jazzy` only for the sensor bridge). `ego_ws/`
holds the simulation-only scripts for the drone's own onboard stack (`~/Projects/ego_planner_ws`).

---

## Contents

1. [Folder layout](#1-folder-layout)
2. [Requirements](#2-requirements)
3. [Running](#3-running)
4. [Worlds](#4-worlds)
5. [Scripts and options](#5-scripts-and-options)
6. [MAVLink: ports and how to connect](#6-mavlink-ports-and-how-to-connect)
7. [The drone and its sensors](#7-the-drone-and-its-sensors)
8. [Navigation profiles and the real FC](#8-navigation-profiles-and-the-real-fc)
9. [Sensors in ROS 2 (the simulated D455)](#9-sensors-in-ros-2-the-simulated-d455)
10. [Testing an algorithm](#10-testing-an-algorithm)
11. [The drone's onboard stack (`ego_ws/`)](#11-the-drones-onboard-stack-ego_ws)
12. [Control from Claude (MCP server `mavlink`)](#12-control-from-claude-mcp-server-mavlink)
13. [Performance](#13-performance)
14. [Adding worlds](#14-adding-worlds)
15. [Troubleshooting](#15-troubleshooting)
16. [Sources and licenses](#16-sources-and-licenses)

---

## 1. Folder layout

```
ardupilot_sim/
├── run_gazebo.sh          Gazebo + the drone (world, --pose, --wind, --infra, --headless)
├── run_sitl.sh            ArduCopter SITL (--gazebo, --nav gps|vicon|extnav, --fc truf)
├── run_ros_bridge.sh      the D455 (+ ground truth) in ROS 2 (--net local|lan, --ideal, --no-rviz)
├── stop_sim.sh            stop everything started by the scripts in this folder
├── worlds/                worlds, without a drone (spawn point: <frame name="ardupilot_sim_spawn">)
│   ├── baylands.sdf           Baylands park (Sunnyvale, CA), GPS, with wind
│   ├── cave_megacavern.sdf    cave (DARPA SubT Finals), GPS-denied
│   ├── ego_forest_real.sdf    natural forest, GPS-denied
│   ├── ego_forest.sdf         textured pillar forest
│   └── ego_forest_plain.sdf   untextured pillar forest
├── models/
│   ├── truf_d455/             THE drone: Iris + ArduPilot + RealSense D455 (the real drone's replica)
│   ├── baylands_park/         Fuel's Baylands model with a Harmonic-compatible tree material
│   ├── ego_textures/, ego_forest_real_textures/   forest textures
├── params/
│   ├── sitl_ports.parm    extra MAVLink ports + default MAVROS stream rates (always loaded)
│   ├── nav_vicon.parm     --nav vicon: position from SITL's simulated vision system
│   ├── nav_extnav.parm    --nav extnav: position from the companion's visual odometry (MAVROS)
│   └── fc_truf.parm       --fc truf: the real drone's FC settings
├── ros/
│   ├── sim_camera.py      the simulated D455 (RealSense topics, noise, wall-clock stamps, /sim/truth)
│   ├── world_map.py       ground-truth obstacle cloud /sim/world_cloud
│   ├── d455.rviz          RViz2: colour, depth, ground truth
│   └── cyclonedds_local.xml / cyclonedds_lan.xml   DDS for an algorithm on this laptop / on the Pi
├── tools/
│   ├── make_world.py          world + drone + wind -> /tmp/ardupilot_sim (called by run_gazebo.sh)
│   ├── check_params.py        are all parameters of a .parm file really set in SITL?
│   ├── set_ekf_origin.py      EKF origin for --nav vicon (started by run_sitl.sh)
│   ├── convert_subt_world.py  builds worlds/cave_megacavern.sdf from the Fuel world
│   └── gen_real_forest.py, gen_textured_world.py   build the forest worlds
├── ego_ws/                the drone's onboard stack on the simulation (section 11)
└── ardupilot_gazebo/      git submodule: ArduPilot/ardupilot_gazebo + plugin build
                           (don't put your own files here)
```

Gazebo searches `/tmp/ardupilot_sim/models` (the drone copy `run_gazebo.sh` writes), `worlds/`,
`models/`, then `ardupilot_gazebo/worlds|models`.

## 2. Requirements

| Component | Location / version |
|---|---|
| ArduPilot (SITL) | `~/Projects/ardupilot` (change with `ARDUPILOT_DIR`), venv `~/venv-ardupilot`; tested with 4.7 |
| Gazebo | Harmonic (`gz sim`, gz-sim 8) |
| ArduPilot Gazebo plugin | `ardupilot_gazebo/build/libArduPilotPlugin.so` (build it, see below) |
| ROS 2 | Jazzy in `/opt/ros/jazzy`: `ros_gz_bridge`, `rviz2`, `tf2_ros`, Python `numpy` (sensor bridge); `mavros` for MAVROS |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`), socket buffers: `/etc/sysctl.d/60-ros2-dds-buffers.conf` (rmem_max/wmem_max 64 MiB) |
| GPU | NVIDIA through PRIME offload (automatic); `GZ_NVIDIA=0` for the integrated GPU |

### Clone and build

`ardupilot_gazebo` is a git submodule (the official ArduPilot repository, pinned to one commit):

```bash
git clone --recursive https://github.com/AgungDjalil/ardupilot-simulation-gazebo-harmonic.git ardupilot_sim
cd ardupilot_sim
# cloned without --recursive?  git submodule update --init
cd ardupilot_gazebo
GZ_VERSION=harmonic cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
```

Models from Gazebo Fuel (the cave is ~1 GB) are downloaded to `~/.gz/fuel` the first time a world
is opened.

## 3. Running

Each script in its own terminal, in this order. To stop: `./stop_sim.sh`.

```bash
./run_gazebo.sh ego_forest_real.sdf             # 1. world + drone (wait for Gazebo)
./run_sitl.sh --gazebo --nav extnav --fc truf   # 2. the FC, set up as the real one
./run_ros_bridge.sh                             # 3. the D455 in ROS 2 (+ RViz2)
# 4. the algorithm under test (section 10), e.g. the drone's own stack (section 11)
```

Other combinations:

```bash
./run_gazebo.sh cave_megacavern.sdf --infra             # cave, with the D455's infrared pair (VIO)
./run_sitl.sh --gazebo --nav vicon                      # fly without an algorithm: perfect vision
./run_gazebo.sh ego_forest.sdf --wind 4,90              # 4 m/s wind towards +y, gusts +-20 %
./run_gazebo.sh baylands.sdf                            # with GPS (and the park's own 5 m/s wind):
./run_sitl.sh --gazebo -l 37.4121731,-121.9988787,0,0   #   SITL home = the park
./run_gazebo.sh                                         # ardupilot_gazebo's iris_runway.sdf
./run_sitl.sh                                           # no Gazebo: SITL's built-in physics
```

QGroundControl connects by itself (UDP 14550). With `--nav vicon` wait for
`set_ekf_origin: EKF origin set ...`; with `--nav extnav` the EKF has a position only once the
algorithm sends its visual odometry (and sets the EKF origin).

## 4. Worlds

Worlds carry no drone: `run_gazebo.sh` (`tools/make_world.py`) adds `models/truf_d455` (model name
`iris`) at the world's spawn point, `<frame name="ardupilot_sim_spawn">`, or at the pose of an iris a
world brings itself (ardupilot_gazebo's worlds, which is removed). `--pose x,y,z,yaw` overrides it.

| World | GPS | Spawn | Notes |
|---|---|---|---|
| `ego_forest_real.sdf` | `--nav extnav` / `vicon` | (0, 0) | Natural forest. Flight area x 0..26, y -8..8; the start and the survey points x ∈ {0, 26}, y ∈ {-8, -4, 0, 4, 8} (radius 1.6 m) are free of trees. |
| `ego_forest.sdf` | `--nav extnav` / `vicon` | (0, 0) | Pillar forest, textured for visual odometry (pillars x 4..22, y -7..7). |
| `ego_forest_plain.sdf` | `--nav extnav` / `vicon` | (0, 0) | The same pillars, untextured (visual odometry fails there). |
| `cave_megacavern.sdf` | `--nav extnav` / `vicon` | (-12, 0) | Louisville Mega Cavern (SubT Finals World 08). Staging area ceiling ~3.8 m; the cave entrance ~8 m ahead (+x). |
| `baylands.sdf` | yes, `-l 37.4121731,-121.9988787,0,0` | (0, 0) | Baylands Park, Sunnyvale CA. **Wind** 5 m/s + gusts. Open grass at the origin; trees south of y = -2. |
| `iris_runway.sdf`, `iris_warehouse.sdf` | yes | the world's iris | From ardupilot_gazebo; their own iris is replaced by `truf_d455`. |

All worlds except Baylands use SITL's default home (CMAC, -35.363262, 149.165237, 584 m), the same
as their `<spherical_coordinates>`.

### Wind

`--wind speed[,direction[,gust]]`: `speed` m/s blowing towards `direction` degrees (0 = +x, 90 = +y),
gusts of ±`gust` % (default 20) over 10 s, direction ±10° over 30 s, plus noise; WindEffects strength
0.3 (measured on the hovering iris: ~2.6° tilt at 5 m/s, ~5° at 10 m/s). It replaces a world's own
wind (`--wind 0`: calm Baylands). The drone feels it (`<enable_wind>`). Change it while running:

```bash
gz topic -t /world/<world name>/wind -m gz.msgs.Wind -p 'linear_velocity: {x: 0, y: 10, z: 0}, enable_wind: true'
```

SITL's own `SIM_WIND_*` parameters don't apply with Gazebo physics.

## 5. Scripts and options

### `run_gazebo.sh [world.sdf] [--pose x,y,z,yaw] [--wind s[,dir[,gust]]] [--infra] [--headless]`

| | |
|---|---|
| Default world | `iris_runway.sdf` |
| `--infra` | also render the D455's infrared stereo pair (costs GPU time) |
| `--headless` | no GUI (server, off-screen rendering) |
| Writes | `/tmp/ardupilot_sim/worlds/<world>.sdf`, `/tmp/ardupilot_sim/models/truf_d455/`, `/tmp/ardupilot_sim/current.json` (read by `run_ros_bridge.sh`) |
| GPU | NVIDIA PRIME offload; `GZ_NVIDIA=0 ./run_gazebo.sh ...` for the integrated GPU |

### `run_sitl.sh [options] [sim_vehicle.py arguments]`

| Option | Effect |
|---|---|
| (none) | ArduCopter with SITL's built-in physics (+ quad), no Gazebo |
| `--gazebo` | `-f gazebo-iris --model JSON`: physics from Gazebo over UDP 9002 |
| `--nav gps` | GPS navigation (default) |
| `--nav vicon` (`--gps-denied`) | no GPS; position from SITL's simulated vision system, `params/nav_vicon.parm`, EKF origin set automatically |
| `--nav extnav` | no GPS; position from the companion's visual odometry over MAVROS (TCP 5763), `params/nav_extnav.parm` |
| `--fc truf` | the real drone's FC settings, `params/fc_truf.parm` (section 8) |
| `-A "..."` | extra arguments for the SITL binary (merged with the script's own) |
| (always) | MAVLink ports as in [section 6](#6-mavlink-ports-and-how-to-connect), `params/sitl_ports.parm` |
| anything else | passed to `sim_vehicle.py`, e.g. `--add-param-file=...`, `-l location` |

Parameters always start clean (`-w`). SITL output: `tail -f /tmp/ArduCopter.log`.
**SITL ignores unknown parameter names without a word**, and ArduPilot renames parameters between
versions; after changing a `.parm` file check it against the running SITL:

```bash
python3 tools/check_params.py params/fc_truf.parm      # ~/venv-ardupilot (pymavlink), TCP 5766
```

### `run_ros_bridge.sh [--net local|lan] [--domain N] [--ideal] [--no-rviz]`

See [section 9](#9-sensors-in-ros-2-the-simulated-d455).

### `stop_sim.sh`

Stops SITL, Gazebo, the sensor bridge and RViz2 started by this folder's scripts. The algorithm under
test is stopped by its own script (e.g. `ego_ws/stop_onboard_local.sh`).

## 6. MAVLink: ports and how to connect

Vehicle: **system ID 1, component ID 1**, MAVLink 2 (the real FC is 4: nothing meant for the
simulation can command it). Each application has its own port, all usable **at the same time**:

| Port | ArduPilot serial | For | How to connect |
|---|---|---|---|
| **UDP 14550** | SERIAL0 (SITL sends to `127.0.0.1:14550`) | **QGroundControl** | automatic, just open QGC |
| **TCP 5762** | SERIAL1 | **MCP server `mavlink`** (Claude) | automatic (`MAVLINK_URL`) |
| **TCP 5763** | SERIAL2 | **MAVROS** (the algorithm's FC link) | `fcu_url:=tcp://127.0.0.1:5763` (from the Pi: `tcp://192.168.50.1:5763`) |
| **TCP 5766** | SERIAL6 | **Mission Planner** / MAVProxy / scripts | TCP, host `127.0.0.1`, port `5766` |
| (internal) | SERIAL5 (`sim:vicon`) | simulated vision system for `--nav vicon` | – |

- The TCP ports are servers on `0.0.0.0`: reachable from the Pi or another PC on the network.
- One TCP port = one client.
- SITL doesn't wait for any connection to start.
- With `--nav vicon`, `tools/set_ekf_origin.py` uses TCP 5766 for a few seconds at start-up and then
  releases it. Connect Mission Planner after that.
- Stream rates on the MAVROS port: `MAV3_*` in `params/sitl_ports.parm` (~20 Hz), with `--fc truf`
  the real FC's (10 Hz position/IMU/attitude, 2 Hz status/battery). ArduPilot 4.7 names them `MAV3_*`
  (4.6: `SR2_*`).

Other internal ports (not MAVLink): UDP 9002 Gazebo ↔ SITL (JSON physics), UDP 5501 SITL RC input,
UDP 9005 IR-Lock simulation.

### MAVROS (ROS 2 Jazzy) — TCP 5763

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5763
```

Change a stream rate at runtime: `ros2 service call /mavros/set_stream_rate
mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 20, on_off: true}"`. Don't connect MAVROS
to 5762 (MCP) or 14550 (QGC).

### Mission Planner — TCP 5766

`cd ~/MissionPlanner && mono MissionPlanner.exe`, top right **TCP** → **Connect** → `127.0.0.1` →
`5766`. From another PC: host = this machine's IP.

### MAVProxy / pymavlink — TCP 5766

```bash
mavproxy.py --master tcp:127.0.0.1:5766          # from ~/venv-ardupilot
python3 -c "from pymavlink import mavutil; m = mavutil.mavlink_connection('tcp:127.0.0.1:5766'); print(m.wait_heartbeat())"
```

More ports: `./run_sitl.sh --gazebo -A "--serial7=udpclient:192.168.1.50:14551"` plus
`SERIAL7_PROTOCOL 2` (in a `.parm` file or set and reboot).

## 7. The drone and its sensors

`models/truf_d455`, the only drone: ardupilot_gazebo's `iris_with_ardupilot` with `iris_with_standoffs`
merged in (so `<enable_wind>` reaches the body), plus the D455 and a ground-truth odometry publisher.
The iris hovers at ~39% throttle, the real drone at 40%. With `--model JSON`, Gazebo sends SITL the
IMU, position, velocity and attitude at every physics step (1 kHz).

| Sensor | Simulated by | Details |
|---|---|---|
| **FC IMU** | Gazebo (`imu_sensor`, 1 kHz) | ArduPilot sees 2 IMUs from it + SITL noise |
| **Barometer, compass** | SITL | 2 barometers, 3 magnetometers |
| **GPS** | SITL | 5 Hz; **off** with `--nav vicon/extnav` |
| **Downward rangefinder** | SITL | from `gazebo-iris.parm`; from the height above home, not the world geometry |
| **Battery** | SITL | 100% unless `--fc truf` (4S 2.9 Ah, discharges, failsafes at 14.4 / 14.0 V) |
| **Vision / ExtNav** | SITL (`sim:vicon`) or your algorithm | `--nav vicon` / `--nav extnav`, section 8 |
| **RealSense D455** | Gazebo (rendered on the GPU) + `ros/sim_camera.py` | Mounted as on the real drone: pitched 22.9° down, rolled -2.5°, 2 cm above the FC; 15 cm forward (real 9 cm: the iris body reaches 14.5 cm). Colour + aligned depth 640×480 @ 15 Hz, HFOV 90°; infrared pair 848×480, HFOV 87°, baseline 95 mm (`--infra`); the D455 IMU from the FC IMU, rotated into the camera. Section 9 |

The mount pose is set in one place, `models/truf_d455/model.sdf` (`d455_link`); `sim_camera.py`
reads it, and the drone's own TF must match (the onboard stack passes `cam_x:=0.15` in simulation).

## 8. Navigation profiles and the real FC

### `--nav vicon` (`params/nav_vicon.parm`)

SITL's simulated vision system takes the pose from the Gazebo physics: GPS-denied flight without an
algorithm (no drift). `GPS1_TYPE 0`, `VISO_TYPE 1`, `SERIAL5_PROTOCOL 2`, `EK3_SRC1_* 6`, noise
`SIM_VICON_P_SD 0.01` / `SIM_VICON_V_SD 0.02`. `tools/set_ekf_origin.py` sets the EKF origin.
Failure tests: `SIM_VICON_GLIT_X/Y/Z` (position jump, m), `SIM_VICON_FAIL 1` (vision lost).

### `--nav extnav` (`params/nav_extnav.parm`)

As the real drone: the position comes only from **your** visual odometry / SLAM, sent over MAVROS
(`/mavros/vision_pose/pose` → `VISION_POSITION_ESTIMATE`). `GPS1_TYPE 0`, `VISO_TYPE 1`,
`EK3_SRC1_POSXY/POSZ/YAW 6`, `VELXY/VELZ 0` (velocity derived), `VISO_DELAY_MS 80`. The algorithm must
also set the EKF origin (`/mavros/global_position/set_gp_origin`), as the drone's `vo_to_mavros`
does; without it the fence (with `--fc truf`) refuses to arm: "Fence requires position".

### `--fc truf` (`params/fc_truf.parm`)

The real drone's FC (MicoAir743v2, ArduCopter 4.6, quad X, 4S), copied from
`ego_planner_ws/fc_params/after_20261019_1613.parm`, translated to ArduPilot 4.7 names where they
changed:

| Group | Settings |
|---|---|
| Battery | 4S (`SIM_BATT_VOLTAGE 16.8`, `SIM_BATT_CAP_AH 2.9`, `MOT_BAT_VOLT_MAX/MIN 17.4/13.2`), low 14.4 V / critical 14.0 V → land, arming ≥ 14.7 V |
| Speeds | `ATC_ANGLE_MAX 30`, `WP_SPD 20`, `WP_SPD_UP/DN 3`, `WP_ACC 3.5`, `LOIT_SPEED_MS 12.5`, `PILOT_SPD_UP 2.5`, `LAND_SPD_MS 0.5`, `RTL_ALT_M 33` |
| Failsafes | `FS_EKF_ACTION 1` (land) at 0.8, `FS_GCS_ENABLE 1`, `FS_THR_ENABLE 0` |
| Fence | circle + altitude + polygon (`FENCE_TYPE 7`), radius 100 m, 80 m high, RTL/land |
| Arming | only GPS lock, board voltage and battery checked (FC `ARMING_CHECK 392` = 4.7 `ARMING_SKIPCHK 2096758`) |
| Vision | `VISO_DELAY_MS 110` (measured on the drone's Pi) |
| Telemetry | MAVROS port rates = the FC's port to the Pi |

Not copied: PIDs and filters (tuned for the real frame; the iris has its own dynamics), sensor
calibrations, serial/RC setup, the system ID.

## 9. Sensors in ROS 2 (the simulated D455)

```bash
./run_ros_bridge.sh                 # algorithm on this laptop: loopback DDS, ROS domain 0, RViz2
./run_ros_bridge.sh --net lan       # algorithm on the drone's Pi (LAN cable, 192.168.50.x)
./run_ros_bridge.sh --ideal         # exact depth and IMU (no D455 noise / holes / bias)
./run_ros_bridge.sh --no-rviz
```

What realsense2_camera publishes on the drone (align_depth, unite_imu_method 2), with wall-clock
stamps:

| ROS 2 topic | Type | Notes |
|---|---|---|
| `/camera/camera/color/image_raw`, `.../color/camera_info` | rgb8 640×480 @ 15 Hz | `camera_color_optical_frame` |
| `/camera/camera/aligned_depth_to_color/image_raw`, `.../camera_info` | 16UC1 mm (0 = none) | same frame and stamp as the colour image |
| `/camera/camera/imu` | `sensor_msgs/Imu` 200 Hz | gyro + accel, no orientation, `camera_imu_optical_frame` |
| `/camera/camera/infra1/image_rect_raw`, `infra2/...`, `camera_info` | mono8 848×480 @ 15 Hz | with `run_gazebo.sh --infra`; infra2 `P[3] = -fx·0.095` |
| TF `camera_link` → `camera_*_optical_frame` | static | `camera_link` itself is placed by the algorithm's TF (on the drone: its camera mount) |
| `/sim/truth` (+ TF `sim_origin` → `sim_truth`) | `nav_msgs/Odometry` 15 Hz | **simulation only**: the true body pose relative to the spawn point (x forward at spawn) |
| `/sim/world_cloud` | `PointCloud2` 1 Hz | **simulation only**: the world's obstacles (primitive shapes, not meshes) in `sim_origin` |

The D455's errors (`ros/sim_camera.py` parameters; off with `--ideal`):

| | Value |
|---|---|
| Depth noise | σ = 0.0025 · z² (measured on the drone's D455: 7.6 cm at 5–6 m; simulated 22 mm at 3 m, 43 mm at 4 m) |
| Depth holes | below 0.45 m, beyond 10 m, 2 % random dropout, both sides of depth jumps > 4 % (flying pixels) |
| IMU | gyro 200 Hz, accel 63 Hz (held), noise 0.0035 rad/s / 0.021 m/s² per sample, constant bias per run (σ 0.2°/s, 0.05 m/s²), accel scale 0.899: the drone's uncalibrated D455 reads \|g\| = 8.82 (`accel_scale:=1.0` after `rs-imu-calibration`) |
| Stamps | wall clock (sim time + an offset tracked from the IMU, per sim time): images keep their capture time, the render + network delay is camera latency (~50 ms) |

RViz2 (`ros/d455.rviz`, fixed frame `sim_origin`): colour, depth, the true pose and the world's
obstacles. The algorithm's terminals need the same DDS settings (section 10).

## 10. Testing an algorithm

1. Start the simulation (section 3) with the navigation profile the algorithm needs: `--nav extnav`
   if it estimates the position itself (VIO/SLAM), `--nav vicon` if it only plans/controls,
   `--nav gps` outdoors.
2. In the algorithm's terminal, the same DDS as the bridge:

   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
   export CYCLONEDDS_URI=file://$HOME/Projects/ardupilot_sim/ros/cyclonedds_local.xml
   ```

3. The algorithm subscribes to the RealSense topics (`/camera/camera/...`) and talks to the FC with
   MAVROS on `tcp://127.0.0.1:5763` (system ID 1). Nothing simulation-specific: on the drone it gets
   the same topics from the real D455 and its FC over MAVROS.
4. Measure it against `/sim/truth` and `/sim/world_cloud` (never feed them to the algorithm).

## 11. The drone's onboard stack (`ego_ws/`)

The drone's own stack (`~/Projects/ego_planner_ws`, launch `drone_onboard.launch.py`: RTAB-Map,
MAVROS, `vo_to_mavros`, EGO-Planner + bridge) flies the simulation unchanged; only the camera driver
is off (`camera:=false`), the FC is SITL (sysid 1, TCP 5763) and the camera is 15 cm forward
(`cam_x:=0.15`).

```bash
./run_gazebo.sh ego_forest_real.sdf
./run_sitl.sh --gazebo --nav extnav --fc truf
# a) on the drone's Raspberry Pi (its CPU), LAN cable plugged in:
./run_ros_bridge.sh --net lan
ssh truf-agen@192.168.50.2 '~/drone/run_onboard.sh --sim'
# b) or on this laptop:
./run_ros_bridge.sh
ego_ws/run_onboard_local.sh                      # stop: ego_ws/stop_onboard_local.sh
# then exactly as with the real drone, from ~/Projects/ego_planner_ws:
./drone_cmd.sh preflight                         # all OK before takeoff
./drone_cmd.sh takeoff 1.0                       # forward 3 / turn 90 / land
./run_drone_log.sh                               # start before takeoff: shows "SIM VO error"
./run_drone_monitor.sh --rviz
```

`drone_env.sh` (`drone_cmd`, `drone_log`, monitor) picks the link: local while
`run_onboard_local.sh` runs, else the LAN cable, else WiFi.

```
ego_ws/
├── run_onboard_local.sh     the onboard stack on this laptop (loopback DDS)
├── stop_onboard_local.sh    stop it (also nodes a failed launch left behind)
├── export_rtabmap_map.sh    the stack's RTAB-Map database (save:=true) -> .ply
├── build.sh                 colcon build of src/ on top of ego_planner_ws (build that one first)
└── src/ego_ardupilot_sim/   survey: lawnmower waypoints through EGO-Planner, to map a whole world
```

Rules, as on the real drone:
- **Restarting the onboard stack = rebooting the FC**: restart SITL first. The EKF keeps the previous
  visual odometry frame (without a magnetometer the VO's yaw is arbitrary), and `preflight` fails
  with "EKF = visual odometry frame: ... deg apart". `takeoff` doesn't wait for `preflight`.
- **Restarting SITL pauses Gazebo** (lockstep): for the drone a camera dropout, EGO-Planner latches
  "Depth Lost" for good. Restart the onboard stack after it.
- Visual odometry silent for > 0.5 s in flight → the bridge lands (failsafe).

`survey` (in the onboard stack's ROS domain 0, after takeoff):
`ros2 run ego_ardupilot_sim survey` with `ego_ws/install` sourced and the DDS of section 10.

## 12. Control from Claude (MCP server `mavlink`)

An MCP server (`~/Projects/mavlink_mcp/server.py`, registered in Claude Code at user scope) connects
to `tcp:127.0.0.1:5762` (env `MAVLINK_URL`). Tools: `status`, `takeoff`, `goto_local`, `goto_global`,
`land`, `rtl`, `set_mode`, `arm`, `disarm`, `get_param`, `set_param`. For the simulation only.

## 13. Performance

- Gazebo physics runs on the CPU; the NVIDIA GPU renders the GUI, the D455 and RViz2.
- **The drone's timing is only realistic at real time (RTF ≈ 1)**: the FC (SITL) runs on sim time, the
  companion on the wall clock. Check: `gz topic -e -t /stats -n 1 | grep real_time_factor`. Other heavy
  programs on the laptop (e.g. a VIO benchmark) push it below 1; `--headless` and no `--infra` help.
- The bridge sends raw images (colour 0.9 MB + depth 0.6 MB per frame, to each reader): ~55 MB/s to
  the Pi over the 1 Gbit/s cable.
- The cave uses the Bullet collision detector (DART's default made the cave mesh ~0.5× real time).

## 14. Adding worlds

1. Copy a template, e.g. `ardupilot_gazebo/worlds/iris_runway.sdf` → `worlds/name.sdf`.
2. Change `<world name>` and the contents. Keep the Physics, Sensors (`ogre2`), Imu and NavSat
   plugins, `<spherical_coordinates>` and `max_step_size` 0.001. Remove its drone.
3. Spawn point on flat ground (else the drone may tip over):
   `<frame name="ardupilot_sim_spawn"><pose degrees="true">x y 0.195 0 0 yaw</pose></frame>`.
4. `./run_gazebo.sh name.sdf`

Rebuilding the cave world from its source:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Finals Prize Round World 08"
python3 tools/convert_subt_world.py \
  "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/finals prize round world 08/1/final_event_08.sdf" \
  worlds/cave_megacavern.sdf
```

The forests: `python3 tools/gen_textured_world.py` (ego_forest.sdf from ego_forest_plain.sdf),
`python3 tools/gen_real_forest.py` (ego_forest_real.sdf).

## 15. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| A profile's parameter seems to do nothing | unknown name, ignored silently (ArduPilot renamed it): `python3 tools/check_params.py params/*.parm` |
| `Arm: Fence requires position` | no EKF origin: with `--nav extnav` the algorithm must send it (`set_gp_origin`) |
| `Crash: Disarming: AngErr=...` right after arming | the drone tipped over in Gazebo (e.g. landed on a log): restart Gazebo (`./stop_sim.sh`) |
| `preflight`: EKF and visual odometry frames degrees apart | the onboard stack was restarted without SITL: restart SITL, then the stack |
| EGO-Planner "Depth Lost! EMERGENCY_STOP" for good | the camera stalled (SITL restarted, RTF far below 1): restart the onboard stack |
| Depth all zeros / Gazebo depth `-inf` | something in front of the camera within 0.1 m (a camera inside the iris body or its own housing) |
| Takeoff fails with `Terrain: clamping offset ...` | SITL's home altitude doesn't match the ground: Baylands needs `-l 37.4121731,-121.9988787,0,0` |
| `Arm: Gyros inconsistent` for a few seconds after start | normal right after SITL starts; wait ~15 s |
| QGC doesn't connect | QGC must listen on UDP 14550 (its default) |
| `No JSON sensor message received` | Gazebo isn't running (start `run_gazebo.sh` first) |
| Port 9002 / 5763 already in use | another simulation is still running: `./stop_sim.sh` |
| MAVROS `connected: true` but position/IMU empty | stream rates on 5763 come from `sitl_ports.parm` / `fc_truf.parm`: start SITL with `run_sitl.sh` |
| The algorithm sees no camera topics | different DDS settings than `run_ros_bridge.sh` (domain, `CYCLONEDDS_URI`, section 10) |

## 16. Sources and licenses

- `worlds/baylands.sdf`, `models/baylands_park`: "Baylands" model by Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/OpenRobotics/fuel/models/Baylands, version 3). The meshes and textures
  are loaded from Fuel; the wrapper only replaces the trees' Ogre 1 material script with a PBR material.
- `models/truf_d455/model.sdf`: generated from ardupilot_gazebo's `models/iris_with_ardupilot/model.sdf`
  (LGPL-3.0).
- `worlds/cave_megacavern.sdf`: "Finals Prize Round World 08", Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/OpenRobotics/fuel/worlds/Finals%20Prize%20Round%20World%2008).
  Changed: competition elements, models that can no longer be downloaded and the Fiducial markers
  removed; ArduPilot systems, the Bullet collision detector and the spawn point added.
- `worlds/ego_forest*.sdf`, `models/ego_textures`, `models/ego_forest_real_textures`: procedurally
  generated (`tools/gen_*.py`).
- `ardupilot_gazebo`: https://github.com/ArduPilot/ardupilot_gazebo (LGPL-3.0).
