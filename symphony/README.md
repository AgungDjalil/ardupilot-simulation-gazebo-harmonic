# Symphony in Gazebo, with gimbal cameras

Symphony's aircraft (G1, G2, ...) flying ArduCopter SITL in Gazebo Harmonic, each with a 3-axis gimbal camera, so
the station can be exercised end to end (arming, flight, gimbal pointing, video, footprint) without a real
aircraft. Nothing in the Symphony repository is changed: the station talks to the simulation exactly as it talks
to the fleet, through MAVLink on the ports its rows carry and the camera URL in `video_rtsp_url`.

```sh
symphony/run.sh start 2          # Baylands, G1/G2 on the front pads of the R23 command vehicle
symphony/run.sh status
symphony/run.sh stop
SYM_SCENE=ground symphony/run.sh start 2   # the earlier layout: on the grass, G2 25 m west
SYM_MODEL=iris symphony/run.sh start 2     # the earlier airframe: Iris with ardupilot_gazebo's small gimbal
```

## The airframe: PX4's x500_gimbal

The aircraft are [PX4's `x500_gimbal`](https://app.gazebosim.org/PX4/fuel/models/x500_gimbal) from Gazebo Fuel:
the Holybro X500 (body, meshes, 2 kg, collisions) with the CGO3 3-axis gimbal camera. `run.sh` downloads it and the
two models it includes (`x500`, `gimbal`) into `~/.gz/fuel` on the first start; the world reads the meshes from
there, so it opens offline afterwards.

PX4 flies it through `MulticopterMotorModel` on PX4's actuator topic, which ArduPilot cannot drive, so
`make_world.py` swaps the PX4 parts for ArduPilot's:

- **Motors**: the ArduPilot plugin spins the rotor joints and `LiftDrag` on each rotor makes the thrust, as on
  ardupilot_gazebo's `iris_with_gimbal` (same gains). PX4's quad X motor order is ArduPilot's, so `FRAME_TYPE 1`
  holds. Hover throttle learned in flight: `MOT_THST_HOVER` ≈ 0.42.
- **IMU**: the Iris' (own link, 1 kHz, rolled 180° for the plugin); the x500's 250 Hz IMU is dropped.
- **Rotors**: the Iris rotor's inertia (izz 1.676e-4) and joint damping 0.003. The x500's own rotor is six times
  lighter than the Iris controller is tuned for; the controller limit-cycled at its torque cap and the propellers
  turned on the ground, disarmed (±12.5 rad/s). The damping sets the top speed (~2.5 N m / damping): 0.003 gives
  thrust/weight ~2.3.
- **Gimbal**: the CGO3's joints take the servo mount's channels: SERVO9 roll → `cgo3_horizontal_arm_joint`,
  SERVO10 pitch → `cgo3_camera_joint`, SERVO11 yaw → `cgo3_vertical_arm_joint`, on `/G<i>/gimbal/cmd_*`, with
  PX4's joint gains. Checked: pitch -90° turns the camera joint to -1.571 rad and the sky leaves the frame.

It stands on a pad with its feet on the deck (model origin 13 mm below the feet). The Iris remains as
`SYM_MODEL=iris`, with the R23 quad's look (`SYM_LOOK`).

## H1: the hexa in the middle bay, with a Livox Avia

The R23's static hexa is replaced by H1, a flying hexa standing on the middle bay's floor (`SYM_HEXA=0` leaves it
out and puts the static one back): [Gambit's Typhoon H480](https://app.gazebosim.org/Gambit/fuel/models/Typhoon%20H480)
from Gazebo Fuel, its CGO3 gimbal removed and a **fixed Livox Avia LiDAR** in its place under the body.

| | |
|---|---|
| MAVLink | `tcp:127.0.0.1:5842` (SERIAL1, for a station row), probe on `tcp:127.0.0.1:5843` |
| SITL | instance `-I 8` (after the eight pads, so its ports do not move with N), `FRAME_CLASS 2` / `FRAME_TYPE 1` (hexa X), `MNT1_TYPE 0` |
| LiDAR | gz topics `/H1/lidar` (LaserScan) and `/H1/lidar/points` (PointCloudPacked: x, y, z, intensity, ring), frame `livox_link` |

- **Airframe**: the Typhoon's nose is its -x (its camera, IMU and propeller handedness agree), so it is included
  turned 180°. Its motors map onto ArduPilot's hexa X (motor 1 right CW … 6 rear-left CW) with the Iris' rotor
  dynamics (LiftDrag, the plugin's gains); the tilted motor axes are set upright and the retractable legs fixed
  down. Its Ogre material scripts (which Harmonic does not read) become plain colours. Flown: GUIDED take-off to 20 m in 30 s, hover
  within ±0.5 m, RTL back onto the bay floor within 1 cm, `MOT_THST_HOVER` ≈ 0.36.
- **Livox Avia**: 91 × 61.2 × 64.8 mm, 498 g, aluminium body with side fins, the teal window, the connector at the
  back. A `gpu_lidar` over its 70.4° × 77.2° field, 160 × 150 rays at 10 Hz = its 240 000 points/s, 1–450 m, 2 cm
  noise. Gazebo has no non-repetitive (rosette) scan, so the pattern is a grid. `SYM_LIDAR_TILT_DEG` sets the
  mount: 90 (default) straight down, 0 straight ahead. From 20 m, nadir: 24 000 returns, 19.1–34.8 m.
- Parked, the LiDAR sees nothing: the bay floor is 0.1 m under it, inside the Avia's 1 m blind zone.
- **Textures**: `texture_h480.py` turns the Typhoon's STLs (no UVs, no parts) into textured OBJ in `models/h480_tex`
  (run.sh does it once): graphite grained-plastic shell, darker belly, black satin arms, brushed dark motor pods,
  dark grey gear, black propellers with a metal hub; parts found by radius from the centre, box-projected UVs.
- **Rotor margin**: the rotors' joint damping is 0.0035 (the model's 0.005 capped them at ~500 rad/s under the
  controller's 2.5 N m: thrust/weight 1.1, and the first survey leg in the park's wind saturated three motors and
  rolled it over). Now ~2.3.

### Mapping with Symphony

`livox_forwarder.py` is H1's LiDAR Pi: it takes the Avia's cloud from `/H1/lidar/points`, the autopilot's pose
(ATTITUDE, LOCAL_POSITION_NED at 50 Hz on SERIAL2, 5843) and serves points in local ENU on
`ws://127.0.0.1:8766`, in Symphony's `gcs/velodyne_forwarder.py` message format, thinned to its 10 000 points/s.
run.sh starts it (`SYM_LIDAR_WS_PORT`, `SYM_LIDAR_LIVE_MAX_PTS`) with this folder's `.venv` (websockets, pymavlink).
The station needs, with no change to Symphony:

1. a vehicle row for H1 (Kelola, or `POST /api/vehicles`): `udp_host` `tcp:127.0.0.1:5842`, copter, and
   `payload_lidar_brand` set (the station maps with the first aircraft that has one);
2. `LIDAR_WS_URL=ws://127.0.0.1:8766` in the station's environment, e.g.
   `LIDAR_WS_URL=ws://127.0.0.1:8766 MAVLINK_ENABLED=1 scripts/dev.sh`.

Tested 2026-11-10: a grid survey from the station (`/api/sar/generate` grid, 100 x 80 m over the R23, 25 m, 25 m
spacing; validate, upload, arm, launch in AUTO), 7 waypoints, RTL into the bay: the station's map took ~750 000
points (0.2 m voxels); `scripts/check-gcs-lidar.mjs` passes (live cloud in the LiDAR tab, OBJ and LAZ export,
saved map on the 2D map and the 3D panel). The forwarder's own cloud is accurate: ground 0.11 m (p10-p90) in 2 m
cells, R23 deck 1.00 m (Gazebo 1.005), canopy roof 1.89 m (1.91). The station's saved map is coarser: 0.40 m, and
points taken while climbing or descending land up to ~15 m low or ~6 m high, because lidar_map.py measures height
from the ground it estimates under the aircraft, and that estimate (the mode of the last 6000 returns within
4 m) trails by ~17 s of returns with a nadir LiDAR at this rate.

## The neighbourhood

`make_town.py` (called by make_world.py; `SYM_TOWN=0` leaves it out) fills the Baylands park with OpenRobotics'
city models from Gazebo Fuel (run.sh downloads them once): about 240 buildings and 190 vehicles, deterministic
(`SYM_TOWN_SEED`).

- Rows of shophouses, shops, houses or civic buildings along the park's paths and roads, fronts 6 m from the
  street's edge, following its curve, cars parked along the kerb (`SYM_TOWN_ROW_BUILDINGS`, default 170).
- More buildings on the open land between, with cars in front (`SYM_TOWN_BUILDINGS`, default 70), and traffic on the
  park's roads and parking lots (`SYM_TOWN_VEHICLES`, default 150 vehicles in all).
- Nothing past the edge of the map (the ground mesh's own extent, less 4 m), on water, paths, trees or the park's
  structures, or on steep ground; nothing within 25 m of the R23, nothing over 12 m tall within 90 m of it.
- Each asset is a static wrapper: the Fuel visuals, textures that Fuel gave only through Ogre material scripts
  re-applied as PBR maps, one box collision from the mesh bounds.

Rendering cost: run.sh renders on the NVIDIA GPU (PRIME offload, `GZ_NVIDIA=0` for the integrated one) and turns the
sun's shadows off with the town (`SYM_SHADOWS=1` keeps them). Measured 2026-11-10 with G1, G2, H1: real-time factor
~1.0 headless (`SYM_HEADLESS=1`), ~0.6–0.8 with the Gazebo GUI open, which renders the whole town as well. On the
integrated GPU it fell to 0.03.

## The vehicle: the N°COMMAND pickup (default)

`models/ncommand_r23`, built by `make_ncommand.py` (run.sh builds it once) from the team's renders
(`mobil_ncommand_semua_sudut`, 2026-11-10): a low-poly double-cab pickup in sage green, a box canopy over the bed
with N°COMMAND on its sides, the side panels down as light grey decks (main section and extension each side), black
bumper and grille with orange indicators, snorkel on the right A-pillar, black running boards, dark six-spoke
wheels. One flat-shaded OBJ, boxes for collisions. Same frame, deck height (1.095 m) and pad centres as the R23
below, so every spawn and every station row stays as it was. H1 stands on the middle of the canopy roof (2.02 m).
`SYM_VEHICLE=hilux` brings back the R23 Hilux (H1 then in its open middle bay).

**ArUco pads, visual only** (nothing detects them in the simulation yet): the fleet's board, rendered by Symphony's
own `gcs/companion/landing_marker_aruco.py` (imported read-only; needs OpenCV ≥ 4.7, hence this folder's
`.venv`): `DICT_ARUCO_MIP_36H12`, six markers of three sizes, the 718 × 526 mm banner (`dock6`, read from 8 m),
textured with explicit UVs so a marker is never mirrored seen from above.

| Pad | Aircraft | ArUco ids |
|---|---|---|
| 1–5 | G1–G5 | 1–6, 11–16, 21–26, 31–36, 41–46 (the fleet's swarm pads) |
| 6 | H1, canopy roof | 51–56 (the fleet's hexacopter pad) |
| 7–9 | G6–G8 | 61–66, 71–76, 81–86 (not printed for the fleet) |

## The R23 scene

The N°COMMAND R23 as on the render on Symphony's landing page (`public/landing/r23-seq`): a Toyota Hilux Double Cab
in olive drab, a darker canopy over the bed with N°COMMAND on its rear face and its roof slid back 1.5 m, side
panels opened flat, the main cassettes slid out 1 m and the outer extensions 0.88 m beyond them, eight pads with
black foot markers on light grey decks, the hexa in its bay, a snorkel. It stands
level on the grass at the Baylands origin, front bumper south (`SYM_R23_POSE`, default `0,0,-0.09,0`; the ground
falls ~2.5° to the west). Geometry from Symphony's `PanggungR23.tsx` (CAD frame: metres, Z up, Y towards the rear,
X to the left).

`run.sh start N` spawns one aircraft per pad, in this order, each facing outwards:

| Aircraft | Pad (vehicle frame) | Start (station base) |
|---|---|---|
| G1 | main left front (+1.42, 4.115) | 37.4122100, -121.9988627 |
| G2 | main right front (-1.42, 4.115) | 37.4122100, -121.9988948 |
| G3, G4 | main left/right rear (±1.42, 4.905) | 37.4122171, -121.9988627 / -121.9988948 |
| G5, G6 | outer left/right front (±2.30, 4.115) | 37.4122100, -121.9988527 / -121.9989047 |
| G7, G8 | outer left/right rear (±2.30, 4.905) | 37.4122171, -121.9988527 / -121.9989047 |

The station has rows for G1 and G2 only. G3..G8 listen on 5782, 5792, ... and serve their cameras on 8603..8608.
With 8 aircraft the world runs at a real-time factor of ~0.3 with 1280x720 cameras and ~0.7 with
`SYM_CAM_W=640 SYM_CAM_H=360` (rendering eight cameras is the cost); with 2 it runs at 1.0.

With `SYM_MODEL=iris` the aircraft fly Iris physics (links, inertia, motors, the ArduPilot plugin) with the R23
quad's look (`SYM_LOOK=iris` for the Iris'); at tilt 0 the top quarter of that frame shows the quad's own arms and
13" propellers, and from -45° down the view is clear. The vehicle's own hexa mesh is scenery, shown only with
`SYM_HEXA=0`; otherwise H1 flies from the bay (below).

The gimbal has no collisions: its meshes hang 2 cm above the feet, inside Bullet's contact margin of a deck, and
testing them every step slowed the world to a real-time factor of 0.04.

Models, rebuilt from Symphony's web models (read only) by:

```sh
node symphony/convert_r23.mjs        # public/models/r23/*.glb -> models/r23_hilux, models/r23_quad (OBJ)
python3 symphony/texture_r23.py      # parts, colours, textures -> *_tex.obj + .mtl, textures/*.png
```

The GLBs are meshopt-compressed, quantized and instanced, which Gazebo cannot read, and carry no parts or UVs: the
parts (Hilux: olive paint, glass, tyres, rims, lights, trim; quad: carbon frame, light grey motors and payload box,
gear, props; hexa: carbon) are found by position, and the textures are procedural. The vehicle collides as
boxes (hood, cab, bed, canopy walls and roof, the four decks): an OBJ mesh as a collision shape crashes DART's
Bullet detector.

## Probing an aircraft while the station is connected

Each SITL TCP port serves one client. The station holds SERIAL1 (5762, 5772, ...); a second client there takes
the link from it. Scripts, MAVProxy or QGroundControl go to SERIAL2 instead: `tcp:127.0.0.1:5763`, 5773, ...

| Aircraft | MAVLink (station `udp_host`) | Camera (`video_rtsp_url`) | Gazebo topics |
|---|---|---|---|
| G1 | `tcp:127.0.0.1:5762` | `http://127.0.0.1:8601/` | `/G1/camera/image`, `/G1/gimbal/cmd_*`, `/G1/camera/zoom` |
| G2 | `tcp:127.0.0.1:5772` | `http://127.0.0.1:8602/` | `/G2/...` |

- **Gimbal**: the x500's CGO3 (`SYM_MODEL=iris`: ardupilot_gazebo's `gimbal_small_3d`), driven by ArduPilot as
  a servo mount (`MNT1_TYPE 1`, SERVO9..11 = roll, pitch, yaw). The station's `DO_MOUNT_CONTROL` / `GIMBAL_MANAGER_PITCHYAW` move it, and
  ArduPilot reports `GIMBAL_DEVICE_ATTITUDE_STATUS`. Range: pitch -135..45, yaw -160..160, roll -30..30.
- **Camera**: the SIYI A8 mini as `gcs/payload.py` models it, 82.56° horizontal at 1280x720, 15 fps
  (`SYM_CAM_*` in `make_world.py`). `camera_bridge.py` serves it as MJPEG over HTTP, plus `/snapshot.jpg`.
- **Zoom** exists only in Gazebo: `gz topic -t /G1/camera/zoom -m gz.msgs.Double -p 'data: 4'`. Nothing maps a
  MAVLink zoom command onto it.

## Differences from the fleet

- On the aircraft the video is RTSP (H.264) from the Pi's MediaMTX at `rtsp://<pi>:8554/main`. Here it is MJPEG
  over HTTP, because no RTSP server and no H.264 encoder that GstCameraPlugin can use (`x264enc`) are installed.
  The station's decode path (`video_registry.py`, OpenCV/FFmpeg) is the same. RTSP-specific behaviour (TCP
  transport, FFmpeg's RTSP latency options, `rtsp_proxy.py`) is not exercised.
- The real gimbal is a SIYI A8 behind the Pi (`gcs/swarm/siyi_gimbal.py`). Here it is an ArduPilot servo mount,
  so anything going through the SIYI protocol or the companion is not covered.
- The pads of one vehicle are 2.8 m apart. The station's SAR area split was written for pads at least 20 m apart
  (Symphony's `scripts/sitl/README.md`); with the R23 it gets what the field will give it.
- The R23 is static: no cassette or canopy motion, and it does not drive.
- The airframe is an X500, not the fleet's quad: a 2 kg class frame with 13" props, close but not its mass,
  inertia or motors. The gimbal is a CGO3 shape carrying the SIYI A8 mini's lens.
- The park's 5 m/s wind blows.
