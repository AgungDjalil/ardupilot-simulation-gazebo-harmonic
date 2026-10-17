# ardupilot_sim — Simulasi ArduPilot SITL + Gazebo

Project khusus simulasi: ArduCopter SITL dengan Gazebo Harmonic, world gua dan hutan untuk terbang
**tanpa GPS**, drone dengan kamera **Intel RealSense D435**, bridge kamera ke **ROS 2 Jazzy / RViz2**,
dan kontrol lewat **MAVLink** (QGroundControl, Mission Planner, MAVProxy, MAVROS, pymavlink, MCP).

Tidak bergantung pada `~/Projects/ego_planner_ws`. Simulasi inti tidak butuh ROS; ROS 2 hanya
dipakai oleh `run_ros_bridge.sh`.

---

## Daftar isi

1. [Struktur folder](#1-struktur-folder)
2. [Prasyarat](#2-prasyarat)
3. [Cara menjalankan](#3-cara-menjalankan)
4. [World yang tersedia](#4-world-yang-tersedia)
5. [Skrip dan opsinya](#5-skrip-dan-opsinya)
6. [MAVLink: port dan cara terhubung](#6-mavlink-port-dan-cara-terhubung)
7. [Sensor pada drone](#7-sensor-pada-drone)
8. [Terbang tanpa GPS (`--gps-denied`)](#8-terbang-tanpa-gps---gps-denied)
9. [Kamera di ROS 2 / RViz2](#9-kamera-di-ros-2--rviz2)
10. [Kontrol lewat Claude (MCP server `mavlink`)](#10-kontrol-lewat-claude-mcp-server-mavlink)
11. [Performa: CPU vs GPU](#11-performa-cpu-vs-gpu)
12. [Menambah world dan vehicle](#12-menambah-world-dan-vehicle)
13. [Troubleshooting](#13-troubleshooting)
14. [Sumber dan lisensi](#14-sumber-dan-lisensi)

---

## 1. Struktur folder

```
ardupilot_sim/
├── run_gazebo.sh          Gazebo (argumen: nama world)
├── run_sitl.sh            ArduCopter SITL (--gazebo, --gps-denied)
├── run_ros_bridge.sh      kamera D435 -> ROS 2 + RViz2 (--points, --no-rviz)
├── stop_sim.sh            hentikan semua yang dijalankan skrip di folder ini
├── worlds/                world sendiri
│   ├── cave_megacavern.sdf    gua (DARPA SubT Finals), tanpa GPS
│   └── ego_forest_real.sdf    hutan (dari ego_planner_ws), tanpa GPS
├── models/                model sendiri
│   ├── iris_with_d435/        Iris + ArduPilot + RealSense D435 (drone utama)
│   ├── realsense_d435/        kamera RealSense D435
│   └── ego_forest_real_textures/  tekstur hutan
├── params/
│   ├── sitl_ports.parm    port MAVLink tambahan + stream rate MAVROS (selalu dimuat)
│   └── gps_denied.parm    parameter ArduPilot untuk terbang tanpa GPS
├── ros/
│   ├── bridge.yaml        ros_gz_bridge: RGB, depth, camera_info, clock
│   ├── bridge_points.yaml ros_gz_bridge: point cloud (opsional)
│   └── d435.rviz          konfigurasi RViz2
├── tools/
│   ├── convert_subt_world.py  membuat worlds/cave_megacavern.sdf dari world Fuel
│   └── set_ekf_origin.py      set EKF origin (dipakai otomatis oleh --gps-denied)
└── ardupilot_gazebo/      clone upstream ArduPilot/ardupilot_gazebo + build plugin
                           (jangan taruh file sendiri di sini)
```

`worlds/` dan `models/` dicari Gazebo **lebih dulu** daripada `ardupilot_gazebo/worlds|models`.

## 2. Prasyarat

| Komponen | Lokasi / versi |
|---|---|
| ArduPilot (SITL) | `~/Projects/ardupilot` (ubah dengan env `ARDUPILOT_DIR`), venv `~/venv-ardupilot` |
| Gazebo | Harmonic (`gz sim`, gz-sim 8) |
| Plugin ArduPilot-Gazebo | `ardupilot_gazebo/build/libArduPilotPlugin.so` (sudah di-build) |
| ROS 2 (opsional) | Jazzy di `/opt/ros/jazzy`, paket `ros_gz_bridge`, `rviz2` |
| GPU | NVIDIA via PRIME offload (otomatis); `GZ_NVIDIA=0` untuk iGPU |

### Clone dan build

`ardupilot_gazebo` adalah git submodule (repo resmi ArduPilot, dikunci ke satu commit):

```bash
git clone --recursive https://github.com/AgungDjalil/ardupilot-simulation-gazebo-harmonic.git ardupilot_sim
cd ardupilot_sim
# sudah di-clone tanpa --recursive?  git submodule update --init
```

Build plugin (sekali setelah clone, dan setelah submodule diperbarui):

```bash
cd ardupilot_gazebo
GZ_VERSION=harmonic cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
```

Model dari Gazebo Fuel (gua ±1 GB) diunduh otomatis ke `~/.gz/fuel` saat world pertama kali dibuka.

## 3. Cara menjalankan

Setiap skrip di terminal sendiri, berurutan. Berhenti: `./stop_sim.sh`.

### a. Gua tanpa GPS + kamera di RViz2

```bash
./run_gazebo.sh cave_megacavern.sdf      # terminal 1, tunggu jendela Gazebo terbuka
./run_sitl.sh --gazebo --gps-denied      # terminal 2
./run_ros_bridge.sh                      # terminal 3 (opsional): RViz2
```

### b. Hutan tanpa GPS + kamera di RViz2

```bash
./run_gazebo.sh ego_forest_real.sdf
./run_sitl.sh --gazebo --gps-denied
./run_ros_bridge.sh
```

### c. Landasan pacu dengan GPS (world bawaan ardupilot_gazebo)

```bash
./run_gazebo.sh                          # = iris_runway.sdf
./run_sitl.sh --gazebo
```

### d. Tanpa Gazebo (fisika bawaan SITL, paling ringan)

```bash
./run_sitl.sh
```

Setelah SITL jalan, **QGroundControl tersambung otomatis** (UDP 14550). Siap terbang bila status
EKF hijau; di mode `--gps-denied` tunggu baris `set_ekf_origin: EKF origin set ...` di terminal 2.

## 4. World yang tersedia

| World | Drone | Kamera | GPS | Keterangan |
|---|---|---|---|---|
| `cave_megacavern.sdf` | `iris_with_d435` | D435 | pakai `--gps-denied` | Gua Louisville Mega Cavern (SubT Finals World 08). Start di staging area (x = -12), mulut gua ±8 m di depan (+x). Atap staging area ±3.8 m. |
| `ego_forest_real.sdf` | `iris_with_d435` | D435 | pakai `--gps-denied` | Hutan alami dari ego_planner_ws. Start di (0, 0). Area terbang x 0..26, y -8..8; yang dijamin bebas pohon hanya titik start dan titik survey x ∈ {0, 26}, y ∈ {-8, -4, 0, 4, 8} (radius 1.6 m). |
| `iris_runway.sdf` | `iris_with_gimbal` | kamera gimbal 640×480 | ya | Bawaan ardupilot_gazebo. |
| `iris_warehouse.sdf` | `iris_with_gimbal` | kamera gimbal | ya | Bawaan ardupilot_gazebo. |
| `zephyr_runway.sdf` | `zephyr_with_ardupilot` (pesawat) | – | ya | `./run_sitl.sh --gazebo -v ArduPlane -f gazebo-zephyr` (belum diuji di project ini). |

Semua world memakai lokasi home SITL default (CMAC, -35.363262, 149.165237, 584 m), sama dengan
`<spherical_coordinates>` di world.

Tidak ada penghindar rintangan otomatis: di gua dan hutan terbangkan pelan dan pendek.

## 5. Skrip dan opsinya

### `run_gazebo.sh [world.sdf]`

| | |
|---|---|
| Default world | `iris_runway.sdf` |
| Pencarian world/model | `worlds/`, `models/`, lalu `ardupilot_gazebo/worlds`, `ardupilot_gazebo/models` |
| GPU | NVIDIA PRIME offload; `GZ_NVIDIA=0 ./run_gazebo.sh ...` untuk iGPU |

### `run_sitl.sh [opsi] [argumen sim_vehicle.py]`

| Opsi | Efek |
|---|---|
| (tanpa opsi) | ArduCopter dengan fisika bawaan SITL (quad +), tanpa Gazebo |
| `--gazebo` | `-f gazebo-iris --model JSON`: fisika dari Gazebo lewat UDP 9002 |
| `--gps-denied` | GPS mati, posisi dari simulator vision (ExtNav) di SERIAL5, `params/gps_denied.parm`, EKF origin diset otomatis |
| `-A "..."` | argumen tambahan untuk binary SITL (digabung, tidak menimpa setelan skrip) |
| (selalu) | port MAVLink sesuai [bagian 6](#6-mavlink-port-dan-cara-terhubung), `params/sitl_ports.parm` |
| lainnya | diteruskan ke `sim_vehicle.py`, mis. `-v ArduPlane -f gazebo-zephyr`, `--add-param-file=...`, `-L lokasi` |

Parameter selalu mulai bersih (`-w`, eeprom dihapus). Output SITL: `tail -f /tmp/ArduCopter.log`.

### `run_ros_bridge.sh [--points] [--no-rviz]`

Lihat [bagian 9](#9-kamera-di-ros-2--rviz2).

### `stop_sim.sh`

Mematikan SITL, Gazebo, bridge dan RViz2 yang dijalankan skrip folder ini. Gazebo dari launch
EGO-Planner tidak disentuh.

## 6. MAVLink: port dan cara terhubung

Vehicle: **system ID 1, component ID 1**, MAVLink 2. Setiap aplikasi punya port sendiri, jadi
semuanya bisa tersambung **bersamaan**:

| Port | Serial ArduPilot | Untuk | Cara terhubung |
|---|---|---|---|
| **UDP 14550** | SERIAL0 (SITL mengirim ke `127.0.0.1:14550`) | **QGroundControl** | otomatis, cukup buka QGC |
| **TCP 5762** | SERIAL1 | **MCP server `mavlink`** (Claude) | otomatis (`MAVLINK_URL`) |
| **TCP 5763** | SERIAL2 | **MAVROS** | `fcu_url:=tcp://127.0.0.1:5763` |
| **TCP 5766** | SERIAL6 | **Mission Planner** / MAVProxy / skrip | TCP, host `127.0.0.1`, port `5766` |
| (internal) | SERIAL5 (`sim:vicon`) | simulator vision `--gps-denied` | – |

- Port TCP adalah server di `0.0.0.0`: bisa juga diakses dari PC lain di jaringan
  (`<IP laptop ini>:5766`, mis. Mission Planner di Windows), selama firewall mengizinkan.
- Satu port TCP = satu klien. Klien kedua di port yang sama harus menunggu yang pertama putus.
- SITL tidak menunggu koneksi apa pun untuk mulai; jalankan klien (QGC, MAVROS, Mission Planner)
  setelah SITL jalan.
- Dengan `--gps-denied`, `tools/set_ekf_origin.py` memakai TCP 5766 beberapa detik saat start (sampai
  `EKF origin set`), lalu melepasnya. Sambungkan Mission Planner setelah itu.
- SERIAL6 dan stream rate MAVROS diatur di `params/sitl_ports.parm` (selalu dimuat oleh
  `run_sitl.sh`).

Port internal lain (bukan MAVLink): UDP 9002 Gazebo ↔ SITL (JSON fisika), UDP 5501 input RC SITL,
UDP 9005 simulasi IR-Lock.

### QGroundControl

Buka QGC; drone muncul sendiri (QGC mendengarkan UDP 14550 secara default). Jangan buat comm link
UDP 5760 — itu bukan port simulasi ini.

### MAVROS (ROS 2 Jazzy) — TCP 5763

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5763
```

- `/mavros/state` → `connected: true`. Stream data di port ini sudah diatur
  (`MAV3_*` di `params/sitl_ports.parm`): `/mavros/local_position/pose` dan `/mavros/imu/data`
  ±20 Hz, `/mavros/battery` 2 Hz.
- Ubah rate saat jalan: `ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate
  "{stream_id: 0, message_rate: 20, on_off: true}"`.
- Tanpa GPS (`--gps-denied`) posisi lokal tetap ada di `/mavros/local_position/*`; `global_position`
  dihitung dari EKF origin.
- Jangan pakai MAVROS di 5762 (milik MCP) atau 14550 (milik QGC).

### Mission Planner — TCP 5766

Mission Planner ada di `~/MissionPlanner` (jalan dengan `mono`):

```bash
cd ~/MissionPlanner && mono MissionPlanner.exe
```

Di pojok kanan atas pilih **TCP** → **Connect** → host `127.0.0.1` → port `5766`.
Dari PC Windows lain: host = IP laptop ini (`hostname -I`), port `5766`.

### MAVProxy / pymavlink — TCP 5766

```bash
mavproxy.py --master tcp:127.0.0.1:5766          # dari ~/venv-ardupilot
python3 -c "from pymavlink import mavutil; m = mavutil.mavlink_connection('tcp:127.0.0.1:5766'); print(m.wait_heartbeat())"
```

MAVProxy dan Mission Planner berbagi 5766; kalau keduanya perlu bersamaan, teruskan dari MAVProxy:
`mavproxy.py --master tcp:127.0.0.1:5766 --out udp:127.0.0.1:14551` lalu Mission Planner pakai
**UDP** port 14551.

### Menambah output MAVLink

Port lain bisa ditambahkan ke SITL dengan `-A`, mis. UDP ke companion computer di jaringan
(SERIAL7, protokol MAVLink harus diaktifkan juga):

```bash
./run_sitl.sh --gazebo -A "--serial7=udpclient:192.168.1.50:14551"
# lalu set parameter SERIAL7_PROTOCOL = 2 (QGC/MAVProxy/MCP set_param) dan reboot,
# atau tambahkan "SERIAL7_PROTOCOL 2" ke params/sitl_ports.parm
```

## 7. Sensor pada drone

Drone utama `models/iris_with_d435` = `iris_with_ardupilot` (ardupilot_gazebo) + RealSense D435.
Dengan `--model JSON`, Gazebo mengirim ke SITL setiap langkah fisika (1 kHz): data IMU, posisi,
kecepatan dan attitude. Sensor lain disimulasikan oleh SITL dari keadaan fisika itu.

| Sensor | Disimulasikan oleh | Detail |
|---|---|---|
| **IMU** (gyro + akselerometer) | Gazebo (`imu_sensor`, 1000 Hz) | ArduPilot melihat 2 IMU (`INS_GYR_ID`, `INS_GYR2_ID`) dari data yang sama + noise SITL |
| **Barometer** | SITL | 2 barometer (`BARO1_DEVID`, `BARO2_DEVID`) |
| **Kompas** | SITL | 3 magnetometer (`COMPASS_DEV_ID..3`) |
| **GPS** | SITL | GPS1 5 Hz (`SIM_GPS1_HZ`). **Mati** dengan `--gps-denied` (`GPS1_TYPE 0`) |
| **Rangefinder bawah** | SITL | `RNGFND1_TYPE 1`, menghadap bawah, maks 50 m (dari `gazebo-iris.parm`, untuk precision landing). Dihitung dari ketinggian di atas home, **bukan** dari geometri world, jadi tidak akurat di atas lantai gua yang tidak rata |
| **Baterai** | SITL | `BATT_MONITOR 4`; dengan Gazebo nilainya tetap 100% |
| **Vision / ExtNav** | SITL (`sim:vicon`) | Hanya dengan `--gps-denied`: posisi, kecepatan, yaw 50 Hz + noise. Lihat bagian 8 |
| **Kamera RGB-D RealSense D435** | Gazebo (`rgbd_camera`, render di GPU) | Di hidung drone (x = 0.11 m), menghadap lurus ke depan. 640×480 @ 30 Hz, HFOV 60° (fx = fy = 554.25), depth 0.1–10 m, noise σ 0.007. Topik Gazebo `d435/image`, `d435/depth_image`, `d435/points`, `d435/camera_info`; frame `d435_link` |

World runway/warehouse memakai `iris_with_gimbal`: sensor yang sama kecuali kamera, diganti kamera
gimbal 3 sumbu (640×480, HFOV 2.0 rad).

## 8. Terbang tanpa GPS (`--gps-denied`)

`params/gps_denied.parm` (sama dengan `copter-vicon.parm` di autotest ArduPilot, plus noise):

| Parameter | Nilai | Arti |
|---|---|---|
| `GPS1_TYPE`, `SIM_GPS1_ENABLE` | 0 | tidak ada GPS |
| `VISO_TYPE` | 1 | visual odometry lewat MAVLink |
| `SERIAL5_PROTOCOL` | 2 | MAVLink di SERIAL5 (`sim:vicon`) |
| `EK3_SRC1_POSXY/VELXY/POSZ/VELZ/YAW` | 6 | EKF3: semua dari External Navigation |
| `SIM_VICON_P_SD` / `SIM_VICON_V_SD` | 0.01 m / 0.02 m/s | noise pengukuran vision |

Simulator vision mengambil pose drone dari fisika Gazebo; ini pengganti VIO/SLAM sungguhan
(T265, VOXL, RTAB-Map) tanpa drift. Tanpa GPS EKF tidak punya origin global, jadi
`tools/set_ekf_origin.py` mengirim `SET_GPS_GLOBAL_ORIGIN` (posisi awal simulator) lewat TCP 5766
sampai dikonfirmasi; setelah itu QGC menampilkan posisi drone dan mode GUIDED/LOITER bisa dipakai.

Uji kegagalan saat terbang (QGC → Parameters, MAVProxy `param set`, atau MCP `set_param`):

| Parameter | Efek |
|---|---|
| `SIM_VICON_GLIT_X` / `_Y` / `_Z` | posisi vision meloncat (m) |
| `SIM_VICON_FAIL 1` | vision hilang |
| `SIM_VICON_P_SD`, `SIM_VICON_V_SD` | noise lebih besar |

## 9. Kamera di ROS 2 / RViz2

```bash
./run_ros_bridge.sh              # RGB + depth + RViz2
./run_ros_bridge.sh --points     # + point cloud (berat, lihat bagian 11)
./run_ros_bridge.sh --no-rviz    # bridge saja
```

| Topik ROS 2 | Tipe | Keterangan |
|---|---|---|
| `/d435/image` | `sensor_msgs/Image` (rgb8) | warna 640×480 |
| `/d435/depth_image` | `sensor_msgs/Image` (32FC1) | jarak dalam meter |
| `/d435/camera_info` | `sensor_msgs/CameraInfo` | intrinsik |
| `/d435/points` | `sensor_msgs/PointCloud2` | hanya dengan `--points` |
| `/clock` | `rosgraph_msgs/Clock` | waktu simulasi (`use_sim_time`) |

Frame `d435_link`: x depan, y kiri, z atas. RViz2 (`ros/d435.rviz`) menampilkan panel RGB dan depth;
centang "D435 points" bila memakai `--points`. Hanya butuh `/opt/ros/jazzy`, memakai RMW default.

Topik dari terminal lain:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 run rqt_image_view rqt_image_view /d435/image
```

## 10. Kontrol lewat Claude (MCP server `mavlink`)

MCP server di `~/Projects/mavlink_mcp/server.py` (terdaftar di Claude Code, scope user), terhubung
ke `tcp:127.0.0.1:5762` (env `MAVLINK_URL`). Tool: `status`, `takeoff`, `goto_local`,
`goto_global`, `land`, `rtl`, `set_mode`, `arm`, `disarm`, `get_param`, `set_param`.
Contoh permintaan: "takeoff 1.5 m lalu maju 8 m ke timur dan mendarat".

## 11. Performa: CPU vs GPU

- **Fisika Gazebo selalu di CPU** (DART/Bullet tidak punya versi GPU). GPU NVIDIA dipakai untuk
  rendering GUI, kamera D435 dan RViz2.
- Gua: collision detector bawaan DART membuat mesh gua jadi bottleneck (~0.5× real time);
  `cave_megacavern.sdf` memakai `<collision_detector>bullet</collision_detector>` (3–6× real time,
  dibatasi 1×).
- Point cloud (`--points`): ±300 ribu titik/frame dikemas di CPU dan dikirim lewat DDS; simulasi
  bisa turun di bawah real time. RGB + depth saja tetap ±1× real time.
- Cek real-time factor: `gz topic -e -t /stats -n 1 | grep real_time_factor`.

## 12. Menambah world dan vehicle

### World baru

1. Salin template, mis. `ardupilot_gazebo/worlds/iris_runway.sdf` → `worlds/nama.sdf`.
2. Ganti `<world name>` dan isinya. Pertahankan plugin Physics, Sensors (`ogre2`), Imu, NavSat,
   `<spherical_coordinates>` dan `max_step_size` 0.001.
3. Masukkan drone: `<include><uri>model://iris_with_d435</uri><pose>x y 0.195 0 0 yaw</pose></include>`
   di lantai datar (kalau tidak, drone bisa terguling saat muncul).
4. `./run_gazebo.sh nama.sdf`

World gua dibuat ulang dari sumbernya:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Finals Prize Round World 08"
python3 tools/convert_subt_world.py \
  "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/finals prize round world 08/1/final_event_08.sdf" \
  worlds/cave_megacavern.sdf
```

### Vehicle baru

1. Salin template ke `models/nama/`: `ardupilot_gazebo/models/iris_with_ardupilot` (multicopter),
   `zephyr_with_ardupilot` (pesawat), atau gabungkan seperti `models/iris_with_d435`
   (`<include merge="true">` model dasar + sensor + `fixed` joint).
2. Blok `<plugin name="ArduPilotPlugin">` di `model.sdf`:
   - `fdm_port_in` 9002 (vehicle kedua 9012 + SITL `-I1`, dst.),
   - `imuName` = sensor IMU model itu,
   - satu `<control channel="N">` per motor/servo, urut sesuai output SERVO(N+1) ArduPilot.
3. Frame SITL dan parameter:

   ```bash
   ./run_sitl.sh --gazebo -v ArduPlane -f gazebo-zephyr
   ./run_sitl.sh --gazebo --add-param-file=$HOME/Projects/ardupilot_sim/params/nama.parm
   ```

## 13. Troubleshooting

| Gejala | Penyebab / solusi |
|---|---|
| `Arm: Gyros inconsistent` beberapa detik setelah start | normal saat SITL baru jalan; tunggu ±15 detik lalu arm lagi |
| `PrecLand: Failsafe Measures` saat mendarat | precision landing aktif (`gazebo-iris.parm`) tapi tidak ada target IR-Lock; aman diabaikan |
| QGC tidak tersambung | QGC harus mendengarkan UDP 14550 (default). Comm link UDP 5760 tidak berguna (5760 bukan port SITL ini) |
| `run_sitl.sh --gazebo` tidak jalan / `No JSON sensor message received` | Gazebo belum jalan atau world tidak punya drone dengan ArduPilotPlugin (port 9002) |
| Port 9002 / 5762 sudah dipakai | simulasi lain masih jalan (mis. EGO-Planner): `./stop_sim.sh` atau `stop_ego_ardupilot.sh` |
| Mode `--gps-denied` tidak bisa arm di GUIDED | tunggu `set_ekf_origin: EKF origin set` di terminal SITL; TCP 5766 harus bebas saat start (sambungkan Mission Planner setelahnya) |
| MAVROS `connected: true` tapi topik posisi/IMU kosong | stream rate port 5763 diatur `params/sitl_ports.parm`; pastikan SITL dijalankan lewat `run_sitl.sh`, atau panggil `/mavros/set_stream_rate` |
| World gagal dimuat (`Failed to load a world`) | lihat `[Err]` di output Gazebo; model Fuel bisa hilang/privat — lihat daftar yang dibuang di `tools/convert_subt_world.py` |
| Simulasi lambat | lihat bagian 11; matikan `--points`, tutup GUI Gazebo bila tidak perlu |

## 14. Sumber dan lisensi

- `worlds/cave_megacavern.sdf`: "Finals Prize Round World 08", Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/OpenRobotics/fuel/worlds/Finals%20Prize%20Round%20World%2008).
  Diubah: elemen kompetisi, model yang tidak bisa diunduh, dan penanda Fiducial dibuang; sistem
  ArduPilot, Bullet collision detector dan drone ditambahkan.
- `models/realsense_d435`: "Intel RealSense D435", bmchale / Open Robotics, CC BY 4.0
  (https://app.gazebosim.org/bmchale/fuel/models/Intel%20RealSense%20D435). Diubah: mesh diputar
  90° (lensa aslinya menghadap -y), pose dan topik sensor.
- `worlds/ego_forest_real.sdf`, `models/ego_forest_real_textures`: disalin dari
  `ego_planner_ws/src/ego_ardupilot` (dibuat oleh `tools/gen_real_forest.py` di sana); drone diganti
  `iris_with_d435`.
- `ardupilot_gazebo`: https://github.com/ArduPilot/ardupilot_gazebo (LGPL-3.0).
