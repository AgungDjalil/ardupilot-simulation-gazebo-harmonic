#!/usr/bin/env bash
# Symphony's aircraft in Gazebo Harmonic with a 3-axis gimbal camera each: ArduCopter SITL per aircraft on the
# ports the station's rows already carry (tcp:127.0.0.1:5762, 5772, ...), a gimbal ArduPilot drives as a servo
# mount (MNT1_TYPE 1, SERVO9..11), and the gimbal camera served as MJPEG on http://127.0.0.1:8601/, 8602, ...
# for the aircraft's video_rtsp_url. Nothing in the Symphony repository is touched; this replaces its
# scripts/sitl/run-sitl.sh --gazebo for flights that need the payload.
#
#   symphony/run.sh start [N]     # N aircraft (default 2, at most 8), each spawned on its own pad of the R23
#                                 # command vehicle in Baylands: G1/G2 main front left/right, G3/G4 main rear,
#                                 # G5..G8 the outer extension pads
#   symphony/run.sh stop
#   symphony/run.sh status
#
#   SYM_WORLD=~/Projects/ardupilot_sim/worlds/<world>.sdf   another world (home from its <spherical_coordinates>)
#   SYM_R23_POSE="x,y,z,yaw" where the R23 stands (Baylands default "0,0,-0.09,0": level on the grass at the origin)
#   SYM_SCENE=ground         the aircraft on the ground 25 m apart instead, Iris visuals (the earlier layout);
#   SYM_SPAWNS="x,y,z;..."   then places them exactly (Baylands default: "0,0,0.12;-25,0,-0.68", G2 rests level)
#   SYM_HEADLESS=1           Gazebo server only (the cameras still render)
#   SYM_MODEL=iris           the earlier airframe (ardupilot_gazebo's iris_with_gimbal); default x500: PX4's
#                            x500_gimbal from Gazebo Fuel, downloaded on the first start (see make_world.py)
#   SYM_VEHICLE=hilux        the earlier R23 Hilux; default ncommand: the team's N°COMMAND pickup with ArUco pads
#                            (visual only), H1 on its canopy roof (make_ncommand.py)
#   SYM_TOWN=0               without the neighbourhood (make_town.py: ~240 buildings, ~190 vehicles from Gazebo Fuel;
#                            SYM_TOWN_ROW_BUILDINGS, SYM_TOWN_BUILDINGS, SYM_TOWN_VEHICLES, SYM_TOWN_SEED)
#   SYM_SHADOWS=1            keep the sun's shadows with the town (off by default: they halve the real-time factor)
#   GZ_NVIDIA=0              render on the integrated GPU (default: NVIDIA through PRIME offload)
#   SYM_HEXA=0               without H1, the hexa in the R23's middle bay: a Typhoon H480 (Gazebo Fuel) with a fixed
#                            Livox Avia LiDAR, SITL -I 8, station port tcp:127.0.0.1:5842 (probe 5843), LiDAR on
#                            gz topic /H1/lidar/points; SYM_LIDAR_TILT_DEG=0 points it ahead (default 90: down)
#
# Then the station in live mode (it already lists G1 and G2 at these ports):
#   cd ~/Documents/symphony/gcs && MAVLINK_ENABLED=1 TTS_PROVIDER=none DETECT_AUTOSTART=0 PORT=5050 .venv/bin/python3 app.py
#   cd ~/Documents/symphony && npm run dev
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); SIM=$(dirname "$HERE")
AG="${ARDUPILOT_GAZEBO:-$SIM/ardupilot_gazebo}"
DIR="${SYM_DIR:-$HOME/.cache/symphony-gimbal-sim}"
WORLD="${SYM_WORLD:-$SIM/worlds/baylands.sdf}"
if [ "$(basename "$WORLD")" = baylands.sdf ]; then
  : "${SYM_SPAWNS:=0,0,0.12;-25,0,-0.68}"
  # The grass under the vehicle falls ~2.5 deg to the west: level, the wheels are within 7 cm of it.
  : "${SYM_R23_POSE:=0,0,-0.09,0}"
fi
export SYM_SPAWNS="${SYM_SPAWNS:-}" SYM_R23_POSE="${SYM_R23_POSE:-}"
SYMPHONY_SITL="$HOME/.cache/symphony-sitl"   # where Symphony's run-sitl.sh keeps the binary and copter.parm
BIN_URL="https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter"
PARM_URL="https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm"
CAM_PORT_BASE=8601
mkdir -p "$DIR"

case "${1:-status}" in
  start)
    n="${2:-2}"
    if ss -ltn | awk '{print $4}' | grep -qE ':(57[6-9][0-9]|584[0-9]|860[0-9]|8766)$'; then
      echo "ports 5760.. or 8601.. are taken: stop the other simulation first"
      echo "  (Symphony's: ~/Documents/symphony/scripts/sitl/run-sitl.sh stop; this one: $0 stop)"; exit 1
    fi
    [ -f "$AG/build/libArduPilotPlugin.so" ] || { echo "no ArduPilot plugin in $AG/build (see ardupilot_gazebo's README)"; exit 1; }
    # The x500 (default airframe): PX4's x500_gimbal and the two models it includes, from Gazebo Fuel into its cache.
    if [ "${SYM_MODEL:-x500}" = x500 ]; then
      fuel="${SYM_PX4_FUEL:-$HOME/.gz/fuel/fuel.gazebosim.org/px4/models}"
      for m in x500_gimbal x500 gimbal; do
        ls -d "$fuel/$m"/[0-9]* > /dev/null 2>&1 && continue
        echo "downloading $m from Gazebo Fuel…"
        gz fuel download -u "https://fuel.gazebosim.org/1.0/PX4/models/$m" > /dev/null \
          || { echo "could not download $m (https://app.gazebosim.org/PX4/fuel/models/$m)"; exit 1; }
      done
    fi
    # The hexa: Gambit's Typhoon H480 (Fuel keeps it under its lower-cased name, with the space).
    if [ "${SYM_HEXA:-1}" = 1 ] && ! ls -d "${SYM_H480_FUEL:-$HOME/.gz/fuel/fuel.gazebosim.org/gambit/models/typhoon h480}"/[0-9]* > /dev/null 2>&1; then
      echo "downloading Typhoon H480 from Gazebo Fuel…"
      # gz reports "Unable to load SDF file" afterwards: the model's model.config names typhoon_h480.sdf, the file
      # is model.sdf. make_world.py reads model.sdf, so that message does not matter.
      gz fuel download -u "https://fuel.gazebosim.org/1.0/Gambit/models/Typhoon H480" > /dev/null 2>&1 || true
      ls -d "${SYM_H480_FUEL:-$HOME/.gz/fuel/fuel.gazebosim.org/gambit/models/typhoon h480}"/[0-9]* > /dev/null 2>&1 \
        || { echo "could not download Typhoon H480 (https://app.gazebosim.org/Gambit/fuel/models/Typhoon%20H480)"; exit 1; }
    fi
    # The neighbourhood (make_town.py): OpenRobotics' city models from Gazebo Fuel, downloaded once (~40 MB).
    if [ "${SYM_TOWN:-1}" = 1 ]; then
      for m in "house 1" "house 2" "house 3" "fast food" "fire station" "gas station" "grocery store" "law office" \
               "office building" "police station" "post office" "salon" "school" "thrift shop" "apartment" \
               "water tower" "hatchback" "hatchback blue" "hatchback red" "suv" "pickup" "ambulance" "bus" "fire truck"; do
        ls -d "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/$m"/[0-9]* > /dev/null 2>&1 && continue
        echo "downloading $m from Gazebo Fuel…"
        gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/$m" > /dev/null 2>&1 || echo "  ($m not downloaded: left out)"
      done
    fi
    # The N°COMMAND pickup (default vehicle): made once by make_ncommand.py, which renders the ArUco pads with
    # Symphony's own gcs/companion/landing_marker_aruco.py (needs OpenCV >= 4.7: this folder's .venv).
    if [ "${SYM_VEHICLE:-ncommand}" = ncommand ] && [ ! -f "$SIM/models/ncommand_r23/model.sdf" ]; then
      echo "building the N°COMMAND pickup…"; "$HERE/.venv/bin/python" -I "$HERE/make_ncommand.py" > /dev/null
    fi
    # Its textures: the STLs as textured OBJ in models/h480_tex, made once (texture_h480.py, a few seconds).
    if [ "${SYM_HEXA:-1}" = 1 ] && [ ! -f "$SIM/models/h480_tex/meshes/main_body_remeshed_v3.obj" ]; then
      echo "texturing the Typhoon H480…"; python3 -I "$HERE/texture_h480.py" > /dev/null
    fi
    if [ ! -x "$DIR/arducopter" ]; then
      if [ -x "$SYMPHONY_SITL/arducopter" ]; then cp "$SYMPHONY_SITL/arducopter" "$DIR/"
      else echo "downloading arducopter SITL (stable, x86_64)…"; curl -sf --max-time 600 -o "$DIR/arducopter" "$BIN_URL"; chmod +x "$DIR/arducopter"; fi
    fi
    if [ ! -f "$DIR/copter.parm" ]; then
      if [ -f "$SYMPHONY_SITL/copter.parm" ]; then cp "$SYMPHONY_SITL/copter.parm" "$DIR/"
      else curl -sf --max-time 60 -o "$DIR/copter.parm" "$PARM_URL"; fi
    fi
    # The fleet's battery setup, as Symphony's run-sitl.sh loads it: a 4S LiHV pack and both battery failsafes
    # RTL. The station's readiness gate refuses to arm without it.
    printf '%s\n' 'SIM_BATT_VOLTAGE 17.4' 'BATT_ARM_VOLT 14.7' 'BATT_LOW_VOLT 14.5' 'BATT_CRT_VOLT 14.0' \
      'BATT_FS_LOW_ACT 2' 'BATT_FS_CRT_ACT 2' 'BRD_SD_MISSION 64' > "$DIR/fleet.parm"
    # BRD_SD_MISSION: SITL's EEPROM holds only 650 mission items ("FC: Only 650 items are supported"), and a
    # survey of the town at Symphony's line spacing runs past that (H1's mapping mission was 752). 64 KB of
    # mission.stg on the SITL filesystem holds ~4300 items at 15 bytes each, as a Pixhawk's SD card would.
    # The Iris is an X frame. The gimbal is ardupilot_gazebo's config/gazebo-iris-gimbal.parm without its RC
    # inputs (RC6..8 options): the station points it over MAVLink, and RC overrides on 6..8 would move it.
    # The limits must stay these: the plugin maps SERVO 1100..1900 onto exactly these joint ranges.
    printf '%s\n' 'FRAME_CLASS 1' 'FRAME_TYPE 1' \
      'MNT1_TYPE 1' 'MNT1_PITCH_MIN -135' 'MNT1_PITCH_MAX 45' 'MNT1_ROLL_MIN -30' 'MNT1_ROLL_MAX 30' \
      'MNT1_YAW_MIN -160' 'MNT1_YAW_MAX 160' \
      'SERVO9_FUNCTION 8' 'SERVO10_FUNCTION 7' 'SERVO11_FUNCTION 6' > "$DIR/iris_gimbal.parm"
    # The hexa: hexa X, motors 1..6 on SERVO1..6 (the defaults for that frame), no mount (the LiDAR is fixed).
    printf '%s\n' 'FRAME_CLASS 2' 'FRAME_TYPE 1' 'MNT1_TYPE 0' > "$DIR/hexa.parm"

    rm -rf "$DIR/gazebo"; mkdir -p "$DIR/gazebo"
    out=$(python3 -I "$HERE/make_world.py" "$n" "$DIR/gazebo" "$AG" "$WORLD")
    world=$(sed -n 1p <<< "$out")
    IFS=, read -r HOME_LAT HOME_LON HOME_ALT <<< "$(sed -n 2p <<< "$out")"
    wd=$(cd "$(dirname "$WORLD")" && pwd)
    gui=(); [ "${SYM_HEADLESS:-0}" = 1 ] && gui=(-s --headless-rendering)
    ( # Render (GUI, gimbal cameras, LiDAR) on the NVIDIA GPU through PRIME offload, as ../run_gazebo.sh does: on the
      # integrated GPU the neighbourhood's ~400 models held the world to a real-time factor of 0.03..1.2.
      # GZ_NVIDIA=0 keeps the integrated GPU.
      if [ "${GZ_NVIDIA:-1}" = 1 ]; then
        export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only
        export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
      fi
      export GZ_VERSION=harmonic GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
      export GZ_SIM_RESOURCE_PATH="$DIR/gazebo/models:$wd:$wd/../models:$AG/models:$AG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
      setsid nohup gz sim -v3 -r "${gui[@]}" "$world" > "$DIR/gazebo/gz.log" 2>&1 & echo $! > "$DIR/gazebo/pid" )
    echo "gazebo: pid $(cat "$DIR/gazebo/pid"), world $world, log $DIR/gazebo/gz.log"

    cams=()
    for i in $(seq 0 $((n - 1))); do
      d="$DIR/i$i"; mkdir -p "$d"
      printf 'SYSID_THISMAV %d\n' $((i + 1)) > "$d/sysid.parm"
      # A subshell that waits for arducopter and logs how it ended; the pid file is the subshell's, and `stop`
      # kills its child too. (arducopter exec'd in place of the subshell died a second after this script
      # returned, with nothing in its log, so the exit is now written down.)
      ( cd "$d"; setsid nohup "$DIR/arducopter" --model JSON:127.0.0.1 --speedup 1 -I "$i" \
          --home "$HOME_LAT,$HOME_LON,$HOME_ALT,0" \
          --defaults "$DIR/copter.parm,$DIR/fleet.parm,$d/sysid.parm,$DIR/iris_gimbal.parm" \
          --serial0=tcp:0:nowait --serial1=tcp:2:nowait > "$d/sitl.log" 2>&1
        echo "arducopter exited: $? ($(date +%T))" >> "$d/sitl.log" ) < /dev/null > /dev/null 2>&1 & echo $! > "$d/pid"
      cams+=("G$((i + 1)):$((CAM_PORT_BASE + i))")
      echo "G$((i + 1)): SITL pid $(cat "$d/pid"), station port $((5762 + 10 * i)), camera http://127.0.0.1:$((CAM_PORT_BASE + i))/"
    done
    if [ "${SYM_HEXA:-1}" = 1 ]; then
      # H1 on instance 8, after the pads' quads, so its ports (5840.., JSON 9082) do not depend on N.
      d="$DIR/h1"; mkdir -p "$d"
      printf 'SYSID_THISMAV %d\n' 9 > "$d/sysid.parm"
      ( cd "$d"; setsid nohup "$DIR/arducopter" --model JSON:127.0.0.1 --speedup 1 -I 8 \
          --home "$HOME_LAT,$HOME_LON,$HOME_ALT,0" \
          --defaults "$DIR/copter.parm,$DIR/fleet.parm,$d/sysid.parm,$DIR/hexa.parm" \
          --serial0=tcp:0:nowait --serial1=tcp:2:nowait > "$d/sitl.log" 2>&1
        echo "arducopter exited: $? ($(date +%T))" >> "$d/sitl.log" ) < /dev/null > /dev/null 2>&1 & echo $! > "$d/pid"
      echo "H1: SITL pid $(cat "$d/pid"), station port 5842 (probe 5843), LiDAR gz topic /H1/lidar/points"
      # Its LiDAR Pi: the Avia's cloud placed with the autopilot's pose (SERIAL2, 5843) and served the way
      # Symphony's velodyne_forwarder.py serves it. Station: LIDAR_WS_URL=ws://127.0.0.1:8766.
      py="$HERE/.venv/bin/python"; [ -x "$py" ] || py=python3
      ( nohup "$py" "$HERE/livox_forwarder.py" --ws-port "${SYM_LIDAR_WS_PORT:-8766}" --mavlink tcp:127.0.0.1:5843 \
          --tilt-deg "${SYM_LIDAR_TILT_DEG:-90}" --live-max-pts "${SYM_LIDAR_LIVE_MAX_PTS:-10000}" \
          > "$DIR/livox_forwarder.log" 2>&1 & echo $! > "$DIR/livox_forwarder.pid" )
      echo "H1 LiDAR forwarder: pid $(cat "$DIR/livox_forwarder.pid"), ws://127.0.0.1:${SYM_LIDAR_WS_PORT:-8766}, log $DIR/livox_forwarder.log"
    fi
    # The bridge needs the gz Python bindings (ROS 2 Jazzy's gz vendor packages), hence no -I here.
    ( nohup python3 "$HERE/camera_bridge.py" "${cams[@]}" > "$DIR/camera_bridge.log" 2>&1 & echo $! > "$DIR/camera_bridge.pid" )
    echo "camera bridge: pid $(cat "$DIR/camera_bridge.pid"), log $DIR/camera_bridge.log"
    echo "home $HOME_LAT,$HOME_LON; wait about a minute for the EKF before arming"
    # Where each aircraft starts: the station's base for it (Kelola, or the vehicles row's base_lat/base_lon).
    sed -n '3,$p' <<< "$out" | sed 's/^/start /' ;;
  stop)
    for p in "$DIR"/i*/pid "$DIR"/h1/pid "$DIR/camera_bridge.pid" "$DIR/livox_forwarder.pid"; do
      [ -f "$p" ] || continue
      kill "$(cat "$p")" 2>/dev/null && echo "stopped $(cat "$p")"; rm -f "$p"
    done
    # The pid files are the waiting subshells': the aircraft themselves are every arducopter run from this
    # directory's binary (not Symphony's run-sitl.sh ones, which run from theirs).
    ours=()
    for p in $(pgrep -x arducopter || true); do
      [ "$(readlink "/proc/$p/exe")" = "$DIR/arducopter" ] && ours+=("$p")
    done
    if [ ${#ours[@]} -gt 0 ]; then
      kill "${ours[@]}" 2>/dev/null || true; echo "stopped arducopter ${ours[*]}"
    fi
    # gz sim runs the server and the GUI as children; setsid made it their group leader.
    p="$DIR/gazebo/pid"; [ -f "$p" ] && kill -- "-$(cat "$p")" 2>/dev/null && echo "stopped gazebo $(cat "$p")"; rm -f "$p"
    # Until they are gone: a SITL still holding its TCP ports makes the next start's instance fail to bind.
    for t in $(seq 1 10); do
      alive=(); for p in "${ours[@]}"; do kill -0 "$p" 2>/dev/null && alive+=("$p"); done
      [ ${#alive[@]} -eq 0 ] && break
      [ "$t" = 10 ] && kill -9 "${alive[@]}" 2>/dev/null; sleep 0.5
    done ;;
  status)
    for p in "$DIR"/i*/pid "$DIR"/h1/pid "$DIR/gazebo/pid" "$DIR/camera_bridge.pid" "$DIR/livox_forwarder.pid"; do
      [ -f "$p" ] && { kill -0 "$(cat "$p")" 2>/dev/null && echo "$p: running ($(cat "$p"))" || echo "$p: dead"; }
    done
    ss -ltn 2>/dev/null | awk '{print $4}' | grep -E ":(57[6-9][0-9]|584[0-9]|86[0-9][0-9]|8766)$" | tr '\n' ' '; echo
    [ -f "$DIR/camera_bridge.log" ] && tail -n 1 "$DIR/camera_bridge.log"
    [ -f "$DIR/livox_forwarder.log" ] && tail -n 1 "$DIR/livox_forwarder.log" ;;
  *) echo "usage: $0 start [N] | stop | status"; exit 2 ;;
esac
