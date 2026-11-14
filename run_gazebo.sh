#!/usr/bin/env bash
# Terminal 1: Gazebo Harmonic with the drone (no ROS needed). Then terminal 2: ./run_sitl.sh --gazebo,
# and for the camera in ROS 2: ./run_ros_bridge.sh.
#
#   ./run_gazebo.sh [world.sdf] [options]
#     world.sdf           from worlds/ (ego_forest_real.sdf, cave_megacavern.sdf, baylands.sdf, ...)
#                         or ardupilot_gazebo/worlds (default iris_runway.sdf)
#     --pose x,y,z,yaw    spawn point (yaw in degrees) instead of the world's
#     --wind s[,dir[,g]]  wind of s m/s blowing towards dir degrees (0 = +x, 90 = +y), gusts of +-g %
#                         (default 20); --wind 0 turns the world's own wind (baylands) off
#     --infra             also render the D455's infrared stereo pair (VIO; costs GPU time)
#     --headless          no GUI (server only, rendering off-screen)
#
# The drone is always the real drone's replica, models/truf_d455 (Iris + RealSense D455 mounted as
# on the drone), model name "iris". Worlds carry no drone: tools/make_world.py adds it at the
# world's spawn point and writes /tmp/ardupilot_sim/current.json for run_ros_bridge.sh.
# This project's worlds and models live in worlds/ and models/ (searched before ardupilot_gazebo's).
#
# The plugin is built standalone in ardupilot_gazebo/build:
#   cd ardupilot_gazebo && cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
#
# Rendering (GUI + camera sensors) runs on the NVIDIA GPU through PRIME offload, not the integrated
# GPU. GZ_NVIDIA=0 ./run_gazebo.sh to use the integrated GPU. Physics always runs on the CPU.
DIR="$(cd "$(dirname "$0")" && pwd)"
WORLD=iris_runway.sdf
GEN=()
SERVER=()
while [ $# -gt 0 ]; do
  case "$1" in
    --pose|--wind) GEN+=("$1" "$2"); shift 2 ;;
    --infra) GEN+=(--infra); shift ;;
    --headless) SERVER=(-s --headless-rendering); shift ;;
    -*) echo "Unknown option $1"; exit 1 ;;
    *) WORLD="$1"; shift ;;
  esac
done
GENERATED=$(python3 "$DIR/tools/make_world.py" "$WORLD" "${GEN[@]}") || exit 1
if [ "${GZ_NVIDIA:-1}" = 1 ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  export __VK_LAYER_NV_optimus=NVIDIA_only
fi
AG="$DIR/ardupilot_gazebo"
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
# /tmp/ardupilot_sim/models first: the drone copy make_world.py wrote (infrared pair on/off).
export GZ_SIM_RESOURCE_PATH="/tmp/ardupilot_sim/models:$DIR/models:$DIR/worlds:$AG/models:$AG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "World: $GENERATED  (drone + options: /tmp/ardupilot_sim/current.json)"
exec gz sim -v4 -r "${SERVER[@]}" "$GENERATED"
