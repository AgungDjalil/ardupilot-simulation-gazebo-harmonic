#!/usr/bin/env bash
# Terminal 1: Gazebo Harmonic dengan quadcopter iris dari ardupilot_gazebo (tanpa ROS).
# Lalu Terminal 2: ./run_sitl.sh --gazebo. QGroundControl tersambung otomatis lewat UDP 14550.
#
#   ./run_gazebo.sh                       world iris_runway.sdf (default)
#   ./run_gazebo.sh iris_warehouse.sdf    world lain dari ardupilot_gazebo/worlds
#   ./run_gazebo.sh cave_megacavern.sdf   world sendiri dari worlds/ (gua SubT, untuk --gps-denied)
#
# World dan model sendiri ditaruh di worlds/ dan models/ (dicari lebih dulu daripada ardupilot_gazebo).
#
# Plugin di-build standalone di ardupilot_gazebo/build:
#   cd ardupilot_gazebo && cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
#
# Render (GUI + sensor kamera) di GPU NVIDIA lewat PRIME offload, bukan iGPU AMD.
# GZ_NVIDIA=0 ./run_gazebo.sh untuk memakai iGPU. Fisika tetap dihitung di CPU.
DIR="$(cd "$(dirname "$0")" && pwd)"
if [ "${GZ_NVIDIA:-1}" = 1 ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  export __VK_LAYER_NV_optimus=NVIDIA_only
fi
AG="$DIR/ardupilot_gazebo"
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$DIR/models:$DIR/worlds:$AG/models:$AG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -v4 -r "${1:-iris_runway.sdf}"
