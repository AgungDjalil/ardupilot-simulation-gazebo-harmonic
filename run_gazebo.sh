#!/usr/bin/env bash
# Terminal 1: Gazebo Harmonic with an Iris quadcopter (no ROS needed).
# Then terminal 2: ./run_sitl.sh --gazebo. QGroundControl connects automatically over UDP 14550.
#
#   ./run_gazebo.sh                       world iris_runway.sdf (default)
#   ./run_gazebo.sh iris_warehouse.sdf    another world from ardupilot_gazebo/worlds
#   ./run_gazebo.sh cave_megacavern.sdf   a world of this project from worlds/ (SubT cave, for
#                                         --gps-denied)
#
# This project's worlds and models live in worlds/ and models/ (searched before ardupilot_gazebo's).
#
# The plugin is built standalone in ardupilot_gazebo/build:
#   cd ardupilot_gazebo && cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
#
# Rendering (GUI + camera sensors) runs on the NVIDIA GPU through PRIME offload, not the integrated
# GPU. GZ_NVIDIA=0 ./run_gazebo.sh to use the integrated GPU. Physics always runs on the CPU.
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
