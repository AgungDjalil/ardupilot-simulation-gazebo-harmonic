#!/usr/bin/env bash
# Terminal 3 (optional): bridge the RealSense D435 camera from Gazebo to ROS 2 Jazzy + RViz2.
# Run it after ./run_gazebo.sh with a world that uses the iris_with_d435 model
# (e.g. cave_megacavern.sdf).
#
#   ./run_ros_bridge.sh              RGB + depth + RViz2
#   ./run_ros_bridge.sh --points     also the point cloud (/d435/points); tick "D435 points" in RViz.
#                                    Heavy: ~300k points per frame are packed on the CPU, the
#                                    simulation may slow down (real-time factor < 1).
#   ./run_ros_bridge.sh --no-rviz    bridge only
#
# ROS 2 topics: /d435/image, /d435/depth_image (32FC1, metres), /d435/camera_info, /clock
# (frame_id d435_link: x forward, y left, z up). See ros/bridge.yaml.
# Only needs ROS 2 from /opt/ros/jazzy.
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
RVIZ=1
POINTS=0
for a in "$@"; do
  case "$a" in
    --no-rviz) RVIZ=0 ;;
    --points) POINTS=1 ;;
  esac
done

PIDS=()
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$DIR/ros/bridge.yaml" &
PIDS+=($!)
if [ $POINTS = 1 ]; then
  ros2 run ros_gz_bridge parameter_bridge --ros-args -r __node:=ros_gz_bridge_points \
    -p config_file:="$DIR/ros/bridge_points.yaml" &
  PIDS+=($!)
fi
trap 'kill ${PIDS[*]} 2>/dev/null' EXIT
if [ $RVIZ = 1 ]; then
  # RViz on the NVIDIA GPU, like Gazebo (GZ_NVIDIA=0 for the integrated GPU).
  if [ "${GZ_NVIDIA:-1}" = 1 ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  fi
  rviz2 -d "$DIR/ros/d435.rviz" --ros-args -p use_sim_time:=true
else
  wait
fi
