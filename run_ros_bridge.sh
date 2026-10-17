#!/usr/bin/env bash
# Terminal 3 (opsional): bridge kamera RealSense D435 dari Gazebo ke ROS 2 Jazzy + RViz2.
# Jalankan setelah ./run_gazebo.sh dengan world yang memakai model iris_with_d435
# (mis. cave_megacavern.sdf).
#
#   ./run_ros_bridge.sh              RGB + depth + RViz2
#   ./run_ros_bridge.sh --points     juga point cloud (/d435/points); centang "D435 points" di RViz.
#                                    Berat: ~300 ribu titik/frame dikemas di CPU, simulasi bisa
#                                    melambat (real-time factor < 1).
#   ./run_ros_bridge.sh --no-rviz    bridge saja
#
# Topik ROS 2: /d435/image, /d435/depth_image (32FC1, meter), /d435/camera_info, /clock
# (frame_id d435_link: x depan, y kiri, z atas). Lihat ros/bridge.yaml.
# Hanya memakai ROS 2 dari /opt/ros/jazzy, tidak bergantung pada ego_planner_ws.
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
  # RViz di GPU NVIDIA, seperti Gazebo (GZ_NVIDIA=0 untuk iGPU).
  if [ "${GZ_NVIDIA:-1}" = 1 ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  fi
  rviz2 -d "$DIR/ros/d435.rviz" --ros-args -p use_sim_time:=true
else
  wait
fi
