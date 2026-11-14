#!/usr/bin/env bash
# Terminal 3: the drone's sensors in ROS 2, exactly as on the real drone. Run it after ./run_gazebo.sh.
# The simulated RealSense D455 (ros/sim_camera.py) publishes what realsense2_camera publishes on the
# drone: /camera/camera/color/*, /camera/camera/aligned_depth_to_color/*, /camera/camera/imu (and
# /camera/camera/infra1|2/* with run_gazebo.sh --infra), wall-clock stamps, the D455's noise. An
# algorithm subscribes to these and talks to the FC over MAVROS (TCP 5763), as on the drone; it runs
# unchanged on the drone afterwards. Simulation-only extras: /sim/truth, /sim/world_cloud.
#
#   ./run_ros_bridge.sh [options]
#     --net local     the algorithm runs on this laptop (default): loopback DDS (ros/cyclonedds_local.xml)
#     --net lan       it runs on the drone's Raspberry Pi: DDS over the LAN cable (ros/cyclonedds_lan.xml)
#     --domain N      ROS domain (default 0, the real drone's onboard domain)
#     --ideal         exact depth and IMU (no D455 noise, holes, bias, accel scale)
#     --no-rviz       no RViz (ros/d455.rviz: colour, depth, ground truth)
#
# Use the same settings in the algorithm's terminal: ROS_DOMAIN_ID, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp,
# CYCLONEDDS_URI=file://<this folder>/ros/cyclonedds_local.xml (or the algorithm's own equivalent).
# Only needs ROS 2 from /opt/ros/jazzy.
DIR="$(cd "$(dirname "$0")" && pwd)"
NET=local
DOMAIN=0
RVIZ=1
CAMERA_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --net) NET="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --ideal) CAMERA_ARGS=(-p realistic:=false); shift ;;
    --no-rviz) RVIZ=0; shift ;;
    *) echo "Unknown option $1"; exit 1 ;;
  esac
done
CURRENT=/tmp/ardupilot_sim/current.json
[ -f "$CURRENT" ] || { echo "No $CURRENT: start ./run_gazebo.sh first."; exit 1; }
case "$NET" in
  local) DDS="$DIR/ros/cyclonedds_local.xml" ;;
  lan)
    if ! ip -4 -br addr show enp193s0 2>/dev/null | grep -q '^enp193s0 *UP .* 192\.168\.50\.1/'; then
      echo "No LAN cable to the Pi (enp193s0 is not up with 192.168.50.1; NetworkManager \"LAN-Share\")."; exit 1
    fi
    DDS="$DIR/ros/cyclonedds_lan.xml" ;;
  *) echo "Unknown --net $NET (local, lan)"; exit 1 ;;
esac
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$DOMAIN
export CYCLONEDDS_URI="file://$DDS"
WORLD=$(python3 -c "import json; print(json.load(open('$CURRENT'))['world_name'])")
INFRA=$(python3 -c "import json; print(int(json.load(open('$CURRENT'))['infra']))")
echo "World $WORLD, ROS domain $DOMAIN, DDS $NET ($DDS)"

# One ros_gz_bridge process per stream: a stalled one doesn't hold up the others. Gazebo -> /sim/gz/*,
# read only by sim_camera.py.
STREAMS=(
  "/d455/image|sensor_msgs/msg/Image|gz.msgs.Image|/sim/gz/image"
  "/d455/depth_image|sensor_msgs/msg/Image|gz.msgs.Image|/sim/gz/depth"
  "/d455/camera_info|sensor_msgs/msg/CameraInfo|gz.msgs.CameraInfo|/sim/gz/camera_info"
  "/world/$WORLD/model/iris/link/imu_link/sensor/imu_sensor/imu|sensor_msgs/msg/Imu|gz.msgs.IMU|/sim/gz/imu"
  "/model/iris/odometry|nav_msgs/msg/Odometry|gz.msgs.Odometry|/sim/gz/odometry"
)
if [ "$INFRA" = 1 ]; then
  for n in 1 2; do
    STREAMS+=("/d455/infra$n/image|sensor_msgs/msg/Image|gz.msgs.Image|/sim/gz/infra$n"
              "/d455/infra$n/camera_info|sensor_msgs/msg/CameraInfo|gz.msgs.CameraInfo|/sim/gz/infra$n/camera_info")
  done
fi
PIDS=()
trap 'kill ${PIDS[*]} 2>/dev/null' EXIT
for s in "${STREAMS[@]}"; do
  IFS='|' read -r GZ_TOPIC ROS_TYPE GZ_TYPE OUT <<< "$s"
  ros2 run ros_gz_bridge parameter_bridge "$GZ_TOPIC@$ROS_TYPE[$GZ_TYPE" \
    --ros-args -r "__node:=gz_bridge${OUT//\//_}" -r "$GZ_TOPIC:=$OUT" --log-level warn &
  PIDS+=($!)
done
python3 "$DIR/ros/sim_camera.py" --ros-args "${CAMERA_ARGS[@]}" &
PIDS+=($!)
python3 "$DIR/ros/world_map.py" &
PIDS+=($!)
if [ $RVIZ = 1 ]; then
  # RViz on the NVIDIA GPU, like Gazebo (GZ_NVIDIA=0 for the integrated GPU).
  if [ "${GZ_NVIDIA:-1}" = 1 ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  fi
  rviz2 -d "$DIR/ros/d455.rviz"
else
  wait
fi
