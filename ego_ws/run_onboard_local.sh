#!/usr/bin/env bash
# The real drone's onboard stack on this laptop, flying the simulation: what ~/drone/run_onboard.sh
# runs on the Pi (ego_planner_ws launch/drone_onboard.launch.py: RTAB-Map, MAVROS, vo_to_mavros,
# EGO-Planner + bridge, domain_bridge), unchanged, with the simulated D455 and SITL as the FC.
# For testing on the laptop's CPU; ~/drone/run_onboard.sh --sim on the Pi tests it on the drone's.
#   cd ~/Projects/ardupilot_sim
#   ./run_gazebo.sh ego_forest_real.sdf                 # 1. world + drone
#   ./run_sitl.sh --gazebo --nav extnav --fc truf       # 2. the FC
#   ./run_ros_bridge.sh                                 # 3. the D455 (--net local)
#   ego_ws/run_onboard_local.sh [launch arguments]      # 4. this
#   then from ego_planner_ws as with the real drone: ./drone_cmd.sh preflight / takeoff 1.0 /
#   forward 3 / turn 90 / land, ./run_drone_log.sh, ./run_drone_monitor.sh --rviz
#   (drone_env.sh picks the local link while this runs).
# Restarting this = rebooting the FC: restart SITL first (its EKF keeps the previous VO frame).
# Stop: Ctrl+C.
EGO_PLANNER_WS=${EGO_PLANNER_WS:-$HOME/Projects/ego_planner_ws}
if pgrep -f '[d]rone_onboard.launch.py' > /dev/null; then
  echo "The onboard stack is already running on this laptop."; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$EGO_PLANNER_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Onboard domain 0 and the monitor's domain 1, both on loopback (as ardupilot_sim's --net local).
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="file://$EGO_PLANNER_WS/install/ego_ardupilot/share/ego_ardupilot/config/cyclonedds.xml"
# As ~/drone/run_onboard.sh --sim: no camera driver, SITL as the FC (sysid 1, MAVROS port 5763),
# the drone replica's camera 15 cm forward (models/truf_d455).
exec ros2 launch ego_ardupilot drone_onboard.launch.py camera:=false fcu_url:=tcp://127.0.0.1:5763 \
    tgt_system:=1 cam_x:=0.15 "$@"
