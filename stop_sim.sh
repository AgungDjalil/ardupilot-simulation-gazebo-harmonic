#!/usr/bin/env bash
# Stop SITL, Gazebo, the sensor bridge (ros_gz_bridge, sim_camera, world_map) and RViz2 started by
# the run_*.sh scripts in this folder. The algorithm under test (e.g. the drone's onboard stack) is
# stopped by its own script.
# A Gazebo started by another launch (e.g. EGO-Planner: gz sim -r -v 2 ...) is left alone.
GZ=$(pgrep -f 'gz sim -v4 -r')
GZ_CHILDREN=$(for p in $GZ; do pgrep -P "$p"; done)
pkill -f 'rviz2 -d .*ardupilot_sim/ros/d455.rviz' 2>/dev/null
pkill -f 'ardupilot_sim/ros/(sim_camera|world_map).py' 2>/dev/null
pkill -f 'parameter_bridge .*__node:=gz_bridge_sim_gz' 2>/dev/null
pkill -f 'tools/set_ekf_origin.py' 2>/dev/null
pkill -f 'sim_vehicle.py -v ArduCopter' 2>/dev/null
pkill -x arducopter 2>/dev/null
pkill -x arduplane 2>/dev/null
[ -n "$GZ" ] && kill $GZ 2>/dev/null
sleep 2
pkill -9 -x arducopter 2>/dev/null
pkill -9 -x arduplane 2>/dev/null
[ -n "$GZ$GZ_CHILDREN" ] && kill -9 $GZ $GZ_CHILDREN 2>/dev/null
exit 0
