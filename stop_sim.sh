#!/usr/bin/env bash
# Hentikan SITL, Gazebo, bridge ROS 2 dan RViz2 yang dijalankan oleh run_*.sh di folder ini.
# Gazebo dari launch EGO-Planner (gz sim -r -v 2 ...) tidak disentuh.
GZ=$(pgrep -f 'gz sim -v4 -r')
GZ_CHILDREN=$(for p in $GZ; do pgrep -P "$p"; done)
pkill -f 'rviz2 -d .*ardupilot_sim/ros/d435.rviz' 2>/dev/null
pkill -f 'parameter_bridge .*ardupilot_sim/ros/bridge(_points)?.yaml' 2>/dev/null
pkill -f 'sim_vehicle.py -v ArduCopter' 2>/dev/null
pkill -x arducopter 2>/dev/null
pkill -x arduplane 2>/dev/null
[ -n "$GZ" ] && kill $GZ 2>/dev/null
sleep 2
pkill -9 -x arducopter 2>/dev/null
pkill -9 -x arduplane 2>/dev/null
[ -n "$GZ$GZ_CHILDREN" ] && kill -9 $GZ $GZ_CHILDREN 2>/dev/null
exit 0
