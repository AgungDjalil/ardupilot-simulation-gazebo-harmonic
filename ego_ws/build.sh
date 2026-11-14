#!/usr/bin/env bash
# Build this workspace (ego_ardupilot_sim) as an overlay on ego_planner_ws; build that one first.
# Extra arguments go to colcon build.
cd "$(dirname "$0")"
EGO_PLANNER_WS=${EGO_PLANNER_WS:-$HOME/Projects/ego_planner_ws}
source /opt/ros/jazzy/setup.bash
source "$EGO_PLANNER_WS/install/setup.bash"
exec colcon build "$@"
