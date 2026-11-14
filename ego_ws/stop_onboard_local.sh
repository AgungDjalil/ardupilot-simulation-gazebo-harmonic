#!/usr/bin/env bash
# Stop run_onboard_local.sh (the onboard stack on this laptop), also when running in the background.
# Its nodes are recorded first: a background launch dies at once on SIGTERM without stopping its
# nodes. The nodes get SIGTERM (clean stop, RTAB-Map saves its database), KILL only if still
# alive after 30 s.
LAUNCH='[d]rone_onboard.launch.py'
# Nodes a launch left behind when it failed while starting (seen: domain_bridge missing; they were
# reparented and kept commanding the FC). Only this stack's own executables.
ORPHANS='install/ego_ardupilot/lib/ego_ardupilot/(vo_to_mavros|bridge|camera_preview|pi_health|cloud_compress)|install/ego_planner/lib/ego_planner/(ego_planner_node|traj_server)|domain_bridge .*drone_bridge[.]yaml'
orphans() {
  pkill -TERM -f "$ORPHANS" 2>/dev/null && sleep 2
  pkill -KILL -f "$ORPHANS" 2>/dev/null
  exit 0
}
NODES=""
for LP in $(pgrep -f "$LAUNCH"); do NODES="$NODES $(pgrep -P "$LP")"; done
pkill -TERM -f "$LAUNCH" || orphans
[ -n "$NODES" ] && kill -TERM $NODES 2>/dev/null
for i in $(seq 1 30); do
  ALIVE=$(for p in $NODES; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
  [ -z "$ALIVE" ] && ! pgrep -f "$LAUNCH" > /dev/null && orphans
  sleep 1
done
pkill -KILL -f "$LAUNCH"
[ -n "$ALIVE" ] && kill -KILL $ALIVE 2>/dev/null
orphans
