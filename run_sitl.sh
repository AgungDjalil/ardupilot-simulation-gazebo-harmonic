#!/usr/bin/env bash
# ArduPilot SITL (ArduCopter). MAVLink ports (see README section 6):
#   UDP 14550 (SERIAL0, SITL sends)  QGroundControl, connects automatically
#   TCP 5762  (SERIAL1)              MCP server "mavlink" (~/Projects/mavlink_mcp)
#   TCP 5763  (SERIAL2)              MAVROS
#   TCP 5766  (SERIAL6)              Mission Planner / MAVProxy
# SITL starts right away without waiting for a TCP connection.
#
#   ./run_sitl.sh                         SITL's built-in physics (no Gazebo)
#   ./run_sitl.sh --gazebo                the drone in Gazebo; start ./run_gazebo.sh first
#   --nav gps                             navigation with GPS (default)
#   --nav vicon   (alias --gps-denied)    no GPS: position from SITL's simulated vision system, from
#                                         the physics (ExtNav, no drift), params/nav_vicon.parm;
#                                         EKF origin set automatically
#   --nav extnav                          no GPS: position from YOUR visual odometry / SLAM on a
#                                         companion computer via MAVROS vision_pose (TCP 5763), as on
#                                         the real drone, params/nav_extnav.parm; the companion sets
#                                         the EKF origin
#   --fc truf                             the real drone's FC settings (4S battery, speeds, failsafes,
#                                         fence, telemetry rates on the MAVROS port), params/fc_truf.parm
#   e.g. the real drone's onboard stack: ./run_sitl.sh --gazebo --nav extnav --fc truf
# Other arguments are passed to sim_vehicle.py, e.g. -v ArduPlane -f gazebo-zephyr or
# --add-param-file=params/xxx.parm.
#
# Output arducopter: tail -f /tmp/ArduCopter.log
DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/Projects/ardupilot}
FRAME=()
NAV=gps
FC=
# sim_vehicle.py only uses the last -A, so a user's -A is merged into this one.
SITL_ARGS="--serial0=udpclient:127.0.0.1:14550 --serial6=tcp:6"
ARGS=(--add-param-file="$DIR/params/sitl_ports.parm")
while [ $# -gt 0 ]; do
  a="$1"; shift
  if [ "$a" = "-A" ] || [ "$a" = "--sitl-instance-args" ]; then
    SITL_ARGS="$SITL_ARGS $1"; shift
  elif [[ "$a" == --sitl-instance-args=* ]]; then
    SITL_ARGS="$SITL_ARGS ${a#*=}"
  elif [ "$a" = "--gazebo" ]; then
    FRAME=(-f gazebo-iris --model JSON)
  elif [ "$a" = "--gps-denied" ]; then
    NAV=vicon
  elif [ "$a" = "--nav" ]; then
    NAV="$1"; shift
  elif [ "$a" = "--fc" ]; then
    FC="$1"; shift
  else
    ARGS+=("$a")
  fi
done
[ -f "$HOME/venv-ardupilot/bin/activate" ] && source "$HOME/venv-ardupilot/bin/activate"
case "$NAV" in
  gps) ;;
  vicon)
    echo "GPS-denied: GPS off, EKF3 uses external navigation (simulated vision on SERIAL5)"
    SITL_ARGS="$SITL_ARGS --serial5=sim:vicon:"
    ARGS+=(--add-param-file="$DIR/params/nav_vicon.parm")
    # Without GPS the EKF has no origin; it is set over SERIAL6 (TCP 5766) once SITL is up, then the
    # port is released for Mission Planner. (Not 5763: MAVROS may already be connected there.)
    python3 "$DIR/tools/set_ekf_origin.py" --url tcp:127.0.0.1:5766 &
    ;;
  extnav)
    echo "GPS-denied: GPS off, EKF3 waits for the companion's visual odometry (MAVROS vision_pose, TCP 5763)"
    ARGS+=(--add-param-file="$DIR/params/nav_extnav.parm") ;;
  *) echo "Unknown --nav $NAV (gps, vicon, extnav)"; exit 1 ;;
esac
case "$FC" in
  "") ;;
  truf)
    echo "FC: the real drone's settings (params/fc_truf.parm)"
    ARGS+=(--add-param-file="$DIR/params/fc_truf.parm") ;;
  *) echo "Unknown --fc $FC (truf)"; exit 1 ;;
esac
export PATH="$ARDUPILOT_DIR/Tools/autotest:$PATH"
cd "$ARDUPILOT_DIR"
echo "ArduCopter SITL output: /tmp/ArduCopter.log  (MAVLink -> UDP 127.0.0.1:14550)"
exec env -u DISPLAY -u TMUX -u STY -u ZELLIJ -u SITL_RITW_TERMINAL \
    sim_vehicle.py -v ArduCopter "${FRAME[@]}" --no-mavproxy -w \
    -A "$SITL_ARGS" "${ARGS[@]}"
