#!/usr/bin/env bash
# ArduPilot SITL (ArduCopter). MAVLink ports (see README section 6):
#   UDP 14550 (SERIAL0, SITL sends)  QGroundControl, connects automatically
#   TCP 5762  (SERIAL1)              MCP server "mavlink" (~/Projects/mavlink_mcp)
#   TCP 5763  (SERIAL2)              MAVROS
#   TCP 5766  (SERIAL6)              Mission Planner / MAVProxy
# SITL starts right away without waiting for a TCP connection.
#
#   ./run_sitl.sh                         SITL's built-in physics (no Gazebo)
#   ./run_sitl.sh --gazebo                Iris in Gazebo; start ./run_gazebo.sh first
#   ./run_sitl.sh --gazebo --gps-denied   no GPS: position from the simulated vision system
#                                         (ExtNav), see params/gps_denied.parm; EKF origin set
#                                         automatically
# Other arguments are passed to sim_vehicle.py, e.g. -v ArduPlane -f gazebo-zephyr or
# --add-param-file=params/xxx.parm.
#
# Output arducopter: tail -f /tmp/ArduCopter.log
DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/Projects/ardupilot}
FRAME=()
GPS_DENIED=0
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
    GPS_DENIED=1
  else
    ARGS+=("$a")
  fi
done
[ -f "$HOME/venv-ardupilot/bin/activate" ] && source "$HOME/venv-ardupilot/bin/activate"
if [ $GPS_DENIED = 1 ]; then
  echo "GPS-denied: GPS off, EKF3 uses external navigation (simulated vision on SERIAL5)"
  SITL_ARGS="$SITL_ARGS --serial5=sim:vicon:"
  ARGS+=(--add-param-file="$DIR/params/gps_denied.parm")
  # Without GPS the EKF has no origin; it is set over SERIAL6 (TCP 5766) once SITL is up, then the
  # port is released for Mission Planner. (Not 5763: MAVROS may already be connected there.)
  python3 "$DIR/tools/set_ekf_origin.py" --url tcp:127.0.0.1:5766 &
fi
export PATH="$ARDUPILOT_DIR/Tools/autotest:$PATH"
cd "$ARDUPILOT_DIR"
echo "ArduCopter SITL output: /tmp/ArduCopter.log  (MAVLink -> UDP 127.0.0.1:14550)"
exec env -u DISPLAY -u TMUX -u STY -u ZELLIJ -u SITL_RITW_TERMINAL \
    sim_vehicle.py -v ArduCopter "${FRAME[@]}" --no-mavproxy -w \
    -A "$SITL_ARGS" "${ARGS[@]}"
