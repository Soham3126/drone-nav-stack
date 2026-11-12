#!/usr/bin/env bash
# Start Gazebo, ArduPilot SITL and the ROS stack in one tmux session "sim" (guidebook 4.2).
# Usage: scripts/sim_up.sh <world> <profile> [out_dir]
#   env: HEADLESS=1 (default) | 0     FLOW=0 (default) | 1     MANGLE=<yaml> (truth profile)
#        SLAM=true (default) | false  (gps profile: run Cartographer alongside or not)
#        GATE=1  pausable gates on /scan and /imu before Cartographer (A1.11)
set -euo pipefail
WORLD=${1:?world, e.g. w0_box_room}; PROFILE=${2:?profile: gps|truth|slam|full}
OUT=${3:-$HOME/drone_results/manual_$(date +%Y%m%d_%H%M%S)}
REPO=$HOME/drone_ws/src; P=$REPO/drone_bringup/params/ardupilot
SIMV=$HOME/ardupilot/Tools/autotest/sim_vehicle.py

[ -f "$REPO/drone_sim/worlds/$WORLD.sdf" ] || { echo "no world $WORLD.sdf in drone_sim/worlds (run make_maze.py)"; exit 1; }
if tmux has-session -t sim 2>/dev/null; then echo "a 'sim' session is already running: scripts/sim_down.sh first"; exit 1; fi
mkdir -p "$OUT"

PARMS="--add-param-file=$P/sitl_base.parm"
case "$PROFILE" in
  gps)             PARMS="$PARMS --add-param-file=$P/sitl_gps.parm" ;;
  truth|slam|full) PARMS="$PARMS --add-param-file=$P/sitl_extnav.parm" ;;
  *) echo "unknown profile $PROFILE"; exit 1 ;;
esac
[ "${FLOW:-0}" = 1 ] && PARMS="$PARMS --add-param-file=$P/sitl_flow.parm"
GZ_ARGS="-v3 -r"; [ "${HEADLESS:-1}" = 1 ] && GZ_ARGS="$GZ_ARGS -s"

tmux new-session -d -s sim -n gz \
  "bash -ic 'ros_env; gz sim $GZ_ARGS $WORLD.sdf 2>&1 | tee $OUT/gz.log'"
sleep 5
tmux new-window -t sim -n sitl \
  "bash -ic 'ap_env; cd $OUT; $SIMV -v ArduCopter -f gazebo-iris --model JSON -w -N $PARMS 2>&1 | tee $OUT/sitl.log'"
sleep 20
tmux new-window -t sim -n ros \
  "bash -ic 'ros_env; ros2 launch drone_bringup sim.launch.py profile:=$PROFILE slam:=${SLAM:-true} world:=$WORLD out_dir:=$OUT mangle:=${MANGLE:-none} gate:=${GATE:-false} 2>&1 | tee $OUT/ros.log'"
echo "up: tmux attach -t sim     down: scripts/sim_down.sh   results: $OUT"
echo "tmux windows: gz (Gazebo), sitl (MAVProxy prompt), ros (ROS nodes).  Switch: Ctrl+b then n.  Leave: Ctrl+b then d."
