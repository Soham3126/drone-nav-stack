#!/usr/bin/env bash
# Re-run Cartographer on a recorded flight with changed settings, then evaluate (guidebook A1.3, A1.9).
# Usage: scripts/carto_replay.sh RUN_DIR NAME 'lua override lines'
#   e.g. scripts/carto_replay.sh ~/drone_results/2026..._A1.3_w1_corridor_plain_abc tw1 \
#          'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.'
#   NAME=base with an empty override replays the current drone_2d.lua unchanged (sanity check).
# Env: RATE=1.0 (bag playback speed). Runs on ROS_DOMAIN_ID 77, so a live simulation is not disturbed.
# Output: RUN_DIR__replay_NAME/ with bag, cpu.csv, metrics.json, eval.png.
set -eo pipefail
# Job control on: without it, bash makes background jobs IGNORE SIGINT, so the "kill -INT" used
# to stop the recorder and Cartographer did nothing and the script waited forever.
set -m
RUN=$(readlink -f "${1:?run folder}"); NAME=${2:?name}; OVR=${3:-}
REPO=$HOME/drone_ws/src; RATE=${RATE:-1.0}
[ -d "$RUN/bag" ] || { echo "no bag in $RUN"; exit 1; }
OUT=${RUN}__replay_${NAME}
rm -rf "$OUT"; mkdir -p "$OUT/config"
sed '/^return options/d' "$REPO/drone_slam/config/drone_2d.lua" > "$OUT/config/replay.lua"
printf '%s\n' "-- replay override ($NAME):" "$OVR" "return options" >> "$OUT/config/replay.lua"
grep -v '^procedure exit' "$RUN/meta.txt" > "$OUT/meta.txt"
printf 'replay_of=%s\nreplay_name=%s\nreplay_override=%s\nreplay_rate=%s\n' "$(basename "$RUN")" "$NAME" "$OVR" "$RATE" >> "$OUT/meta.txt"
WORLD=$(grep '^world=' "$RUN/meta.txt" | cut -d= -f2)

set +u; source /opt/ros/humble/setup.bash; source "$HOME/drone_ws/install/setup.bash"
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
cd "$OUT"
# stop leftovers of an earlier replay (they would publish a second pose and map on domain 77)
stop_replay() {
  pkill -INT -f "replay.launch.py" 2>/dev/null || true
  pkill -INT -f "bag record -s mcap -o bag /tf /tf_static /map" 2>/dev/null || true
  sleep 1
  pkill -f "configuration_basename replay.lua" 2>/dev/null || true
  [ -n "${CP:-}" ] && kill $CP 2>/dev/null || true
}
stop_pid() {   # SIGINT (clean close), then TERM, then KILL if it is still running
  for sig in INT TERM KILL; do
    kill -$sig "$1" 2>/dev/null || return 0
    for _ in $(seq 1 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
  done
}
stop_replay; sleep 1
trap stop_replay EXIT
ros2 launch "$REPO/scripts/replay.launch.py" config_dir:="$OUT/config" config:=replay.lua > carto.log 2>&1 &
LP=$!
python3 "$REPO/drone_sim/tools/cpu_logger.py" --out cpu.csv & CP=$!
sleep 4
ros2 bag record -s mcap -o bag /tf /tf_static /map /clock /ground_truth/odom > rec.log 2>&1 &
RP=$!
sleep 2
echo "replaying $(basename "$RUN") with '$NAME' at rate $RATE ..."
ros2 bag play "$RUN/bag" --topics /scan /imu /clock /ground_truth/odom --rate "$RATE" > play.log 2>&1 || { echo "bag play FAILED (incomplete recording?):"; tail -5 play.log; stop_pid $RP; stop_pid $LP; stop_pid $CP; exit 1; }
sleep 3                                  # final map
stop_pid $RP; stop_pid $LP; stop_pid $CP
grep -q "Error\|error" carto.log && echo "(Cartographer printed errors: see $OUT/carto.log)"
python3 "$REPO/drone_sim/tools/eval_run.py" "$OUT" --world "$WORLD" | tee report.txt
