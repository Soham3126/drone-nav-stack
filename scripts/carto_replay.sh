#!/usr/bin/env bash
# Re-run Cartographer on a recorded flight with changed settings, then evaluate (guidebook A1.3, A1.9).
# Usage: scripts/carto_replay.sh RUN_DIR NAME 'lua override lines'
#   e.g. scripts/carto_replay.sh ~/drone_results/2026..._A1.3_w1_corridor_plain_abc tw1 \
#          'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.'
#   NAME=base with an empty override replays the current drone_2d.lua unchanged (sanity check).
# Env: RATE=1.0 (bag playback speed); DOMAIN=77 (ROS domain; parallel replays each get their own).
#      A live simulation (domain 0) is never disturbed.
# Output: RUN_DIR__replay_NAME/ with eval_data.npz, cpu.csv, carto.log, metrics.json, eval.png.
set -eo pipefail
# Job control on: without it, bash makes background jobs IGNORE SIGINT, so the "kill -INT" used
# to stop the recorder and Cartographer did nothing and the script waited forever.
set -m
RUN=$(readlink -f "${1:?run folder}"); NAME=${2:?name}; OVR=${3:-}
REPO=$HOME/drone_ws/src; RATE=${RATE:-1.0}
[ -f "$RUN/bag/metadata.yaml" ] || { echo "no complete bag in $RUN (pick a run folder that has metrics.json)"; exit 1; }
OUT=${RUN}__replay_${NAME}
rm -rf "$OUT"; mkdir -p "$OUT/config"
sed '/^return options/d' "$REPO/drone_slam/config/drone_2d.lua" > "$OUT/config/replay.lua"
printf '%s\n' "-- replay override ($NAME):" "$OVR" "return options" >> "$OUT/config/replay.lua"
grep -v '^procedure exit' "$RUN/meta.txt" > "$OUT/meta.txt"
printf 'replay_of=%s\nreplay_name=%s\nreplay_override=%s\nreplay_rate=%s\n' "$(basename "$RUN")" "$NAME" "$OVR" "$RATE" >> "$OUT/meta.txt"
WORLD=$(grep '^world=' "$RUN/meta.txt" | cut -d= -f2)

set +u; source /opt/ros/humble/setup.bash; source "$HOME/drone_ws/install/setup.bash"
export ROS_DOMAIN_ID=${DOMAIN:-77} ROS_LOCALHOST_ONLY=1
cd "$OUT"
stop_pid() {   # SIGINT, then TERM, then KILL if it is still running
  for sig in INT TERM KILL; do
    kill -$sig "$1" 2>/dev/null || return 0
    for _ in $(seq 1 10); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
  done
}
# leftovers of an earlier replay INTO THIS SAME FOLDER only (parallel replays use other folders)
cleanup() {
  pkill -INT -f "config_dir:=$OUT/config" 2>/dev/null || true
  sleep 1
  pkill -9 -f "configuration_directory $OUT/config" 2>/dev/null || true
  pkill -9 -f "replay_collect.py --out $OUT" 2>/dev/null || true
}
cleanup
ros2 launch "$REPO/scripts/replay.launch.py" config_dir:="$OUT/config" config:=replay.lua > carto.log 2>&1 &
LP=$!
if [ "${NOCPU:-0}" = 1 ]; then sleep 99999 & CP=$!       # parallel sweeps: per-process CPU would be mixed
else python3 "$REPO/drone_sim/tools/cpu_logger.py" --out cpu.csv > /dev/null 2>&1 & CP=$!; fi
sleep 4
python3 "$REPO/drone_sim/tools/replay_collect.py" --out "$OUT" > collect.log 2>&1 & GP=$!
sleep 2
echo "replaying $(basename "$RUN") with '$NAME' at rate $RATE (about as long as the flight) ..."
if ! ros2 bag play "$RUN/bag" --topics /scan /imu /clock /ground_truth/odom --rate "$RATE" > play.log 2>&1; then
  echo "bag play FAILED:"; tail -5 play.log; stop_pid $GP; stop_pid $LP; stop_pid $CP; cleanup; exit 1
fi
wait $GP || true                         # the collector stops by itself a few seconds after playback
cat collect.log
stop_pid $LP; stop_pid $CP; cleanup
grep -qi "error" carto.log && echo "(Cartographer log mentions errors: $OUT/carto.log - normal ones at start/stop; send it if results look odd)" || true
python3 "$REPO/drone_sim/tools/eval_run.py" "$OUT" --world "$WORLD" > report.txt 2>&1 || true
RES=$(grep -E "^A[0-9.a-z]+: (PASS|FAIL)" report.txt | tail -1)
[ "${QUIET:-0}" = 1 ] || cat report.txt
echo "done: $NAME on $(basename "$RUN") -> ${RES:-no verdict (see $OUT/report.txt)}"
