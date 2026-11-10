#!/usr/bin/env bash
# Re-run Cartographer on a recorded flight with changed settings, then evaluate (guidebook A1.3, A1.9).
# Usage: scripts/carto_replay.sh RUN_DIR NAME 'lua override lines'
#   e.g. scripts/carto_replay.sh ~/drone_results/2026..._A1.3_w1_corridor_plain_abc tw1 \
#          'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.'
#   NAME=base with an empty override replays the current drone_2d.lua unchanged (sanity check).
# Env: RATE=1.0 (bag playback speed). Runs on ROS_DOMAIN_ID 77, so a live simulation is not disturbed.
# Output: RUN_DIR__replay_NAME/ with bag, cpu.csv, metrics.json, eval.png.
set -eo pipefail
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
ros2 launch "$REPO/scripts/replay.launch.py" config_dir:="$OUT/config" config:=replay.lua > carto.log 2>&1 &
LP=$!
python3 "$REPO/drone_sim/tools/cpu_logger.py" --out cpu.csv & CP=$!
sleep 4
ros2 bag record -s mcap -o bag /tf /tf_static /map /clock /ground_truth/odom > rec.log 2>&1 &
RP=$!
sleep 2
echo "replaying $(basename "$RUN") with '$NAME' at rate $RATE ..."
ros2 bag play "$RUN/bag" --topics /scan /imu /clock /ground_truth/odom --rate "$RATE" > play.log 2>&1
sleep 3                                  # final map
kill -INT $RP; wait $RP 2>/dev/null || true
kill -INT $LP $CP 2>/dev/null; wait $LP $CP 2>/dev/null || true
grep -q "Error\|error" carto.log && echo "(Cartographer printed errors: see $OUT/carto.log)"
python3 "$REPO/drone_sim/tools/eval_run.py" "$OUT" --world "$WORLD" | tee report.txt
