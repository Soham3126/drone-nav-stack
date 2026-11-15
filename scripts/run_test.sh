#!/usr/bin/env bash
# One test run, fully recorded (guidebook 4.2).
# Usage: scripts/run_test.sh <TEST_ID> <world> <profile> [timeout_s]
#   e.g. scripts/run_test.sh A0.3 w0_box_room gps
# Results: ~/drone_results/<date>_<ID>_<world>_<git hash>/ with meta.txt, logs, bag, cpu.csv, report.
set -euo pipefail
ID=$1; WORLD=$2; PROFILE=$3; TMO=${4:-900}
REPO=$HOME/drone_ws/src
GIT=$(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo "-dirty")
OUT=$HOME/drone_results/$(date +%Y%m%d_%H%M%S)_${ID}_${WORLD}_${GIT}
mkdir -p "$OUT"
cp -r "$REPO/drone_bringup/params" "$REPO/drone_slam/config" "$REPO/drone_nav/config" "$OUT/" 2>/dev/null || true
printf 'test=%s\nworld=%s\nprofile=%s\ngit=%s\nflow=%s\nmangle=%s\nheadless=%s\nclock_throttle=%s\n' \
  "$ID" "$WORLD" "$PROFILE" "$GIT" "${FLOW:-0}" "${MANGLE:-none}" "${HEADLESS:-1}" "${CLOCK_THROTTLE:-0}" > "$OUT/meta.txt"

"$REPO/scripts/sim_up.sh" "$WORLD" "$PROFILE" "$OUT"
tmux new-window -t sim -n rec \
  "bash -ic 'ros_env; cd $OUT; ros2 bag record -s mcap -o bag \$(cat $REPO/drone_bringup/config/bag_topics.txt)'"
tmux new-window -t sim -n cpu \
  "bash -ic 'ros_env; python3 $REPO/drone_sim/tools/cpu_logger.py --out $OUT/cpu.csv'"

SCRIPT="$REPO/drone_sim/tests/${ID//./_}.py"
RC=0
if [ -f "$SCRIPT" ]; then
  bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; timeout $TMO python3 $SCRIPT --out $OUT --world $WORLD" < /dev/null || RC=$?
  echo "procedure exit code $RC" | tee -a "$OUT/meta.txt"
else
  echo "No automated procedure for $ID. Perform the manual steps, then press Enter."; read -r
fi
tmux send-keys -t sim:rec C-c 2>/dev/null || true; sleep 3
"$REPO/scripts/sim_down.sh"
if [ -f "$REPO/drone_sim/tools/eval_run.py" ]; then
  bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tools/eval_run.py $OUT --world $WORLD" | tee "$OUT/report.txt"
fi
echo "results: $OUT"
exit $RC
