#!/usr/bin/env bash
# One recorded run of a workstream C test (no Gazebo, no ArduPilot: the C tests work on map files).
# Usage: scripts/run_c.sh <TEST_ID> [label] [-- extra arguments for the test]
#   e.g. scripts/run_c.sh C0          scripts/run_c.sh C1
# Results: ~/drone_results/<date>_<ID>_<label>_<git hash>/ with meta.txt, node logs, result.json, report.txt
set -uo pipefail
ID=$1; LABEL=${2:-fixtures}; shift; [ $# -gt 0 ] && shift; [ "${1:-}" = "--" ] && shift
REPO=$HOME/drone_ws/src
GIT=$(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo "-dirty")
OUT=$HOME/drone_results/$(date +%Y%m%d_%H%M%S)_${ID}_${LABEL}_${GIT}
mkdir -p "$OUT"
printf 'test=%s\nlabel=%s\ngit=%s\n' "$ID" "$LABEL" "$GIT" > "$OUT/meta.txt"
if pgrep -f "map_server|planner_server|lifecycle_manager" >/dev/null; then
  echo "map_server / planner_server already running (another test or the simulation?): stop it first"; exit 1
fi
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; export ROS_DOMAIN_ID=\${C_DOMAIN:-42}; \
  python3 $REPO/drone_sim/tests/${ID}.py --out $OUT $*" < /dev/null
RC=$?
echo "procedure exit code $RC" | tee -a "$OUT/meta.txt"
echo "results: $OUT"
exit $RC
