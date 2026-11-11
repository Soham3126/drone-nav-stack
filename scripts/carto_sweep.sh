#!/usr/bin/env bash
# Replay several recorded runs with several settings and tabulate (guidebook A1.3 step 3, A1.9).
# Usage: [JOBS=4] [RATE=2.0] scripts/carto_sweep.sh SWEEP_FILE RUN_DIR [RUN_DIR ...]
#   JOBS  replays run side by side, each on its own ROS domain (default 1). 4 at RATE=2 uses
#         about one third of a 20-core laptop. With JOBS>1 per-process CPU is not recorded.
# SWEEP_FILE: one setting per line, "name | lua override" (empty override = current config), e.g.
#   base |
#   tw1  | TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
# Settings already replayed into a folder with metrics.json are skipped (resume after a stop);
# FORCE=1 redoes them. Table at the end: python3 drone_sim/tools/sweep_table.py RUN_DIR ...
set -eo pipefail
SW=${1:?sweep file}; shift
REPO=$HOME/drone_ws/src; JOBS=${JOBS:-1}
LIST=$(mktemp)
for RUN in "$@"; do
  RUN=$(readlink -f "$RUN")
  while IFS='|' read -r NAME OVR; do
    NAME=$(echo "$NAME" | xargs); [ -z "$NAME" ] && continue; [[ "$NAME" == \#* ]] && continue
    if [ "${FORCE:-0}" != 1 ] && [ -f "${RUN}__replay_${NAME}/metrics.json" ]; then
      echo "skip (done): $NAME on $(basename "$RUN")"; continue; fi
    printf '%s\t%s\t%s\n' "$RUN" "$NAME" "$(echo "$OVR" | xargs)" >> "$LIST"
  done < "$SW"
done
N=$(wc -l < "$LIST"); echo "$N replays to do, $JOBS at a time, rate ${RATE:-1.0}"
worker() {   # worker k takes every JOBS-th line, on ROS domain 80+k
  local k=$1 i=0
  while IFS=$'\t' read -r RUN NAME OVR; do
    if [ $((i % JOBS)) -eq "$k" ]; then
      DOMAIN=$((80 + k)) QUIET=$([ "$JOBS" -gt 1 ] && echo 1 || echo 0) NOCPU=$([ "$JOBS" -gt 1 ] && echo 1 || echo 0) \
        "$REPO/scripts/carto_replay.sh" "$RUN" "$NAME" "$OVR" < /dev/null || echo "replay $NAME on $(basename "$RUN") did not finish"
    fi
    i=$((i + 1))
  done < "$LIST"
}
for k in $(seq 0 $((JOBS - 1))); do worker "$k" & done
wait
rm -f "$LIST"
python3 "$REPO/drone_sim/tools/sweep_table.py" "$@"
