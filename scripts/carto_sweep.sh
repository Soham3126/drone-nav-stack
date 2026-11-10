#!/usr/bin/env bash
# Replay several recorded runs with several settings and tabulate (guidebook A1.3 step 3, A1.9).
# Usage: scripts/carto_sweep.sh SWEEP_FILE RUN_DIR [RUN_DIR ...]
# SWEEP_FILE: one setting per line, "name | lua override" (empty override = current config), e.g.
#   base |
#   tw1  | TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
#   tw10 | TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
# Then: python3 drone_sim/tools/sweep_table.py RUN_DIR ...
set -eo pipefail
SW=${1:?sweep file}; shift
REPO=$HOME/drone_ws/src
for RUN in "$@"; do
  while IFS='|' read -r NAME OVR; do
    NAME=$(echo "$NAME" | xargs); [ -z "$NAME" ] && continue; [[ "$NAME" == \#* ]] && continue
    "$REPO/scripts/carto_replay.sh" "$RUN" "$NAME" "$(echo "$OVR" | xargs)" || echo "replay $NAME of $RUN failed"
  done < "$SW"
done
python3 "$REPO/drone_sim/tools/sweep_table.py" "$@"
