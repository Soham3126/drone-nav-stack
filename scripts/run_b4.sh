#!/usr/bin/env bash
# B4 (guidebook 4.4): one run per scenario in drone_sim/tests/b4_cases, then the summary.
# Usage: scripts/batch.sh b4 "scripts/run_b4.sh"        (about 12 minutes for the 4 scenarios)
REPO=$HOME/drone_ws/src
for f in "$REPO"/drone_sim/tests/b4_cases/*.yaml; do
  MANGLE=$f "$REPO/scripts/run_test.sh" B4 w2_turns_junctions truth 600
done
echo "=== B4 summary"
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tests/B4.py --summary"
