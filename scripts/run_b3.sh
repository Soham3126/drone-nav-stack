#!/usr/bin/env bash
# B3 (guidebook 4.4): one hover per case in drone_sim/tests/b3_cases, then the summary table.
# Usage: scripts/batch.sh b3 "scripts/run_b3.sh"        (about 25 minutes for the 6 cases)
REPO=$HOME/drone_ws/src
for f in "$REPO"/drone_sim/tests/b3_cases/*.yaml; do
  MANGLE=$f "$REPO/scripts/run_test.sh" B3 w0_box_room truth 600
done
echo "=== B3 summary"
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tests/B3.py --summary"
