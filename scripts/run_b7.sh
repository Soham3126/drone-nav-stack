#!/usr/bin/env bash
# B7 (guidebook 4.4): 3 runs without and 3 with optical-flow fusion, then the comparison (about 30 minutes).
REPO=$HOME/drone_ws/src
for rep in 1 2 3; do
  for fl in 0 1; do
    FLOW=$fl MANGLE=$REPO/drone_sim/tests/b7_slip.yaml "$REPO/scripts/run_test.sh" B7 w2_turns_junctions truth 900
  done
done
echo "=== B7 summary"
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tests/B7.py --summary"
