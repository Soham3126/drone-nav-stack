#!/usr/bin/env bash
# B5 (guidebook 4.4): speeds 0.5/0.8/1.0 m/s x lookahead 0.5/0.6/0.7 m x 3 repeats = 27 runs, then the summary.
# Usage: scripts/batch.sh b5 "scripts/run_b5.sh"        (about 1.5-2 hours)
#        REPEATS=1 scripts/run_b5.sh                     (one pass of the 9 combinations)
REPO=$HOME/drone_ws/src
for rep in $(seq 1 "${REPEATS:-3}"); do
  for sp in 0.5 0.8 1.0; do
    for L in 0.5 0.6 0.7; do
      echo "### B5 repeat $rep: speed $sp, lookahead $L"
      SPEED=$sp LOOK=$L "$REPO/scripts/run_test.sh" B5 w2_turns_junctions truth 900
    done
  done
done
echo "=== B5 summary"
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tests/B5.py --summary"
