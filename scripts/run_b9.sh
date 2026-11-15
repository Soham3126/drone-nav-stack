#!/usr/bin/env bash
# B9 (guidebook 4.4): latency and load, W2, slam profile; phase 1 no load, phase 2 under stress-ng --cpu 4.
# Usage: scripts/batch.sh b9 "scripts/run_b9.sh"       (2 runs, about 12 minutes; needs: sudo apt install stress-ng)
REPO=$HOME/drone_ws/src
for rep in $(seq 1 "${REPEATS:-2}"); do
  echo "### B9 run $rep"
  "$REPO/scripts/run_test.sh" B9 w2_turns_junctions slam 1500
done
