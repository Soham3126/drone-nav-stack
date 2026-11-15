#!/usr/bin/env bash
# B6 (guidebook 4.4): bridge freeze, bridge kill, pose-stream pause (5 s) - 2 runs each - plus one 15 s pose pause
# to document a long outage (beyond ArduPilot's 10 s position timeout). About 18 minutes.
REPO=$HOME/drone_ws/src
for rep in 1 2; do
  B6_CASE=freeze "$REPO/scripts/run_test.sh" B6 w2_turns_junctions truth 600
  B6_CASE=kill   "$REPO/scripts/run_test.sh" B6 w2_turns_junctions truth 600
  B6_CASE=pose GATE=1 "$REPO/scripts/run_test.sh" B6 w2_turns_junctions truth 600
done
B6_CASE=pose_long GATE=1 "$REPO/scripts/run_test.sh" B6 w2_turns_junctions truth 600
