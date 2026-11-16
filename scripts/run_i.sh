#!/usr/bin/env bash
# Integration missions (guidebook 4.6): N full missions on one world, then the table.
# Usage: scripts/run_i.sh <I2|I3|I4|I5|I6> <world> [runs]
#   I2: run_i.sh I2 w3_tree_maze 10          I3: run_i.sh I3 w4_maze_rooms 10
#   I4: run_i.sh I4 w6_width_100 5 (also w6_width_130, w6_width_150, w7_gaps, w5_maze_loop)
#   I5: run_i.sh I5 w8_arena_full 10         I6: one run on each of wr_s101 ... wr_s130 (see I6 in the C guide)
# Each run: scripts/run_test.sh <ID> <world> full <timeout> (Gazebo, SITL, Cartographer, bridge, planner,
# frontier server, mission brain), results in ~/drone_results/<date>_<ID>_<world>_<git>/.
set -uo pipefail
ID=${1:?test id}; WORLD=${2:?world}; N=${3:-1}
REPO=$HOME/drone_ws/src
TMO=1800; [ "$WORLD" = w8_arena_full ] && TMO=2400
for i in $(seq 1 "$N"); do
  echo "### $ID $WORLD run $i of $N"
  "$REPO/scripts/run_test.sh" "$ID" "$WORLD" full "$TMO" || true
done
bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tests/$ID.py --summary $N --world $WORLD" || true
