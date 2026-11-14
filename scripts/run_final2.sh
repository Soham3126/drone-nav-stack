#!/usr/bin/env bash
# Workstream A, last replay check (batch "final2"):
#  1. retry the IMU "down"-update setting under its newer name (pg1, pg30) on the 5 key W4/W3 flights
#  2. regression: replay every other sign-off flight with base, mcd15_n20 and mcd6_n20
# Usage:  cd ~/drone_ws/src && bash scripts/run_final2.sh
# Check:  scripts/batch.sh status      Result: newest ~/drone_results/batch_*_final2.log
set -eo pipefail
cd "$HOME/drone_ws/src"
R=$HOME/drone_results

M='POSE_GRAPH.constraint_builder.max_constraint_distance = 6. POSE_GRAPH.optimize_every_n_nodes = 20'
L='POSE_GRAPH.constraint_builder.max_constraint_distance = 15. POSE_GRAPH.optimize_every_n_nodes = 20'
P='TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant'

printf '# IMU gravity time constant, newer key (the old key had no effect)\npg1 | %s %s = 1.\npg30 | %s %s = 30.\n' \
  "$M" "$P" "$M" "$P" > drone_slam/sweeps/a1_9_grav2.txt
printf '# regression on all other sign-off flights\nbase |\nmcd15_n20 | %s\nmcd6_n20 | %s\n' \
  "$L" "$M" > drone_slam/sweeps/a1_9_regress.txt

RUNS2="$(ls -d $R/*_A1.6_w4_maze_rooms_e7a1f8f $R/*150150_A1.5_w4_maze_rooms_e7a1f8f | grep -v __replay | tr '\n' ' ') \
$(ls -d $R/*_A1.5_w3_tree_maze_e7a1f8f | grep -v __replay | head -1)"
OTHERS=$(ls -d $R/*_e7a1f8f $R/*_d351baa 2>/dev/null | grep -v __replay \
  | grep -v -E 'A1\.1_|A1\.10_|A1\.11_|w4_maze' | tr '\n' ' ')

N1=$(echo $RUNS2 | wc -w); N2=$(echo $OTHERS | wc -w)
echo "key flights: $N1 (expect 5)    other sign-off flights: $N2 (expect about 15-20)"
echo "other flights:"; for d in $OTHERS; do echo "  $(basename $d)"; done
[ "$N1" -ne 5 ] && { echo "STOP: expected 5 key flights"; exit 1; }
[ "$N2" -lt 5 ] && { echo "STOP: too few other flights found"; exit 1; }

scripts/batch.sh final2 "JOBS=5 RATE=2 scripts/carto_sweep.sh drone_slam/sweeps/a1_9_grav2.txt $RUNS2; \
JOBS=5 RATE=2 scripts/carto_sweep.sh drone_slam/sweeps/a1_9_regress.txt $OTHERS; \
echo TABLE_KEY_FLIGHTS_W4_W3; python3 drone_sim/tools/sweep_table.py $RUNS2; \
echo TABLE_OTHER_SIGNOFF_FLIGHTS; python3 drone_sim/tools/sweep_table.py $OTHERS"
