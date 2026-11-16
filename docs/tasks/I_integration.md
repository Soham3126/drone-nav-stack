# Integration series I1-I7 (guidebook 4.6)

The whole stack in Gazebo: ArduPilot SITL, Cartographer (carto-v1), the bridge (on Cartographer's pose), the
Nav2 planner server, the frontier server and the mission brain. Start-up profile `full` in
`drone_bringup/launch/sim.launch.py` (= `slam` + planner + frontier server + mission brain).

| Test | What | How | Status |
|---|---|---|---|
| gate | B5 re-run on the fixed 0.6 m lookahead (the bend-aware follower failed: 23 cm into corners, 2 runs; bridge reverted) | `run_test.sh B5 w2_turns_junctions truth` x3 at 0.8 m/s | **PASS** 3/3 (20261116_230820, _231054, _231330): lateral <= 5.7 cm straight / 11.1 cm corners, clearance >= 12.8 cm, arrival <= 4.9 cm |
| I1 | Hover 60 s + B5's paths on Cartographer's pose (W2): drift < 0.08 m, lateral <= 0.15 m, clearance >= 0.05 m | `run_test.sh I1 w2_turns_junctions slam 900` | pending |
| I2 | W3, 10 full missions | `run_i.sh I2 w3_tree_maze 10` | pending |
| I3 | W4, 10 full missions | `run_i.sh I3 w4_maze_rooms 10` | pending |
| I4 | W6 x3 widths, W7, W5: 5 each; 0.45 m gap never entered | `run_i.sh I4 <world> 5` | pending |
| I5 | W8, 10 missions with flow as decided in B7 (+5 the other way) | `run_i.sh I5 w8_arena_full 10` | pending |
| I6 | 30 random mazes wr_s101-130 | - | pending |
| I7 | Soak: W8 x10 unattended, memory growth < 10 % per node | - | pending |

Per mission (I2-I6, `tests/I2.py` + `tests/i_common.py`), from Gazebo's ground truth: DONE; coverage >= 95 % of
the drone-reachable floor known free in the final /map; landing within 0.3 m of the start; no contact (0.27 m
circle more than 2 cm into a wall); ATE < 0.15 m (Cartographer vs truth while airborne); bridge frame offset
< 0.10 m.
