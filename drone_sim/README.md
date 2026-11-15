# drone_sim

The simulated world and all test code.

| Folder | What it holds |
|---|---|
| `worlds/`, `maps/` | 31 maze worlds (`.sdf`) and their true maps (`.pgm` + `meta.json` with walls, start pose, reachable areas). Made by `tools/make_maze.py` |
| `models/iris_with_lidar/` | The simulated drone: ArduPilot iris + LiDAR, IMU (MPU6500 noise), downward rangefinder |
| `fixtures/` | Test maps for workstream C (full / partial / Cartographer-like per world), made by `tools/make_fixtures.py` |
| `patterns/` | Scripted flight paths per world (for the A tests, flown on simulated GPS) |
| `tests/` | One script per test (`A0_1.py` ... `B9.py`, `C0.py`, `C1.py` ...) plus shared code (`b_common.py`, `a1_common.py`, `sim_common.py`, `c_common.py`) and case files |
| `tools/` | Helper nodes and analysis tools (below) |

Main tools: `make_maze.py` (worlds), `make_fixtures.py` (C test maps), `make_patterns.py`, `scripted_flight.py` (GPS flights), `truth_relay.py`
(exact position), `range_relay.py` (simulated MTF-01P: rangefinder + optical flow over MAVLink), `pose_mangler.py`
(adds delay, noise, jumps to the position), `stream_gate.py` (pauses a topic), `imu_drift_sim.py` (gyro bias drift),
`eval_run.py` (scores a run), `nav_check.py`, `where_error.py`, `render_mazes.py`, `cpu_logger.py`, `cpu_probe.py`.

Tests are run through `scripts/run_test.sh <ID> <world> <profile>`; see `docs/tasks/`.
