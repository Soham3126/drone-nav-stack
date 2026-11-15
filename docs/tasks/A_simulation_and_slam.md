# Workstream A: simulator and SLAM (Cartographer)

Goal: a simulator with exact ground truth, and a map-maker (Cartographer) whose accuracy is known.
Gates: **G1** (A0.1-A0.4) and **G2** (A1.1-A1.9 + `carto-v1` tag) - both done.

How to run any test: `scripts/run_test.sh <ID> <world> gps [timeout_s]` from `~/drone_ws/src`.
The test script is `drone_sim/tests/<ID with _ for .>.py` (A1.10 -> `A1_10.py`); it flies the drone with
`drone_sim/tools/scripted_flight.py` on simulated GPS and judges the run; `drone_sim/tools/eval_run.py` scores the map.

## A0: the platform

| Test | What it checks | Main files | World |
|---|---|---|---|
| A0.1 | Maze generator: worlds and true maps agree, start poses valid | `tools/make_maze.py`, `worlds/`, `maps/` | all |
| A0.2 | LiDAR and IMU give what Cartographer needs | `tests/A0_2.py`, `models/iris_with_lidar/model.sdf` | W0 |
| A0.3 | Ground-truth position is right; the simulation runs in real time | `tests/A0_3.py`, `tools/truth_relay.py` | W0 |
| A0.4 | Scripted flights repeat reliably with a safe wall margin | `tests/A0_4.py`, `tools/scripted_flight.py`, `tools/make_patterns.py`, `patterns/` | W0-W3 |

## A1: SLAM accuracy

| Test | What it checks | Main files | World |
|---|---|---|---|
| A1.1 | Still drone: steady pose, clean first map | `tests/A1_1.py` | W0 |
| A1.2 | Accuracy in an easy room | `tests/A1_2.py` | W0 |
| A1.3 | Slip along a featureless corridor | `tests/A1_3.py` | W1 |
| A1.4 | Heading and position through turns and junctions | `tests/A1_4.py` | W2 |
| A1.5 | Accuracy at mission scale | `tests/A1_5.py`, `tools/nav_check.py` | W3, W4 |
| A1.6 | Strong tilting (fast flight) | `tests/A1_6.py` | W4 |
| A1.7 | Floor seen as wall during take-off/landing | `tests/A1_7.py` | W1, W2 |
| A1.8 | Closing a loop without loop closure | `tests/A1_8.py` | W5 |
| A1.9 | Parameter sweep, choose and freeze `carto-v1` | `scripts/carto_sweep.sh`, `carto_replay.sh`, `tools/sweep_table.py`, `drone_slam/sweeps/` | replays |
| A1.10 | Cartographer's CPU and pose delay -> `VISO_DELAY_MS` | `tests/A1_10.py` | W4 |
| A1.11 | What happens when the LiDAR or IMU stream stops (documented) | `tests/A1_11.py`, `tools/stream_gate.py` | W3 |

## Results and decisions

* **`carto-v1`** (`drone_slam/config/drone_2d.lua`, git tag `carto-v1`): the book's configuration with loop closure on
  but only within 1 m (`max_constraint_distance` 1 m, so look-alike junctions elsewhere cannot be confused) and the
  map optimised every 45 scans (noisy-IMU sweep: W4 worst error 10.1 -> 9.7 cm, navigability 6/6).
* Tried but **not adopted**: `max_constraint_distance` 15 m with `optimize_every_n_nodes` 20 reduced the W4 heading
  creep in replays (worst error 15.2 -> 10.6 cm) but was never committed. All of workstream B ran on `carto-v1`.
  Worth re-testing in the integration tests.
* Confirmation on fresh flights: 7 of 9 passed. The two misses are the known limit: in long featureless corridors
  (W4/W5) the heading creeps slowly and is corrected only now and then. Diagnosed with `tools/where_error.py`.
* Keep the heading constant in missions: turning the nose along the path made the heading drift (A1.5 finding).
* A1.10: effective pose delay about 5 ms (B9 later measured 7.4 ms in flight) -> `VISO_DELAY_MS 10`.
* Simulation realism added along the way: MPU6500 IMU noise and wandering bias (`tools/imu_drift_sim.py`),
  gyro-bias removal on the ground (`drone_slam/scripts/imu_bias_node.py`), RPLIDAR C1 range noise.

Per-run numbers: [`results/INDEX.md`](../../results/INDEX.md). Every change from the book, with its reason: [`HUMBLE_NOTES.md`](../../HUMBLE_NOTES.md).
