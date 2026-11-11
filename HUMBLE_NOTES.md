# Deviations from the guidebook (v1.1)

The guidebook targets Ubuntu 24.04 + ROS 2 Jazzy. This repository runs on
**Ubuntu 22.04 + ROS 2 Humble**. Every difference is listed here; anything not listed
follows the book.

## Machine set-up (chapter 3)
| Book | Here | Why |
|---|---|---|
| `ros-jazzy-*` packages | `ros-humble-*` | Humble is the ROS 2 release for 22.04 |
| `ros-jazzy-ros-gz` | `ros-humble-ros-gzharmonic*` (OSRF repo) | Gazebo Harmonic is not Humble's default Gazebo |
| `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | `ROS_LOCALHOST_ONLY=1` | the Jazzy variable does not exist in Humble |
| `pip install --break-system-packages` | plain `pip install --user` | 22.04 does not mark system Python as externally managed |
| Python 3.12 | Python 3.10 | Ubuntu 22.04 |
| ArduPilot venv `~/venv-ardupilot` | system Python | ArduPilot's installer did not create a venv on this machine |
| — | scipy 1.15 from pip (`~/.local`) | Ubuntu's scipy 1.8 does not work with numpy 2.x |
| — | NVIDIA PRIME variables in `ros_env` | laptop with hybrid graphics; Gazebo must render on the RTX GPU (RTF 0.93 → 0.98) |
| `git describe --tags` = `Copter-4.7.2` | use `git describe --tags --exact-match --match 'Copter-*'` | the 4.7.2 commit carries ~20 tags; plain describe may print another one |

## Code and configuration
| File | Change | Why |
|---|---|---|
| `drone_slam/config/drone_2d.lua` | added `use_pose_extrapolator = true` | required key in cartographer_ros 2.0.900x; missing from the book's listing |
| `drone_nav/config/planner.yaml` | planner plugin `nav2_smac_planner/SmacPlanner2D` | Humble (Nav2 1.1) plugin name form |
| `drone_sim/tests/A0_1.py` | start clearance tested as ≥ 0.475 m to the wall **face** | the book's "≥ 0.5 m from walls" is impossible in a 1.0 m corridor (centre line is 0.475 m from each face, 0.5 m from each wall's centre line) |
| `drone_sim/tools/make_maze.py` | writes two reachability masks: `reachable_free` (all free cells joined to the start) and `reachable_drone` (what the 0.27 m drone circle can sweep) | in W7 the room behind the 0.45 m gap is free space the drone can never enter |
| `drone_sim/tools/make_maze.py` | W8 tall box is a 0.25 m column (not specified in the book) | a 0.4 m box in the middle of a 2 x 2 m room leaves 0.30 m to the room's flight lines, below the 0.35 m scripted-flight margin, and cut the W8 tour to 9 of 225 cells |
| `drone_sim/tools/make_patterns.py` | scripted-flight patterns keep >= 0.35 m from every wall face (0.27 m radius + GPS margin); flown as cell-to-cell straight lines | book says "waypoint YAML per world, generated from meta.json" without a rule |
| `drone_sim/tools/eval_run.py` | "phantom cluster" = 8-connected phantom cells whose bounding-box diagonal is > 0.20 m; A1.2's "no phantom clusters" judged the same way | the book defines length limits only in A1.5/A1.6 |
| all A1 flights | take-off heading held, as the bridge will fly (book 2.9: "yaw = heading at takeoff"); `YAW=travel` turns the nose along each leg as an optional stress test | first A1.5 used nose-along-path: **finding** - Cartographer's heading drifted to 3.7 deg over 84 corners at up to 62 deg/s yaw (ATE 0.24 m, precision 0.60), while the same W3 lap with heading held gave ATE 0.06-0.07 m, precision > 0.99. Keep heading constant in missions; re-test before any yaw-sweep feature |
| `drone_sim/tools/eval_run.py` | heading error "after a turn" = 1 s after the drone starts moving along a new direction (path direction change > 45 deg) | book gives no exact moment; works with heading held or turning |
| `drone_sim/tools/eval_run.py` | A1.1 drift = movement of the **1 s average** pose over 15-60 s; raw jitter reported separately (not judged) | first A1.1 run: no creep (slope 0.1-0.5 mm / 45 s, net 0.7 mm) but ~200 Hz IMU-extrapolated poses shake by 2.3 mm std, 11 mm peaks; the raw-sample maximum measured jitter, not drift. Jitter stays well inside the 2 cm settle check (2.10) |
| `drone_sim/tools/eval_run.py` | known-free = map value 0..64 (Nav2 static layer, trinary) | Cartographer's free cells sit near 40-49; the 25 threshold gave coverage 0 |
| `drone_sim/tools/eval_run.py` (A1.8) | doubling judged on the whole map (99th-percentile distance of mapped walls from true walls) and free width at every seen corridor cell, not only at the loop's closing point | the closing point is not stored in the world description; a whole-map check is stricter and includes it |
| `drone_sim/tests/A1_7.py` | style (b) uses WP_SPD_UP 0.1 m/s; style (c) hovers at 0.3 m, steps 0.3 m to the left of the heading and back | book gives "e.g. WP_SPD_UP lowered" without a value |
| `drone_sim/tools/scripted_flight.py` | locks onto the autopilot's heartbeat (not the first heartbeat seen) | a ground-station heartbeat gave target system 0 in a SITL test |
| `scripts/run_test.sh`, `scripts/batch.sh` | no interactive shells in non-tmux steps; batches run in tmux session "batch" | an interactive shell in a background job is stopped by Linux job control ("Stopped (tty input)") |
| `drone_sim/models/iris_with_lidar/model.sdf` | IMU noise and bias of the BOM's MPU6500, LiDAR noise 0.02 m (RPLIDAR C1) | the noise-free IMU made the simulation optimistic about heading drift; results before this change (git 7d425fb and earlier) used a perfect IMU |
| `drone_sim/tools/nav_check.py` | added: navigability check (map vs truth in drone-centre space: intrusion into walls, cells cut off) | A1.5's global-accuracy limits do not say whether a map is safe to plan in; decides how much W4's A1.5 shortfall matters |
| `scripts/sim_up.sh` | calls `sim_vehicle.py` by full path; refuses to start twice; `SLAM=false` option | robustness |

## To verify on first use
* `WP_SPD`, `RNGFND1_MIN`, `ARMING_SKIPCHK` are Copter 4.7 names taken from the book. After `sim_up.sh`,
  in the `sitl` tmux window run `param show WP_SPD` etc. An unknown name means the 4.7 rename differs.
* Nav2 Humble's `ComputePathToPose` result may not carry `error_code` (codes 205-208 in the book).
  Checked when the mission brain is written.

## Gemini prototype (archived, not used)
Kept in `~/drone_gemini_archive_<date>/` for reference. Replaced because:
* `iris_with_lidar` included only `iris_with_standoffs` plus a hand-copied ArduPilot plugin
  (missing lift-drag and IMU wiring); the book includes the full `iris_with_ardupilot`.
* `carto_2d.lua` published `odom` and relied on hand-made static transforms; no height filter,
  motion filter or loop-closure-off settings.
* `drone_autonomy` used MAVROS, swapped x/y in the ENU→NED conversion (wrong for our map frame,
  see book 2.3), NavFn with `allow_unknown: true`, radius 0.35 m, and explore_lite as commander.
