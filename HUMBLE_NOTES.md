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
| `drone_slam/scripts/imu_bias_node.py` | added: gyro bias removal (`/imu_raw` -> `/imu`), estimated whenever the drone is still on the ground | noisy-IMU flights: heading crept +0.01 deg/s to 3 deg (39 cm error) in the bad W4/A1.6 runs; standard practice on real drones |
| `drone_sim/tools/imu_drift_sim.py` | sim only: wandering gyro bias (Gauss-Markov, 1e-4 rad/s, 300 s) on top of Gazebo's constant bias | otherwise ground calibration would remove the simulated bias completely; replace with real MPU6500 values |
| `drone_sim/tests/A1_7.py` | style (c): side-step of 0.15 m at 0.5 m height | a 0.3 m step hits the wall of the 1 m corridor (drone radius 0.27 m): the earlier 'disarmed in flight' was a collision |
| `drone_sim/tools/eval_run.py` (A1.8) | judged by local wall checks around the drone (nav_check): misplaced and missing walls p99 < 0.10 m | the global wall offset measured overall drift, and corridor_free_width was shown wrong (0.3 m in the box room) |
| `drone_sim/tools/stream_gate.py` | lives in drone_sim (book lists it under B) and is wired before Cartographer with `GATE=1` | needed for A1.11 before drone_bridge exists |
| `drone_sim/tests/A1_10.py` | effective delay found by matching velocities (not positions) of the received pose stream against ground truth | drift and loop-closure corrections shift positions by a constant and biased a position-based fit (synthetic check: 63 ms true -> 70 ms found) |
| `drone_sim/tests/A1_11.py` | pauses during hover and during flight: imu 2 s, scan 2 s, imu 10 s, scan 10 s, 15 s apart; drone flies on GPS throughout | book gives the pauses but not the order or spacing |
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
| `drone_slam/config/drone_2d.lua` (A1.9, W4 heading creep) | `max_constraint_distance` 1 -> 15 m, `optimize_every_n_nodes` 45 -> 20 | `where_error.py` showed W4 error is slow heading creep (to 3 deg at 1.5 m/s) corrected only rarely; the distance is measured from each submap's origin, so 1 m allowed corrections near a few points only. Replays (8 W4/W3 flights): worst ATE 15.2 -> 10.6 cm, no phantoms. Regression on 30 other sign-off flights: A1.2-A1.5 unchanged; A1.8 (W5) worst wall misplacement 13.7 -> 10.9 cm (both settings borderline vs 10 cm). Ruled out: rotation_weight 40 (broken maps), no IMU, IMU gravity time constant (both keys), smaller submaps, freer correlative rotation. A1.7 replays unusable (short take-off bags), real flights stand |
| `drone_sim/tools/where_error.py`, `render_mazes.py`, `scripts/run_final2.sh` | added: error-along-path plot, maze images, final regression batch | diagnosis tools |
| `drone_bridge/` (B0) | maths and command logic split into `frames.py`, `carrot.py`, `offset.py`, `commands.py` (no ROS, no MAVLink library) so B0 tests them without a simulator; `commands.py` talks to any object with `send_command()`/`recv()` | book lists frames.py and carrot.py; the offset estimator and command logic are also B0 items, so they get their own testable files |
| `drone_bridge/carrot.py` | closest-point search looks at most `2L + 0.5` m ahead of the current progress | the book says "search forward from the last one"; without a limit a U-turn or loop passing close by would make the carrot jump ahead |
| `drone_sim/tools/pose_mangler.py` | delayed poses keep their original time stamp; roll/pitch preserved, yaw step applied about the vertical; `/pose_mangler/reset_counter` published for B4's "step with reset_counter incremented" | a slow SLAM delivers old-stamped poses; the bridge copies the counter into VISION_POSITION_ESTIMATE |
| `drone_bridge/mavlink_link.py` | MAVLink connection split from `bridge_node.py` (no ROS) and tested against a fake TCP autopilot: heartbeat, sys 1 / comp 191, command ACK and refusal text, reconnect < 5 s | B1 step 4's reconnect logic can be proven before SITL; a closed TCP socket is detected at once (pymavlink returns "nothing" immediately instead of waiting) |
| `drone_bridge/bridge_node.py` | `/bridge/link_stats` (JSON text, every 5 s: received Hz per message, connection count) added | B1 needs the received rates; the book does not name a topic for them |
| `drone_bridge/bridge_node.py` | `takeoff` returns after ACK **and** height within 0.1 m (20 s limit); `land` after mode 9 **and** disarm (60 s limit); `hold` = position target at the current FC position, repeated at 10 Hz until a new path arrives | book's "Confirmed by" column, made explicit |
| `scripts/sim_up.sh` | SITL command saved to `<out>/sitl_cmd.sh`; SLAM off by default in the truth profile | B1/B2 restart ArduPilot alone; Cartographer is not used when the pose is ground truth |
| `drone_sim/tests/B1.py`, `B2.py`, `b_common.py` | bridge's own HEARTBEAT counted on SITL's spare port 5763 (ArduPilot forwards it); B2 runs its cases in the order happy path -> ArduPilot stopped -> restart + arm at once | stopping ArduPilot is also how the refusal case gets a fresh, not-yet-ready EKF |
| `drone_bringup/params/ardupilot/sitl_extnav.parm` | `VISO_DELAY_MS 10` | A1.10 measured ~5 ms effective delay |
| `drone_sim/tests/b_common.py` (first B1/B2 run) | ArduPilot is stopped with `pkill` after Ctrl+C; the test's spare-port link sends a GCS HEARTBEAT | first run: MAVProxy ignores Ctrl+C and arducopter outlived its tmux window, so "restart" never happened; ArduPilot forwards messages only to links it has heard from, so the bridge's HEARTBEAT was invisible on 5763 |
| `drone_bridge/bridge_node.py` | height = rangefinder only when inside its min/max, else the EKF height; raw DISTANCE_SENSOR fields in `/bridge/link_stats` | first run: DISTANCE_SENSOR read 0 throughout, so take-off was judged "not reached" |
| `drone_bridge/mavlink_link.py` (second B1/B2 run) | "connected" = autopilot HEARTBEAT within the last 2 s (book 2.10), checked on every pass; the bridge forgets mode/armed/EKF/origin on every (re)connect; refusal STATUSTEXT collected for 0.5 s after the ACK | second run: after ArduPilot was killed the bridge kept reporting connected for >15 s and kept the old "EKF ok" flags, so B2 armed too early after a restart; one refusal arrived with its text after the ACK |
| `drone_bringup/params/ardupilot/sitl_base.parm` | `RNGFND1_TYPE 100` (SITL rangefinder), `RNGFND1_ORIENT 25` | gazebo-iris's analog sonar read 0 cm even at 1.1 m, so EKF3's rangefinder height source (EK3_SRC1_POSZ 2) had nothing; height came from the barometer fallback |
| `drone_sim/tests/b_common.py` | ArduPilot is found and stopped by process name only | MAVProxy renames its tmux window, so "sim:sitl" no longer exists by that name |
| `drone_sim/models/iris_with_lidar/model.sdf`, `gz_bridge.yaml`, bridge `rangefinder_topic` (third B1/B2 run) | downward one-beam range sensor (0.02-8 m, 20 Hz) in Gazebo; the bridge forwards it to ArduPilot as DISTANCE_SENSOR; `sitl_extnav.parm`: `RNGFND1_TYPE 10` (MAVLink), `RNGFND1_ORIENT 25`; `RNGFND1_TYPE 100` removed again | SITL type 100 gave "Rangefinder 1: No Data" with the JSON (Gazebo) model and gazebo-iris's analog sonar reads 0, so EKF3's rangefinder height (EK3_SRC1_POSZ 2) had no sensor. Stand-in for the MTF-01P ToF, which on the real drone is wired to the flight controller, not the bridge |
| `drone_sim/tests/B2.py` | arm retried every 5 s for up to 60 s while ArduPilot's own pre-flight checks settle after a restart (tries recorded) | "Arm: Accels inconsistent" / "Battery unhealthy" right after a SITL restart; book 2.10's "FC ready" check (no PreArm text in the last 5 s) waits for the same thing |
| `drone_sim/tests/B1.py` | link loss also proven by the bridge's connection counter going up; status trace around the restart saved | the 15 s status watch missed the loss twice although the reconnect was then measured |
| B1, B2 | **PASS** (commit 9089bc4): rates within 1 %, reconnect 0.10 s, happy path / refusal / timeout 3/3 each. After a SITL restart ArduPilot needed 3-4 arm tries (~15 s: "Accels/Gyros inconsistent") | first GPS-free flights; taking off to 1.1 m gave a true height of 1.02-1.03 m and the downward sensor reads 14 cm on the ground (sensor 5 cm below base_link) - ground offset to be settled in B8 |
| `drone_sim/tests/B3.py`, `b3_cases/`, `scripts/run_b3.sh` | six one-factor cases (delay 0/50/100/150 ms at noise 0; noise 2/5 cm at delay 50 ms) instead of the full 4x3 grid; VISO_DELAY_MS set to 50 for B3 through the spare port; drift = max distance from the hover start, oscillation = max distance from the hover mean (ground truth) | the book's pass rules only involve the matched case, delay mismatch at noise 0 and noise at matched delay |
| B3 | matched case 2/3 strictly (drift 4.1 / 5.1 / 3.1 cm, offset 1.5 / 2.1 / 1.4 cm; the miss is 1 mm on each); delay 0-150 ms and noise 2 cm all held within 4-8 cm; 5 cm noise -> 10 cm drift. One earlier matched run drifted 67 cm (offset 23 cm) and did not recur in 3 repeats - cause unknown, watched via the new hover plots | accepted in substance; VISO_DELAY_MS is not critical within +-100 ms |
| `pose_mangler.py` | services `/pose_mangler/step_now`, `/pose_mangler/drift_now` start the configured step/drift when called | B4/B7 need the disturbance while hovering at a known place, not at a fixed time after start-up |
| `drone_bridge/bridge_node.py` | `lookahead_m` can be changed while running (`ros2 param set /bridge lookahead_m 0.6`) | B5 sweeps L without restarting |
| `drone_sim/tests/B4.py` | hover point: the W2 crossroads, reached with the carrot; baseline 20 s, then 30 s after the step; documents drone movement, FC position jump, offset transient | book: "hover at the centre of a junction" |
| `drone_sim/tests/B5.py` | four paths in one flight: straight 8 m; east L-turn; crossroads -> T-junction -> right; south-arm U-turn -> home. Lateral error judged on straights only (more than L + 0.3 m from a corner); corners judged by clearance >= 0.08 m | the book's own corner-cut bound (0.354 L = 0.18-0.25 m) exceeds its 0.12 m lateral limit, so the two can only both hold if the lateral limit applies away from corners; clearance is the safety-relevant number at corners |
| B4 | **PASS** (undisturbed offset 0.8-1.8 cm). 0.3 m pose step without reset: drone moved back 0.30 m but **overshot to 0.61 m** first; FC position absorbed it at ~14 cm/s. Same step with reset_counter+1: moved 0.30 m with **no overshoot** (peak 0.31). 5 deg yaw step: drone moved 3 cm; the frame offset absorbed the rotation (4.6 deg, 39 cm at ~5 m from the origin) | a large SLAM jump fed without a reset can swing the drone twice as far as the jump - relevant for the integration failsafe (consider raising reset_counter on pose jumps > ~0.15 m) |
| B5 | **PASS 27/27**: straight-line error <= 7.6 cm, corners <= 14.2 cm, clearance >= 11.7 cm, arrival <= 6 cm everywhere. Speed is limited by the lookahead, not WP_SPD (L 0.5: ~108 s whatever WP_SPD; L 0.7: 78 s). **Chosen: lookahead 0.6 m, WP_SPD 0.8 m/s** (91 s, clearance >= 11.7 cm) | bridge default lookahead 0.6; WP_SPD 0.8 in sitl_extnav.parm |
| `drone_sim/tools/range_relay.py` (replaces the bridge's rangefinder forwarding) | the simulated downward rangefinder reaches ArduPilot through its own MAVLink link (MAVProxy extra output udp 14570, `--out` added in sim_up.sh) | B6 freezes and kills the bridge: on the real drone the MTF-01P is wired to the flight controller and keeps working, so in simulation it must not go through the bridge either |
| `sitl_extnav.parm` | `RNGFND1_GNDCLR 0.145` | B3/B5 hovered ~0.15 m below target: the rangefinder reads 14.5 cm when landed; B8 reads the parameter back to confirm the name |
| `drone_bridge` | `ground_clearance_m` (0.145 in simulation): status height = rangefinder - clearance | status height should be height above the ground |
| `drone_sim/tests/B6.py` | pose pause done with stream_gate.py as a pose gate in the truth profile (`GATE=1`); after a kill a new bridge is started by the test and the rest of the path re-sent | book: "Repeat, killing the bridge outright" - resuming needs a new process |
| `drone_sim/tests/B7.py` | W2 only; the four scenarios (hover, straight, synthetic slip, flow loss) in one flight per run, 3 runs with and 3 without flow | W1's straight corridor is the same as W2's start corridor |
