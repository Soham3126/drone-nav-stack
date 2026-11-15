# Indoor maze drone: navigation stack

A GPS-free navigation stack for a small quadcopter that explores indoor mazes: it maps with a 2D LiDAR
(Cartographer SLAM), flies on its own position through ArduPilot, and will plan routes and explore on its own.
Built and tested in simulation first, following a team *Navigation Stack Guidebook*, on **Ubuntu 22.04 + ROS 2 Humble**,
**Gazebo Harmonic** and **ArduPilot Copter 4.7.2 SITL**.

| Workstream | What it delivers | Status |
|---|---|---|
| A: simulator and SLAM | Maze worlds with exact ground truth; Cartographer with measured accuracy | Done: frozen as tag `carto-v1` |
| B: flight interface | Bridge between ROS and the flight controller; ArduPilot settings for flying without GPS | Done: tag `g3-flight-interface`, one open item (B6) |
| C: planner and mission | Safe routes, frontier finding, mission logic | In progress ([guide](docs/tasks/C_planner_and_mission.md)) |

Start here:
* [`docs/PROJECT_SUMMARY.md`](docs/PROJECT_SUMMARY.md): the whole project on one page
* [`docs/tasks/`](docs/tasks): every test of workstreams A and B: what it checks, its files, how to run it, its result
* [`results/INDEX.md`](results/INDEX.md): the final run of every test (reports and numbers)
* [`HUMBLE_NOTES.md`](HUMBLE_NOTES.md): every difference from the guidebook, and why
* [`setup/INSTALL.md`](setup/INSTALL.md): installing everything on a fresh computer

## Repository layout

This repository is the `src/` folder of a ROS 2 workspace: clone it to `~/drone_ws/src` (the scripts expect that path).

| Folder | What it holds |
|---|---|
| `drone_description/` | The drone's body and sensor positions (URDF) |
| `drone_interfaces/` | Our ROS message and service types (bridge status, flight commands, frontiers) |
| `drone_sim/` | Maze generator (`tools/make_maze.py`), 31 maze worlds and their true maps, the simulated drone, test tools and **all test scripts** (`tests/A*.py`, `tests/B*.py`) |
| `drone_slam/` | Cartographer configuration `carto-v1`, launch file, gyro-bias removal, parameter-sweep records |
| `drone_bringup/` | Launches everything for a profile; ArduPilot parameter files and the Lua fallback script (`params/ardupilot/`) |
| `drone_bridge/` | The ROS-to-flight-controller bridge (MAVLink) and its 64 unit tests |
| `drone_nav/` | Route-planner configuration (for workstream C) |
| `scripts/` | Start/stop the simulation, run a recorded test, run batches, collect results |
| `setup/` | Install guide and the `ros_env` / `ap_env` shell shortcuts |
| `docs/`, `results/` | Summary, per-test guides, final test results |

## Quick start (after [setup/INSTALL.md](setup/INSTALL.md))

```
ros_env
cd ~/drone_ws && colcon build --symlink-install && cd src
scripts/run_test.sh B8 w0_box_room truth 1500       # one recorded test: 7 take-offs, judged automatically
scripts/batch.sh b6 "scripts/run_b6.sh"              # a batch of runs in the background
scripts/batch.sh status                              # its progress
scripts/sim_up.sh w2_turns_junctions slam            # or start the simulation by hand (tmux attach -t sim)
scripts/sim_down.sh
```
Profiles: `gps` (flies on simulated GPS, used to test SLAM), `truth` (flies on the simulator's exact position
through the bridge), `slam` (flies on Cartographer's position through the bridge). Every run is saved in
`~/drone_results/<date>_<test>_<world>_<git commit>/` with its settings, logs, recording, CPU use and report.

## How it fits together

LiDAR + IMU -> **Cartographer** (map and position, 200 Hz) -> **bridge** -> ArduPilot (MAVLink: position 30 Hz,
targets 10 Hz) -> motors. The bridge also follows paths with a "carrot" 0.6 m ahead and runs commands (arm, take off,
land, hold). If our position stops, a Lua script on the flight controller switches to the optical-flow sensor and
the rangefinder, and lands after 8 s.

## Real drone (the target hardware)

Quadcopter with EMAX ECO II motors, MicoAir743 flight controller, 4S LiPo, Raspberry Pi 5 + AI HAT+, RPLIDAR C1,
MicoAir MTF-01P optical flow + rangefinder, MPU6500 IMU, IMX708 camera, M8N GPS, Flysky receiver, SiK telemetry.
Settings marked `RE-SET` in `drone_bringup/params/ardupilot/sitl_extnav.parm` must be measured on the real drone
(see the summary).

The guidebook itself is not part of this repository.
