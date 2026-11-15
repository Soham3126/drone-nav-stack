# scripts

Run from `~/drone_ws/src`. All of them expect the repository at that path and the `ros_env` / `ap_env` shortcuts
(`setup/env.sh`).

| Script | Job |
|---|---|
| `sim_up.sh <world> <profile> [out]` / `sim_down.sh` | Start / stop Gazebo, ArduPilot SITL and ROS in tmux session `sim`. Options by environment: `HEADLESS`, `FLOW`, `MANGLE`, `SLAM`, `GATE`, `CLOCK_THROTTLE` |
| `run_test.sh <ID> <world> <profile> [timeout]` | One recorded test run: settings, logs, recording, CPU use, report in `~/drone_results/` |
| `batch.sh <name> "<commands>"` / `batch.sh status` | Run many tests in the background (tmux session `batch`), check progress |
| `run_b3.sh` ... `run_b9.sh`, `run_final2.sh` | The batches used for B3-B9 and the A regression |
| `carto_sweep.sh`, `carto_replay.sh`, `replay.launch.py` | Cartographer parameter sweeps on recorded flights (A1.9) |
| `profile_bridge.sh`, `probe_cpu.sh` | Where the bridge spends its CPU (B9) |
| `collect_results.sh` | Copy each test's final run summary into `results/` |
