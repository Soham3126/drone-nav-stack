# drone_slam

Cartographer 2D SLAM configuration, frozen as git tag `carto-v1`.

* `config/drone_2d.lua`: the configuration `carto-v1` (loop closure only within 1 m, optimised every 45 scans; publishes map -> base_link directly)
* `launch/slam.launch.py`: starts Cartographer and the occupancy grid
* `scripts/imu_bias_node.py`: removes gyro bias, measured while the drone is still on the ground (`/imu_raw` -> `/imu`)
* `sweeps/`: records of the A1.3 and A1.9 parameter sweeps that chose the configuration
