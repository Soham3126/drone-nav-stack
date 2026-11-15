# drone_bridge

The bridge between ROS and the flight controller (ArduPilot, over MAVLink). Python, no C++.

| File | Job |
|---|---|
| `drone_bridge/bridge_node.py` | The ROS node: sends our position (VISION_POSITION_ESTIMATE, 30 Hz, never stale or repeated), follows `/bridge/follow_path` with the carrot, runs `/bridge/command` (guided, arm, takeoff, land, hold), publishes `/bridge/status` (10 Hz) and `/bridge/link_stats` (every 5 s) |
| `drone_bridge/mavlink_link.py` | The MAVLink connection: reader thread, heartbeat watch, reconnect |
| `drone_bridge/commands.py` | Commands with acknowledgement, 3 s wait, 2 retries, refusal messages |
| `drone_bridge/carrot.py` | Path following: a target point 0.6 m ahead along the path; arrival check |
| `drone_bridge/offset.py` | Tracks the offset between the SLAM map frame and the flight controller's frame |
| `drone_bridge/frames.py` | Coordinate conversions (ROS FLU/ENU to ArduPilot NED, angles) |
| `test/` | 64 unit tests (test B0), no simulator needed |

Parameters (ROS): `fc_url` (tcp:127.0.0.1:5762), `pose_source` (tf or topic), `pose_topic`, `lookahead_m` (0.6, can
be changed live), `pose_timeout_s` (0.3), `ground_clearance_m`, `flight_height_m`.

Run the unit tests: `cd ~/drone_ws && colcon test --packages-select drone_bridge && colcon test-result --verbose`
