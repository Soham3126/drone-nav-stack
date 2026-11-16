# drone_mission

The mission brain (guidebook 2.8): the drone's only commander. Take off, explore until no frontiers remain,
return to the start, land.

| File | Job |
|---|---|
| `drone_mission/mission_core.py` | The decisions, no ROS: states WAIT_READY -> TAKEOFF -> HOVER_CHECK -> EXPLORE -> RETURN -> LAND -> DONE (STOP on any error: lands) and the exploration loop |
| `drone_mission/mission_brain.py` | The node: `/bridge/status`, `/frontiers`, `/map`, TF and the planner in; `/bridge/follow_path`, `/bridge/command`, `/mission/state`, `/mission/events` out; services `/mission/start`, `/mission/abort` |
| `launch/stub.launch.py` | The whole decision layer on the stub world (a fast stand-in drone, `drone_sim/tools/stub_world.py`) |
| `test/` | 7 unit tests of the decisions (`colcon test --packages-select drone_mission`) |

## Exploration loop (every 0.5 s)

1. Every 2 s, or when the goal is reached: take the frontiers (minus blacklisted ones), keep the current goal
   unless another is clearly cheaper (20 % and at least 1.5 cost units, checked on real path lengths), plan a path
   to the goal's middle cell and send it to the bridge.
2. A frontier the planner cannot reach: try points on the way to it ("approach"), else blacklist it for 30 s
   (3 failures: for good).
3. No progress toward the goal for 20 s: blacklist it.
4. No frontiers left for 3 updates: RETURN to (0, 0), LAND.
5. Stuck next to a wall (no path to anywhere): back off 0.4 m along the way it came.
6. `/frontiers` silent: keep the current goal, choose nothing new. `/bridge/status` silent for 3 s: STOP.

Try it (watch in RViz):
```
ros2 launch drone_mission stub.launch.py world:=w3_tree_maze speed_factor:=2 rviz:=true
ros2 service call /mission/start std_srvs/srv/Trigger
```
