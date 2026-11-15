# drone_frontier

The edge-finder (guidebook 2.7): finds frontiers (where explored free space meets unknown space) in `/map` and
publishes them once a second, cheapest first. It sends no commands; the mission brain chooses among them.

| File | Job |
|---|---|
| `drone_frontier/frontier_core.py` | The search, no ROS: map conversion (free < 65, wall >= 65, -1 unknown), reachable free space from the drone, frontier cells, groups, size filter (0.55 m), cost |
| `drone_frontier/frontier_server.py` | The node: `/map` + TF map -> base_link in; `/frontiers`, `/frontiers/markers`, `/frontiers/search_ms` out |
| `test/` | 9 unit tests (`colcon test --packages-select drone_frontier`) |

Run: `ros2 run drone_frontier frontier_server` (parameters: `rate_hz`, `free_threshold`, `occupied_threshold`,
`min_frontier_size`, `potential_scale`, `gain_scale`). Tested on all fixtures by test C3.
