# Workstream C: route-finder, edge-finder and mission brain

Goal: a planner configuration, a frontier server and a mission brain proven on hundreds of mazes, fast, before they
ever fly. Gate **G4** = C1-C6 pass. **In progress.**

The C tests need no Gazebo and no ArduPilot: they run on map files ("fixtures"), so a test takes seconds to minutes.
How to run any of them: `scripts/run_c.sh <ID>` from `~/drone_ws/src` (results in `~/drone_results/<date>_<ID>_...`).

| Test | What it checks | Files | Result |
|---|---|---|---|
| C0 | Test maps for every world: full, partial (with real frontiers), Cartographer-like; each loads in Nav2's map_server exactly | `drone_sim/tools/make_fixtures.py`, `drone_sim/fixtures/`, `tests/C0.py` | **PASS**: 29 worlds, 87 of 87 fixtures identical cell by cell |
| C1 | Planner: safe, centred paths where they exist, refusal where they don't; planning time | `drone_nav/config/planner.yaml`, `tests/C1.py`, `tests/c1_cases.py` | **PASS** 36/36: corridors centred within 2.6 cm, 0.45 m gap refused, 0.60 m passed, W8 planning p95 18.6 ms |
| C2 | Cost-map threshold 65 vs Nav2's default 100 on Cartographer-like maps | `tests/C2.py` | **PASS**: at 65 every wall blocked (100 %), 0 false walls, 0 leaks; at 100 all walls gone (142,520 leak cells) |
| C3 | Frontier detection: every real opening, nothing else, under 50 ms | `drone_frontier/`, `tests/C3.py` | **PASS**: every passable opening found in 116 fixtures, 0 false frontiers, max 3.6 ms on 320 x 320; with the book's threshold 25: frontiers on 29 of 29 fully explored maps |
| C4 | Mission brain on the stub: 7 worlds x 10 start poses must explore >= 98 %, return within 0.3 m, land, no contacts, calm goal switching | `drone_mission/`, `drone_sim/tools/stub_world.py`, `tests/C4.py`, `tests/c4_one.py` | **PASS** 70/70 (20261116_160045): coverage 100 %, 0 contacts, home within 0.15 m; median mission 243-394 s, W8 775 s (longest 906 s of 1200); confirmed after the C5 fix: **PASS** 70/70 (20261116_192805), W8 median 827 s, longest 937 s |
| C5 | 100 random mazes (wr_s1-100: 8-15 m, corridors 1.0-1.5 m, rooms 0-6, loops 0-2), once each: >= 97 complete, every failure classified | `tests/C5.py` (mazes 21-100 generated on demand, not in git) | **PASS 100/100** (20261116_183506): exploration 2.06 s/m2 median, 3.01 s/m2 p90 (the baseline for frontier scoring); missions median 324 s, longest 814 s. First run 99/100 (wr_s5 at 72 %, fixed: see below) |
| C6 | Helper failures, each 3 times: planner refuses the chosen frontier; frontier behind a 0.45 m gap (W7); arm refused; /frontiers silent 10 s; /bridge/status stops | `tests/C6.py`, `tools/fault_planner.py` (planner relay), stub faults `fault_arm_refuse`, `fault_status_stop_s`, mission brain `planner_action` | **PASS** 15/15 (20261116_212814); planner refusal handled by blacklist 2x, by approach 1x; 1 graze in the 0.60 m gap (not judged, see below) |
| C7 | Timing on W8 at real-time speed: planner p95 < 200 ms, frontier search < 50 ms, mission loop 0.5 +- 0.05 s, CPU per process recorded for the Pi 5 | `tests/C7.py`, `c4_one.py --timing` (CPU from /proc incl. children), brain records request->answer times and loop marks | **PASS** (20261116_220725): planner p95 10 ms (max 20; request->answer p95 12 ms), frontier search p95 2.9 ms (max 6), loop p5-p95 0.50-0.52 s (max 5.8 s); CPU % of one core, mean/p95: brain 17/20, stub 8/10, frontier 7/8, planner 3/4, lifecycle 1/2; W8 DONE in 762 s |

## Fixtures (C0)

`make_fixtures.py` writes `drone_sim/fixtures/<world>/` for W2-W8 and the random mazes wr_s1-wr_s20:

* `full`: the true map (walls 100, free 0, outside unknown)
* `partial`: what a 360 deg, 10 m LiDAR sees from the lattice cells within 2.5 m of the start; the rest is unknown,
  so it has real frontiers. `openings.json` lists every frontier with its size and whether it is a real opening
  the drone can reach (`passable`)
* `carto`: Cartographer-like values: free 0-30, walls 60-95, a halo of 31-59 next to walls, 1% random speckle
  with values 0-64 (below the wall threshold; see the C1 finding below)

Files are in map_server's raw mode (pixel = value 0-100, 255 = unknown), map frame = world frame. They are
deterministic: re-running the script gives the same files.

## Frontier server (`drone_frontier`)

`frontier_server` reads `/map` and the drone's position (TF map -> base_link), and once a second publishes
`/frontiers` (cheapest first), RViz markers on `/frontiers/markers` and its own search time on
`/frontiers/search_ms`. The search (`frontier_core.py`, no ROS, 9 unit tests) is explore_lite's algorithm:
convert the map to exact free/wall/unknown, keep the free space reachable from the drone, frontier cells = unknown
cells next to it, 8-connected groups, drop groups under 0.55 m, cost = 3 x distance - 1 x size.

## Stub world and mission brain (C4)

`drone_sim/tools/stub_core.py` / `stub_world.py` replace Gazebo, ArduPilot, Cartographer and the bridge with a fast
stand-in: the drone follows paths with the bridge's own carrot follower (0.6 m) at 0.5 m/s with a 0.3 s lag, a
360 deg 10 m LiDAR is raycast in the true maze, the map is published at 1 Hz with Cartographer-like values, and
`/bridge/status`, `/bridge/command`, `/bridge/follow_path` behave like the bridge's. It runs on its own clock, 5x
real time in C4. It referees: contacts (drone circle vs true walls), coverage, distance flown.

`tests/c4_offline.py` couples the same stub, frontier search and mission brain with a grid planner that imitates
Smac 2D, without ROS: this is how the brain's logic was developed (35 offline missions) before the ROS test.

## Differences from the book (Humble)

* Nav2 Humble's ComputePathToPose returns no error code (205-208 arrived in a later Nav2): a failure is an aborted
  goal; the reason is in `planner_server.log`.
* C1 gives each request an explicit start pose (`use_start: true`) instead of moving the static transform, so one
  planner instance serves all cases.
* Clearance along a path is judged with a tolerance of one map cell (0.05 m), because the map is drawn in 5 cm cells.
* Paths on partial maps are judged against the walls the partial map shows; their clearance to the true walls is
  reported beside it (see the finding below).
* The frontier search is written in Python with numpy/scipy (whole-map operations) instead of copying explore_lite's
  C++ files: same algorithm, no C++ build, and a 320 x 320 map takes about 3 ms.
* Frontier server `free_threshold` 65 instead of 25. Cartographer's free cells in our simulation sit at 40-49
  (workstream A), so with 25 almost all explored space would count as unknown; and the planner's cost map already
  treats every observed value below 65 as free - the two must agree on what "free" means. C3 records what 25 does.
* C2 judges wall cells "blocked" (wall or too close to one for the drone's centre) and reports "lethal" beside it:
  with walls drawn at 60-95, one wall cell in seven is below 65, but its neighbours still close it off.
* Planner `cost_travel_multiplier` 5.0 instead of 2.0 (C4): with 2.0 paths hugged inside corners, and the carrot
  (which cuts corners by up to 0.21 m) took the stub drone into walls; 5.0 keeps paths nearer the corridor centre.
* Mission brain additions to the book's loop, each found necessary on the offline missions:
  - goal = the frontier's middle cell (always next to known free space), not its centroid (can lie deep in unknown);
  - blacklist entries expire after 30 s and become final after 3 failures: a frontier seen through a doorway at an
    angle is often unreachable at first and reachable a minute later;
  - "approach": when a frontier cannot be planned to, fly to points on the way to it (along known free cells);
  - "escape": no path to anywhere (the drone drifted into a wall's inflation) -> back off 0.4 m along its trail;
  - switching rule: the alternative must be cheaper by 20 % AND 1.5 cost units, checked on real path lengths
    (straight-line ranking alone made the drone alternate between two frontiers every 22 s), and a goal given up
    is not chosen again for 10 s;
  - progress is measured along the planned path, not in a straight line (winding paths looked like no progress);
  - "escape" (no path from where the drone is: it is inside the planner's lethal ring) flies to the nearest map
    cell 0.45 m from every wall, through known free cells (backing off along the trail rocked between two spots
    that were both too close, W4 in the first ROS run);
  - every path is moved away from walls before it is sent (points closer than 0.5 m to a known wall are pushed
    straight away from it, as far as the passage allows), and the part still ahead is re-checked against each
    new map (fixed and re-sent if it passes within 0.35 m): the planner's cost map is 1-2 s behind the map in
    flight, and its smoothing pulls paths into inside corners;
  - clearance is measured to every wall-looking cell (value >= 60), not only to the planner's walls (>= 65): a
    wall's tip cell is often 60-64, and the three 2 cm touches in the second ROS run were all at wall tips;
  - paths are planned from the drone's current position (use_start with the TF pose) and trimmed to start at the
    drone: the cost map's own robot pose lagged, at 5x stub speed by up to 0.5 m, so a new path could begin on
    the far side of a wall end and the carrot cut round it (the last 2.5 cm touch, W5);
  - the bridge's "arrived" flag counts only within 0.5 m of the end of the path just sent (for a moment after a
    new path it still describes the old one: in W5 all 5 return attempts "arrived" at once and the drone
    landed 7.6 m from home), and a return that ends more than 0.3 m from home is a STOP, never DONE;
  - the wall touches that remained (always a 2-3 cm graze while rounding a wall END) came from the bridge's
    carrot follower itself: flying straight at a point 0.6 m ahead, the drone ends up to 0.26 m inside a
    0.4 m turn. The follower now shortens its lookahead on bends (drone_bridge carrot.py, parameter
    follow_max_cut_m 0.05, lookahead never below 0.25 m): 0.07 m inside on the same turn, unchanged on
    straights. Closest pass in the offline missions went from about 0 cm to 9-16 cm. A shortened lookahead
    alone made the drone crawl on bends (the flight controller's speed follows the distance to its target:
    0.33 m/s on average instead of 0.5, missions 20-75 % longer, W8 past its 20 min), so the target sent is
    moved out along the same direction to the full lookahead distance: full speed, same clearance (48 test
    paths: 0.50 m/s, closest pass 9 cm, typical 13 cm; fixed lookahead: 5 cm). LATER (B5 re-run in Gazebo):
    under ArduPilot the moved-out target sent the drone 23 cm into corners, so the real bridge went back to the
    fixed lookahead; only the stub keeps the bend-aware follower (C4 tests the decision logic "independent of
    SLAM and flight quality", as the book says; real contacts are measured in I2-I5). This changed B code:
    the bridge unit tests pass (2 new), B5 (path following in Gazebo) should be re-run before integration;
  - real ROS runs refused 2-4x more plans than the offline copy ("no path": 31 against 7 in W6 1.0 m) and flew at
    0.17-0.42 m/s on average against 0.44: the planner's cost map was refreshed at 1 Hz from a /map that is
    itself 1 Hz, so it lagged the map by up to 2 s and refused frontiers that had just appeared. Cost map
    update_frequency is now 5 Hz, and a frontier's FIRST "no path" blacklists it for 6 s, not 30 s;
  - the return home retries for up to 300 s (escape, re-plan, points on the way home, wait 1 s) instead of
    5 instant attempts: in one ROS run all 5 were refused within 0.15 s and the drone stopped 5 m from home;
  - C5 wr_s5 (DONE at 72 % coverage): a long frontier ran from the room the drone was in down a strip of free
    cells the LiDAR had seen diagonally down a corridor; its middle cell was on that strip, too narrow to plan
    along, so the brain approached it, failed 3 times and dropped the whole frontier. Now, when a frontier's
    middle cannot be planned to, its nearest point to the drone (already in /frontiers) is tried first.
    Offline: wr_s5 100 % (reproduced the 72 % before); all 100 mazes pass, longest mission 1093 s -> 882 s;
  - C6 judging (decided 2026-11-16, "follow the book"): wall contacts are reported, not judged. W7's 0.60 m gap
    leaves the 0.54 m drone 3 cm a side; passing it the stub drone came within 2 cm in every offline run and
    grazed by 3-4 cm in one ROS run. Known limitation: the drone may lightly touch the sides of a gap less than
    ~0.66 m wide (C1 still expects a path through 0.60 m, as the book does). Revisit with real flight data.
    A planner refusal counts as handled if the frontier is blacklisted OR got round with approach points, with
    another goal flown within 20 s (1 of 3 ROS runs went round it: the brain's extra "approach" recovery);
  - "A -> B -> A" counts only real changes of mind (B still an open frontier when the drone went back to A).
* First ROS runs of C4 (lessons): the pre-flight "map present" check needs only 20 % of the 2 m square around the
  drone known (walls hide the rest); rclpy refuses one line of code logging at two severities (the brain's
  thread died at its first warning); ament_python packages must be rebuilt after every code change, even with
  --symlink-install.
* "recorded" fixtures (final maps from workstream A's recordings in `~/drone_replay_bags`) are not made yet.

## Findings

* **One false wall cell blocks a corridor.** First C1 run: the Cartographer-like maps had speckle of any value
  (0-100), so 0.4% of free cells read as walls (>= 65). Each one, inflated by the drone's 0.27 m radius, closes a
  1 m corridor (it leaves 0.18 m either side of the centre line for the drone's centre), and no path existed on any
  of them. Speckle is now kept below 65. For integration: a single phantom wall cell from Cartographer in a
  corridor would stop the planner, so phantom cells (workstream A, A1.2/A1.7) matter more than their count suggests.
* First C1 run: 0.60 m gap passed on the full map (clearance -0.9 cm, within the one-cell tolerance); planning time
  p95 15.5 ms on W8 (limit 200 ms); goal inside a wall -> path to free space 0.33 m away; start 0.2 m from a wall ->
  no path (recorded).
* **The planner hugs unknown space.** Second C1 run, W8 partial map: the path passed 16 cm too close to a wall corner
  that the partial map did not show yet (rays had passed the corner without hitting it). The planner refuses to
  enter unknown cells but keeps no distance from them (only walls are inflated). In flight the LiDAR sees the corner
  as the drone gets closer and the mission brain re-plans every 2 s, which should catch it; the alternative is
  Nav2's `inflate_unknown: true`, which keeps paths 0.27 m from unknown space but makes frontier goals 0.3 m into
  the unknown unreachable with the 0.5 m tolerance. To be decided with the mission brain (C4), on the stub.
