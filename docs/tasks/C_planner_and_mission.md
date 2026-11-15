# Workstream C: route-finder, edge-finder and mission brain

Goal: a planner configuration, a frontier server and a mission brain proven on hundreds of mazes, fast, before they
ever fly. Gate **G4** = C1-C6 pass. **In progress.**

The C tests need no Gazebo and no ArduPilot: they run on map files ("fixtures"), so a test takes seconds to minutes.
How to run any of them: `scripts/run_c.sh <ID>` from `~/drone_ws/src` (results in `~/drone_results/<date>_<ID>_...`).

| Test | What it checks | Files | Result |
|---|---|---|---|
| C0 | Test maps for every world: full, partial (with real frontiers), Cartographer-like; each loads in Nav2's map_server exactly | `drone_sim/tools/make_fixtures.py`, `drone_sim/fixtures/`, `tests/C0.py` | **PASS**: 29 worlds, 87 of 87 fixtures identical cell by cell |
| C1 | Planner: safe, centred paths where they exist, refusal where they don't; planning time | `drone_nav/config/planner.yaml`, `tests/C1.py`, `tests/c1_cases.py` | **PASS** 36/36: corridors centred within 2.6 cm, 0.45 m gap refused, 0.60 m passed, W8 planning p95 18.6 ms |
| C2 | Cost-map threshold 65 vs Nav2's default 100 on Cartographer-like maps | `tests/C2.py` | pending |
| C3 | Frontier detection: every real opening, nothing else, under 50 ms | `drone_frontier/`, `tests/C3.py` | pending (offline check of the search code: 116/116 fixtures, 0 false, max 2.8 ms) |
| C4-C6 | Mission brain on a fast stand-in drone; 100 random mazes; helper failures | - | not started |
| C7 | Timing of the decision layer | - | not started |

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
