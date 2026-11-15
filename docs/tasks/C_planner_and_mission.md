# Workstream C: route-finder, edge-finder and mission brain

Goal: a planner configuration, a frontier server and a mission brain proven on hundreds of mazes, fast, before they
ever fly. Gate **G4** = C1-C6 pass. **In progress.**

The C tests need no Gazebo and no ArduPilot: they run on map files ("fixtures"), so a test takes seconds to minutes.
How to run any of them: `scripts/run_c.sh <ID>` from `~/drone_ws/src` (results in `~/drone_results/<date>_<ID>_...`).

| Test | What it checks | Files | Result |
|---|---|---|---|
| C0 | Test maps for every world: full, partial (with real frontiers), Cartographer-like; each loads in Nav2's map_server exactly | `drone_sim/tools/make_fixtures.py`, `drone_sim/fixtures/`, `tests/C0.py` | pending |
| C1 | Planner: safe, centred paths where they exist, refusal where they don't; planning time | `drone_nav/config/planner.yaml`, `tests/C1.py`, `tests/c1_cases.py` | pending |
| C2 | Cost-map threshold 65 vs Nav2's default 100 on Cartographer-like maps | - | not started |
| C3 | Frontier detection: every real opening, nothing else | - | not started |
| C4-C6 | Mission brain on a fast stand-in drone; 100 random mazes; helper failures | - | not started |
| C7 | Timing of the decision layer | - | not started |

## Fixtures (C0)

`make_fixtures.py` writes `drone_sim/fixtures/<world>/` for W2-W8 and the random mazes wr_s1-wr_s20:

* `full`: the true map (walls 100, free 0, outside unknown)
* `partial`: what a 360 deg, 10 m LiDAR sees from the lattice cells within 2.5 m of the start; the rest is unknown,
  so it has real frontiers. `openings.json` lists every frontier with its size and whether it is a real opening
  the drone can reach (`passable`)
* `carto`: Cartographer-like values: free 0-30, walls 60-95, a halo of 31-59 next to walls, 1% random speckle

Files are in map_server's raw mode (pixel = value 0-100, 255 = unknown), map frame = world frame. They are
deterministic: re-running the script gives the same files.

## Differences from the book (Humble)

* Nav2 Humble's ComputePathToPose returns no error code (205-208 arrived in a later Nav2): a failure is an aborted
  goal; the reason is in `planner_server.log`.
* C1 gives each request an explicit start pose (`use_start: true`) instead of moving the static transform, so one
  planner instance serves all cases.
* Clearance along a path is judged with a tolerance of one map cell (0.05 m), because the map is drawn in 5 cm cells.
* "recorded" fixtures (final maps from workstream A's recordings in `~/drone_replay_bags`) are not made yet.
