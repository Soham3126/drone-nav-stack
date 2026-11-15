# Drone Navigation Stack: Project Summary

15 Nov 2026

## Overview

Two of the guidebook's three workstreams are done in simulation: the drone maps mazes with SLAM and flies on its
own position through a tested bridge. Workstream C (route planner, frontier finder, mission brain) is next.

Everything runs in simulation first, on Ubuntu 22.04 with ROS 2 Humble, Gazebo Harmonic and ArduPilot Copter 4.7.2
(the book targets 24.04 + Jazzy; every difference is in [`HUMBLE_NOTES.md`](../HUMBLE_NOTES.md)).

| Workstream | What it delivers | Status |
| --- | --- | --- |
| A: simulator and SLAM | Maze worlds with exact ground truth; Cartographer with known accuracy | Closed; Cartographer frozen as `carto-v1` |
| B: flight interface | Bridge between ROS and the flight controller; ArduPilot settings for flying without GPS | Closed; gate G3 met, B6 open |
| C: planner and mission | Safe paths, exploration frontiers, mission logic | Not started |

## Workstream A: simulator and SLAM

Cartographer maps the mazes accurately and is frozen as `carto-v1`; its one known weakness is slow heading creep
in long, featureless corridors.

**What was built**
- A maze generator that writes Gazebo worlds and their true maps (W0 box room to W8 full arena, plus random mazes),
  and a simulated drone with LiDAR, IMU and downward rangefinder.
- Realistic sensor errors: MPU6500 noise and wandering gyro bias, RPLIDAR C1 range noise; gyro-bias removal while
  the drone is on the ground.
- Scripted test flights on simulated GPS, a recorded-run system (every run saved with its settings, data and
  report) and map scoring against the true map.

| Item | Result |
| --- | --- |
| Platform tests A0.1-A0.4 | Passed (gate G1) |
| SLAM tests A1.1-A1.11 | Done; gate G2 met |
| `carto-v1` | Book configuration with loop closure on but short-range (`max_constraint_distance` 1 m), optimised every 45 scans; tuned for the noisy IMU (W4 worst error 9.7 cm) |
| Tried, not adopted | `max_constraint_distance` 15 m with `optimize_every_n_nodes` 20 improved W4 replays (worst error 15.2 to 10.6 cm) but was never committed; all B tests ran on `carto-v1`. Re-evaluate in integration |
| Confirmation on fresh flights | 7 of 9 passed; the 2 misses are the corridor heading creep (W4/W5) |
| Pose delay (A1.10) | About 5 ms; with the bridge 7.4 ms (B9) |

**Lessons:** keep the heading constant during missions (turning the nose along the path made Cartographer's
heading drift); a 1 m corridor leaves little room (a 0.3 m side-step test hit the wall, so tests use 0.15 m).

## Workstream B: flight interface

The drone flies without GPS on our own position: it holds, follows paths within 8 cm on straights, takes off to
the right height and survives short position outages; one safety item (B6) stays open.

The bridge (`drone_bridge`) sends our position to ArduPilot 30 times a second, steers along paths with a "carrot"
0.6 m ahead, and runs commands (arm, take off, land, hold) with acknowledgements and retries.

| Test | What it checks | Result |
| --- | --- | --- |
| B0 | Bridge maths and logic | Pass, 64/64 unit tests |
| B1 | Link, telemetry rates, reconnect | Pass: rates within 1%, reconnect 0.1 s |
| B2 | Commands: success, refusal, timeout | Pass, 3/3 each |
| B3 | Hover with position delay and noise | Pass in substance: drift 3-5 cm |
| B4 | Sudden SLAM position jumps | Pass; jumps must carry `reset_counter` or the drone overshoots |
| B5 | Path following | Pass 27/27; chosen lookahead 0.6 m, speed 0.8 m/s |
| B6 | Bridge frozen, killed, or position stopped | Open: within 10 cm in 19 of 24 runs, 2 wall contacts |
| B7 | Fuse optical flow all the time? | Decided no; flow only as outage backup |
| B8 | Take-off height | Pass 7/7, within 4 cm |
| B9 | Delays and CPU load | Pass by the book's rule (rates within 4% under load); `VISO_DELAY_MS` 10 |

**Losing position safely.** When positions stop, ArduPilot notices within 0.3 s and a Lua script switches it to
optical-flow velocity and rangefinder height; positions back for 1 s switch it back; after 8 s it lands.

**B6 open item.** On flow alone the drone drifts 3-19 cm sideways in 5 s, while W2's corridor leaves 20.5 cm
between propeller tips and wall; both wall contacts followed 18-19 cm drift. The drift comes from our home-made
simulated flow sensor; the real MTF-01P will decide it.

**Bugs the tests caught**
- The bridge resent its last position when the stream stopped: the drone flew 6.4 m into a wall. Fixed: never send
  a position twice or older than 0.3 s.
- A reconnect threw away the path after a freeze. Fixed: only a real autopilot reboot clears it.
- Take-off stopped 14.5 cm low. Fixed: the bridge takes over after lift-off and trims the height.
- The bridge used 45% of a CPU core, almost all in ROS's multi-threaded scheduler. Fixed with simpler scheduling;
  the remaining cost is mostly the simulation clock, which the real drone does not have.

## Final settings and what to re-measure on the real drone

| Setting | Value | Where | Re-measure on hardware? |
| --- | --- | --- | --- |
| Position source | ExtNav position and heading, rangefinder height | `sitl_extnav.parm` (`EK3_SRC1_*`) | No |
| `VISO_DELAY_MS` | 10 ms (measured 7.4 ms) | `sitl_extnav.parm` | Yes: Pi 5 + Cartographer |
| `RNGFND1_GNDCLR` | 0.145 m | `sitl_extnav.parm` | Yes: sensor height when landed |
| Flow fallback | `FLOW_TYPE 5`, `EK3_FLOW_DELAY 50` | `sitl_extnav.parm`, `extnav_fallback.lua` | Yes: MTF-01P delay and noise |
| Speed and lookahead | `WP_SPD` 0.8 m/s, lookahead 0.6 m | `sitl_extnav.parm`, bridge | Check in the arena |
| Cartographer | `carto-v1` | `drone_slam/config/drone_2d.lua` | CPU and delay on the Pi 5 |
| Bridge CPU | 38-40% of a core in simulation, ~28 points of it the simulation clock (10.5-16% with a slower clock copy); aim under 10% | bridge | Yes: on the Pi 5 |

Before the first real flight, confirm the Lua fallback script is on the flight controller and enabled (`SCR_ENABLE 1`).

## Open items and next steps

- [ ] B6: settle the flow-fallback sideways drift with the real MTF-01P (measure its delay and noise, re-run B6)
- [ ] Corridor heading creep (W4/W5): watch it in the integration tests
- [ ] Bridge CPU and pose delay: measure on the Pi 5
- [ ] Workstream C: route planner (C1-C2), frontier finder (C3), mission brain (C4-C6)
- [ ] Integration tests I1-I7, then the hardware chapter
