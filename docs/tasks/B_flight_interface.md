# Workstream B: flight interface (the bridge)

Goal: a bridge between ROS and the flight controller that is proven correct, and ArduPilot settings that hold
position and follow paths on our own position (no GPS).
Gate **G3** (B0-B6 and B8 pass, B7 decided): **met, with B6 as an open item** (see below).

The bridge: `drone_bridge/drone_bridge/`
* `bridge_node.py` - the ROS node: sends our position to ArduPilot (VISION_POSITION_ESTIMATE, 30 Hz), follows paths
  with the carrot, runs commands (`/bridge/command`: guided, arm, takeoff, land, hold), publishes `/bridge/status`
  and `/bridge/link_stats`
* `mavlink_link.py` (connection), `commands.py` (command + ACK + retries), `carrot.py` (path following),
  `offset.py` (SLAM-frame vs flight-controller-frame offset), `frames.py` (coordinate conversions)
* Flight-controller side: `drone_bringup/params/ardupilot/sitl_extnav.parm` and `lua/extnav_fallback.lua`

How to run: `scripts/run_test.sh <ID> <world> truth [timeout]`, or the batch scripts `scripts/run_b3.sh` ... `run_b9.sh`
through `scripts/batch.sh`. Shared test code: `drone_sim/tests/b_common.py`.

| Test | What it checks | Files | Result |
|---|---|---|---|
| B0 | Bridge maths and logic without a simulator | `drone_bridge/test/` (64 tests: `colcon test --packages-select drone_bridge`) | **PASS** 64/64 |
| B1 | Link and telemetry: rates, reconnect | `tests/B1.py` | **PASS**: rates within 1%, reconnect 0.1 s |
| B2 | Commands: happy path, refusal, timeout | `tests/B2.py` | **PASS** 3/3 each |
| B3 | Hover with pose delay and noise | `tests/B3.py`, `b3_cases/`, `run_b3.sh`, `tools/pose_mangler.py` | **PASS in substance**: drift 3-5 cm; 0-150 ms delay all held |
| B4 | Sudden position jumps (SLAM corrections) | `tests/B4.py`, `b4_cases/`, `run_b4.sh` | **PASS**: offset 0.8-1.8 cm; a jump without reset_counter overshoots (0.61 m for 0.3 m), with it none |
| B5 | Path following, speed and lookahead | `tests/B5.py`, `run_b5.sh` | **PASS** 27/27. Chosen: lookahead 0.6 m, `WP_SPD` 0.8 m/s |
| B6 | Companion computer freezes / dies / position stops | `tests/B6.py`, `run_b6.sh`, `tools/range_relay.py`, `tools/stream_gate.py`, `lua/extnav_fallback.lua` | **Open**: within 10 cm in 19 of 24 runs; 2 wall contacts |
| B7 | Fuse optical-flow velocity all the time? | `tests/B7.py`, `b7_slip.yaml`, `run_b7.sh` | **Decided: no.** Flow only as the outage fallback |
| B8 | Take-off height and rangefinder | `tests/B8.py` | **PASS** 7/7: hover within 4 cm of target |
| B9 | Delays and CPU load | `tests/B9.py`, `run_b9.sh` | **PASS** by the book: rates within 4% under load. Pose delay 7.4 ms -> `VISO_DELAY_MS 10`. Bridge CPU (aim < 10%): 45% at first, 38-40% after the fixes with the full simulation clock, 10.5-16% with a 100 Hz clock copy (so ~28 points are the simulation clock, absent on the drone). Open: re-measure on the Pi 5 |

## How the drone survives losing its position (B6, B7)

When our position stops (bridge frozen or killed, SLAM stalled), ArduPilot notices within 0.3 s and the Lua script
switches its estimator to "source set 2": optical-flow velocity + rangefinder height + the same heading source.
The drone holds where it was told to stop. When positions return it switches back after 1 s. After 8 s without
a position it lands (ArduPilot itself gives up its estimated position at about 10 s).

**B6 open item.** On flow alone the drone drifts 3-19 cm sideways in 5 s. W2's corridor leaves only 20.5 cm between
the propeller tips and each wall, and both wall contacts came after 18-19 cm drift. The drift comes from our
home-made simulated flow sensor; the real MTF-01P will decide it. Settle on hardware: measure the MTF-01P's delay
and noise, set `EK3_FLOW_DELAY` / flow noise, re-run B6.

## Bugs the tests caught (fixed)

* The bridge resent its last position when the stream stopped: the drone flew 6.4 m into a wall (B6). Now a position
  is sent once and never when older than 0.3 s.
* A reconnect after a freeze threw away the path (B6). Now only a real autopilot reboot clears it.
* Take-off stopped 14.5 cm low: ArduPilot ends its take-off on the raw rangefinder. The bridge now takes over after
  lift-off and trims the height (B8).
* The bridge used 45% of a CPU core, almost all in ROS's multi-threaded scheduler (B9). Now two simple
  single-threaded schedulers and a plain `/tf` reader. In simulation the biggest remaining cost is the simulation clock
  (1000 messages a second, which the real drone does not have). A 100 Hz copy of it brought the bridge to 10.5-16%,
  but shifted ArduPilot's height by 4 cm, so it is off (`CLOCK_THROTTLE=1` to try it).

## Final settings (`sitl_extnav.parm`)

ExtNav position and heading (`EK3_SRC1_POSXY 6`, `EK3_SRC1_YAW 6`), rangefinder height (`EK3_SRC1_POSZ 2`),
`VISO_DELAY_MS 10`, `RNGFND1_GNDCLR 0.145`, `WP_SPD 0.8`; fallback set 2 = flow velocity (`FLOW_TYPE 5`,
`EK3_FLOW_DELAY 50`), `SCR_ENABLE 1`. Marked "RE-SET" in the file: values that must be measured on the real drone.

Per-run numbers: [`results/INDEX.md`](../../results/INDEX.md). Every change from the book, with its reason: [`HUMBLE_NOTES.md`](../../HUMBLE_NOTES.md).
