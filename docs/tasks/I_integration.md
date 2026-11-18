# Integration series I1-I7 (guidebook 4.6)

The whole stack in Gazebo: ArduPilot SITL, Cartographer (carto-v1), the bridge (on Cartographer's pose), the
Nav2 planner server, the frontier server and the mission brain. Start-up profile `full` in
`drone_bringup/launch/sim.launch.py` (= `slam` + planner + frontier server + mission brain).

| Test | What | How | Status |
|---|---|---|---|
| gate | B5 re-run on the fixed 0.6 m lookahead (the bend-aware follower failed: 23 cm into corners, 2 runs; bridge reverted) | `run_test.sh B5 w2_turns_junctions truth` x3 at 0.8 m/s | **PASS** 3/3 (20261116_230820, _231054, _231330): lateral <= 5.7 cm straight / 11.1 cm corners, clearance >= 12.8 cm, arrival <= 4.9 cm |
| I1 | Hover 60 s + B5's paths on Cartographer's pose (W2): drift < 0.08 m, lateral <= 0.15 m, clearance >= 0.05 m | `run_test.sh I1 w2_turns_junctions slam 900` | **PASS** (20261117_011042): hover drift 5.3 cm; lateral <= 6.5 cm straight / 10.4 cm corners; clearance >= 13.6 cm; Cartographer ATE 1.9 cm RMS (5.8 cm max) |
| I2 | W3, 10 full missions | `run_i.sh I2 w3_tree_maze 10` | **PASS** 10/10 after fixes 1-9 (batch ending 20261118_002415): coverage >= 99.4 %, 0 contacts, landing <= 0.26 m, ATE <= 12.5 cm, offset <= 0.087 m, 304-443 s; see below |
| gate 2 | B5 with the bend limit the `full` profile uses | `MAXCUT=0.05 SPEED=0.8 LOOK=0.6 run_test.sh B5 w2_turns_junctions truth 900` x3 | **PASS** 3/3 (20261117_030230, _030515, _030758): lateral <= 5.1 cm straight / 6.1 cm corners, clearance >= 15.4 cm, arrival <= 4.5 cm |
| I3 | W4, 10 full missions | `run_i.sh I3 w4_maze_rooms 10` | pending |
| I4 | W6 x3 widths, W7, W5: 5 each; 0.45 m gap never entered | `run_i.sh I4 <world> 5` | pending |
| I5 | W8, 10 missions with flow as decided in B7 (+5 the other way) | `run_i.sh I5 w8_arena_full 10` | pending |
| I6 | 30 random mazes wr_s101-130 | - | pending |
| I7 | Soak: W8 x10 unattended, memory growth < 10 % per node | - | pending |

Per mission (I2-I6, `tests/I2.py` + `tests/i_common.py`), from Gazebo's ground truth: DONE; coverage >= 95 % of
the drone-reachable floor known free in the final /map; landing within 0.3 m of the start; no contact (0.27 m
circle more than 2 cm into a wall); ATE < 0.15 m (Cartographer vs truth while airborne); bridge frame offset
< 0.10 m.

## First full mission (I2, W3, 20261117_011503)

Worked end to end: DONE, 100 % of the map (95.2 % by i_common's stricter rule), landed 0.14 m from the start,
ATE 5.8 cm. Failed: 4 wall touches (2-4.5 cm) and bridge frame offset 0.18 m. Cause (from the bag, the ROS log
and the brain's events): at 180.7 s the brain sent a path that started the other way; the bridge's target jumped
about 1.2 m from in front of the drone to behind it; ArduPilot reversed hard (37 deg tilt, 1.45 m/s) into the
wall, the tilted 2D LiDAR gave Cartographer a 17 cm pose jump, hence the frame offset. Same pattern, milder, at
63 s. Fixes: the bridge's target now moves at most 1.0 m/s (`target_slew_mps`; ordinary following at 0.8 m/s is
untouched, a jumped target is glided to); the brain no longer re-sends a "fixed" path that is no better (20 of
its 52 re-sends changed nothing).

## Second I2 run: crash on the way home (20261117_012742)

The mission explored 95 % of W3, then, 42 s into the flight home, the drone started spinning (yaw -130 deg/s, then
+250 deg/s) and flipped into a wall; ArduPilot disarmed (crash check). Not the path, not the new target limiter:
ArduPilot's own log (PSCN/PSCE) shows the target 0.41 m from the walls and the drone within 5-8 cm of it.

Cause: the drone's propellers. The Gazebo iris body fits the book's 0.27 m circle (its collision shape reaches
0.272 m), but each propeller has a collision disc of 0.10 m radius centred 0.24-0.26 m from the middle, so the
propeller edges reach 0.355 m. When the spin began the drone's centre was 0.34 m from a wall end - a legal 7 cm
margin for our 0.27 m airframe - and the front propeller disc touched it; the distance from that propeller's
centre to the wall then stayed at exactly 0.10 m while the drone pivoted round the contact. The same touch at 69 s
had turned the drone 12 deg. The book plans for the new airframe (360 mm, 7-inch propellers, 0.27 m) and notes
the iris is bigger; it expected the simulation to be "slightly pessimistic", not to catch walls with propellers
that our drone does not have.

Fixes:
* The simulated drone now collides like our airframe: `drone_sim/tools/make_compact_iris.py` copies
  ardupilot_gazebo's iris_with_standoffs and iris_with_ardupilot (from `~/ardupilot_gazebo/models`, at every
  `sim_up.sh`) with only the propeller collision discs shrunk so their edges end at 0.270 m; `iris_with_lidar`
  includes the copy. Names, mass, inertia, lift and motors are unchanged.
* Corner cutting: the bridge's fixed 0.6 m carrot cut wall ends to 0.32-0.36 m centre-to-wall (practice harness
  with the real follower: margin 4.7-5.6 cm on W3), while the C4-C7 practice runs had used the bend limit
  (5 cm) all along (margin 12.9-13.6 cm). The `full` profile now sets `follow_max_cut_m` 0.05 (the bridge
  default stays 0 = the book's follower). The bend limit was untested under ArduPilot, so B5 is re-run with it
  (`MAXCUT=0.05`); the earlier failure was the separate keep_speed option, which stays stub-only. Missions take
  about 30 % longer (the drone slows at bends).

## Third I2 run (20261117_033155, fix 2): no crash; three near misses

DONE, landed 0.10 m from the start, tilt <= 8.9 deg, ATE 8.4 cm, map precision 98 % / recall 99.7 %, no phantom
walls. Failed: coverage 93.9 %, one touch 2.2 cm deep (limit 2 cm), frame offset 0.101 m (limit 0.10 m).

* Coverage was measured wrongly. The book (4.6, "Coverage") takes the reachable free space *eroded 0.1 m*;
  i_common used it unshrunk. Of the 6.1 % "missed", not one cell was unknown: all were 1-2 cell strips along wall
  faces that the 5 cm map draws as wall. With the book's erosion: 99.6 % (run 3). i_common now erodes 0.1 m.
* Frame offset: the bridge reported the translation between the frames at the map origin. Its 0.5-0.8 deg yaw
  estimate, 8-9 m out, made that up to 0.10 m while the frames disagreed by 3-7 cm where the drone was (replayed
  from ArduPilot's log). The bridge now reports the disagreement at the drone's position (the error a target sent
  to ArduPilot actually has; degrees unchanged). Replayed: run 3 0.07 m, run 2 0.06 m, run 1 still 0.13 m (its
  real 17 cm pose jump).
* The touch (86.3 s, wall face y = 4.0 m near x = 1.85 m): Cartographer was within 3 cm, and ArduPilot followed
  its target within 4 cm, so the target itself went 0.25 m from the wall. At 85.8 s the brain's re-check found the
  path given 2 s earlier passing 0.11 m from walls (by the then-newer map) and sent a fixed one; 10 ms later the
  routine 2 s re-choice sent a fresh planner path to the same goal, restarting the follower. The paths were not
  recorded, so which one led the drone in cannot be shown; re-planned on the final map, both keep >= 0.38 m.
  Changes: no re-choice in the same loop as a path fix; every path sent (and fixed) goes into the events
  (`sent`: points every 0.2 m + predicted track clearance), so the next touch can be traced.

## I2 x3 after fix 3 (20261117_034904, _035520, _040153): 2/3

Runs 1 and 3 PASS (coverage 99.8 / 100 %, ATE 7.1 / 4.5 cm, offset 6.2 / 6.1 cm, landing 0.26 / 0.25 m).
Run 2: one contact, 10.7 cm deep, at world (5.27, 3.47). The new path log shows why. At 101.6 s the brain sent
a path whose second leg ran from beside the drone straight through a wall end to the far side (its own predicted
track clearance: 0.00 m). The planner's path went round the wall end and came back on the wall's other side,
passing near the drone *through* the wall; `send` started the path at the path point nearest the drone - which
was on that return leg - and dropped everything before it. The drone followed the shortcut into the wall.
14 of the run's 112 paths crossed a wall cell (some only through walls not yet seen, which is harmless).

Fixes: the nearest point is taken only among points the drone can see (the straight line to it crosses no wall
cell) and, when a new path is sent, no more than 1 m further along than the path start's distance from the
drone; a path whose moved points (keep_clear) would make a leg cross a wall is sent unmoved instead (also for
path fixes); a path that still crosses a wall cell is logged as a warning. Unit test reproduces the cut.

## I2 x10 after fix 4 (20261117_042..._055945): 7/10

No crash; ATE 3.7-9.5 cm; offset <= 0.102 m. Fails: run 2 (1 contact 3.6 cm, offset 0.102 m, 1064 s),
run 6 (2 contacts 2-3 cm, 1030 s), run 10 (coverage 77 %). Run 1 passed but took 972 s.

Cause shared by all three: frontiers behind a wall. The map marks a strip of free cells just beyond some wall
faces (and outside the outer wall); a frontier there cannot be reached from this side. The planner (tolerance
0.5 m) still returns a path - to the nearest point it can reach, on the near side, 0.6-0.9 m short - the drone
hovered there 20 s and the goal was dropped. 21 of the 24 goals dropped in the three runs were such goals (the
straight line from the path's end to the goal crosses a wall). Runs 2 and 6 spent ~600 s on them; in run 10 the
real entrance to a 10 m2 region belonged to such a frontier, which was dropped for good, leaving 23 % unknown.
Contacts: run 2's came from a path that hugged a wall 7 cm off (fix 4's whole-path fallback put the planner's
path back when one moved point made a leg cross the wall end); run 6's two came from planner paths running
into the outer wall towards goals outside it.

Fixes: a plan whose end cannot see its frontier (or that crosses a wall) does not count; the frontier's nearest
point is tried, then the approach through known free space (no longer along a straight line through walls);
moved points are put back only on legs that cross a wall (`untangle`); a path that still crosses a wall is flown
only up to the wall (`cut_at_wall`); the drone at its path's end (0.15 m) counts as arrived even if the bridge's
latch was reset by a re-send. Practice runs: W3 x3, W4, wr_s1/5/9/13 all 100 %, no contact.

## I2 x10 after fix 5, run 1 (20261117_080445): contact 3.4 cm

Mission 311 s, coverage 100 %, 3 goals dropped (fix 5 works). Contact at world (6.43, 3.73): the goal was a
frontier lying where a wall turned out to be (map (6.08, 3.53), on the wall y = 4.0 once seen). The path ended
0.04 m from that wall; when the wall appeared the re-check found nothing it could move (a point inside a wall
has no "away" direction) and, by design, did not re-send - so the drone flew on to the end and touched.
Fix 6: when the re-check cannot improve a path that runs to within 0.22 m of a wall (drone radius 0.27 m), the
path is cut short before that point (`path_cut` event); the same cut is applied when a path is sent.

## I2 x10 after fix 5 (20261117_080445 ... _085945): 6/10 -> fix 7 (includes fix 6)

All missions 140-406 s (fix 5 removed the 1000 s ones). Fails: runs 1, 8, 10 one contact each (2.2-3.6 cm);
run 4 coverage 33 % (DONE at 140 s).

* Contacts, all three the same: the goal was a frontier lying where a wall turned out to be; the path ended on
  that wall (wall distance 0.00 m in the final map). When the wall appeared, the re-check could not move a
  point that sits inside a wall and did not re-send; the drone flew on. Fix 6/7: the re-check (and every send)
  stops a path short of any point within 0.22 m of a wall (`path_cut`; down to "stop here" when the wall is
  right ahead; a drone already closer is only kept from getting closer still), and a fixed path that still
  crosses a wall is flown only up to it. Replayed on the three runs' maps: all three paths now end 0.45-0.50 m
  from the walls.
* Run 4: fix 5's rule "a plan that cannot see its goal from its end does not count" was too strict. Only one
  frontier (15 m long) was left; its middle could not be planned to, the plan to its nearest point and every
  approach point ended out of sight of them, so the frontier was dropped three times and the mission ended.
  Such a plan now still counts if it moves the drone at least 0.5 m (it explores on the way); only a plan that
  leaves the drone where it is goes nowhere. Replayed: the approach is accepted (drone moves 1.07 m).

## I2 x10 after fix 7 (20261117_1915xx ... _200913): 6/10 -> fix 8

Fails: run 3 coverage 77 %, run 4 coverage 47 %, run 6 landing 0.34 m, run 10 one contact (3.0 cm).

* Early end (runs 3 and 4, and x10 run 10 before): each time the last frontier was given up three times in
  ~40 s while the drone hovered in one place (so the map never changed), and the mission ended. The way into
  the unknown part was 1-2 m away, round a wall end. Fix: before the mission may end with given-up frontiers,
  it flies once per frontier to the point it can reach (0.35 m from walls) that is closest to the frontier
  *through known open space*, clears that frontier's record, and tries again (`sweep`, at most 8). Replayed on
  every recorded run at the moment exploration ended: in the three low-coverage runs the sweep point lies in
  the open space leading into the unknown part (0.65-1.47 m from the frontier); runs that ended at 100 % had
  nothing to sweep, or one frontier outside the outer wall (one wasted trip).
* Landing 0.34 m: exploration ended 0.23 m from home (SLAM), inside the 0.3 m tolerance, so nothing was flown;
  SLAM error and the landing added 0.11 m. The return now always aims for 0.15 m (the final check stays 0.3 m).
* Contact (run 10): a wall corner entered the map 0.35 m before contact at 0.55 m/s (map 1 Hz). Decision
  (Soham, 17 Nov): add a close-range LiDAR guard to the bridge - not in the book. `drone_bridge/guard.py`: at
  10 Hz, from the raw scan (20 Hz), the distance the drone's body (0.30 m disc) can travel straight on before
  touching anything; if that is less than its braking distance (v^2 / 2 m/s^2 + 0.08 m), the target is frozen
  where the drone is until the way has been clear for 0.5 s. Only the swept band counts, so corridor walls and
  wall ends being turned round do not trigger it. Replayed on recorded flights against the true walls: no
  trigger in two clean missions (~10 000 samples), and a trigger 0.6-0.7 s before each of two real contacts.
  On for the `full` profile (`guard_scan_topic /scan`); the bridge's default is off.

## I2 x10 after fix 8 (20261117_2140xx ... _225705): 9/10

Coverage >= 99.6 % in all ten (the sweep works), landings <= 0.26 m, no contact in nine. Run 4 took 742 s (a
27 m trip to the far region of W3; no sweep was needed). The LiDAR guard never fired in the nine passing runs.

Run 9 (20261117_224941) FAIL: 4 contacts, frame offset 0.377 m. Not the planner or the follower: flying the
return path along the bottom corridor at 0.6 m/s, the drone drifted sideways (+y) at up to 0.46 m/s for ~0.8 s
while ArduPilot believed it was flying straight on (its velocity: 0.05 m/s sideways). Cartographer saw the drift
(its error stayed 9 cm); the EKF's position innovation grew to 0.26 m and it caught up only after ~3 s, by
which time the drone was on the wall y = 1.0. The true tilt was ~2.5 deg more than the autopilot's attitude
showed, and the drift started with a jolt in the vertical accelerometer (-7.9 / -11.6 m/s2, 316.7-316.8 s) with
nothing within 0.47 m of the drone. A velocity disagreement like this was found in 2 of 16 recorded missions
(the other, 20261117_051832 at 270.5 s, coincided with a contact). Taken as an autopilot-estimate / simulator
fault, not fixed here; if it recurs, the EKF settings (B-series) are the place to look.
The LiDAR guard took its direction from ArduPilot's velocity and so looked the wrong way until the drone was on
the wall; it now uses the velocity of the SLAM pose (replayed: triggers 0.4 s earlier in run 9; still no false
trigger in the 742 s run). It cannot by itself undo a wrong autopilot estimate.

## I2 PASS (batch ending 20261118_002415, fix 9)

10/10: coverage 99.4-100 %, no contact (closest 9.1 cm), landing 0.01-0.26 m, ATE 4.9-12.5 cm, frame offset
0.059-0.087 m, missions 304-443 s. No flight-controller drift like run 9 of the previous batch.

## I3 x10 (batch ending 20261118_022008, commit 88cf1a8): 9/10 -> fix 10

No contact in any run; coverage 97.5-100 %; ATE 4.7-14.3 cm; missions 468-885 s. Run 1 (20261118_003805) explored
100 % but ended in STOP "could not get home: 0.33 m away": fix 8 made the return aim for 0.15 m, and every path
home ended 0.34 m from home - keep_clear moves all of a path's points away from walls, the last one too, and home
is 0.35 m from a wall in the map (0.475 m in truth; the map draws walls thicker). Fix 10: on the way home the
last point is put back on home (if the straight leg to it is clear), in sends and in re-checks. Landings were
0.25-0.30 m in four runs (W4's SLAM error near home is 10-14 cm); with the path now ending on home, the drone
lands where SLAM puts home rather than up to 0.34 m short of it.

## I3 after fix 10 (batch ending 20261118_063046, commit 3b85b5a): 9/10 -> Cartographer change

Landings now 0.01-0.11 m (fix 10). One failure: run 9 (20261118_062229) ATE 17.1 cm > 15 cm; everything else fine.
Cartographer drifted to ~25 cm over 3-4 minutes in the far rooms and corrected only near the start (loop closure
search limited to 1 m since A1.9). Settled offline with `scripts/carto_replay.sh` on the recorded flights (no new
missions): 7 settings on the failed run; `max_constraint_distance` 15 m + `optimize_every_n_nodes` 20 was best
(17.1 -> 13.2 cm) and did not harm three passed W4 and four passed W3 flights (all equal or lower). Adopted in
`drone_slam/config/drone_2d.lua`; see A notes. Pi 5 CPU for it still to be measured.

## I3 on f2d6406 (batch ending 20261119_013734): 8/10 -> guard keep-off distance

ATE no longer fails (max 14.2 cm), landings 0.01-0.07 m. Runs 1 and 3: one contact each (2.1 and 2.9 cm), both the
same way. The target the bridge sent was 0.40 m and 0.30 m from the walls in the map, but the drone ended 0.257
and 0.245 m from them in truth: W4's map and pose disagree locally by 5-15 cm, and ArduPilot overshoots a stopping
target by ~5 cm. The LiDAR guard fired 5-6 times per mission but only looked straight ahead along the motion.
Fix: the guard now also keeps the *target* at least 0.31 m (`guard_keep_m`) from every point the LiDAR sees,
pushing it straight off the nearest wall (the drone slides along walls instead of stopping). The scan is used
relative to the drone, so SLAM's offsets do not matter. Replayed on four recorded flights against the true walls:
targets changed in 0-0.2 % of samples, never closer than 0.308 m to a true wall. Consequence: gaps narrower than
0.62 m can no longer be flown through (I4's W7 gap widths to be checked against this).
