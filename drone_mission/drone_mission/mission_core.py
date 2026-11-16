"""mission_core.py - the mission brain's decisions (guidebook 2.8), without ROS.

The brain is the only commander. It is written as a plain sequence of states, run in its own thread; everything
it needs from the outside world goes through an "interface" object, so the same code runs in the ROS node
(mission_brain.py), in the offline harness and in unit tests. Times are simulation seconds (iface.now()).

States: WAIT_READY -> TAKEOFF -> HOVER_CHECK -> EXPLORE -> RETURN -> LAND -> DONE;  STOP on any internal error
(requests LAND).

Interface (iface) methods:
  now(), sleep(dt)                         simulation time
  pose() -> (x, y) or None                 drone position in the map frame
  status() -> dict or None, status_age()   the bridge's /bridge/status as a dict
  frontiers() -> (seq, list)               latest /frontiers (seq increases with every message); items are dicts
                                           with 'middle', 'centroid' (x, y), 'cost', 'size_m'
  frontiers_age()                          seconds since the last /frontiers message
  plan(goal_xy) -> [(x, y), ...] or None   a path from the drone to the goal (planner server)
  follow(path)                             send a path to the bridge (empty list = stop)
  command(name, value) -> (ok, message)    the bridge's /bridge/command (blocking, as the bridge's)
  checks() -> dict name -> (ok, detail)    preflight checks that need the outside world (map, planner, ...)
  log(event, **data)                       mission events (state changes, goals, blacklist, warnings)
"""
import math
from dataclasses import dataclass, field


@dataclass
class Params:
    flight_height_m: float = 1.1
    settle_time_s: float = 15.0
    settle_tol_m: float = 0.02
    hover_check_s: float = 3.0
    hover_height_tol_m: float = 0.1
    hover_hold_tol_m: float = 0.1
    loop_period_s: float = 0.5
    rechoose_period_s: float = 2.0
    progress_timeout_s: float = 20.0
    progress_min_m: float = 0.2
    switch_hysteresis: float = 0.2
    switch_min_gain: float = 1.5          # cost units (3 per metre): the best must be at least this much cheaper
    switch_back_s: float = 12.0           # a goal given up for another is not chosen again for this long
    potential_scale: float = 3.0          # as the frontier server's: cost per metre (for path-based comparisons)
    empty_frontier_updates: int = 3
    blacklist_radius_m: float = 0.3
    blacklist_first_ttl_s: float = 6.0    # ... after a first "no path"
    blacklist_ttl_s: float = 30.0         # a blacklisted frontier may be tried again after this long ...
    blacklist_max_tries: int = 3          # ... but not after it failed this many times
    escape_m: float = 0.4                 # stuck next to a wall, no map: back off this far along the way we came
    arrived_check_m: float = 0.5          # the bridge's "arrived" counts only this close to the sent path's end
    path_clear_m: float = 0.50            # move path points to at least this far (centre to wall) where there is room
    carrot_lookahead_m: float = 0.3       # the bridge's carrot on bends (lookahead_m 0.6, shortened: follow_max_cut_m)
    track_min_m: float = 0.34             # the predicted flown track (centre) must stay this far from walls:
                                          # drone radius 0.27 + 7 cm; else the path is widened and re-sent
    escape_clear_m: float = 0.45          # stuck next to a wall: fly to the nearest cell this far from any wall
    approach_step_m: float = 0.5          # frontier not reachable yet: try points this far apart toward the drone
    approach_min_gain_m: float = 0.8      # ... but only if that gets the drone at least this much closer
    approach_max_tries: int = 12          # planner calls per approach at most
    same_goal_radius_m: float = 1.0       # a frontier this close to the current goal is the same frontier, moved
    goal_reached_m: float = 0.3
    home_x: float = 0.0
    home_y: float = 0.0
    home_tol_m: float = 0.3
    status_timeout_s: float = 3.0
    frontier_silence_warn_s: float = 3.0
    return_attempts: int = 5              # paths home flown at most ...
    return_timeout_s: float = 300.0       # ... within this time


class Stop(Exception):
    """Internal error: the brain goes to STOP (and requests LAND)."""


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class Goal:
    xy: tuple                 # where we plan to (the frontier's middle cell)
    cost: float
    since: float              # chosen at
    best_d: float             # smallest distance to it seen so far
    best_t: float             # when that distance was last improved by progress_min_m
    path: list = None         # the latest path to it


@dataclass
class Stats:
    goal_switches: list = field(default_factory=list)     # (t, xy, abandoned previous) per new goal
    blacklisted: list = field(default_factory=list)
    plans: int = 0
    plan_failures: int = 0


class MissionCore:
    def __init__(self, iface, params=None):
        self.io = iface
        self.p = params or Params()
        self.state = 'WAIT_READY'
        self.start_requested = False
        self.abort_requested = False
        self.goal = None
        self.blacklist = []           # dicts: xy, tries, until, why
        self.trail = []               # recent drone positions (for backing off)
        self.escapes = 0
        self.stuck_n = 0              # escapes since the last successful plan
        self.path_fixes = 0
        self.left = []                # (t, xy) of goals given up for another
        self.stats = Stats()
        self.reason = ''

    # ------------------------------------------------------------------ helpers
    def set_state(self, s, **why):
        self.state = s
        self.io.log('state', state=s, **why)

    def st(self):
        """Fresh bridge status, or STOP if the bridge has gone silent (C6: STOP after 3 s)."""
        s = self.io.status()
        if s is None or self.io.status_age() > self.p.status_timeout_s:
            raise Stop(f'no /bridge/status for {self.io.status_age():.1f} s')
        return s

    def blacklisted(self, xy):
        """Excluded: blacklisted less than blacklist_ttl_s ago, or failed blacklist_max_tries times.
        (A frontier that cannot be reached now often can be later, when more of the map is known.)"""
        now = self.io.now()
        for b in self.blacklist:
            if dist(xy, b['xy']) <= self.p.blacklist_radius_m:
                return now < b['until'] or b['tries'] >= self.p.blacklist_max_tries
        return False

    def excluded_for_good(self, xy):
        return any(dist(xy, b['xy']) <= self.p.blacklist_radius_m and b['tries'] >= self.p.blacklist_max_tries
                   for b in self.blacklist)

    def add_blacklist(self, xy, why):
        b = next((b for b in self.blacklist if dist(xy, b['xy']) <= self.p.blacklist_radius_m), None)
        if b is None:
            b = dict(xy=tuple(xy), tries=0, until=0.0, why=why)
            self.blacklist.append(b)
        b['tries'] += 1
        # a first refusal is often only the planner's cost map lagging the map (a frontier just seen): retry soon
        b['until'] = self.io.now() + (self.p.blacklist_first_ttl_s if b['tries'] == 1 and why == 'no path'
                                      else self.p.blacklist_ttl_s)
        b['why'] = why
        self.stats.blacklisted.append((round(self.io.now(), 2), [round(v, 2) for v in xy], why, b['tries']))
        self.io.log('blacklist', xy=[round(v, 2) for v in xy], why=why, tries=b['tries'],
                    final=b['tries'] >= self.p.blacklist_max_tries)

    def wait(self, cond, timeout, period=0.1):
        t0 = self.io.now()
        while self.io.now() - t0 < timeout:
            if cond():
                return True
            if self.abort_requested:
                raise Stop('abort requested')
            self.io.sleep(period)
        return cond()

    # ------------------------------------------------------------------ the mission
    def run(self):
        try:
            self.wait_ready()
            self.takeoff()
            if not self.hover_check():
                self.land('hover check failed')
                return self.state
            self.explore()
            self.go_home()
            self.land('mission complete')
        except Stop as e:
            self.reason = str(e)
            self.set_state('STOP', reason=self.reason)
            try:
                self.io.follow([])
                ok, msg = self.io.command('land', 0.0)
                self.io.log('stop_land', ok=ok, message=msg)
            except Exception as e2:  # noqa: BLE001
                self.io.log('stop_land_failed', error=str(e2))
        return self.state

    def wait_ready(self):
        self.set_state('WAIT_READY')
        hist, last = [], {}
        while True:
            if self.abort_requested:
                raise Stop('abort requested')
            now = self.io.now()
            s = self.io.status()
            pose = self.io.pose()
            ok = {}
            ok['fc_link'] = (s is not None and s['fc_connected'] and self.io.status_age() < 2.0, '')
            ok['ekf_origin'] = (s is not None and s['origin_set'], '')
            ok['ekf_position'] = (s is not None and s['ekf_position_ok'], '')
            ok['on_ground_disarmed'] = (s is not None and not s['armed'], '')
            if pose is not None:
                hist.append((now, pose))
                hist = [h for h in hist if now - h[0] <= self.p.settle_time_s]
            span = hist[-1][0] - hist[0][0] if hist else 0.0
            moved = max((dist(h[1], hist[-1][1]) for h in hist), default=9.0)
            ok['settled'] = (span >= self.p.settle_time_s - 0.6 and moved < self.p.settle_tol_m,
                             f'{span:.0f} s, moved {moved * 100:.1f} cm')
            ok.update(self.io.checks())
            all_ok = all(v[0] for v in ok.values())
            if {k: v[0] for k, v in ok.items()} != last:
                last = {k: v[0] for k, v in ok.items()}
                self.io.log('checks', **{k: bool(v[0]) for k, v in ok.items()})
            if all_ok and self.start_requested:
                return
            self.io.sleep(0.5)

    def takeoff(self):
        self.set_state('TAKEOFF')
        for name, val in (('guided', 0.0), ('arm', 0.0), ('takeoff', self.p.flight_height_m)):
            ok, msg = self.io.command(name, val)
            self.io.log('command', name=name, ok=ok, message=msg)
            if not ok:
                raise Stop(f'{name} refused: {msg}')
        self.home = self.io.pose()

    def hover_check(self):
        self.set_state('HOVER_CHECK')
        t0, pts = self.io.now(), []
        while self.io.now() - t0 < self.p.hover_check_s:
            pts.append(self.io.pose())
            self.io.sleep(0.1)
        s = self.st()
        h_ok = abs(s['height_m'] - self.p.flight_height_m) <= self.p.hover_height_tol_m
        hold = max(dist(p, pts[0]) for p in pts if p is not None)
        ok = h_ok and hold <= self.p.hover_hold_tol_m
        self.io.log('hover_check', ok=ok, height=round(s['height_m'], 3), hold_m=round(hold, 3))
        return ok

    # ------------------------------------------------------------------ exploration (book 2.8)
    def explore(self):
        self.set_state('EXPLORE')
        empty, last_seq_empty, last_choice = 0, None, -1e9
        silent_warned = False
        while True:
            if self.abort_requested:
                raise Stop('abort requested')
            now = self.io.now()
            s = self.st()
            pose = self.io.pose()
            if pose is None:
                raise Stop('no drone pose')
            if not self.trail or dist(self.trail[-1], pose) >= 0.05:
                self.trail = (self.trail + [pose])[-200:]
            seq, fr = self.io.frontiers()
            if self.io.frontiers_age() > self.p.frontier_silence_warn_s:
                # C6: frontiers stopped: keep the current goal, choose nothing new, never RETURN for missing data
                if not silent_warned:
                    self.io.log('warning', what='no /frontiers', age_s=round(self.io.frontiers_age(), 1))
                    silent_warned = True
                self.check_progress(now, pose)
                self.io.sleep(self.p.loop_period_s)
                continue
            silent_warned = False
            if self.goal is not None and self.goal.path:
                self.goal.path = self.recheck(self.goal.path, pose)
            reached = self.goal is not None and (
                (s.get('path_arrived') and self.goal.path and dist(pose, self.goal.path[-1]) <= self.p.arrived_check_m)
                or dist(pose, self.goal.xy) <= self.p.goal_reached_m)
            if self.goal is None or reached or now - last_choice >= self.p.rechoose_period_s:
                cands = [f for f in fr if not self.blacklisted(f['middle'])]
                waiting = [f for f in fr if not self.excluded_for_good(f['middle'])]
                if not cands and waiting:
                    # only frontiers that failed recently: wait for their retry time, do not end the mission
                    empty = 0
                    if reached:
                        self.goal = None
                elif not cands:
                    if seq != last_seq_empty:
                        empty += 1
                        last_seq_empty = seq
                    if empty >= self.p.empty_frontier_updates:
                        self.io.log('explore_done', empty_updates=empty)
                        self.io.follow([])
                        self.goal = None
                        return
                    if reached:
                        self.goal = None
                else:
                    empty = 0
                    self.choose(cands, now, pose)
                    last_choice = now
            self.check_progress(now, pose)
            self.io.sleep(self.p.loop_period_s)

    def choose(self, cands, now, pose):
        cands = sorted(cands, key=lambda f: f['cost'])
        best = cands[0]
        order = cands
        if self.goal is not None:
            same = min(cands, key=lambda f: dist(f['middle'], self.goal.xy))
            if dist(same['middle'], self.goal.xy) <= self.p.same_goal_radius_m:
                # switch only if the best is clearly cheaper: 20 % of the cost, and at least switch_min_gain
                # (costs are 3 x distance - size, often near zero, where 20 % alone means nothing)
                margin = max(self.p.switch_hysteresis * abs(same['cost']), self.p.switch_min_gain)
                keep = best is same or best['cost'] > same['cost'] - margin
                if not keep:
                    # the ranking uses straight-line distance; in a maze the "closer" frontier can be much
                    # farther to fly to. Before switching, compare what is left to fly to each (path lengths).
                    alt = self.io.plan(best['middle'])
                    self.stats.plans += 1
                    if alt:
                        w = self.p.potential_scale
                        cur_cost = w * remaining(self.goal.path, pose, self.goal.xy) - same['size_m']
                        alt_cost = w * path_len(alt) - best['size_m']
                        keep = alt_cost > cur_cost - max(self.p.switch_hysteresis * abs(cur_cost),
                                                         self.p.switch_min_gain)
                if keep:
                    order = [same] + [f for f in cands if f is not same]      # keep it (hysteresis)
        # never go straight back to a goal given up less than switch_back_s ago (no A -> B -> A)
        recent = [xy for t, xy in self.left if now - t < self.p.switch_back_s]
        if self.goal is not None:
            order = [f for f in order if dist(f['middle'], self.goal.xy) <= self.p.same_goal_radius_m
                     or not any(dist(f['middle'], xy) <= self.p.same_goal_radius_m for xy in recent)] or order
        failed = []
        for f in order:
            path = self.io.plan(f['middle'])
            self.stats.plans += 1
            near = f.get('nearest')
            if not path and near is not None and dist(near, f['middle']) > self.p.goal_reached_m:
                # a long frontier's middle can lie where the drone cannot go (e.g. on a strip the LiDAR saw
                # down a corridor at an angle) while the part nearest the drone is easy to reach: go there
                path = self.io.plan(near)
                self.stats.plans += 1
            if not path:
                path = self.approach(f['middle'], pose)
            if path:
                keeping = self.goal is not None and dist(f['middle'], self.goal.xy) <= self.p.same_goal_radius_m
                if not keeping:
                    # abandoned: the old goal is still an open frontier (a real change of mind, which C4 limits);
                    # otherwise it was reached, explored away or dropped
                    abandoned = self.goal is not None and any(
                        dist(c['middle'], self.goal.xy) <= self.p.same_goal_radius_m for c in cands)
                    if self.goal is not None:
                        self.left.append((now, self.goal.xy))
                        self.left.append((now, getattr(self.goal, 'xy0', self.goal.xy)))   # where it was chosen
                        self.left = self.left[-20:]
                    self.goal = Goal(xy=tuple(f['middle']), cost=f['cost'], since=now,
                                     best_d=remaining(path, pose, f['middle']), best_t=now, path=path)
                    self.goal.xy0 = tuple(f['middle'])
                    self.stats.goal_switches.append((round(now, 2), [round(v, 2) for v in f['middle']], abandoned))
                    self.io.log('goal', xy=[round(v, 2) for v in f['middle']], cost=round(f['cost'], 2),
                                size_m=round(f['size_m'], 2), path_m=round(path_len(path), 2), abandoned=abandoned)
                else:
                    self.goal.xy, self.goal.cost, self.goal.path = tuple(f['middle']), f['cost'], path
                self.goal.path = path = self.send(path)
                self.stuck_n = 0
                for g in failed:                                 # the better ones that failed before it
                    self.add_blacklist(g['middle'], 'no path')
                return True
            self.stats.plan_failures += 1
            failed.append(f)
        # every candidate failed: if even the way home fails, the drone itself is the problem (it has drifted
        # too close to a wall, where the planner refuses to start). Back off along the way we came instead of
        # blacklisting good frontiers.
        if len(failed) == len(order) and not self.io.plan((self.p.home_x, self.p.home_y)):
            self.escape(pose)
            return False
        for f in failed:
            self.add_blacklist(f['middle'], 'no path')
            if self.goal is not None and dist(f['middle'], self.goal.xy) <= self.p.same_goal_radius_m:
                self.goal = None
        return False

    def approach(self, goal, pose):
        """The frontier itself cannot be planned to - usually it was seen through a doorway or gap at an angle,
        so the known free space only joins up through a strip too narrow for the drone. Plan instead to points
        on the way there: along the shortest route through known free cells (ignoring the drone's size) when
        the map is available, else along the straight line, every approach_step_m back from the frontier.
        Flying there shows more of the map, and the next re-plan usually reaches the frontier itself.
        None if no such point gets the drone at least approach_min_gain_m further."""
        route = None
        grid = self.io.map_grid() if hasattr(self.io, 'map_grid') else None
        if grid is not None:
            route = free_route(*grid, pose, goal)
        if not route:
            d = dist(goal, pose)
            n = int(d / 0.05)
            route = [(pose[0] + (goal[0] - pose[0]) * i / max(n, 1), pose[1] + (goal[1] - pose[1]) * i / max(n, 1))
                     for i in range(n + 1)]
        # walk back from the frontier end of the route
        acc, last = 0.0, route[-1]
        tries = 0
        for q in reversed(route[:-1]):
            acc += dist(q, last)
            last = q
            if acc < self.p.approach_step_m:
                continue
            acc = 0.0
            if dist(q, pose) < self.p.approach_min_gain_m:
                return None
            path = self.io.plan(q)
            self.stats.plans += 1
            tries += 1
            if path and dist(path[-1], pose) >= self.p.approach_min_gain_m:
                self.io.log('approach', frontier=[round(v, 2) for v in goal], via=[round(v, 2) for v in q],
                            tries=tries)
                return path
            if tries >= self.p.approach_max_tries:
                return None
        return None

    def send(self, path):
        """Follow `path` after moving its points away from walls (see keep_clear), using the newest map."""
        pose = self.io.pose()
        if pose is not None and len(path) >= 2:
            # start the path at the drone: drop what lies behind it (the planner's start can lag the drone)
            k = nearest_index(path, pose)
            if dist(path[k], pose) <= 1.0:
                path = [tuple(pose)] + list(path[k + 1:]) if k + 1 < len(path) else [tuple(pose), path[-1]]
        grid = self.io.map_grid() if hasattr(self.io, 'map_grid') else None
        if grid is not None:
            path = make_clear(*grid, path, self.p.path_clear_m, self.p.carrot_lookahead_m, self.p.track_min_m)
        self.io.follow(path)
        return path

    def recheck(self, path, pose):
        """The planner works on a cost map that is 1-2 s behind the map (in flight: walls seen moments ago are
        missing from it). If the track the drone will fly along the path ahead (carrot) passes closer than track_min_m to a wall,
        move it away and send it again."""
        grid = self.io.map_grid() if hasattr(self.io, 'map_grid') else None
        if grid is None or len(path) < 2:
            return path
        g, res, origin = grid
        k = nearest_index(path, pose)
        ahead = path[k:]
        before = carrot_track_clearance(g, res, origin, ahead, self.p.carrot_lookahead_m)
        if before >= self.p.track_min_m:
            return path
        fixed = make_clear(g, res, origin, ahead, self.p.path_clear_m, self.p.carrot_lookahead_m,
                           self.p.track_min_m)
        self.path_fixes += 1
        self.io.log('path_fix', n=self.path_fixes, track_before=round(before, 3),
                    track_after=round(carrot_track_clearance(g, res, origin, fixed, self.p.carrot_lookahead_m), 3))
        self.io.follow(fixed)
        return fixed

    def escape(self, pose):
        """The planner refuses to start: the drone is closer to a wall than its own radius (in the cost map's
        "lethal" ring). Fly to the nearest map cell that is clearly clear (escape_clear_m from any wall),
        through known free cells. Without a map, back off escape_m along the way we came."""
        self.escapes += 1
        self.stuck_n += 1
        if self.stuck_n > 20:
            raise Stop('stuck: no path from here, 20 escapes')
        route = None
        grid = self.io.map_grid() if hasattr(self.io, 'map_grid') else None
        if grid is not None:
            route = clear_route(*grid, pose, self.p.escape_clear_m)
        if route and dist(route[-1], pose) < 0.05:
            route = None                         # already clear by the map: the planner disagrees, back off instead
        if not route:
            back = next((q for q in reversed(self.trail) if dist(q, pose) >= self.p.escape_m), None)
            route = [pose, back] if back is not None else None
        self.io.log('escape', at=[round(v, 2) for v in pose],
                    to=None if not route else [round(v, 2) for v in route[-1]], n=self.stuck_n)
        if not route:
            return
        self.goal = None
        end = route[-1]
        self.io.follow(route)
        self.wait(lambda: dist(self.io.pose(), end) <= 0.1
                  or (self.st().get('path_arrived') and dist(self.io.pose(), end) <= self.p.arrived_check_m), 8.0)

    def check_progress(self, now, pose):
        """Blacklist the goal if the distance to it has not dropped by progress_min_m in progress_timeout_s.
        "Distance" = what is left of the latest path plus the step from its end to the goal (a straight-line
        distance would not drop on a winding path, and the goal would be dropped wrongly)."""
        g = self.goal
        if g is None:
            return
        d = remaining(g.path, pose, g.xy)
        if d <= g.best_d - self.p.progress_min_m:
            g.best_d, g.best_t = d, now
        elif now - g.best_t >= self.p.progress_timeout_s:
            self.add_blacklist(g.xy, f'no progress for {self.p.progress_timeout_s:.0f} s (distance {d:.2f} m)')
            self.goal = None

    # ------------------------------------------------------------------ return and land
    def go_home(self):
        self.set_state('RETURN')
        home = (self.p.home_x, self.p.home_y)
        attempt, failures, t_end = 0, 0, self.io.now() + self.p.return_timeout_s
        while attempt < self.p.return_attempts and self.io.now() < t_end:
            pose = self.io.pose()
            if dist(pose, home) <= self.p.home_tol_m:
                break
            path = self.io.plan(home)
            if not path:
                # the planner refused: usually the drone is in its lethal ring (escape), or its cost map has not
                # caught up with the map yet (wait a moment). Points on the way home are the fallback.
                failures += 1
                self.io.log('warning', what='no path home', failures=failures)
                self.escape(pose)
                path = self.io.plan(home) or self.approach(home, self.io.pose())
                if not path:
                    self.io.sleep(1.0)
                    continue
            attempt += 1
            path = self.send(path)
            best, best_t = remaining(path, pose, home), self.io.now()
            while True:
                self.io.sleep(0.5)
                if self.abort_requested:
                    raise Stop('abort requested')
                s = self.st()
                pose = self.io.pose()
                path = self.recheck(path, pose)
                d = remaining(path, pose, home)
                if (s.get('path_arrived') and dist(pose, path[-1]) <= self.p.arrived_check_m) \
                        or dist(pose, home) <= self.p.home_tol_m * 0.5:
                    break
                if d < best - self.p.progress_min_m:
                    best, best_t = d, self.io.now()
                elif self.io.now() - best_t > self.p.progress_timeout_s:
                    self.io.log('warning', what='no progress home', distance=round(d, 2))
                    break
        pose = self.io.pose()
        d = dist(pose, home)
        self.io.log('home', distance=round(d, 3))
        if d > self.p.home_tol_m:
            raise Stop(f'could not get home: {d:.2f} m away ({attempt} paths flown, {failures} planner refusals)')

    def land(self, why):
        self.set_state('LAND', why=why)
        self.io.follow([])
        ok, msg = self.io.command('land', 0.0)
        self.io.log('command', name='land', ok=ok, message=msg)
        if not ok:
            raise Stop(f'land: {msg}')
        self.set_state('DONE')


def path_len(path):
    return sum(dist(a, b) for a, b in zip(path, path[1:]))


def free_route(grid, res, origin, start, goal, free_below=65):
    """Shortest 8-connected route through known free cells (value 0..free_below-1) from start to the free cell
    nearest the goal, ignoring the drone's size; list of (x, y) or None. grid rows = y from the origin up."""
    import numpy as np
    from collections import deque
    g = np.asarray(grid)
    free = (g >= 0) & (g < free_below)
    H, W = g.shape
    ox, oy = origin
    cell = lambda p: (int((p[1] - oy) / res), int((p[0] - ox) / res))  # noqa: E731
    s, t = cell(start), cell(goal)
    if not (0 <= s[0] < H and 0 <= s[1] < W):
        return None
    if not (0 <= t[0] < H and 0 <= t[1] < W):
        return None
    r0, c0 = max(0, t[0] - 3), max(0, t[1] - 3)
    win = free[r0:t[0] + 4, c0:t[1] + 4]
    if not win.any():
        return None
    rr, cc = np.nonzero(win)
    k = int(np.argmin((rr + r0 - t[0]) ** 2 + (cc + c0 - t[1]) ** 2))
    t = (int(rr[k] + r0), int(cc[k] + c0))
    free[s] = True
    prev = np.full((H, W), -1, np.int64)
    prev[s] = s[0] * W + s[1]
    q = deque([s])
    while q:
        r, c = q.popleft()
        if (r, c) == t:
            break
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr2, cc2 = r + dr, c + dc
                if (dr or dc) and 0 <= rr2 < H and 0 <= cc2 < W and free[rr2, cc2] and prev[rr2, cc2] < 0:
                    prev[rr2, cc2] = r * W + c
                    q.append((rr2, cc2))
    if prev[t] < 0:
        return None
    out, cur = [], t[0] * W + t[1]
    while True:
        r, c = divmod(cur, W)
        out.append((ox + (c + 0.5) * res, oy + (r + 0.5) * res))
        if (r, c) == s:
            break
        cur = int(prev[r, c])
    out.reverse()
    return out


_EDT_CACHE = {}


WALL_LIKE = 60   # for clearance: every cell that looks like wall (Cartographer gives wall cells 60..100, and a
                 # wall's tip cell is often below the planner's 65), so paths keep their margin at wall ends


def wall_field(grid, res, occupied=WALL_LIKE):
    """Distance (m) from each cell to the nearest occupied cell, and its gradient (d/drow, d/dcol): the
    direction straight away from the nearest walls. Cached per grid array."""
    import numpy as np
    from scipy import ndimage
    key = (id(grid), grid.shape, res)
    hit = _EDT_CACHE.get(key)
    if hit is not None and hit[0] is grid:
        return hit[1], hit[2]
    occ = np.asarray(grid) >= occupied
    if not occ.any():
        d, grad = np.full(occ.shape, 99.0), None
    else:
        d = ndimage.distance_transform_edt(~occ) * res
        grad = np.gradient(ndimage.gaussian_filter(d, 1.0))
    _EDT_CACHE.clear()
    _EDT_CACHE[key] = (grid, d, grad)
    return d, grad


def _cell(p, res, origin, shape):
    r, c = int((p[1] - origin[1]) / res), int((p[0] - origin[0]) / res)
    return min(max(r, 0), shape[0] - 1), min(max(c, 0), shape[1] - 1)


def min_wall_dist(grid, res, origin, pts):
    d, _ = wall_field(grid, res)
    return min((float(d[_cell(p, res, origin, d.shape)]) for p in pts), default=99.0)


def keep_clear(grid, res, origin, path, want):
    """Move each path point that is closer than `want` to a known wall straight away from the walls (up the
    wall-distance slope) until it is `want` away or no longer gets clearer (in a narrow passage it ends in the
    middle). The planner's path hugs inside corners (its smoothing) and is planned on a cost map that lags the
    map in flight; the bridge's carrot cuts corners further."""
    import numpy as np
    d, grad = wall_field(grid, res)
    if grad is None:
        return list(path)
    gr, gc = grad
    out = []
    for p in path:
        x, y = float(p[0]), float(p[1])
        for _ in range(12):
            r, c = _cell((x, y), res, origin, d.shape)
            d0 = float(d[r, c])
            if d0 >= want:
                break
            vx, vy = float(gc[r, c]), float(gr[r, c])
            n = float(np.hypot(vx, vy))
            if n < 1e-6:
                break
            step = min(want - d0, 0.05)
            nx, ny = x + vx / n * step, y + vy / n * step
            r2, c2 = _cell((nx, ny), res, origin, d.shape)
            if float(d[r2, c2]) < d0 or grid[r2, c2] < 0:          # no clearer, or into unknown space
                break
            x, y = nx, ny
        out.append((x, y))
    return out


def carrot_track_clearance(grid, res, origin, path, lookahead):
    """Smallest wall distance along the track the bridge's carrot follower will fly: from each path point the
    drone heads straight for the point `lookahead` metres further along the path, so round a wall end it cuts
    inside the path (by about r(1 - cos(L/2r)) on a turn of radius r)."""
    import numpy as np
    d, _ = wall_field(grid, res)
    if len(path) < 2:
        return min_wall_dist(grid, res, origin, path)
    P = np.asarray(path, float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    best = 99.0
    for i in range(0, len(P), 2):
        j = int(np.searchsorted(s, s[i] + lookahead))
        j = min(j, len(P) - 1)
        a, b = P[i], P[j]
        n = max(2, int(np.hypot(*(b - a)) / (res / 2)))
        for t in np.linspace(0.0, 1.0, n):
            q = a + (b - a) * t
            best = min(best, float(d[_cell(q, res, origin, d.shape)]))
    return best


def make_clear(grid, res, origin, path, want, lookahead, track_min):
    """keep_clear (the carrot's corner cutting is limited in the bridge itself: follow_max_cut_m)."""
    return keep_clear(grid, res, origin, path, want)


def nearest_index(path, pose):
    return min(range(len(path)), key=lambda i: (path[i][0] - pose[0]) ** 2 + (path[i][1] - pose[1]) ** 2)


def clear_route(grid, res, origin, start, clear_m, occupied=65):
    """Route through known free cells from start to the nearest free cell at least clear_m from every
    occupied cell; list of (x, y) (start first) or None."""
    import numpy as np
    from collections import deque
    from scipy import ndimage
    g = np.asarray(grid)
    occ = g >= occupied
    free = (g >= 0) & ~occ
    H, W = g.shape
    ox, oy = origin
    r0, c0 = int((start[1] - oy) / res), int((start[0] - ox) / res)
    if not (0 <= r0 < H and 0 <= c0 < W):
        return None
    wall = g >= WALL_LIKE
    clear = ndimage.distance_transform_edt(~wall) * res >= clear_m if wall.any() else free
    free[r0, c0] = True
    prev = np.full((H, W), -1, np.int64)
    prev[r0, c0] = r0 * W + c0
    q, t = deque([(r0, c0)]), None
    while q:
        r, c = q.popleft()
        if clear[r, c] and free[r, c]:
            t = (r, c)
            break
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if (dr or dc) and 0 <= rr < H and 0 <= cc < W and free[rr, cc] and prev[rr, cc] < 0:
                    prev[rr, cc] = r * W + c
                    q.append((rr, cc))
    if t is None:
        return None
    out, cur = [], t[0] * W + t[1]
    while True:
        r, c = divmod(cur, W)
        out.append((ox + (c + 0.5) * res, oy + (r + 0.5) * res))
        if (r, c) == (r0, c0):
            break
        cur = int(prev[r, c])
    out.reverse()
    out[0] = tuple(start)
    return out


def remaining(path, pose, goal):
    """Length of the path from the point closest to the drone to its end, plus the end-to-goal gap."""
    if not path:
        return dist(pose, goal)
    i = min(range(len(path)), key=lambda k: dist(path[k], pose))
    return dist(pose, path[i]) + path_len(path[i:]) + dist(path[-1], goal)
