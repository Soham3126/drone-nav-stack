"""Unit tests for the mission brain's decisions (no ROS): a fake world that answers instantly."""
from drone_mission.mission_core import MissionCore, Params


class Fake:
    """Drone teleports along paths at 1 m/s; frontiers are scripted by the test."""

    def __init__(self, frontiers=None, plan_ok=lambda g: True, arm_ok=True, status_until=1e9, fr_until=1e9):
        self.t, self.p = 0.0, (0.0, 0.0)
        self.path, self.cmds, self.events = [], [], []
        self.fr = frontiers if frontiers is not None else (lambda t, p: [])
        self.plan_ok, self.arm_ok = plan_ok, arm_ok
        self.status_until, self.fr_until = status_until, fr_until
        self.seq = 0
        self.armed, self.z = False, 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt
        self.seq += 1
        if self.path:                                    # move 1 m/s toward the path end
            ex, ey = self.path[-1]
            dx, dy = ex - self.p[0], ey - self.p[1]
            d = (dx * dx + dy * dy) ** 0.5
            s = min(d, dt)
            self.p = (self.p[0] + dx / d * s, self.p[1] + dy / d * s) if d > 1e-9 else self.p

    def pose(self):
        return self.p

    def status(self):
        arrived = bool(self.path) and abs(self.p[0] - self.path[-1][0]) + abs(self.p[1] - self.path[-1][1]) < 1e-6
        return dict(fc_connected=True, mode='GUIDED', armed=self.armed, ekf_position_ok=True, origin_set=True,
                    height_m=self.z, path_arrived=arrived)

    def status_age(self):
        return 0.0 if self.t < self.status_until else self.t - self.status_until

    def frontiers(self):
        return self.seq, self.fr(self.t, self.p)

    def frontiers_age(self):
        return 0.0 if self.t < self.fr_until else self.t - self.fr_until

    def plan(self, goal):
        return [self.p, tuple(goal)] if self.plan_ok(goal) else None

    def follow(self, path):
        self.path = list(path)

    def command(self, name, value):
        self.cmds.append(name)
        if name == 'arm':
            if not self.arm_ok:
                return False, 'PreArm: test refusal'
            self.armed = True
        if name == 'takeoff':
            self.z = value
        if name == 'land':
            self.armed, self.z = False, 0.0
        return True, 'ok'

    def checks(self):
        return {}

    def log(self, event, **data):
        self.events.append((round(self.t, 2), event, data))


def fr(x, y, cost):
    return dict(middle=(x, y), centroid=(x, y), cost=cost, size_m=1.0)


def run(io, **params):
    m = MissionCore(io, Params(settle_time_s=1.0, **params))
    m.start_requested = True
    return m, m.run()


def test_happy_path_explores_returns_and_lands():
    # one frontier at (3, 0) until the drone has been there, then nothing
    seen = {'x': False}

    def frontiers(t, p):
        if abs(p[0] - 3.0) < 0.05:
            seen['x'] = True
        return [] if seen['x'] else [fr(3.0, 0.0, 9.0)]
    io = Fake(frontiers)
    m, final = run(io)
    assert final == 'DONE'
    assert io.cmds == ['guided', 'arm', 'takeoff', 'land']
    assert abs(io.p[0]) < 0.3 and abs(io.p[1]) < 0.3          # came home
    states = [d['state'] for _, e, d in io.events if e == 'state']
    assert states == ['WAIT_READY', 'TAKEOFF', 'HOVER_CHECK', 'EXPLORE', 'RETURN', 'LAND', 'DONE']


def test_planner_failure_blacklists_and_takes_the_next():
    def frontiers(t, p):
        return [fr(5.0, 0.0, 1.0), fr(0.0, 4.0, 2.0)] if t < 40 else []
    io = Fake(frontiers, plan_ok=lambda g: abs(g[0]) < 1e-9)          # only points on the y axis are reachable
    m, final = run(io)
    assert final == 'DONE'
    bl = [d for _, e, d in io.events if e == 'blacklist']
    assert bl and bl[0]['xy'] == [5.0, 0.0]
    goals = [d['xy'] for _, e, d in io.events if e == 'goal']
    assert goals[0] == [0.0, 4.0]


def test_arm_refused_stops_and_requests_land():
    io = Fake(arm_ok=False)
    m, final = run(io)
    assert final == 'STOP' and 'arm refused' in m.reason
    assert io.cmds[-1] == 'land'


def test_status_silence_stops_after_3_s():
    io = Fake(lambda t, p: [fr(50.0, 0.0, 1.0)], status_until=30.0)
    m, final = run(io)
    assert final == 'STOP' and 'no /bridge/status' in m.reason
    assert io.cmds[-1] == 'land'


def test_frontier_silence_keeps_goal_and_does_not_return():
    io = Fake(lambda t, p: [fr(30.0, 0.0, 1.0)], fr_until=25.0)
    m = MissionCore(io, Params(settle_time_s=1.0))
    m.start_requested = True
    orig = io.sleep

    def sl(dt):
        orig(dt)
        if io.t > 45:
            m.abort_requested = True
    io.sleep = sl
    m.run()
    states = [d['state'] for _, e, d in io.events if e == 'state']
    assert 'RETURN' not in states
    assert any(e == 'warning' and d.get('what') == 'no /frontiers' for _, e, d in io.events)
    assert io.p[0] > 15.0                                  # it kept flying to the goal it had


def test_hysteresis_keeps_goal_when_alternative_only_slightly_cheaper():
    def frontiers(t, p):
        if t > 60:
            return []
        # two frontiers far away with costs that swap by 5 % every second
        a, b = (1.0, 1.05) if int(t) % 2 else (1.05, 1.0)
        return [fr(80.0, 0.0, a), fr(0.0, 80.0, b)]          # too far to reach before they vanish at 60 s
    io = Fake(frontiers)
    m, final = run(io, progress_timeout_s=100.0)
    goals = [d['xy'] for _, e, d in io.events if e == 'goal']
    assert len(goals) == 1


def test_unreachable_frontier_is_dropped_by_the_progress_timeout_and_mission_ends():
    # a frontier the planner "reaches" only to a tolerance point 1 m short (behind a 0.45 m gap): no progress
    io = Fake(lambda t, p: [fr(3.0, 0.0, 1.0)])
    io.plan = lambda g: [io.p, (2.0, 0.0)] if tuple(g) != (0.0, 0.0) else [io.p, (0.0, 0.0)]   # home is reachable
    m, final = run(io)
    assert final == 'DONE'
    bl = [d for _, e, d in io.events if e == 'blacklist']
    assert bl and bl[-1]['final']


def test_keep_clear_moves_points_off_a_wall_but_stays_in_a_narrow_corridor():
    import numpy as np
    from drone_mission.mission_core import keep_clear, min_wall_dist
    g = np.zeros((40, 80), int)
    g[:2, :] = 100                     # walls at the bottom (y < 0.1) and top (y >= 1.9): a 1.8 m corridor
    g[38:, :] = 100
    path = [(x * 0.1, 0.35) for x in range(5, 60)]
    out = keep_clear(g, 0.05, (0.0, 0.0), path, 0.5)
    assert min_wall_dist(g, 0.05, (0.0, 0.0), out) >= 0.45
    n = np.zeros((14, 80), int)        # 0.6 m wide passage: the middle is the best it can do
    n[:2, :] = 100
    n[12:, :] = 100
    out = keep_clear(n, 0.05, (0.0, 0.0), [(x * 0.1, 0.2) for x in range(5, 60)], 0.5)
    assert all(0.25 <= y <= 0.45 for _, y in out)


def test_escape_route_ends_clear_of_walls():
    import numpy as np
    from drone_mission.mission_core import clear_route, min_wall_dist
    g = np.zeros((40, 40), int)
    g[30:, :] = 100                    # wall along the top (y >= 1.5)
    r = clear_route(g, 0.05, (0.0, 0.0), (1.0, 1.35), 0.45)
    assert r[0] == (1.0, 1.35) and min_wall_dist(g, 0.05, (0.0, 0.0), [r[-1]]) >= 0.45



def test_return_home_survives_a_few_planner_refusals():
    # the planner refuses the first 4 requests for home (its cost map still catching up), then plans
    seen = {'x': False, 'refused': 0}

    def frontiers(t, p):
        if abs(p[0] - 3.0) < 0.05:
            seen['x'] = True
        return [] if seen['x'] else [fr(3.0, 0.0, 9.0)]
    io = Fake(frontiers)

    def plan(g):
        if tuple(g) == (0.0, 0.0) and seen['refused'] < 4:
            seen['refused'] += 1
            return None
        return [io.p, tuple(g)]
    io.plan = plan
    io.map_grid = lambda: None
    m, final = run(io)
    assert final == 'DONE' and abs(io.p[0]) < 0.3


def test_unreachable_middle_falls_back_to_the_frontiers_nearest_point():
    # a long frontier whose middle (6, 0) cannot be planned to, but whose nearest point (2, 0) can
    seen = {'x': False}

    def frontiers(t, p):
        if abs(p[0] - 2.0) < 0.05:
            seen['x'] = True
        return [] if seen['x'] else [dict(middle=(6.0, 0.0), centroid=(6.0, 0.0), nearest=(2.0, 0.0), cost=1.0,
                                          size_m=5.0)]
    io = Fake(frontiers, plan_ok=lambda g: tuple(g) != (6.0, 0.0))
    io.map_grid = lambda: None
    m, final = run(io)
    assert final == 'DONE' and seen['x']
    assert not [d for _, e, d in io.events if e == 'blacklist']


def test_path_start_is_not_taken_from_the_far_side_of_a_wall():
    """I2 run 5: the planner's path went round a wall end and came back on the wall's other side, passing
    0.3 m from the drone through the wall; the path was cut there and the drone flew into the wall."""
    import numpy as np
    import math
    from drone_mission.mission_core import nearest_index, crosses_wall
    res, origin = 0.05, (0.0, 0.0)
    g = np.zeros((60, 60), dtype=np.int16)
    g[30, 0:40] = 100                                   # wall along y = 1.5 from x = 0 to 2
    grid = (g, res, origin)
    pose = (1.0, 1.6)                                   # drone just above the wall
    # the planner's start lags behind the drone (x = 0.4); the path runs 0.3 m above the wall to x = 2.2 ...
    up = [(0.4 + 0.1 * i, 1.9) for i in range(19)]
    down = [(2.3, 1.9 - 0.1 * i) for i in range(1, 6)]  # ... round its end ...
    back = [(2.2 - 0.1 * i, 1.35) for i in range(19)]   # ... and back underneath, 0.25 m from the drone
    path = up + down + back
    k_old = min(range(len(path)), key=lambda i: (path[i][0] - pose[0]) ** 2 + (path[i][1] - pose[1]) ** 2)
    assert k_old >= len(up) + len(down)                 # the old search took the leg under the wall
    k = nearest_index(path, pose, grid, max_along=math.dist(path[0], pose) + 1.0)
    assert k == 6 and path[k] == (1.0, 1.9)
    assert crosses_wall(grid, [pose] + path[k_old + 1:]) and not crosses_wall(grid, [pose] + path[k + 1:])


def test_wall_helpers_untangle_and_cut():
    import numpy as np
    from drone_mission.mission_core import untangle, cut_at_wall, crosses_wall
    g = np.zeros((60, 60), dtype=np.int16)
    g[30, 0:40] = 100                                   # wall along y = 1.5, x 0..2
    grid = (g, 0.05, (0.0, 0.0))
    raw = [(1.8, 1.7), (2.1, 1.7), (2.3, 1.5), (2.1, 1.3)]
    moved = [(1.8, 1.8), (1.9, 1.7), (1.9, 1.3), (2.1, 1.2)]   # two neighbours pushed apart across the wall
    assert crosses_wall(grid, moved) and not crosses_wall(grid, raw)
    out = untangle(grid, raw, moved)
    assert not crosses_wall(grid, out) and out[0] == moved[0]   # only the bad leg's points went back
    through = [(1.0, 1.9), (1.0, 1.7), (1.0, 1.2), (1.0, 1.0)]
    assert cut_at_wall(grid, through) == through[:2]


def test_goal_behind_a_wall_is_not_taken_as_reached_by_a_short_plan():
    """I2 x10: the planner returns the nearest reachable point, on this side of a wall, for a frontier strip
    just behind it; the drone then hovered 20 s there and the goal was dropped."""
    import numpy as np
    from drone_mission.mission_core import MissionCore
    g = np.zeros((60, 60), dtype=np.int16)
    g[30, 0:40] = 100
    grid = (g, 0.05, (0.0, 0.0))

    class IO:
        def map_grid(self):
            return grid
    m = MissionCore.__new__(MissionCore)
    m.io = IO()
    from drone_mission.mission_core import Params
    m.p = Params()
    target = (1.0, 1.4)                                 # just behind the wall
    assert m.leads_to([(1.0, 2.5), (1.0, 2.0)], target) is None
    assert m.leads_to([(2.5, 2.5), (2.4, 1.6)], (2.4, 1.2)) is not None    # open space, in sight


def test_path_ending_on_a_newly_seen_wall_is_cut_short():
    import numpy as np
    from drone_mission.mission_core import cut_before_contact
    g = np.zeros((60, 60), dtype=np.int16)
    g[40, :] = 100                                      # wall along y = 2.0
    grid = (g, 0.05, (0.0, 0.0))
    path = [(1.0, 1.0 + 0.1 * i) for i in range(10)]    # ends at y = 1.9, 0.1 m from the wall
    cut = cut_before_contact(grid, path, 0.22)
    assert cut is not None and cut[-1][1] < 1.8 and len(cut) >= 2
    assert cut_before_contact(grid, path[:5], 0.22) is None


def test_reach_toward_goes_round_the_wall_not_to_its_back():
    """The point offered for a given-up frontier is the one nearest it through open space."""
    import numpy as np
    from drone_mission.mission_core import reach_toward
    g = np.zeros((80, 80), dtype=np.int16)              # 4 m x 4 m, all known free
    g[40, 0:60] = 100                                   # wall along y = 2.0 from x = 0 to 3 (open beyond x = 3)
    grid = (g, 0.05, (0.0, 0.0))
    pose = (1.0, 1.0)                                   # below the wall
    target = (1.0, 2.2)                                 # frontier just above it
    q = reach_toward(*grid, pose, target, 0.35)
    assert q is not None and q[1] > 2.0                 # on the frontier's side, reached round the wall end
