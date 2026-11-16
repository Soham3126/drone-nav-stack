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
