"""Close-range LiDAR guard (I2)."""
import math

from drone_bridge.guard import time_to_contact_m, stop_distance_m

N = 500
AMIN, AINC = -math.pi, 2 * math.pi / N


def scan_of(segments, x, y, yaw, rmax=12.0):
    """Synthetic body-frame scan of wall segments ((x1, y1), (x2, y2)) from (x, y, yaw)."""
    out = []
    for i in range(N):
        a = yaw + AMIN + i * AINC
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1), (x2, y2) in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        out.append(best if best < rmax else float('inf'))
    return out


def test_wall_straight_ahead():
    s = scan_of([((1.0, -2), (1.0, 2))], 0, 0, 0)
    assert abs(time_to_contact_m(s, AMIN, AINC, 0.0, (1, 0), 0.30) - 0.70) < 0.02


def test_corridor_walls_beside_do_not_count():
    s = scan_of([((-5, 0.475), (5, 0.475)), ((-5, -0.475), (5, -0.475))], 0, 0, 0)
    assert time_to_contact_m(s, AMIN, AINC, 0.0, (1, 0), 0.30) == math.inf


def test_turning_round_a_wall_end_at_040_does_not_stop():
    # wall along x from -3 to 0 at y = 0; drone 0.40 m beyond its end, flying round it (tangent +y)
    s = scan_of([((-3, 0), (0, 0))], 0.40, 0.0, 0)
    assert time_to_contact_m(s, AMIN, AINC, 0.0, (0, 1), 0.30) == math.inf


def test_braking_on_a_late_wall_stops_before_contact():
    """Fly at 0.55 m/s toward a corner the planner did not know; ArduPilot brakes at ~1 m/s2 once the target is
    frozen. The drone (0.27 m) must not touch."""
    wall = [((1.2, -1), (1.2, 1))]
    x, v, dt, frozen = 0.0, 0.55, 0.1, False
    for _ in range(100):
        free = time_to_contact_m(scan_of(wall, x, 0, 0), AMIN, AINC, 0.0, (1, 0), 0.30)
        if free < stop_distance_m(v, 1.0, 0.08):
            frozen = True
        if frozen:
            v = max(0.0, v - 1.0 * dt)
        x += v * dt
    assert frozen and 1.2 - x > 0.27


def test_clip_keeps_the_drone_off_a_wall_ahead():
    from drone_bridge.guard import clip_target
    s = scan_of([((1.0, -2), (1.0, 2))], 0, 0, 0)            # wall 1 m ahead
    tx, ty = clip_target(s, AMIN, AINC, 0.0, (0, 0), (0.9, 0), 0.31)
    assert abs(tx - 0.69) < 0.02 and abs(ty) < 0.02


def test_clip_slides_along_a_wall():
    from drone_bridge.guard import clip_target
    s = scan_of([((-5, 0.35), (5, 0.35))], 0, 0, 0)          # wall 0.35 m to the left
    tx, ty = clip_target(s, AMIN, AINC, 0.0, (0, 0), (0.5, 0.2), 0.31)
    assert abs(tx - 0.5) < 0.03 and ty < 0.06               # still moves along it, but not much closer


def test_clip_leaves_corridor_flight_alone():
    from drone_bridge.guard import clip_target
    s = scan_of([((-5, 0.475), (5, 0.475)), ((-5, -0.475), (5, -0.475))], 0, 0, 0)
    tx, ty = clip_target(s, AMIN, AINC, 0.0, (0, 0), (0.6, 0.0), 0.31)
    assert abs(tx - 0.6) < 0.01 and abs(ty) < 0.01


def test_clip_lets_the_drone_through_a_062_m_gap():
    from drone_bridge.guard import clip_target
    s = scan_of([((0.5, 0.31), (0.6, 0.31)), ((0.5, -0.31), (0.6, -0.31))], 0, 0, 0)
    tx, ty = clip_target(s, AMIN, AINC, 0.0, (0, 0), (0.6, 0.0), 0.31)
    assert tx > 0.5 and abs(ty) < 0.02


def test_clip_pushes_out_of_a_corner():
    from drone_bridge.guard import clip_target, math as _m
    s = scan_of([((1.0, -2), (1.0, 1.0)), ((-2, 1.0), (1.0, 1.0))], 0, 0, 0)
    tx, ty = clip_target(s, AMIN, AINC, 0.0, (0, 0), (0.9, 0.9), 0.31)
    assert 1.0 - tx >= 0.30 and 1.0 - ty >= 0.30
