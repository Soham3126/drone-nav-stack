"""B0.2 carrot.py: straight, 90 deg corner (cut <= 0.354 L), U-turn, short path, duplicate points,
index never decreases when pushed back, end-of-path behaviour, arrival."""
import math

import pytest

from drone_bridge.carrot import CarrotFollower, corner_cut

L = 0.5


def follower(points, lookahead=L):
    c = CarrotFollower(lookahead)
    c.set_path(points)
    return c


def fly_along_x(c, x_from, x_to, y=0.0, step=0.05):
    """Move the drone continuously (as it does at 10 Hz), not by teleporting."""
    x = x_from
    while x < x_to - 1e-9:
        c.update(x, y)
        x += step
    return c.update(x_to, y)


def test_straight_carrot_exactly_L_ahead():
    c = follower([(0, 0), (8, 0)])
    x0 = 0.0
    for x in [0.0, 1.0, 2.5, 4.0, 7.0]:
        cx, cy = fly_along_x(c, x0, x)
        x0 = x
        assert cx == pytest.approx(x + L)
        assert cy == pytest.approx(0.0)


def test_straight_with_sideways_offset():
    c = follower([(0, 0), (8, 0)])
    cx, cy = fly_along_x(c, 0.0, 3.0, y=0.2)   # drone 0.2 m beside the line: carrot is on the line
    assert (cx, cy) == pytest.approx((3.5, 0.0))


@pytest.mark.parametrize('lookahead', [0.5, 0.6, 0.7])
def test_corner_cut_at_most_0354_L(lookahead):
    corner = (5.0, 0.0)
    c = follower([(0, 0), corner, (5, 5)], lookahead)
    worst = 0.0
    x = 0.0
    while x <= 5.0 + 1e-9:                 # drone exactly on the path, approaching the corner
        p = (x, 0.0)
        q = c.update(*p)
        if q[1] > 1e-9:                    # the line drone->carrot spans the corner: it cuts inside
            worst = max(worst, corner_cut(p, q, corner))
        x += 0.01
    y = 0.0
    while y <= 2.0:                        # after the corner both are on the second leg: no cut
        q = c.update(5.0, y)
        assert q[0] == pytest.approx(5.0)
        y += 0.01
    assert worst <= 0.354 * lookahead + 1e-3
    assert worst == pytest.approx(lookahead / (2 * math.sqrt(2)), abs=2e-3)   # the bound is reached


def test_u_turn_does_not_jump_to_return_leg():
    # out along y=0, U-turn, back along y=0.3 (return leg passes close to the outbound leg)
    path = [(0, 0), (4, 0), (4, 0.3), (0, 0.3)]
    c = follower(path)
    c.update(1.0, 0.15)                   # halfway between the legs, early in the path
    assert c.s < 2.0                      # still on the outbound leg (s ~ 1), not the return leg (s ~ 7)
    cx, cy = c.update(1.0, 0.15)
    assert cy == pytest.approx(0.0)


def test_carrot_goes_round_u_turn():
    c = follower([(0, 0), (4, 0), (4, 0.3), (0, 0.3)])
    for x in [i * 0.1 for i in range(41)]:
        c.update(x, 0.0)
    cx, cy = c.update(4.0, 0.0)
    assert (cx, cy) == pytest.approx((4.0, 0.3)) or cx < 4.0     # carrot already on/after the turn
    assert c.s == pytest.approx(4.0, abs=0.05)


def test_path_shorter_than_L():
    c = follower([(0, 0), (0.2, 0)])
    assert c.update(0, 0) == pytest.approx((0.2, 0))      # carrot clamped to the final point


def test_single_point_path():
    c = follower([(1, 2)])
    assert c.update(0, 0) == pytest.approx((1, 2))
    assert c.length == 0.0


def test_duplicate_points_are_removed():
    c = follower([(0, 0), (0, 0), (1, 0), (1, 0), (1, 0), (2, 0)])
    assert c.pts == [(0, 0), (1, 0), (2, 0)]
    assert c.length == pytest.approx(2.0)
    assert c.update(0.2, 0) == pytest.approx((0.7, 0))


def test_index_never_decreases_when_pushed_back():
    c = follower([(0, 0), (8, 0)])
    fly_along_x(c, 0.0, 4.0)
    s_before = c.s
    cx, _ = c.update(2.0, 0)              # drone pushed 2 m backwards
    assert c.s == pytest.approx(s_before)
    assert cx == pytest.approx(4.5)       # the carrot does not come back either


def test_window_limits_far_jumps():
    # a sudden jump far along the path (e.g. a SLAM pose jump) cannot advance progress past the window
    c = follower([(0, 0), (10, 0)])
    c.update(9.0, 0)
    assert c.s <= c.window + 1e-9


def test_end_of_path_carrot_stays_on_final_point():
    c = follower([(0, 0), (2, 0)])
    x0 = 0.0
    for x in [1.6, 1.8, 2.0, 2.3]:
        assert fly_along_x(c, x0, x) == pytest.approx((2.0, 0.0))
        x0 = x


def test_arrival_needs_radius_speed_and_time():
    c = follower([(0, 0), (2, 0)])
    assert not c.check_arrived(1.80, 0, 0.0, 0.0)        # 0.2 m away: outside 0.15 m
    assert not c.check_arrived(1.90, 0, 0.3, 1.0)        # close but moving
    assert not c.check_arrived(1.90, 0, 0.05, 2.0)       # slow since t=2
    assert not c.check_arrived(1.92, 0, 0.05, 2.9)
    assert c.check_arrived(1.95, 0, 0.05, 3.0)           # 1 s slow and close
    assert c.check_arrived(5, 5, 1.0, 9.0)               # latched


def test_arrival_timer_resets_when_leaving():
    c = follower([(0, 0), (2, 0)])
    c.check_arrived(1.95, 0, 0.0, 0.0)
    c.check_arrived(1.95, 0, 0.5, 0.5)                   # speed up: timer reset
    assert not c.check_arrived(1.95, 0, 0.0, 1.2)
    assert c.check_arrived(1.95, 0, 0.0, 2.2)


def test_no_path_errors_and_helpers():
    c = CarrotFollower()
    assert not c.active
    with pytest.raises(ValueError):
        c.update(0, 0)
    with pytest.raises(ValueError):
        c.point_at(0)
    assert c.distance_to_end(0, 0) == 0.0
    assert c.closest_progress(0, 0) == 0.0
    assert not c.check_arrived(0, 0, 0, 0)


def test_point_at_and_corner_cut_helpers():
    c = follower([(0, 0), (1, 0), (1, 1)])
    assert c.point_at(-1) == (0, 0)
    assert c.point_at(1.5) == pytest.approx((1, 0.5))
    assert c.point_at(99) == (1, 1)
    assert corner_cut((0, 0), (0, 0), (3, 4)) == pytest.approx(5)


def test_set_path_resets_progress():
    c = follower([(0, 0), (5, 0)])
    fly_along_x(c, 0, 3)
    c.set_path([(0, 0), (0, 5)])
    assert c.s == 0.0 and not c.arrived
    assert c.update(0, 1) == pytest.approx((0, 1.5))
