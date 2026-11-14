"""B0.1 frames.py: FLU -> NED -> FLU round trip, yaw wrap at +-pi, the table of guidebook 2.3."""
import math
import random

import pytest

from drone_bridge import frames as F


def test_round_trip_1000_random_poses():
    rng = random.Random(1)
    for _ in range(1000):
        x, y, z = (rng.uniform(-50, 50) for _ in range(3))
        r, p, yw = rng.uniform(-math.pi, math.pi), rng.uniform(-1.5, 1.5), rng.uniform(-math.pi, math.pi)
        ned = F.pose_flu_to_ned(x, y, z, r, p, yw)
        back = F.pose_ned_to_flu(*ned)
        for a, b in zip((x, y, z), back[:3]):
            assert a == pytest.approx(b, abs=1e-12)
        for a, b in zip((r, p, yw), back[3:]):
            assert abs(F.wrap(a - b)) < 1e-12


@pytest.mark.parametrize('flu,ned', [
    ((1, 0, 0), (1, 0, 0)),      # forward position x -> north = x
    ((0, 1, 0), (0, -1, 0)),     # sideways y (left) -> east = -y
    ((0, 0, 1), (0, 0, -1)),     # height z (up) -> down = -z
    ((2, -3, 1.1), (2, 3, -1.1)),
])
def test_table_positions(flu, ned):
    assert F.flu_to_ned_xyz(*flu) == pytest.approx(ned)
    assert F.ned_to_flu_xyz(*ned) == pytest.approx(flu)


def test_table_angles():
    # heading: yaw_ned = -yaw ; roll / pitch -> roll / -pitch
    assert F.flu_to_ned_rpy(0.1, 0.2, 0.3) == pytest.approx((0.1, -0.2, -0.3))
    assert F.ned_to_flu_rpy(0.1, -0.2, -0.3) == pytest.approx((0.1, 0.2, 0.3))
    # a drone facing +y (left, yaw +90 deg in ROS) faces west in NED (yaw -90 deg)
    assert F.flu_to_ned_rpy(0, 0, math.pi / 2)[2] == pytest.approx(-math.pi / 2)


def test_wrap():
    assert F.wrap(math.pi) == pytest.approx(math.pi)
    assert F.wrap(-math.pi) == pytest.approx(math.pi)          # one representation for 180 deg
    assert F.wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert F.wrap(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert F.wrap(7 * math.pi) == pytest.approx(math.pi)
    # differences across the +-180 deg seam are small, not ~360 deg
    a, b = math.radians(179), math.radians(-179)
    assert math.degrees(F.wrap(a - b)) == pytest.approx(-2)
    assert math.degrees(F.wrap(b - a)) == pytest.approx(2)


def test_yaw_near_seam_converts_cleanly():
    for deg in (179.9, -179.9, 180.0):
        y = math.radians(deg)
        n = F.flu_to_ned_rpy(0, 0, y)[2]
        assert abs(F.wrap(n + y)) < 1e-12
        assert -math.pi < n <= math.pi


def test_quaternion_round_trip():
    rng = random.Random(2)
    for _ in range(500):
        r, p, yw = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        q = F.rpy_to_quat(r, p, yw)
        assert sum(c * c for c in q) == pytest.approx(1.0)
        r2, p2, y2 = F.quat_to_rpy(*q)
        assert abs(F.wrap(r - r2)) < 1e-9 and abs(F.wrap(p - p2)) < 1e-9 and abs(F.wrap(yw - y2)) < 1e-9


def test_quat_gimbal_lock_clamped():
    q = F.rpy_to_quat(0, math.pi / 2, 0)
    assert F.quat_to_rpy(*q)[1] == pytest.approx(math.pi / 2)


def test_rotate2d():
    assert F.rotate2d(math.pi / 2, 1, 0) == pytest.approx((0, 1))
    assert F.rotate2d(-math.pi / 2, 1, 0) == pytest.approx((0, -1))
