"""B0.3 offset estimator: recovers a known translation and rotation within 1 mm / 0.01 deg;
the low-pass converges within the expected time; time matching in the 1 s buffer."""
import math
import random

import pytest

from drone_bridge.frames import rotate2d, wrap
from drone_bridge.offset import OffsetEstimator, PoseBuffer


def fc_from_slam(n, e, yaw, dpsi, dn, de):
    rn, re = rotate2d(dpsi, n, e)
    return rn + dn, re + de, wrap(yaw + dpsi)


@pytest.mark.parametrize('dpsi_deg,dn,de', [(0, 0, 0), (5, 0.3, -0.2), (-170, 1.5, 2.0), (179.9, -0.01, 0.02)])
def test_recovers_known_offset(dpsi_deg, dn, de):
    rng = random.Random(3)
    est = OffsetEstimator(alpha=0.1)
    dpsi = math.radians(dpsi_deg)
    for _ in range(300):                      # 10 s at 30 Hz
        n, e, y = rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3)
        fn, fe, fy = fc_from_slam(n, e, y, dpsi, dn, de)
        est.update(fn, fe, fy, n, e, y)
    assert est.dn == pytest.approx(dn, abs=1e-3)
    assert est.de == pytest.approx(de, abs=1e-3)
    assert math.degrees(abs(wrap(est.dpsi - dpsi))) < 0.01
    # and the transform maps points correctly both ways
    p = (2.0, -1.0)
    q = est.slam_to_fc(*p)
    assert q == pytest.approx(fc_from_slam(*p, 0, dpsi, dn, de)[:2], abs=1e-3)
    assert est.fc_to_slam(*q) == pytest.approx(p, abs=1e-9)


def test_low_pass_convergence_time():
    # factor 0.1 per sample at 30 Hz: 95 % of a step after ln(0.05)/ln(0.9) = 28.4 samples (~0.95 s)
    est = OffsetEstimator(alpha=0.1)
    est.update(0, 0, 0, 0, 0, 0)
    k = 0
    while True:
        k += 1
        est.update(1.0, 0, 0, 0, 0, 0)
        if est.dn >= 0.95:
            break
    assert k == math.ceil(math.log(0.05) / math.log(0.9))
    assert k / 30.0 < 1.0


def test_identity_when_frames_agree():
    est = OffsetEstimator()
    for i in range(50):
        est.update(i * 0.1, 0.2, 0.3, i * 0.1, 0.2, 0.3)
    m, d = est.magnitude()
    assert m < 1e-9 and d < 1e-9


def test_not_ready_passthrough():
    est = OffsetEstimator()
    assert not est.ready
    assert est.slam_to_fc(1, 2) == (1, 2)
    assert est.fc_to_slam(1, 2) == (1, 2)
    assert all(math.isnan(v) for v in est.magnitude())


def test_yaw_filter_across_seam():
    est = OffsetEstimator(alpha=0.1)
    est.update(0, 0, math.radians(179), 0, 0, 0)
    for _ in range(100):
        est.update(0, 0, math.radians(-179), 0, 0, 0)
    assert math.degrees(est.dpsi) == pytest.approx(-179, abs=0.01)   # went the short way across 180


def test_buffer_closest_and_span():
    b = PoseBuffer(span=1.0)
    for i in range(100):
        b.add(i * 0.02, i, 0, 0)          # 2 s at 50 Hz
    assert b.t[0] >= b.t[-1] - 1.0 - 1e-9 # only the last second kept
    dt, v = b.closest(1.5)
    assert v[0] == 75 and abs(dt) < 1e-9
    dt, v = b.closest(1.511)
    assert v[0] == 76                     # 1.52 is closer than 1.50
    assert b.closest(5.0, max_gap=0.1) is None
    assert b.closest(1.98 + 0.05, max_gap=0.1)[1][0] == 99


def test_buffer_out_of_order_and_empty():
    b = PoseBuffer()
    assert b.closest(0) is None
    b.add(1.0, 1, 0, 0)
    b.add(0.5, 0.5, 0, 0)
    assert b.t == [0.5, 1.0] and len(b) == 2
