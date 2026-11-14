"""B0.5 pose_mangler.py: delay, noise, drift, step and dropout produce exactly the configured effect."""
import math
import os
import statistics
import sys

import pytest

HERE = os.path.dirname(os.path.realpath(__file__))   # realpath: works with colcon --symlink-install
TOOLS = os.path.join(HERE, '..', '..', 'drone_sim', 'tools')
sys.path.insert(0, os.path.abspath(TOOLS))
from pose_mangler import Mangler  # noqa: E402


def test_passthrough_by_default():
    m = Mangler()
    assert m.apply(1.0, 1, 2, 3, 0.5) == pytest.approx((1, 2, 3, 0.5))


def test_delay_exact():
    m = Mangler(delay=0.1)
    for i in range(10):
        m.push(i * 0.02, i, 0, 0, 0)
    assert m.pop_ready(0.099) == []
    out = m.pop_ready(0.10)
    assert [p[0] for _, p, _ in out] == [0]          # only the first sample, at exactly +0.1 s
    out = m.pop_ready(0.18 + 0.1)
    assert [p[0] for _, p, _ in out] == list(range(1, 10))
    assert m.queue == []


def test_delay_keeps_original_stamp_and_extra():
    m = Mangler(delay=0.05)
    m.push(2.0, 0, 0, 0, 0, extra='q')
    (ts, _, ex), = m.pop_ready(2.05)
    assert ts == 2.0 and ex == 'q'


def test_noise_statistics():
    m = Mangler(noise=0.02, seed=7)
    xs, ys = [], []
    for i in range(20000):
        x, y, z, yaw = m.apply(i * 0.01, 0, 0, 1.1, 0.3)
        xs.append(x)
        ys.append(y)
        assert z == 1.1 and yaw == 0.3                # noise only on x, y
    assert statistics.pstdev(xs) == pytest.approx(0.02, rel=0.03)
    assert statistics.pstdev(ys) == pytest.approx(0.02, rel=0.03)
    assert abs(statistics.mean(xs)) < 0.001


def test_noise_reproducible_with_seed():
    a = [Mangler(noise=0.05, seed=3).apply(0, 0, 0, 0, 0) for _ in range(2)]
    assert a[0] == a[1]


def test_drift_rate_and_hold():
    # B7 synthetic slip: 0.1 m/s lengthwise for 3 s starting at 10 s
    m = Mangler(drift_start=10, drift_duration=3, drift_vx=0.1)
    assert m.apply(9.9, 0, 0, 0, 0)[0] == pytest.approx(0)
    assert m.apply(11.0, 0, 0, 0, 0)[0] == pytest.approx(0.1)
    assert m.apply(12.5, 0, 0, 0, 0)[0] == pytest.approx(0.25)
    assert m.apply(13.0, 0, 0, 0, 0)[0] == pytest.approx(0.3)
    assert m.apply(60.0, 0, 0, 0, 0)[0] == pytest.approx(0.3)      # holds after the drift ends


def test_step_translation_and_yaw():
    # B4: 0.3 m step in x at t = 30 s; 5 deg yaw step
    m = Mangler(step_time=30, step_x=0.3, step_yaw=math.radians(5))
    assert m.apply(29.99, 1, 1, 0, 0) == pytest.approx((1, 1, 0, 0))
    x, y, z, yaw = m.apply(30.0, 1, 1, 0, 0)
    assert (x, y) == pytest.approx((1.3, 1))
    assert math.degrees(yaw) == pytest.approx(5)
    assert m.reset_counter == 0                       # step_reset not requested


def test_step_yaw_wraps():
    m = Mangler(step_time=0, step_yaw=math.radians(10))
    yaw = m.apply(1, 0, 0, 0, math.radians(175))[3]
    assert math.degrees(yaw) == pytest.approx(-175)


def test_step_with_reset_counter_once():
    m = Mangler(step_time=5, step_x=0.3, step_reset=True)
    m.apply(4, 0, 0, 0, 0)
    assert m.reset_counter == 0
    m.apply(5, 0, 0, 0, 0)
    m.apply(6, 0, 0, 0, 0)
    assert m.reset_counter == 1


def test_dropout_window():
    m = Mangler(dropout_start=2, dropout_duration=1)
    assert m.apply(1.99, 0, 0, 0, 0) is not None
    assert m.apply(2.0, 0, 0, 0, 0) is None
    assert m.apply(2.99, 0, 0, 0, 0) is None
    assert m.apply(3.0, 0, 0, 0, 0) is not None
    m.push(2.5, 0, 0, 0, 0)
    assert m.queue == []                               # dropped inputs are never queued
