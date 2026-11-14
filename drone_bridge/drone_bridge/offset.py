"""Frame-offset tracking (guidebook 2.9). Pure maths, no ROS. Tested by test_offset.py (B0).

On each LOCAL_POSITION_NED + ATTITUDE from the flight controller (FC):
  take the SLAM pose closest in time (1 s buffer), converted to NED;
  dpsi = yaw_FC - yaw_SLAM;  d = p_FC - R(dpsi) * p_SLAM;  low-pass both (factor 0.1 at 30 Hz).
Every target:  p_FC = R(dpsi) * p_SLAM + d.
All positions here are horizontal NED (north, east); yaws are NED yaws.
"""
import bisect
import math

from .frames import rotate2d, wrap


class PoseBuffer:
    """Time-stamped SLAM poses (t, n, e, yaw_ned) for the last `span` seconds."""

    def __init__(self, span=1.0):
        self.span = span
        self.t = []
        self.v = []

    def add(self, t, n, e, yaw):
        if self.t and t < self.t[-1]:          # out of order: keep the buffer sorted
            i = bisect.bisect(self.t, t)
            self.t.insert(i, t)
            self.v.insert(i, (n, e, yaw))
        else:
            self.t.append(t)
            self.v.append((n, e, yaw))
        cut = bisect.bisect_left(self.t, self.t[-1] - self.span)
        if cut:
            del self.t[:cut]
            del self.v[:cut]

    def closest(self, t, max_gap=None):
        """(dt, (n, e, yaw)) of the sample closest to t, or None if empty / further than max_gap."""
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.t)]
        j = min(cands, key=lambda k: abs(self.t[k] - t))
        dt = self.t[j] - t
        if max_gap is not None and abs(dt) > max_gap:
            return None
        return dt, self.v[j]

    def __len__(self):
        return len(self.t)


class OffsetEstimator:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.dpsi = None
        self.dn = None
        self.de = None

    @property
    def ready(self):
        return self.dpsi is not None

    def update(self, fc_n, fc_e, fc_yaw, slam_n, slam_e, slam_yaw):
        """One FC sample matched with one SLAM sample (both NED). Returns the instantaneous (dpsi, dn, de)."""
        dpsi = wrap(fc_yaw - slam_yaw)
        rn, re = rotate2d(dpsi, slam_n, slam_e)
        dn, de = fc_n - rn, fc_e - re
        if self.dpsi is None:
            self.dpsi, self.dn, self.de = dpsi, dn, de
        else:
            a = self.alpha
            self.dpsi = wrap(self.dpsi + a * wrap(dpsi - self.dpsi))
            self.dn += a * (dn - self.dn)
            self.de += a * (de - self.de)
        return dpsi, dn, de

    def slam_to_fc(self, n, e):
        """SLAM-frame NED point -> FC local NED point."""
        if self.dpsi is None:
            return n, e
        rn, re = rotate2d(self.dpsi, n, e)
        return rn + self.dn, re + self.de

    def fc_to_slam(self, n, e):
        if self.dpsi is None:
            return n, e
        return rotate2d(-self.dpsi, n - self.dn, e - self.de)

    def magnitude(self):
        """(metres, degrees) for /bridge/status."""
        if self.dpsi is None:
            return float('nan'), float('nan')
        return math.hypot(self.dn, self.de), math.degrees(abs(self.dpsi))
