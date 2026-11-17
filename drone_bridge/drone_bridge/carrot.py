"""Carrot follower (guidebook 2.9). Pure maths, no ROS. Tested by test_carrot.py (B0).

Every 0.1 s:
 1. find the path point closest to the drone, searching FORWARD from the last one (never backwards);
 2. walk forward along the path by the lookahead L to get the carrot;
 3. (caller) converts the carrot to the FC frame and sends it as a position target;
 4. at the path's end the carrot stays on the final point; "arrived" = within 0.15 m and slower than
    0.1 m/s for 1 s.
At a 90 deg corner the line drone->carrot cuts inside the path by at most 0.354 * L (0.21 m for L = 0.6).
With max_cut set (C4), the lookahead is shortened on bends (never below min_lookahead) until the path between
the drone's place on it and the carrot stays within max_cut of the straight line drone -> carrot: on straights
the full lookahead, round a wall end a short one, so the drone no longer cuts inside the path there.
keep_speed=True moves a shortened target out along the same direction to the full lookahead distance. That is
for the stub only (its point drone's speed follows the target distance): under ArduPilot (B5 re-run in Gazebo,
2026-11-16) the moved-out target lay beyond the inside of each corner and the drone flew 23 cm into the wall.
The bridge itself uses the fixed lookahead that B5 passed with (follow_max_cut_m 0 = off).

Coordinates are 2D (x, y) in the map frame. The search window keeps the closest-point search from
jumping to a later part of the path that happens to pass nearby (U-turns, loops).
"""
import math


class CarrotFollower:
    def __init__(self, lookahead=0.5, arrive_radius=0.15, arrive_speed=0.1, arrive_time=1.0,
                 search_window=None, max_cut=None, min_lookahead=0.25, keep_speed=False):
        self.L = float(lookahead)
        self.keep_speed = keep_speed         # move a shortened carrot out to L (stub only, see below)
        self.max_cut = max_cut               # None: fixed lookahead (the book's follower)
        self.min_L = min(float(min_lookahead), self.L)
        self.arrive_radius = arrive_radius
        self.arrive_speed = arrive_speed
        self.arrive_time = arrive_time
        # how far ahead of the current progress (path length, m) the closest-point search may look
        self.window = search_window if search_window is not None else 2.0 * self.L + 0.5
        self.clear()

    # ---------------------------------------------------------------- path handling
    def clear(self):
        self.pts = []
        self.cum = []
        self.s = 0.0           # progress along the path (m), never decreases
        self.seg = 0           # index of the segment containing s
        self._slow_since = None
        self.arrived = False

    def set_path(self, points):
        """points: iterable of (x, y). Consecutive duplicates are dropped."""
        self.clear()
        for p in points:
            p = (float(p[0]), float(p[1]))
            if not self.pts or math.hypot(p[0] - self.pts[-1][0], p[1] - self.pts[-1][1]) > 1e-6:
                self.pts.append(p)
        self.cum = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    @property
    def length(self):
        return self.cum[-1] if self.cum else 0.0

    @property
    def active(self):
        return bool(self.pts)

    def point_at(self, s):
        """Point at path length s (clamped to the path)."""
        if not self.pts:
            raise ValueError('no path')
        if len(self.pts) == 1 or s <= 0.0:
            return self.pts[0]
        if s >= self.length:
            return self.pts[-1]
        i = self._segment_of(s)
        a, b = self.pts[i], self.pts[i + 1]
        seg_len = self.cum[i + 1] - self.cum[i]
        t = (s - self.cum[i]) / seg_len
        return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])

    def _segment_of(self, s):
        i = self.seg
        while i + 1 < len(self.cum) - 1 and self.cum[i + 1] <= s:
            i += 1
        while i > 0 and self.cum[i] > s:
            i -= 1
        return i

    # ---------------------------------------------------------------- main step
    def closest_progress(self, x, y):
        """Path length of the point closest to (x, y), searched forward from the current progress
        and at most `window` metres ahead. Never returns less than the current progress."""
        if len(self.pts) < 2:
            return 0.0
        best_d, best_s = float('inf'), self.s
        limit = self.s + self.window
        i = self.seg
        while i < len(self.pts) - 1 and self.cum[i] <= limit:
            a, b = self.pts[i], self.pts[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            seg_len = self.cum[i + 1] - self.cum[i]
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / (seg_len * seg_len)
            t = min(1.0, max(0.0, t))
            s = self.cum[i] + t * seg_len
            if s < self.s:                       # never search backwards
                s = self.s
                px, py = self.point_at(s)
            else:
                px, py = a[0] + t * dx, a[1] + t * dy
            if s > limit:
                break
            d = math.hypot(x - px, y - py)
            if d < best_d - 1e-12:
                best_d, best_s = d, s
            i += 1
        return max(self.s, best_s)

    def update(self, x, y):
        """Advance progress for the drone at (x, y); returns the carrot (x, y)."""
        if not self.pts:
            raise ValueError('no path')
        self.s = self.closest_progress(x, y)
        self.seg = self._segment_of(self.s)
        la = self.lookahead_now((x, y))
        cx, cy = self.point_at(self.s + la)
        if self.keep_speed and la < self.L - 1e-9 and self.length - self.s > self.L:
            # shortened on a bend: keep the DIRECTION to the near carrot but put the target the full lookahead
            # away, so the flight controller keeps its speed (its speed follows the distance to the target; a
            # target 0.25 m away means 0.25 m/s). Near the path's end the carrot itself is the target.
            d = math.hypot(cx - x, cy - y)
            if d > 1e-6 and d < self.L:
                k = self.L / d
                cx, cy = x + (cx - x) * k, y + (cy - y) * k
        return cx, cy

    def lookahead_now(self, drone=None):
        """The lookahead for the current progress: L, shortened on a bend if max_cut is set (the path up to
        the carrot must stay within max_cut of the line the drone will fly: drone -> carrot)."""
        if self.max_cut is None or len(self.pts) < 3:
            return self.L
        L = self.L
        a = drone if drone is not None else self.point_at(self.s)
        while L > self.min_L + 1e-9:
            if self._bulge(a, self.s, self.s + L) <= self.max_cut:
                break
            L = max(self.min_L, L - 0.05)
        return L

    def _bulge(self, a, s0, s1):
        """Largest distance of the path between lengths s0 and s1 from the straight line a -> point_at(s1)."""
        b = self.point_at(s1)
        worst = 0.0
        i = self._segment_of(s0) + 1
        while i < len(self.pts) and self.cum[i] < s1:
            worst = max(worst, corner_cut(a, b, self.pts[i]))
            i += 1
        return worst

    def distance_to_end(self, x, y):
        if not self.pts:
            return 0.0
        e = self.pts[-1]
        return math.hypot(x - e[0], y - e[1])

    def check_arrived(self, x, y, speed, t):
        """Latching 'arrived': within arrive_radius of the end and slower than arrive_speed for arrive_time."""
        if self.arrived:
            return True
        if self.pts and self.distance_to_end(x, y) <= self.arrive_radius and speed < self.arrive_speed:
            if self._slow_since is None:
                self._slow_since = t
            if t - self._slow_since >= self.arrive_time:
                self.arrived = True
        else:
            self._slow_since = None
        return self.arrived


def corner_cut(p, q, corner):
    """Distance from `corner` to the segment p-q (how far the line drone->carrot cuts inside)."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(corner[0] - p[0], corner[1] - p[1])
    t = min(1.0, max(0.0, ((corner[0] - p[0]) * dx + (corner[1] - p[1]) * dy) / L2))
    return math.hypot(corner[0] - (p[0] + t * dx), corner[1] - (p[1] + t * dy))


def slew(prev, target, max_step):
    """Move prev toward target by at most max_step (m); prev None -> target. The bridge passes every carrot
    through this before sending it, so a new path that starts somewhere else (often behind the drone, when the
    mission brain picks a goal the other way) is not a sudden jump of the target: ArduPilot turned such jumps into
    hard reversals (I2, first full mission: 37 deg tilt, 1.45 m/s, into a wall)."""
    if prev is None:
        return tuple(target)
    dx, dy = target[0] - prev[0], target[1] - prev[1]
    d = math.hypot(dx, dy)
    if d <= max_step or d < 1e-9:
        return tuple(target)
    return (prev[0] + dx / d * max_step, prev[1] + dy / d * max_step)
