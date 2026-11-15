"""Unit tests for the frontier search (no ROS)."""
import time

import numpy as np

from drone_frontier.frontier_core import Params, convert, search

RES = 0.05


def room(h=60, w=60):
    """A closed room: walls on the border, free inside (map values 0 / 100)."""
    m = np.zeros((h, w), np.int16)
    m[0, :] = m[-1, :] = m[:, 0] = m[:, -1] = 100
    return m


def test_closed_room_has_no_frontier():
    fr, info = search(room(), RES, (0.0, 0.0), (1.5, 1.5))
    assert fr == [] and info['reachable_free'] > 0


def test_corridor_half_known_gives_one_frontier_at_its_end():
    m = np.full((20, 200), -1, np.int16)            # 1 m x 10 m corridor, x along columns
    m[0, :] = m[-1, :] = 100
    m[1:-1, :100] = 0                                # first 5 m seen
    fr, _ = search(m, RES, (0.0, 0.0), (0.5, 0.5))
    assert len(fr) == 1
    f = fr[0]
    assert abs(f.centroid[0] - 5.025) < 1e-6           # column 100's centre
    assert abs(f.size_m - 18 * RES) < 1e-6            # the 18 free rows of the corridor
    assert abs(f.distance_m - np.hypot(5.025 - 0.5, min(abs(0.075 - 0.5), 0.425))) < 0.06


def test_narrow_opening_is_ignored_and_wide_one_kept():
    m = room(60, 60)
    m[20:40, 59] = 0                                 # wall gap on the right
    out = np.full((60, 80), -1, np.int16)
    out[:, :60] = m
    narrow = out.copy()
    narrow[20:40, 59] = 100
    narrow[28:36, 59] = 0                            # 8 cells = 0.40 m
    assert search(narrow, RES, (0, 0), (1.5, 1.5))[0] == []
    wide = narrow.copy()
    wide[25:40, 59] = 0                              # 15 cells = 0.75 m
    fr, _ = search(wide, RES, (0, 0), (1.5, 1.5))
    assert len(fr) == 1 and abs(fr[0].size_m - 0.75) < 1e-6


def test_unknown_next_to_unreachable_free_space_is_not_a_frontier():
    m = room(60, 120)
    m[:, 60] = 100                                   # dividing wall: right half is closed off
    m[10:50, 80:100] = -1                            # unknown hole on the far side
    fr, _ = search(m, RES, (0, 0), (1.0, 1.0))
    assert fr == []


def test_cartographer_values_need_the_matching_threshold():
    m = np.full((20, 200), -1, np.int16)
    m[0, :] = m[-1, :] = 80                          # walls 60-95
    m[1:-1, :100] = 45                               # Cartographer free cells sit near 40-49
    assert len(search(m, RES, (0, 0), (0.5, 0.5), Params(free_threshold=65))[0]) == 1
    assert search(m, RES, (0, 0), (0.5, 0.5), Params(free_threshold=25))[0] == []   # the integration trap


def test_convert_values():
    p = Params(free_threshold=25, occupied_threshold=65)
    c = convert(np.array([-1, 0, 24, 25, 64, 65, 100]), p)
    assert list(c) == [-1, 0, 0, -1, -1, 100, 100]


def test_robot_inside_a_wall_starts_from_the_nearest_free_cell():
    m = np.full((20, 200), -1, np.int16)
    m[0, :] = m[-1, :] = 100
    m[1:-1, :100] = 0
    fr, info = search(m, RES, (0.0, 0.0), (0.5, 0.0))   # robot on the bottom wall
    assert len(fr) == 1 and info['start_cell'][0] == 1


def test_sorted_cheapest_first():
    m = np.full((20, 400), -1, np.int16)
    m[0, :] = m[-1, :] = 100
    m[1:-1, 100:300] = 0                              # known middle part: frontiers at both ends
    fr, _ = search(m, RES, (0, 0), (14.0, 0.5))       # robot nearer the right end (x = 15 m)
    assert len(fr) == 2
    assert fr[0].cost <= fr[1].cost and fr[0].centroid[0] > fr[1].centroid[0]


def test_speed_on_a_320x320_map():
    rng = np.random.default_rng(1)
    m = np.zeros((320, 320), np.int16)
    m[::20, :] = 100
    m[:, ::20] = 100
    m[::20, 10::20] = 0                              # doors
    m[:, 10::20][::20] = 0
    m[160:, :] = -1                                  # top half unknown
    m[rng.random(m.shape) < 0.01] = -1               # 1 % speckle
    search(m, RES, (0, 0), (0.5, 0.5))
    t0 = time.perf_counter()
    for _ in range(5):
        search(m, RES, (0, 0), (0.5, 0.5))
    assert (time.perf_counter() - t0) / 5 < 0.05
