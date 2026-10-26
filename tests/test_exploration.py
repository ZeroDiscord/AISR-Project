"""Small checks for the mapping, frontier and planning modules.

    python -m pytest tests        (or: python tests/test_exploration.py)
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers", "scarab"))

from tomb import config  # noqa: E402
from tomb.exploration import detect_frontiers, frontier_mask, information_gain  # noqa: E402
from tomb.mapping import FREE, OCCUPIED, UNKNOWN, OccupancyGrid  # noqa: E402
from tomb.planning import Costmap, astar  # noqa: E402


def room_state(n=config.MAP_N):
    """A 4 x 4 m known room in the middle of the map, walls all round, unknown outside."""
    s = np.full((n, n), UNKNOWN, dtype=np.int8)
    c = n // 2
    s[c - 40:c + 40, c - 40:c + 40] = OCCUPIED
    s[c - 39:c + 39, c - 39:c + 39] = FREE
    return s, c


def test_grid_marks_hit_and_free_cells():
    g = OccupancyGrid()
    for _ in range(5):
        g.update(0.0, 0.0, 0.0, np.array([2.0]), np.array([0.0]))
    s = g.state()
    col, row = g.to_cell(2.0, 0.0)
    assert s[row, col] == OCCUPIED
    col, row = g.to_cell(1.0, 0.0)
    assert s[row, col] == FREE
    col, row = g.to_cell(3.0, 0.0)
    assert s[row, col] == UNKNOWN


def test_closed_room_has_no_frontier_until_a_door_opens():
    s, c = room_state()
    assert not frontier_mask(s).any()
    s[c - 10:c + 10, c + 39] = FREE                 # 1 m doorway in the east wall
    fr = detect_frontiers(s, OccupancyGrid())
    # the two door-edge cells touch the wall and do not count
    assert len(fr) == 1 and fr[0].size == 18


def test_half_seen_wall_is_not_a_frontier():
    s, c = room_state()
    s[c - 5:c + 5, c + 38] = UNKNOWN                # a stretch of wall face never confirmed
    assert not frontier_mask(s).any()


def test_astar_goes_round_a_wall():
    s, c = room_state()
    s[c - 39:c + 20, c] = OCCUPIED                  # wall with a gap at the top
    cm = Costmap(s)
    start, goal = (c - 20, c - 20), (c - 20, c + 20)
    path, _ = astar(cm, start, goal)
    assert path is not None and path[0] == start and path[-1] == goal
    assert max(r for r, _ in path) > c + 20         # had to go round through the gap
    assert all(cm.traversable[r, k] for r, k in path)


def test_astar_reports_no_path():
    s, c = room_state()
    s[c - 39:c + 39, c] = OCCUPIED                  # wall right across the room
    path, _ = astar(Costmap(s), (c, c - 20), (c, c + 20))
    assert path is None


def test_wavefront_and_astar_agree_on_diagonal_squeezes():
    """Two traversable regions touching only at a corner: neither planner may pass."""
    s, c = room_state()
    cm = Costmap(s)
    cm.traversable[:] = False
    cm.traversable[c - 10:c, c - 10:c] = True      # top-left block
    cm.traversable[c:c + 10, c:c + 10] = True      # bottom-right block, touching diagonally
    d = cm.wavefront((c - 5, c - 5))
    assert not np.isfinite(d[c + 5, c + 5])
    path, _ = astar(cm, (c - 5, c - 5), (c + 5, c + 5))
    assert path is None


def test_information_gain_counts_only_visible_unknown():
    s, c = room_state()
    g = OccupancyGrid()
    x, y = g.to_world(c, c)
    assert information_gain(s, g, float(x), float(y)) == 0.0     # closed room: nothing to see
    s[c - 10:c + 10, c + 39] = FREE
    assert information_gain(s, g, float(x), float(y)) > 0.5      # the doorway reveals area


def test_wavefront_close_to_euclidean():
    s, c = room_state()
    cm = Costmap(s)
    d = cm.wavefront((c, c))
    assert math.isclose(d[c, c + 20], 20 * config.MAP_RES, rel_tol=0.01)
    assert abs(d[c + 20, c + 20] - math.hypot(20, 20) * config.MAP_RES) < 0.07 * d[c + 20, c + 20]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
