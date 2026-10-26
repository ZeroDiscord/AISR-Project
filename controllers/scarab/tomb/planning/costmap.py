"""Configuration-space costmap built from the occupancy grid.

* clearance:   distance from every cell to the nearest OCCUPIED cell (m)
* traversable: FREE cells with clearance > INFLATE_RADIUS, i.e. places the
               robot's centre can be without touching anything
* penalty:     extra step cost that grows as clearance shrinks towards the
               inflation radius, so planned paths keep to the middle of
               corridors instead of grazing walls
"""

import numpy as np
from scipy import ndimage

from .. import config
from ..mapping import FREE, OCCUPIED


def shifted(a, dr, dc):
    """b[i, j] = a[i - dr, j - dc], False where that falls outside the array."""
    b = np.zeros_like(a)
    n, m = a.shape
    b[max(dr, 0):n + min(dr, 0), max(dc, 0):m + min(dc, 0)] = \
        a[max(-dr, 0):n + min(-dr, 0), max(-dc, 0):m + min(-dc, 0)]
    return b


class Costmap:
    def __init__(self, state, res=config.MAP_RES):
        self.res = res
        self.clearance = ndimage.distance_transform_edt(state != OCCUPIED) * res
        self.traversable = (state == FREE) & (self.clearance > config.INFLATE_RADIUS)
        soft = config.CLEARANCE_SOFT - config.INFLATE_RADIUS
        self.penalty = config.CLEARANCE_WEIGHT * np.clip(
            (config.CLEARANCE_SOFT - self.clearance) / soft, 0.0, 1.0)

    def nearest_traversable(self, row, col, max_cells=12):
        """The robot can end up inside the inflation zone (e.g. after turning
        near a wall); planning then starts from the closest safe cell."""
        n = self.traversable.shape[0]
        if 0 <= row < n and 0 <= col < n and self.traversable[row, col]:
            return row, col
        r0, r1 = max(row - max_cells, 0), min(row + max_cells + 1, n)
        c0, c1 = max(col - max_cells, 0), min(col + max_cells + 1, n)
        rr, cc = np.nonzero(self.traversable[r0:r1, c0:c1])
        if len(rr) == 0:
            return None
        k = np.argmin((rr + r0 - row) ** 2 + (cc + c0 - col) ** 2)
        return int(rr[k] + r0), int(cc[k] + c0)

    def wavefront(self, start):
        """Path distance (m) from start to every traversable cell, inf if unreachable.

        Breadth-first wave expanded with whole-array operations, alternating
        4- and 8-connected steps; this 'octagonal' metric stays within ~6% of
        the Euclidean path length and is fast enough to run every re-plan.
        Diagonal steps obey the same no-corner-cutting rule as A*, so every
        cell the wave reaches is also reachable by A*.
        """
        rows = np.nonzero(self.traversable.any(axis=1))[0]
        cols = np.nonzero(self.traversable.any(axis=0))[0]
        r0, r1 = rows[0], rows[-1] + 1
        c0, c1 = cols[0], cols[-1] + 1
        trav = self.traversable[r0:r1, c0:c1]
        dist = np.full(trav.shape, np.inf)
        wave = np.zeros_like(trav)
        wave[start[0] - r0, start[1] - c0] = True
        dist[wave] = 0.0
        seen = wave.copy()
        step = 0
        while wave.any():
            step += 1
            grown = wave.copy()
            grown[1:, :] |= wave[:-1, :]
            grown[:-1, :] |= wave[1:, :]
            grown[:, 1:] |= wave[:, :-1]
            grown[:, :-1] |= wave[:, 1:]
            if step % 2 == 0:
                for dr, dc in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                    grown |= shifted(wave, dr, dc) & shifted(trav, dr, 0) & shifted(trav, 0, dc)
            wave = grown & trav & ~seen
            dist[wave] = step * self.res
            seen |= wave
        full = np.full(self.traversable.shape, np.inf)
        full[r0:r1, c0:c1] = dist
        return full
