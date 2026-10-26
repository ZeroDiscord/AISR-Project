"""Log-odds occupancy grid with a vectorised inverse sensor model.

For every beam the cells it passes through get L_FREE added, and the cell
where it ends gets L_OCC added (only if the beam actually hit something
within LIDAR_RANGE_CAP). Beams are sampled every half cell, which visits the
same cells as a Bresenham line traversal but runs as one NumPy operation for
the whole scan.
"""

import numpy as np

from .. import config

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


class OccupancyGrid:
    def __init__(self, res=config.MAP_RES, n=config.MAP_N, origin=config.MAP_ORIGIN):
        self.res = res
        self.n = n
        self.origin = np.asarray(origin, dtype=float)
        self.log_odds = np.zeros((n, n), dtype=np.float32)   # indexed [row = y, col = x]

    # ---------------------------------------------------------- frames
    def to_cell(self, x, y):
        """World coordinates -> (col, row) integer indices."""
        col = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        return col, row

    def to_world(self, col, row):
        """Cell indices -> world coordinates of the cell centre."""
        return (self.origin[0] + (np.asarray(col) + 0.5) * self.res,
                self.origin[1] + (np.asarray(row) + 0.5) * self.res)

    # ---------------------------------------------------------- update
    def update(self, sx, sy, syaw, ranges, angles):
        """Integrate one scan taken from sensor pose (sx, sy, syaw)."""
        r = np.asarray(ranges, dtype=float)
        cap = config.LIDAR_RANGE_CAP
        finite = np.isfinite(r)
        hit = finite & (r >= config.LIDAR_MIN_RANGE) & (r <= cap)
        # a return past the cap proves free space up to the cap. inf readings are
        # discarded: inside a closed tomb they only come from rendering seams in
        # the Webots lidar (or objects closer than its minimum range)
        open_beam = finite & (r > cap)
        use = hit | open_beam
        if not use.any():
            return
        length = np.where(hit, r, cap)[use]
        theta = syaw + np.asarray(angles)[use]
        ct, st = np.cos(theta), np.sin(theta)

        # free cells: samples along each beam, stopping short of the end cell
        step = self.res * 0.5
        t = np.arange(0.0, length.max(), step)
        along = t[None, :] < (length[:, None] - step)
        fx = sx + ct[:, None] * t[None, :]
        fy = sy + st[:, None] * t[None, :]
        free = self._flat(fx[along], fy[along])

        # occupied cells: end points of beams that hit something
        hit_used = hit[use]
        occ = self._flat(sx + ct[hit_used] * length[hit_used],
                         sy + st[hit_used] * length[hit_used])

        free = np.setdiff1d(free, occ, assume_unique=False)
        lo = self.log_odds.reshape(-1)
        lo[free] += config.L_FREE
        lo[occ] += config.L_OCC
        np.clip(self.log_odds, config.L_MIN, config.L_MAX, out=self.log_odds)

    def _flat(self, x, y):
        col, row = self.to_cell(x, y)
        ok = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return np.unique(row[ok] * self.n + col[ok])

    # ---------------------------------------------------------- queries
    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def state(self):
        """int8 grid of UNKNOWN / FREE / OCCUPIED using the P_FREE / P_OCC thresholds."""
        p = self.probability()
        s = np.full(p.shape, UNKNOWN, dtype=np.int8)
        s[p < config.P_FREE] = FREE
        s[p > config.P_OCC] = OCCUPIED
        return s

    def known_area(self):
        """Area (m^2) classified as free or occupied."""
        return float((self.state() != UNKNOWN).sum()) * self.res ** 2
