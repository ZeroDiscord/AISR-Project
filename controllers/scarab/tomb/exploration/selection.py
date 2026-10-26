"""Choosing which frontier to explore next.

Three strategies are compared in the report:
  nearest  - shortest path distance (Yamauchi 1997 baseline)
  largest  - biggest frontier cluster
  utility  - information gain discounted by distance,
             U = IG * exp(-lambda * d)  (Gonzalez-Banos & Latombe 2002),
             with IG estimated by casting rays from the goal over the map.

All strategies share two robustness rules:
  hysteresis   - keep the current goal (tracked as its frontier recedes) for at
                 least COMMIT_S, and after that switch only to a goal scoring
                 HYSTERESIS times better, so the robot does not flip-flop
                 between frontiers on opposite sides of the tomb;
  blacklisting - goals that turned out unreachable, timed out or were already
                 visited are never chosen again.
"""

import math

import numpy as np
from scipy import ndimage

from .. import config
from ..mapping import OCCUPIED, UNKNOWN
from .frontiers import wall_mask


def attach_targets(frontiers, distance, grid, reach=config.FRONTIER_REACH):
    """Give each frontier a reachable target cell near its centroid.

    Frontier cells often lie inside the inflation zone (along walls), so the
    target is the reachable cell within `reach` of the cluster that is closest
    to the cluster centroid.
    """
    k = int(math.ceil(reach / grid.res))
    disk = np.hypot(*np.mgrid[-k:k + 1, -k:k + 1]) <= k
    n = grid.n
    for f in frontiers:
        r0, r1 = max(f.rows.min() - k, 0), min(f.rows.max() + k + 1, n)
        c0, c1 = max(f.cols.min() - k, 0), min(f.cols.max() + k + 1, n)
        mask = np.zeros((r1 - r0, c1 - c0), dtype=bool)
        mask[f.rows - r0, f.cols - c0] = True
        near = ndimage.binary_dilation(mask, structure=disk)
        cand = near & np.isfinite(distance[r0:r1, c0:c1])
        if not cand.any():
            continue
        rr, cc = np.nonzero(cand)
        rr, cc = rr + r0, cc + c0
        x, y = grid.to_world(cc, rr)
        j = int(np.argmin((x - f.centroid[0]) ** 2 + (y - f.centroid[1]) ** 2))
        f.target = (int(rr[j]), int(cc[j]))
        f.target_xy = (float(x[j]), float(y[j]))
        f.distance = float(distance[rr[j], cc[j]])


def information_gain(state, grid, x, y, rays=config.IG_RAYS, max_range=config.LIDAR_RANGE_CAP,
                     walls=None):
    """Area (m^2) of UNKNOWN cells a 360-degree scan from (x, y) could see.

    Rays stop at the first wall cell (OCCUPIED, or unknown right next to an
    occupied cell, see frontiers.wall_mask) so they cannot leak through
    half-seen walls into the unknown outside the tomb; other unknown cells are
    treated as see-through (the optimistic assumption of frontier exploration).
    Each ray sample stands for the ring-sector r * dtheta * dr around it, so
    the sum is an area estimate that does not depend on the number of rays.
    """
    if walls is None:
        walls = wall_mask(state)
    th = np.linspace(0, 2 * math.pi, rays, endpoint=False)
    dr = grid.res * 0.75
    t = np.arange(grid.res, max_range, dr)
    col, row = grid.to_cell(x + np.cos(th)[:, None] * t, y + np.sin(th)[:, None] * t)
    inside = (col >= 0) & (col < grid.n) & (row >= 0) & (row < grid.n)
    s = np.full(col.shape, OCCUPIED, dtype=np.int8)
    s[inside] = state[row[inside], col[inside]]
    wall = np.ones(col.shape, dtype=bool)
    wall[inside] = walls[row[inside], col[inside]]
    blocked = np.maximum.accumulate(wall, axis=1)
    seen = (s == UNKNOWN) & ~blocked
    area = np.broadcast_to(t * (2 * math.pi / rays) * dr, seen.shape)
    return float(area[seen].sum())


class FrontierSelector:
    def __init__(self, strategy):
        if strategy not in ("nearest", "largest", "utility"):
            raise ValueError(f"unknown strategy {strategy}")
        self.strategy = strategy
        self.blacklist = []

    def is_blacklisted(self, xy):
        return any(math.hypot(xy[0] - bx, xy[1] - by) < config.BLACKLIST_RADIUS
                   for bx, by in self.blacklist)

    def ban(self, xy):
        if xy is not None and not self.is_blacklisted(xy):
            self.blacklist.append(tuple(xy))

    def _score(self, f):
        if self.strategy == "nearest":
            return 1.0 / (f.distance + 0.25)
        if self.strategy == "largest":
            return f.size * config.MAP_RES
        return f.gain * math.exp(-config.UTILITY_LAMBDA * f.distance)

    def candidates(self, frontiers):
        return [f for f in frontiers if f.target is not None and not self.is_blacklisted(f.target_xy)]

    def choose(self, frontiers, state, grid, current_xy=None, may_switch=True):
        """Best candidate, honouring hysteresis. Returns (frontier or None, ranked list).

        current_xy is the goal being pursued; may_switch is False while it is
        still inside its commitment period.
        """
        cands = self.candidates(frontiers)
        # every strategy skips frontiers that would reveal almost nothing (corners
        # behind statues, gaps narrower than the robot); this is also what lets
        # the robot decide on its own that the tomb is finished
        walls = wall_mask(state) if cands else None
        for f in cands:
            f.gain = information_gain(state, grid, *f.target_xy, walls=walls)
        cands = [f for f in cands if f.gain >= config.IG_MIN]
        if not cands:
            return None, []
        for f in cands:
            f.score = self._score(f)
        ranked = sorted(cands, key=lambda f: f.score, reverse=True)
        best = ranked[0]
        if current_xy is not None:
            same = sorted((f for f in cands
                           if math.hypot(f.target_xy[0] - current_xy[0], f.target_xy[1] - current_xy[1])
                           < config.GOAL_TRACK_RADIUS),
                          key=lambda f: math.hypot(f.target_xy[0] - current_xy[0], f.target_xy[1] - current_xy[1]))
            if same and (not may_switch or best.score < config.HYSTERESIS * same[0].score):
                keep = same[0]
                ranked.remove(keep)
                ranked.insert(0, keep)
                return keep, ranked
        return best, ranked
