"""Deciding which frontier to head for next.

The report compares three strategies:
  nearest  - least path distance (the Yamauchi 1997 baseline)
  largest  - the frontier cluster with the most cells
  utility  - information gain damped by distance,
             U = IG * exp(-lambda * d)  (Gonzalez-Banos & Latombe 2002),
             where IG comes from casting rays across the map at the goal.

Two robustness rules apply to all of them:
  hysteresis   - hold on to the present goal (following it as its frontier
                 retreats) for COMMIT_S at minimum, and thereafter move only
                 to a goal scoring HYSTERESIS times higher, which keeps the
                 robot from oscillating between frontiers at opposite ends
                 of the tomb;
  blacklisting - a goal that proved unreachable, ran out of time or has
                 already been visited is never picked again.
"""

import math

import numpy as np
from scipy import ndimage

from .. import config
from ..mapping import OCCUPIED, UNKNOWN
from .frontiers import wall_mask


def attach_targets(frontiers, distance, grid, reach=config.FRONTIER_REACH):
    """Assign every frontier a reachable target cell close to its centroid.

    Because frontier cells frequently sit within the inflation zone (hugging
    walls), the target chosen is whichever reachable cell within `reach` of
    the cluster lies nearest the cluster centroid.
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
    """Area (m^2) of UNKNOWN cells a 360-degree scan at (x, y) could reveal.

    Each ray halts at its first wall cell (OCCUPIED, or unknown immediately
    adjacent to an occupied one, cf. frontiers.wall_mask), which prevents
    leakage through half-seen walls out into the unknown beyond the tomb;
    every other unknown cell is assumed transparent, the optimistic premise
    behind frontier exploration. A ray sample represents the ring sector
    r * dtheta * dr surrounding it, so the total is an area estimate that is
    independent of how many rays are cast.
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
        """Top candidate subject to hysteresis. Returns (frontier or None, ranked list).

        current_xy is whichever goal is currently being pursued; may_switch
        stays False while that goal remains inside its commitment period.
        """
        cands = self.candidates(frontiers)
        # all strategies pass over frontiers promising almost no new ground (nooks
        # behind statues, gaps too tight for the robot); this doubles as the test
        # by which the robot concludes for itself that the tomb is done
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
