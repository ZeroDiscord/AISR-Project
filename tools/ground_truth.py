"""Rasterise a tomb layout into ground-truth occupancy grids.

Two grids are produced:
  * a fine grid (1 cm) used by the 2D simulator to cast lidar rays;
  * a coarse grid in the controller's map frame (config.MAP_*), used to score
    the robot's map. A coarse cell is occupied if any part of an obstacle
    falls inside it, which is what a lidar beam ending in that cell reports.
"""

import math

import numpy as np
from scipy import ndimage

from layouts import Box, Cyl, WALL_T
import _paths  # noqa: F401  (makes the controller package importable)
from tomb import config

FINE_RES = 0.01


def _paint(occ, origin, res, ob):
    """Mark the cells covered by one obstacle footprint (in place)."""
    ny, nx = occ.shape
    if isinstance(ob, Cyl):
        reach = ob.r
    else:
        reach = 0.5 * math.hypot(ob.sx, ob.sy)
    i0 = max(int((ob.x - reach - origin[0]) / res) - 1, 0)
    i1 = min(int((ob.x + reach - origin[0]) / res) + 2, nx)
    j0 = max(int((ob.y - reach - origin[1]) / res) - 1, 0)
    j1 = min(int((ob.y + reach - origin[1]) / res) + 2, ny)
    if i0 >= i1 or j0 >= j1:
        return
    xs = origin[0] + (np.arange(i0, i1) + 0.5) * res
    ys = origin[1] + (np.arange(j0, j1) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    dx, dy = X - ob.x, Y - ob.y
    if isinstance(ob, Cyl):
        inside = dx * dx + dy * dy <= ob.r * ob.r
    else:
        c, s = math.cos(ob.yaw), math.sin(ob.yaw)
        u = c * dx + s * dy
        v = -s * dx + c * dy
        inside = (np.abs(u) <= ob.sx / 2) & (np.abs(v) <= ob.sy / 2)
    occ[j0:j1, i0:i1] |= inside


def fine_grid(layout):
    """1 cm occupancy grid covering the tomb including its outer walls."""
    half_w = layout.inner_w / 2 + WALL_T + 0.1
    half_h = layout.inner_h / 2 + WALL_T + 0.1
    origin = (-half_w, -half_h)
    nx = int(math.ceil(2 * half_w / FINE_RES))
    ny = int(math.ceil(2 * half_h / FINE_RES))
    occ = np.zeros((ny, nx), dtype=bool)
    for ob in layout.obstacles:
        _paint(occ, origin, FINE_RES, ob)
    return occ, origin


def coarse_from_fine(fine, fine_origin):
    """Max-pool the fine grid into the controller's map frame."""
    res, n = config.MAP_RES, config.MAP_N
    k = int(round(res / FINE_RES))
    out = np.zeros((n, n), dtype=bool)
    # offset of the fine grid inside the map frame, in fine cells
    oi = int(round((fine_origin[0] - config.MAP_ORIGIN[0]) / FINE_RES))
    oj = int(round((fine_origin[1] - config.MAP_ORIGIN[1]) / FINE_RES))
    big = np.zeros((n * k, n * k), dtype=bool)
    big[oj:oj + fine.shape[0], oi:oi + fine.shape[1]] = fine
    out[:] = big.reshape(n, k, n, k).any(axis=(1, 3))
    return out


def reachable(occ, start_xy):
    """Free cells connected to the start cell (the area there is to explore)."""
    labels, _ = ndimage.label(~occ)
    i = int((start_xy[0] - config.MAP_ORIGIN[0]) / config.MAP_RES)
    j = int((start_xy[1] - config.MAP_ORIGIN[1]) / config.MAP_RES)
    return labels == labels[j, i]


START_CLEARANCE = 0.5     # experiment start poses keep this far from obstacles (m)
N_STARTS = 5
STARTS_SEED = 2026


def start_poses(occ, reach, first, n=N_STARTS, seed=STARTS_SEED):
    """n start poses spread over the tomb by farthest-point sampling.

    Pose 0 is the layout's own start. The others are reachable cells with at
    least START_CLEARANCE to any obstacle, each as far as possible from the
    poses already chosen; headings are drawn from a fixed-seed generator so
    every run of the experiments uses the same poses.
    """
    clear = ndimage.distance_transform_edt(~occ) * config.MAP_RES
    rows, cols = np.nonzero(reach & (clear >= START_CLEARANCE))
    xs = config.MAP_ORIGIN[0] + (cols + 0.5) * config.MAP_RES
    ys = config.MAP_ORIGIN[1] + (rows + 0.5) * config.MAP_RES
    rng = np.random.default_rng(seed)
    poses = [tuple(first)]
    nearest = np.hypot(xs - first[0], ys - first[1])
    for _ in range(n - 1):
        k = int(np.argmax(nearest))
        poses.append((float(xs[k]), float(ys[k]), float(rng.uniform(-math.pi, math.pi))))
        nearest = np.minimum(nearest, np.hypot(xs - xs[k], ys - ys[k]))
    return np.array(poses)


def build(layout):
    fine, fine_origin = fine_grid(layout)
    occ = coarse_from_fine(fine, fine_origin)
    reach = reachable(occ, layout.start[:2])
    return {
        "starts": start_poses(occ, reach, layout.start),
        "name": layout.name,
        "fine_occ": fine,
        "fine_origin": np.array(fine_origin),
        "fine_res": FINE_RES,
        "occ": occ,
        "reachable": reach,
        "map_origin": np.array(config.MAP_ORIGIN),
        "map_res": config.MAP_RES,
        "start": np.array(layout.start),
    }
