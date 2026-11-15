"""Bayesian log-odds 2D occupancy grid mapping with vectorised beam casting.

Theoretical Background:
-----------------------
Occupancy grid mapping represents the environment as a field of independent,
binary random variables m_i in {FREE, OCCUPIED} (Moravec & Elfes, 1985).

Log-Odds Representation:
    To avoid floating-point underflow and simplify recursive Bayesian updates,
    cell occupancy probabilities p(m_i) are mapped to log-odds:
        L(m_i) = ln( p(m_i) / (1 - p(m_i)) )

    The inverse transformation recovering probabilities is:
        p(m_i) = 1 / (1 + exp(-L(m_i))) = 1 - 1 / (1 + exp(L(m_i)))

Recursive Bayesian Measurement Update:
    Assuming a static prior p(m_i) = 0.5 (equal prior probability of free vs
    occupied space), the initial log-odds L_0 = ln(0.5 / 0.5) = 0.
    Under the standard inverse sensor model, each range measurement z_t from
    sensor pose x_t updates the log-odds additively:
        L_t(m_i) = L_{t-1}(m_i) + inv_sensor_model(m_i, z_t, x_t)

    * Cells traversed by a lidar beam receive negative evidence (L_FREE = -0.40).
    * Cells containing the beam endpoint receive positive evidence (L_OCC = +0.85).
    * Log-odds are clamped to [L_MIN, L_MAX] = [-5.0, 5.0] to prevent infinite
      confidence and allow the map to recover from transient occlusion or noise.

Vectorised Beam Casting:
    Rather than executing sequential Bresenham line traversals across 180 rays
    in Python loops, beam ranges are discretised at half-cell resolution
    (dr = res / 2). The traversal samples for all beams are evaluated as a single
    vectorised NumPy array operation, providing real-time update rates (~1 ms).
"""

import math
from typing import Optional, Tuple, Union

import numpy as np

from .. import config

# Cell state classifications
UNKNOWN: int = -1
FREE: int = 0
OCCUPIED: int = 1


class OccupancyGrid:
    """2D Log-Odds Occupancy Grid.

    Parameters
    ----------
    res : float
        Grid cell side length in metres (default: config.MAP_RES = 0.05 m).
    n : int
        Grid dimensions in cells along each axis (default: config.MAP_N = 320).
    origin : tuple of float
        World (x, y) coordinates of the bottom-left corner of cell [0, 0]
        (default: config.MAP_ORIGIN = (-8.0, -8.0) m).
    """

    def __init__(
        self,
        res: float = config.MAP_RES,
        n: int = config.MAP_N,
        origin: Tuple[float, float] = config.MAP_ORIGIN,
    ):
        self.res: float = float(res)
        self.n: int = int(n)
        self.origin: np.ndarray = np.asarray(origin, dtype=float)
        # 2D array indexed by [row = y, col = x]
        self.log_odds: np.ndarray = np.zeros((self.n, self.n), dtype=np.float32)

    # ------------------------------------------------------------------ frames
    def to_cell(
        self, x: Union[float, np.ndarray], y: Union[float, np.ndarray]
    ) -> Tuple[Union[int, np.ndarray], Union[int, np.ndarray]]:
        """Convert continuous world coordinates to discrete grid indices.

        Parameters
        ----------
        x, y : float or array-like
            World positions in metres.

        Returns
        -------
        col, row : int or ndarray of int
            Grid indices where col is along x and row is along y.
        """
        col = np.floor((np.asarray(x, dtype=float) - self.origin[0]) / self.res).astype(int)
        row = np.floor((np.asarray(y, dtype=float) - self.origin[1]) / self.res).astype(int)
        if np.ndim(col) == 0:
            return int(col), int(row)
        return col, row

    def to_world(
        self, col: Union[int, np.ndarray], row: Union[int, np.ndarray]
    ) -> Tuple[Union[float, np.ndarray], Union[float, np.ndarray]]:
        """Convert discrete grid indices to continuous world coordinates at cell centres.

        Parameters
        ----------
        col, row : int or array-like
            Grid indices.

        Returns
        -------
        x, y : float or ndarray of float
            Coordinates of cell centres in metres.
        """
        x = self.origin[0] + (np.asarray(col, dtype=float) + 0.5) * self.res
        y = self.origin[1] + (np.asarray(row, dtype=float) + 0.5) * self.res
        if np.ndim(x) == 0:
            return float(x), float(y)
        return x, y

    def in_bounds(
        self, col: Union[int, np.ndarray], row: Union[int, np.ndarray]
    ) -> Union[bool, np.ndarray]:
        """Check whether cell indices fall within the map bounds."""
        c = np.asarray(col)
        r = np.asarray(row)
        return (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)

    def _flatten_indices(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Helper to convert world (x, y) arrays into unique, in-bounds flat 1D cell indices."""
        col, row = self.to_cell(x, y)
        valid = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return np.unique(row[valid] * self.n + col[valid])

    # ------------------------------------------------------------------ update
    def update(
        self,
        sx: float,
        sy: float,
        syaw: float,
        ranges: np.ndarray,
        angles: np.ndarray,
    ):
        """Integrate a lidar scan taken from sensor pose (sx, sy, syaw).

        Parameters
        ----------
        sx, sy : float
            Sensor position in the world frame (metres).
        syaw : float
            Sensor heading in the world frame (radians).
        ranges : array-like
            Measured distances per beam (metres).
        angles : array-like
            Angular offset per beam relative to sensor heading (radians).
        """
        r = np.asarray(ranges, dtype=float)
        cap = float(config.LIDAR_RANGE_CAP)
        finite = np.isfinite(r)

        # Valid obstacle returns within operational cap
        hit = finite & (r >= config.LIDAR_MIN_RANGE) & (r <= cap)
        # Returns exceeding cap verify open/free space along the beam up to cap
        open_beam = finite & (r > cap)
        usable = hit | open_beam

        if not usable.any():
            return

        beam_lengths = np.where(hit, r, cap)[usable]
        ray_thetas = syaw + np.asarray(angles, dtype=float)[usable]
        cos_t = np.cos(ray_thetas)
        sin_t = np.sin(ray_thetas)

        # 1. Pass-through free space: sample along each beam at step = res / 2
        step = self.res * 0.5
        max_len = float(beam_lengths.max())
        sample_dists = np.arange(0.0, max_len, step)

        # Discard samples that reach into or past the obstacle cell
        in_free_corridor = sample_dists[None, :] < (beam_lengths[:, None] - step)
        free_x = sx + cos_t[:, None] * sample_dists[None, :]
        free_y = sy + sin_t[:, None] * sample_dists[None, :]
        free_cells = self._flatten_indices(free_x[in_free_corridor], free_y[in_free_corridor])

        # 2. Obstacle endpoints: beams that scored a confirmed hit
        hit_mask = hit[usable]
        occ_x = sx + cos_t[hit_mask] * beam_lengths[hit_mask]
        occ_y = sy + sin_t[hit_mask] * beam_lengths[hit_mask]
        occ_cells = self._flatten_indices(occ_x, occ_y)

        # An obstacle detection takes precedence over pass-through evidence
        free_cells = np.setdiff1d(free_cells, occ_cells, assume_unique=False)

        # 3. Additive log-odds update & saturation clamping
        flat_grid = self.log_odds.reshape(-1)
        flat_grid[free_cells] += config.L_FREE
        flat_grid[occ_cells] += config.L_OCC
        np.clip(self.log_odds, config.L_MIN, config.L_MAX, out=self.log_odds)

    # ----------------------------------------------------------------- queries
    def probability(self) -> np.ndarray:
        """Return the occupancy probability grid P(m_i = OCCUPIED) in [0.0, 1.0]."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def state(self) -> np.ndarray:
        """Classify each cell into UNKNOWN (-1), FREE (0), or OCCUPIED (1)."""
        probs = self.probability()
        classified = np.full(probs.shape, UNKNOWN, dtype=np.int8)
        classified[probs < config.P_FREE] = FREE
        classified[probs > config.P_OCC] = OCCUPIED
        return classified

    def known_area(self) -> float:
        """Return total area (m^2) classified as either FREE or OCCUPIED."""
        classified = self.state()
        return float((classified != UNKNOWN).sum()) * (self.res ** 2)

    def free_area(self) -> float:
        """Return total area (m^2) classified as FREE."""
        return float((self.state() == FREE).sum()) * (self.res ** 2)

    def occupied_area(self) -> float:
        """Return total area (m^2) classified as OCCUPIED."""
        return float((self.state() == OCCUPIED).sum()) * (self.res ** 2)

    def entropy(self, mask: Optional[np.ndarray] = None) -> float:
        """Compute the total Shannon entropy (in bits) of the map.

        Entropy quantifies the information uncertainty across grid cells:
            H(p) = -p * log2(p) - (1 - p) * log2(1 - p)

        Maximum entropy occurs at p = 0.5 (unknown, 1 bit per cell).
        Minimum entropy occurs as p approaches 0.0 or 1.0 (certainty, 0 bits).

        Parameters
        ----------
        mask : boolean ndarray, optional
            Optional mask specifying which cells to include (e.g. reachable space).

        Returns
        -------
        float
            Total map entropy in bits.
        """
        p = self.probability()
        if mask is not None:
            p = p[mask]

        # Numerical safety: clip away from 0.0 and 1.0 to prevent log2(0)
        p = np.clip(p, 1e-7, 1.0 - 1e-7)
        cell_entropy = -p * np.log2(p) - (1.0 - p) * np.log2(1.0 - p)
        return float(cell_entropy.sum())

    def reset(self):
        """Reset the occupancy grid to uniform prior log-odds (0.0)."""
        self.log_odds.fill(0.0)

    def copy(self) -> "OccupancyGrid":
        """Return a deep copy of the occupancy grid."""
        clone = OccupancyGrid(res=self.res, n=self.n, origin=self.origin)
        clone.log_odds[:] = self.log_odds
        return clone
