"""Render the occupancy grid as a 'papyrus map' for the on-robot Display."""

import math

import numpy as np

from .. import config

STONE = np.array([25, 25, 32], dtype=float)      # unknown
PAPYRUS = np.array([232, 212, 165], dtype=float)  # free
INK = np.array([120, 40, 20], dtype=float)        # occupied
GOLD = np.array([222, 175, 60], dtype=np.uint8)
RED = np.array([200, 60, 30], dtype=np.uint8)
LAPIS = np.array([40, 95, 210], dtype=np.uint8)   # frontiers
TEAL = np.array([20, 150, 140], dtype=np.uint8)   # planned path
ASH = np.array([120, 120, 120], dtype=np.uint8)   # blacklisted goals


class Hud:
    def __init__(self, grid, size=256, half_view=7.0):
        """Shows a fixed (2 * half_view) metre square around the world origin."""
        self.grid, self.size, self.half = grid, size, half_view
        px = (np.arange(size) + 0.5) / size * 2 * half_view - half_view
        col, _ = grid.to_cell(px, px)
        _, row = grid.to_cell(px, px[::-1])          # image row 0 is the top (max y)
        self.cols = np.clip(col, 0, grid.n - 1)
        self.rows = np.clip(row, 0, grid.n - 1)
        self.yy, self.xx = np.mgrid[0:size, 0:size]

    def to_px(self, x, y):
        s = self.size / (2 * self.half)
        return (np.asarray(x) + self.half) * s, (self.half - np.asarray(y)) * s

    def _dots(self, img, x, y, colour):
        px, py = self.to_px(x, y)
        px, py = np.asarray(px).astype(int).ravel(), np.asarray(py).astype(int).ravel()
        ok = (px >= 0) & (px < self.size) & (py >= 0) & (py < self.size)
        img[py[ok], px[ok]] = colour

    def _cross(self, img, x, y, colour, r=3):
        cx, cy = self.to_px(x, y)
        for d in range(-r, r + 1):
            self._dots_px(img, cx + d, cy + d, colour)
            self._dots_px(img, cx + d, cy - d, colour)

    def _dots_px(self, img, px, py, colour):
        px, py = int(px), int(py)
        if 0 <= px < self.size and 0 <= py < self.size:
            img[py, px] = colour

    def render(self, pose, trail, overlay=None):
        p = self.grid.probability()[np.ix_(self.rows, self.cols)][..., None]
        # p = 0 -> papyrus, 0.5 -> stone, 1 -> ink
        img = np.where(p < 0.5,
                       PAPYRUS + (STONE - PAPYRUS) * (p / 0.5),
                       STONE + (INK - STONE) * ((p - 0.5) / 0.5)).astype(np.uint8)

        if overlay is not None and len(overlay["frontier_rows"]):
            fx, fy = self.grid.to_world(overlay["frontier_cols"], overlay["frontier_rows"])
            self._dots(img, fx, fy, LAPIS)
        if len(trail) > 1:
            tx, ty = np.asarray(trail).T
            self._dots(img, tx, ty, GOLD)
        if overlay is not None:
            if overlay["path"] is not None:
                self._dots(img, overlay["path"][:, 0], overlay["path"][:, 1], TEAL)
            for bx, by in overlay["blacklist"]:
                self._cross(img, bx, by, ASH, r=2)
            if overlay["goal"] is not None:
                self._cross(img, *overlay["goal"], RED, r=4)

        x, y, yaw = pose
        cx, cy = self.to_px(x, y)
        r = config.ROBOT_RADIUS * self.size / (2 * self.half)
        img[(self.xx - cx) ** 2 + (self.yy - cy) ** 2 <= r * r] = GOLD
        for t in np.linspace(0, 2 * r, 12):
            self._dots_px(img, cx + t * math.cos(yaw), cy - t * math.sin(yaw), RED)
        return img
