"""Frontier detection (Yamauchi, 1997).

A frontier cell is a FREE cell with at least one UNKNOWN 4-neighbour: the
border between what the robot has seen and what it has not. Neighbouring
frontier cells are grouped into clusters with 8-connected labelling, and
clusters smaller than FRONTIER_MIN_CELLS are dropped as map noise.

Unknown cells right next to an OCCUPIED cell are treated as wall, not as open
unknown space: wall faces seen at grazing angles get both hit and pass-through
evidence and can stay 'unknown' for ever. Counting them would create
frontiers that flicker along every wall and can never be cleared.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .. import config
from ..mapping import FREE, OCCUPIED, UNKNOWN

EIGHT = np.ones((3, 3), dtype=bool)


def wall_mask(state):
    """OCCUPIED cells grown by one cell: unknown cells in here belong to a wall."""
    return ndimage.binary_dilation(state == OCCUPIED, structure=EIGHT)


@dataclass(eq=False)          # compare by identity: fields hold NumPy arrays
class Frontier:
    rows: np.ndarray
    cols: np.ndarray
    centroid: tuple                  # world (x, y) of the cluster's mean cell
    target: tuple = None             # reachable (row, col) the robot drives to
    target_xy: tuple = None
    distance: float = np.inf         # wavefront path distance to the target (m)
    gain: float = 0.0                # information gain at the target (m^2 of unknown)
    score: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def size(self):
        return len(self.rows)


def frontier_mask(state, walls=None):
    """Boolean grid of frontier cells."""
    if walls is None:
        walls = wall_mask(state)
    unknown = (state == UNKNOWN) & ~walls
    touches = np.zeros_like(unknown)
    touches[1:, :] |= unknown[:-1, :]
    touches[:-1, :] |= unknown[1:, :]
    touches[:, 1:] |= unknown[:, :-1]
    touches[:, :-1] |= unknown[:, 1:]
    return (state == FREE) & touches


def detect_frontiers(state, grid, min_cells=config.FRONTIER_MIN_CELLS, walls=None):
    """Cluster the frontier cells; returns a list of Frontier objects."""
    mask = frontier_mask(state, walls)
    labels, n = ndimage.label(mask, structure=EIGHT)
    if n == 0:
        return []
    frontiers = []
    for i, sl in enumerate(ndimage.find_objects(labels), start=1):
        rr, cc = np.nonzero(labels[sl] == i)
        if len(rr) < min_cells:
            continue
        rr = rr + sl[0].start
        cc = cc + sl[1].start
        x, y = grid.to_world(cc, rr)
        frontiers.append(Frontier(rows=rr, cols=cc, centroid=(float(x.mean()), float(y.mean()))))
    return frontiers
