"""Frontier detection (Yamauchi, 1997).

A frontier cell is any FREE cell adjoining at least one UNKNOWN 4-neighbour,
which places it on the boundary between the explored tomb and the unexplored
one. Adjacent frontier cells are gathered into clusters by 8-connected
labelling, and a cluster holding fewer than FRONTIER_MIN_CELLS cells is
discarded as map noise.

An unknown cell sitting directly beside an OCCUPIED cell counts as wall
rather than as open unexplored space. Wall surfaces struck at shallow angles
accumulate hit and pass-through evidence alike, which can leave them
'unknown' indefinitely; were they counted, frontiers would shimmer along
every wall with no way of ever clearing them.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .. import config
from ..mapping import FREE, OCCUPIED, UNKNOWN

EIGHT = np.ones((3, 3), dtype=bool)


def wall_mask(state):
    """OCCUPIED dilated by a single cell; unknown cells inside belong to a wall."""
    return ndimage.binary_dilation(state == OCCUPIED, structure=EIGHT)


@dataclass(eq=False)          # identity comparison: these fields hold NumPy arrays
class Frontier:
    rows: np.ndarray
    cols: np.ndarray
    centroid: tuple                  # world (x, y) at the cluster's mean cell
    target: tuple = None             # reachable (row, col) that the robot heads for
    target_xy: tuple = None
    distance: float = np.inf         # wavefront path length out to the target (m)
    gain: float = 0.0                # information gain at that target (m^2 of unknown)
    score: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def size(self):
        return len(self.rows)


def frontier_mask(state, walls=None):
    """Frontier cells as a boolean grid."""
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
    """Group the frontier cells into clusters; returns a list of Frontier objects."""
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
