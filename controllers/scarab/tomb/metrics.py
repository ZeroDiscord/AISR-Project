"""Map quality against the ground truth produced by tools/world_gen.py."""

import os

import numpy as np

from .mapping import FREE, OCCUPIED, UNKNOWN

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
GT_DIR = os.path.join(REPO_ROOT, "ground_truth")


def load_ground_truth(world):
    path = os.path.join(GT_DIR, f"{world}.npz")
    if not os.path.exists(path):
        return None
    return dict(np.load(path))


def explored_fraction(state, gt):
    """Share of the reachable free space that the robot's map has classified."""
    reach = gt["reachable"]
    return float((state[reach] != UNKNOWN).sum()) / float(reach.sum())


def accuracy(state, gt):
    """Share of classified cells inside the tomb whose class matches the truth."""
    inside = gt["reachable"] | gt["occ"]
    known = inside & (state != UNKNOWN)
    if not known.any():
        return float("nan")
    truth = np.where(gt["occ"], OCCUPIED, FREE)
    return float((state[known] == truth[known]).sum()) / float(known.sum())
