"""Pure-pursuit path tracking for the differential drive (Coulter, 1992).

The robot steers along the arc that passes through a look-ahead point on the
path: curvature k = 2 sin(alpha) / L_d, where alpha is the bearing of that
point and L_d its distance. Being a differential drive, the Pioneer turns on
the spot first when the path points far away from its heading.
"""

import math

import numpy as np

from .. import config


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    def __init__(self, lookahead=config.LOOKAHEAD):
        self.lookahead = lookahead
        self.path = None
        self.idx = 0

    def set_path(self, points):
        self.path = np.asarray(points, dtype=float)
        self.idx = 0

    def remaining(self):
        """Distance left along the path from the closest point (m)."""
        if self.path is None:
            return 0.0
        seg = np.diff(self.path[self.idx:], axis=0)
        return float(np.hypot(seg[:, 0], seg[:, 1]).sum())

    def command(self, pose):
        x, y, yaw = pose
        pts = self.path
        d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
        # progress only forwards along the path, within a short search window
        window = d[self.idx:self.idx + 40]
        self.idx += int(np.argmin(window))
        ahead = np.nonzero(d[self.idx:] >= self.lookahead)[0]
        j = self.idx + int(ahead[0]) if len(ahead) else len(pts) - 1
        tx, ty = pts[j]
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(alpha) > config.TURN_IN_PLACE:
            return 0.0, math.copysign(config.TURN_RATE, alpha)
        ld = max(math.hypot(tx - x, ty - y), 0.05)
        to_goal = d[-1]
        v = config.CRUISE_SPEED * min(1.0, 0.3 + to_goal / 0.8)
        w = float(np.clip(v * 2.0 * math.sin(alpha) / ld, -config.MAX_TURN_RATE, config.MAX_TURN_RATE))
        return v, w
