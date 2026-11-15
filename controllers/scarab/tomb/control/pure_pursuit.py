"""Pure-pursuit path-tracking controller for a differential-drive robot.

The controller follows a sequence of planned waypoints by selecting a
look-ahead target on the path and calculating the required steering
curvature.

The target's bearing relative to the robot's heading determines the
steering direction. Linear and angular velocity commands are generated
to guide the robot towards the target.

For large heading errors, the differential-drive robot first rotates
in place before continuing along the path.
"""
import math

import numpy as np

from .. import config

"""Normalize an angle to the interval [-pi, pi]."""
def wrap(a):
    
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    """Track a planned path using the Pure Pursuit algorithm."""
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

        """Calculate linear and angular velocity commands for path tracking.

        Selects a look-ahead target, calculates the heading error and
        determines the required steering command.

        For large heading errors, the robot rotates in place. Otherwise,
        the controller calculates forward speed and angular velocity.

        Args:
            pose: Current robot pose represented by (x, y, yaw).

        Returns:
            A tuple containing linear velocity and angular velocity.
        """
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
