"""Reactive 'follow the open space' behaviour.

This is the v0 explorer that makes the simulation come alive: it steers
towards the direction in which the robot's whole body can travel furthest,
and turns on the spot when blocked. It has no map and no goal, so it does not
guarantee full coverage; frontier exploration + A* will replace it, and it
stays as the baseline and as the safety layer.
"""

import math

import numpy as np

from .. import config

CORRIDOR_HALF = config.ROBOT_RADIUS + 0.05    # half-width of the swept corridor (m)


def scan_points(scan, angles):
    """Lidar returns as (x forward, y left) points relative to the axle centre."""
    ok = np.isfinite(scan) & (scan >= config.LIDAR_MIN_RANGE)
    r, a = scan[ok], angles[ok]
    return np.stack([config.LIDAR_OFFSET_X + r * np.cos(a), r * np.sin(a)], axis=1)


def travel_distance(points, headings, limit=config.LIDAR_RANGE_CAP):
    """How far the axle can move along each heading before the corridor hits a point."""
    d = np.stack([np.cos(headings), np.sin(headings)])          # 2 x M
    fwd = points @ d                                            # N x M
    lat = points[:, :1] * d[1] - points[:, 1:] * d[0]           # N x M (cross product)
    blocking = (fwd > 0) & (np.abs(lat) < CORRIDOR_HALF)
    return np.where(blocking, fwd, limit).min(axis=0, initial=limit)


def front_clearance(scan, angles):
    return float(travel_distance(scan_points(scan, angles), np.zeros(1))[0])


class Wander:
    def __init__(self, rng=None, bias_period=8.0):
        self.rng = rng or np.random.default_rng(0)
        self.bias = 0.0               # preferred heading offset, re-drawn now and then
        self.bias_period = bias_period
        self.bias_timer = 0.0
        self.spin = 0                 # +1 / -1 while turning on the spot, 0 otherwise
        self.headings = np.linspace(math.radians(80), math.radians(-80), 81)

    def command(self, scan, angles, dt):
        pts = scan_points(scan, angles)
        reach = travel_distance(pts, self.headings)
        front = float(reach[len(reach) // 2])

        self.bias_timer += dt
        if self.bias_timer >= self.bias_period:
            self.bias_timer = 0.0
            self.bias = self.rng.uniform(-0.9, 0.9)

        # blocked: rotate towards the more open side until the way ahead clears
        if self.spin == 0 and front < config.STOP_DIST:
            left = reach[self.headings > 0].mean()
            right = reach[self.headings < 0].mean()
            self.spin = 1 if left >= right else -1
        if self.spin != 0:
            if front > config.STOP_DIST * 1.8:
                self.spin = 0
            else:
                return 0.0, 0.9 * self.spin

        score = np.minimum(reach, 3.0) - 0.5 * np.abs(self.headings - self.bias)
        target = float(self.headings[int(np.argmax(score))])
        slow = np.clip((front - config.STOP_DIST) / (config.SLOW_DIST - config.STOP_DIST), 0.15, 1.0)
        v = config.CRUISE_SPEED * slow * max(0.2, math.cos(target))
        w = config.TURN_GAIN * target
        return v, w


def teleop(keys):
    """Arrow keys -> (v, w)."""
    v = 0.4 * ("up" in keys) - 0.25 * ("down" in keys)
    w = 1.2 * ("left" in keys) - 1.2 * ("right" in keys)
    return v, w
