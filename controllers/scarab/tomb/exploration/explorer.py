"""Frontier exploration loop.

Every REPLAN_EVERY_S seconds:
    1. build the C-space costmap from the current map,
    2. compute path distances from the robot (wavefront),
    3. detect and cluster frontiers, give each a reachable target,
    4. pick a goal with the selected strategy (+ hysteresis, blacklist),
    5. plan an A* path to it (falling back to the next candidates).
Every control step:
    follow the path with pure pursuit, stop if the safety check sees an
    obstacle ahead, and give up goals that are blocked or late.
On reaching a goal the robot turns to face its frontier: the SICK only sees
the half-plane in front, so arriving is not the same as looking. A spot is
blacklisted only when it is reached a second time and is still a frontier.
Exploration finishes when no reachable frontier is left.
"""

import math
import time

import numpy as np

from .. import config
from ..control.pure_pursuit import PurePursuit, wrap
from ..control.wander import front_clearance

FACE_TOLERANCE = math.radians(10)
FACE_TIMEOUT = 4.0        # s allowed for turning towards a reached frontier
from ..planning import Costmap, astar
from .frontiers import detect_frontiers
from .selection import FrontierSelector, attach_targets


class FrontierExplorer:
    def __init__(self, grid, strategy):
        self.grid = grid
        self.selector = FrontierSelector(strategy)
        self.follower = PurePursuit()
        self.goal_xy = None
        self.goal_face = None     # centroid of the goal's frontier, to look at on arrival
        self.facing = None        # (x, y) the robot is turning towards, or None
        self.facing_until = 0.0
        self.visits = []          # goals already reached once
        self.goal_since = 0.0     # when the current goal was first chosen
        self.goal_deadline = math.inf
        self.next_plan = 0.0
        self.blocked = 0.0
        self.empty_plans = 0
        self.finished = False
        self.frontiers = []
        self.status = "starting"
        self.stats = {"plans": 0, "goals": 0, "reached": 0, "banned": 0,
                      "plan_ms": [], "astar_ms": [], "expanded": []}

    # ------------------------------------------------------------- control
    def step(self, pose, scan, angles, t, dt):
        if self.finished:
            return 0.0, 0.0
        x, y, yaw = pose
        if self.facing is not None:
            err = wrap(math.atan2(self.facing[1] - y, self.facing[0] - x) - yaw)
            if abs(err) > FACE_TOLERANCE and t < self.facing_until:
                return 0.0, math.copysign(config.TURN_RATE, err)
            self.facing = None
            self.next_plan = t
        if self.goal_xy is not None and math.hypot(x - self.goal_xy[0], y - self.goal_xy[1]) < config.GOAL_REACHED_DIST:
            self.stats["reached"] += 1
            if any(math.hypot(self.goal_xy[0] - vx, self.goal_xy[1] - vy) < config.BLACKLIST_RADIUS
                   for vx, vy in self.visits):
                self.selector.ban(self.goal_xy)      # second arrival and still a frontier: give up
                self.stats["banned"] += 1
            self.visits.append(self.goal_xy)
            face = self.goal_face
            self._drop_goal("looking around", t)
            if face is not None and math.hypot(face[0] - x, face[1] - y) > 0.1:
                self.facing, self.facing_until = face, t + FACE_TIMEOUT
                return 0.0, 0.0
        if self.goal_xy is not None and t > self.goal_deadline:
            self._give_up("goal timed out", t)
        if t >= self.next_plan:
            self._plan(pose, t)
            self.next_plan = t + config.REPLAN_EVERY_S
        if self.follower.path is None:
            # no path yet (e.g. the very first scans): look around on the spot
            return 0.0, (0.0 if self.finished else 0.6 * config.TURN_RATE)

        v, w = self.follower.command(pose)
        if v > 0 and front_clearance(scan, angles) < config.STOP_DIST:
            v = 0.0                                   # safety layer: never drive into something
            self.blocked += dt
            if self.blocked > config.BLOCKED_TIMEOUT:
                self._give_up("blocked", t)
        else:
            self.blocked = max(0.0, self.blocked - dt)
        return v, w

    def _drop_goal(self, why, t):
        self.goal_xy = None
        self.goal_deadline = math.inf
        self.follower.path = None
        self.blocked = 0.0
        self.status = why
        self.next_plan = t                            # choose a new goal right away

    def _give_up(self, why, t):
        self.stats["banned"] += 1
        self.selector.ban(self.goal_xy)
        self._drop_goal(why, t)

    # ------------------------------------------------------------ planning
    def _plan(self, pose, t):
        t0 = time.perf_counter()
        self.stats["plans"] += 1
        grid = self.grid
        state = grid.state()
        costmap = Costmap(state)
        col, row = grid.to_cell(pose[0], pose[1])
        start = costmap.nearest_traversable(int(row), int(col))
        if start is None:
            self.follower.path = None
            self.status = "no safe start cell"
            return
        distance = costmap.wavefront(start)
        frontiers = detect_frontiers(state, grid)
        attach_targets(frontiers, distance, grid)
        self.frontiers = frontiers

        goal, ranked = self.selector.choose(frontiers, state, grid, current_xy=self.goal_xy,
                                            may_switch=t - self.goal_since >= config.COMMIT_S)
        if goal is None:
            self.empty_plans += 1
            self._drop_goal("no reachable frontier", t)
            self.next_plan = t + config.REPLAN_EVERY_S
            if self.empty_plans >= 2:
                self.finished = True
                self.status = "exploration complete"
            return
        self.empty_plans = 0

        for f in ranked[:4]:
            ta = time.perf_counter()
            cells, expanded = astar(costmap, start, f.target)
            self.stats["astar_ms"].append((time.perf_counter() - ta) * 1000)
            self.stats["expanded"].append(expanded)
            if cells is None:
                self.selector.ban(f.target_xy)
                self.stats["banned"] += 1
                continue
            rows, cols = np.array(cells).T
            px, py = grid.to_world(cols, rows)
            self.follower.set_path(np.stack([px, py], axis=1))
            new_goal = (self.goal_xy is None or
                        math.hypot(f.target_xy[0] - self.goal_xy[0], f.target_xy[1] - self.goal_xy[1])
                        >= config.GOAL_TRACK_RADIUS)
            if new_goal:
                self.stats["goals"] += 1
                self.goal_since = t
                length = self.follower.remaining()
                self.goal_deadline = t + length / (0.5 * config.CRUISE_SPEED) + config.GOAL_TIMEOUT_PAD
            self.goal_xy = f.target_xy
            self.goal_face = f.centroid
            self.status = f"{self.selector.strategy}: {len(ranked)} frontiers"
            break
        else:
            self._drop_goal("no path to any frontier", t)
            self.next_plan = t + config.REPLAN_EVERY_S
        self.stats["plan_ms"].append((time.perf_counter() - t0) * 1000)

    # ------------------------------------------------------------- display
    def overlay(self):
        rows = [f.rows for f in self.frontiers]
        cols = [f.cols for f in self.frontiers]
        return {
            "frontier_rows": np.concatenate(rows) if rows else np.zeros(0, int),
            "frontier_cols": np.concatenate(cols) if cols else np.zeros(0, int),
            "path": self.follower.path,
            "goal": self.goal_xy,
            "blacklist": list(self.selector.blacklist),
        }
