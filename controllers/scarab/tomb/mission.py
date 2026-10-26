"""The Scarab's mission loop: sense -> map -> decide -> act -> log.

Keys (click the 3D view first in Webots):
    M        toggle manual / autonomous driving
    arrows   drive in manual mode
    S        save a map snapshot to results/logs
"""

import math
import os
import time

import numpy as np

from . import config, metrics
from .control import Wander, teleop
from .exploration import FrontierExplorer
from .mapping import OccupancyGrid
from .viz import Hud

LOG_DIR = os.path.join(metrics.REPO_ROOT, "results", "logs")
DONE_FRACTION = 0.95      # explored share of the tomb that counts as 'mapped'


class Mission:
    def __init__(self, io, world="unknown", mode="auto", strategy="utility", max_time=0.0,
                 seed=0, quiet=False, shots=False, start=None, log_dir=None):
        """start: index into the tomb's experiment start poses (None = where the robot stands)."""
        if strategy not in config.STRATEGIES:
            raise ValueError(f"strategy must be one of {config.STRATEGIES}")
        self.io, self.world, self.mode, self.strategy = io, world, mode, strategy
        self.max_time = max_time
        self.quiet = quiet
        self.shots = shots and hasattr(io, "export_view")
        self.start = start
        self.log_dir = log_dir or LOG_DIR
        self.grid = OccupancyGrid()
        self.hud = Hud(self.grid)
        self.wander = Wander(np.random.default_rng(seed))
        self.explorer = None if strategy == "wander" else FrontierExplorer(self.grid, strategy)
        self.gt = metrics.load_ground_truth(world)
        self.trail = []
        self.map_ms = []
        self.coverage = []        # (time, explored fraction, distance driven) for the plots
        self.distance = 0.0       # metres driven so far
        self.t95 = None           # first time the tomb was DONE_FRACTION explored
        self.t_finish = None      # time the robot itself decided it was done
        self.keys = set()
        self.steps = 0
        self.done = False
        self._last_hud = -1e9
        self._m_was_down = False
        self._s_was_down = False

    def log(self, msg):
        if not self.quiet:
            print(f"[scarab {self.io.time():7.1f}s] {msg}", flush=True)

    # ------------------------------------------------------------------ loop
    def run(self, quit_sim=False):
        if self.start is not None:
            pose = self.gt["starts"][self.start]
            self.io.place(*pose)
            self.log(f"start pose {self.start}: ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f} rad)")
        self.log(f"entering the {self.world} ({self.strategy}, {self.mode} mode)")
        while self.io.step():
            x, y, yaw = self.io.pose()
            if not np.isfinite(x):            # GPS is not ready on the very first step
                continue
            scan = self.io.scan()
            self.steps += 1
            t = self.io.time()

            if self.steps % config.MAP_UPDATE_EVERY == 0:
                sx = x + config.LIDAR_OFFSET_X * math.cos(yaw)
                sy = y + config.LIDAR_OFFSET_X * math.sin(yaw)
                t0 = time.perf_counter()
                self.grid.update(sx, sy, yaw, scan, self.io.scan_angles)
                self.map_ms.append((time.perf_counter() - t0) * 1000)
            step = math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) if self.trail else 0.0
            if not self.trail or step > 0.5:
                self.trail = [(x, y)]        # first fix, or the jump after being placed at a start pose
                self.distance = 0.0
            elif step > 0.05:
                self.trail.append((x, y))
                self.distance += step

            self._handle_keys()
            if self.mode == "manual":
                v, w = teleop(self.keys)
            elif self.done:
                v, w = 0.0, 0.0
            elif self.explorer is not None:
                v, w = self.explorer.step((x, y, yaw), scan, self.io.scan_angles, t, self.io.dt)
                if self.explorer.finished and not self.done:
                    self._finish(t, "no reachable frontiers left")
            else:
                v, w = self.wander.command(scan, self.io.scan_angles, self.io.dt)
            self.io.drive(v, w)

            if t - self._last_hud >= config.HUD_EVERY_S:
                self._last_hud = t
                self._update_hud((x, y, yaw), t)
            if self.shots and self.steps == 10:
                self.io.export_view(self._fig_path("view_start.jpg"))

            if self.max_time and t >= self.max_time:
                self.log("time limit reached")
                break
            if self.done and self.mode == "auto" and quit_sim:
                break
        else:
            # the simulator stopped us (window closed, world reverted): not a valid result
            path = self.save(tag="aborted")
            self.log(f"simulation ended early at {self.io.time():.1f}s; partial log saved to {path}")
            return path
        self._update_hud(self.io.pose(), self.io.time())
        self.log(self.summary())
        if self.shots:
            self.io.drive(0.0, 0.0)
            self.io.step()                    # Webots redraws the display on the next step
            self.io.export_view(self._fig_path("view_end.jpg"))
            from PIL import Image
            Image.fromarray(self.hud.render(self.io.pose(), self.trail, self._overlay())).save(
                self._fig_path("map_end.png"))
        path = self.save()
        self.log(f"log saved to {path}")
        self.io.finish(quit_sim=quit_sim)
        return path

    def _finish(self, t, why):
        self.done = True
        self.t_finish = t
        self.log(f"mission complete at {t:.0f}s: {why}")

    def path_length(self):
        return self.distance

    def summary(self):
        text = (f"strategy={self.strategy} steps={self.steps} path={self.path_length():.1f}m "
                f"known={self.grid.known_area():.1f}m2")
        if self.map_ms:
            text += f" map_update={np.mean(self.map_ms):.1f}ms"
        if self.gt is not None:
            state = self.grid.state()
            text += (f" explored={100 * metrics.explored_fraction(state, self.gt):.1f}%"
                     f" accuracy={100 * metrics.accuracy(state, self.gt):.1f}%")
        if self.t95 is not None:
            text += f" t95={self.t95:.0f}s"
        if self.t_finish is not None:
            text += f" finished={self.t_finish:.0f}s"
        return text

    def _fig_path(self, name):
        fig_dir = os.path.join(metrics.REPO_ROOT, "results", "figs")
        os.makedirs(fig_dir, exist_ok=True)
        return os.path.join(fig_dir, f"{self.io.source}_{self.world}_{self.strategy}{self._start_tag()}_{name}")

    def _start_tag(self):
        return "" if self.start is None else f"_s{self.start}"

    # -------------------------------------------------------------- helpers
    def _handle_keys(self):
        keys = self.keys = self.io.keys()
        m, s = "m" in keys, "s" in keys
        if m and not self._m_was_down:
            self.mode = "manual" if self.mode == "auto" else "auto"
            self.log(f"switched to {self.mode} mode")
        if s and not self._s_was_down:
            self.log(f"snapshot saved to {self.save(tag='snap')}")
        self._m_was_down, self._s_was_down = m, s

    def _overlay(self):
        return self.explorer.overlay() if self.explorer is not None else None

    def _update_hud(self, pose, t):
        lines = [f"SCARAB  {self.strategy.upper()}  t={t:5.0f}s"]
        if self.gt is not None:
            state = self.grid.state()
            frac = metrics.explored_fraction(state, self.gt)
            self.coverage.append((t, frac, self.distance))
            lines.append(f"tomb mapped {100 * frac:5.1f}%")
            if frac >= DONE_FRACTION and self.t95 is None:
                self.t95 = t
                self.log(f"{100 * frac:.1f}% of the tomb mapped after {t:.0f}s")
                if self.explorer is None:          # the wanderer cannot tell when it is done
                    self._finish(t, "95% mapped")
        else:
            lines.append(f"known {self.grid.known_area():6.1f} m2")
        if self.done:
            lines.append("THE TOMB IS MAPPED!")
        elif self.explorer is not None:
            lines.append(self.explorer.status)
        self.io.show(self.hud.render(pose, self.trail, self._overlay()), lines)

    def save(self, tag="run"):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        name = f"{self.world}_{self.io.source}_{self.strategy}{self._start_tag()}_{tag}_{stamp}.npz"
        path = os.path.join(self.log_dir, name)
        stats = self.explorer.stats if self.explorer is not None else {}
        np.savez_compressed(
            path,
            world=self.world,
            source=self.io.source,
            strategy=self.strategy,
            start=-1 if self.start is None else self.start,
            max_time=self.max_time,
            log_odds=self.grid.log_odds,
            map_origin=self.grid.origin,
            map_res=self.grid.res,
            trail=np.asarray(self.trail),
            coverage=np.asarray(self.coverage),
            map_ms=np.asarray(self.map_ms),
            sim_time=self.io.time(),
            t95=np.nan if self.t95 is None else self.t95,
            t_finish=np.nan if self.t_finish is None else self.t_finish,
            path_length=self.path_length(),
            collisions=getattr(self.io, "collisions", -1),   # -1: not measured (Webots)
            plans=stats.get("plans", 0),
            goals=stats.get("goals", 0),
            goals_reached=stats.get("reached", 0),
            goals_banned=stats.get("banned", 0),
            plan_ms=np.asarray(stats.get("plan_ms", [])),
            astar_ms=np.asarray(stats.get("astar_ms", [])),
            astar_expanded=np.asarray(stats.get("expanded", [])),
        )
        return path
