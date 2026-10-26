"""Fast 2D stand-in for Webots, built on the ground-truth tombs.

It implements the same RobotIO interface as WebotsIO, so the real mission code
runs unchanged: differential-drive kinematics, a 180-ray lidar cast against
the 1 cm ground-truth grid, and collision detection with the robot footprint.
Use it to develop modules and to run many experiments quickly; confirm the
final results in Webots.

    python tools/sim2d.py antechamber --time 300
    python tools/sim2d.py all --time 600
"""

import argparse
import math
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

import _paths
from tomb import config, metrics
from tomb.hal import RobotIO
from tomb.mission import Mission

FIG_DIR = os.path.join(_paths.ROOT, "results", "figs")


class SimIO(RobotIO):
    source = "sim2d"

    def __init__(self, world, range_noise=0.0, seed=0):
        gt = metrics.load_ground_truth(world)
        if gt is None:
            raise SystemExit(f"no ground truth for '{world}': run tools/world_gen.py first")
        self.res = float(gt["fine_res"])
        self.origin = gt["fine_origin"]
        # distance from every cell to the nearest obstacle: used for both
        # sphere-traced lidar rays and footprint collision checks
        self.clearance = ndimage.distance_transform_edt(~gt["fine_occ"]) * self.res
        self.x, self.y, self.yaw = (float(v) for v in gt["start"])
        self.t = 0.0
        self.wheels = (0.0, 0.0)
        self.enc = [0.0, 0.0]
        self.collisions = 0
        self._touching = False
        self.range_noise = range_noise
        self.rng = np.random.default_rng(seed)
        self.last_image = None
        self.last_lines = ()

    # ------------------------------------------------------------- geometry
    def _clear(self, x, y):
        i = np.clip(((np.asarray(x) - self.origin[0]) / self.res).astype(int), 0, self.clearance.shape[1] - 1)
        j = np.clip(((np.asarray(y) - self.origin[1]) / self.res).astype(int), 0, self.clearance.shape[0] - 1)
        return self.clearance[j, i]

    # ------------------------------------------------------------- RobotIO
    def step(self):
        wl, wr = self.wheels
        v = config.WHEEL_RADIUS * (wl + wr) / 2
        w = config.WHEEL_RADIUS * (wr - wl) / config.AXLE_LENGTH
        yaw = self.yaw + w * self.dt
        x = self.x + v * math.cos(self.yaw + w * self.dt / 2) * self.dt
        y = self.y + v * math.sin(self.yaw + w * self.dt / 2) * self.dt
        if self._clear(x, y) >= config.ROBOT_RADIUS:
            self.x, self.y = x, y
            self._touching = False
        elif not self._touching:          # count each new contact once
            self.collisions += 1
            self._touching = True
        self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.enc[0] += wl * self.dt
        self.enc[1] += wr * self.dt
        self.t += self.dt
        return True

    def time(self):
        return self.t

    def pose(self):
        return self.x, self.y, self.yaw

    def scan(self):
        sx = self.x + config.LIDAR_OFFSET_X * math.cos(self.yaw)
        sy = self.y + config.LIDAR_OFFSET_X * math.sin(self.yaw)
        th = self.yaw + self.scan_angles
        c, s = np.cos(th), np.sin(th)
        far = config.LIDAR_MAX_RANGE
        t = np.zeros_like(th)
        for _ in range(200):
            d = self._clear(sx + c * t, sy + s * t)
            active = (d > self.res) & (t < far)
            if not active.any():
                break
            t[active] += np.maximum(d[active] - self.res / 2, self.res / 2)
        if self.range_noise:
            t += self.rng.normal(0, self.range_noise, t.shape)
        return np.where(t >= far, np.inf, t)

    def wheel_angles(self):
        return tuple(self.enc)

    def set_wheel_speeds(self, left, right):
        self.wheels = (left, right)

    def place(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    def show(self, rgb, lines=()):
        self.last_image, self.last_lines = rgb, tuple(lines)


def save_figure(io, path, scale=2):
    img = Image.fromarray(io.last_image).resize(
        (io.last_image.shape[1] * scale, io.last_image.shape[0] * scale), Image.NEAREST)
    d = ImageDraw.Draw(img)
    for i, line in enumerate(io.last_lines):
        d.text((6, 6 + 14 * i), line, fill=(240, 216, 144))
    img.save(path)


def run(world, strategy, sim_time, seed, quiet, start=None, log_dir=None, figure=True):
    io = SimIO(world, seed=seed)
    mission = Mission(io, world=world, mode="auto", strategy=strategy, max_time=sim_time,
                      seed=seed, quiet=quiet, start=start, log_dir=log_dir)
    log_path = mission.run(quit_sim=True)
    if figure:
        os.makedirs(FIG_DIR, exist_ok=True)
        tag = "" if start is None else f"_s{start}"
        save_figure(io, os.path.join(FIG_DIR, f"sim2d_{world}_{strategy}{tag}.png"))

    state = mission.grid.state()
    plan_ms = mission.explorer.stats["plan_ms"] if mission.explorer else []
    fmt = lambda t: f"{t:5.0f}s" if t is not None else "    -"   # noqa: E731
    print(f"{world:12s} {strategy:8s} start={'-' if start is None else start} "
          f"t95={fmt(mission.t95)} finish={fmt(mission.t_finish)} "
          f"path={mission.path_length():5.1f}m "
          f"explored={100 * metrics.explored_fraction(state, mission.gt):5.1f}% "
          f"accuracy={100 * metrics.accuracy(state, mission.gt):5.1f}% "
          f"collisions={io.collisions} "
          f"plan={np.mean(plan_ms) if len(plan_ms) else 0:5.1f}ms", flush=True)
    return log_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world", help="antechamber | labyrinth | treasury | all")
    ap.add_argument("--strategy", default="utility", help="wander | nearest | largest | utility | all")
    ap.add_argument("--time", type=float, default=600.0, help="simulated seconds")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--start", type=int, default=None, help="index of an experiment start pose")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    worlds = ("antechamber", "labyrinth", "treasury") if args.world == "all" else (args.world,)
    strategies = config.STRATEGIES if args.strategy == "all" else (args.strategy,)
    for w in worlds:
        for s in strategies:
            run(w, s, args.time, args.seed, quiet=not args.verbose, start=args.start)


if __name__ == "__main__":
    main()
