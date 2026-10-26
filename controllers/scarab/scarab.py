"""Webots entry point for the Scarab (Pioneer 3-DX + SICK LMS 291).

controllerArgs (set in the world file or the scene tree):
    --world=<name>     tomb name, used for ground-truth scoring and log names
    --mode=auto|manual start autonomous or under keyboard control
    --strategy=<name>  wander | nearest | largest | utility (default)
    --max-time=<s>     stop after this much simulated time (0 = no limit)
    --quit             close Webots when the mission ends (batch experiments)
    --shots            save 3D-view screenshots and the final map to results/figs
    --start=<i>        begin from experiment start pose i (see ground_truth/<world>.png)
    --log-dir=<path>   where to write the run log (default results/logs)

Extra arguments can also come from the SCARAB_ARGS environment variable, so
batch scripts can change a run without editing the world file, e.g.
    SCARAB_ARGS="--max-time=300 --quit" webots --batch worlds/treasury.wbt
"""

import argparse
import os
import shlex
import sys

from tomb import config
from tomb.hal import WebotsIO
from tomb.mission import Mission


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="unknown")
    ap.add_argument("--mode", default="auto", choices=("auto", "manual"))
    ap.add_argument("--strategy", default="utility", choices=config.STRATEGIES)
    ap.add_argument("--max-time", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quit", action="store_true")
    ap.add_argument("--shots", action="store_true")
    ap.add_argument("--start", type=int, default=None, help="index of an experiment start pose")
    ap.add_argument("--log-dir", default=None)
    args = ap.parse_args(sys.argv[1:] + shlex.split(os.environ.get("SCARAB_ARGS", ""), posix=False))

    io = WebotsIO()
    Mission(io, world=args.world, mode=args.mode, strategy=args.strategy, max_time=args.max_time,
            seed=args.seed, shots=args.shots, start=args.start, log_dir=args.log_dir).run(quit_sim=args.quit)


if __name__ == "__main__":
    main()
