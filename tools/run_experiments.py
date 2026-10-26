"""Run the exploration experiments: every tomb x strategy x start pose.

    python tools/run_experiments.py --source sim2d            # 60 runs, in parallel, a few minutes
    python tools/run_experiments.py --source webots           # 60 runs in real Webots, one at a time
    python tools/run_experiments.py --source sim2d --worlds treasury --strategies utility,nearest --starts 0,1

Logs go to results/experiments/<source>/logs/. Runs that already have a log
are skipped, so an interrupted batch can simply be started again (use
--force to redo them). Afterwards, make the figures and tables with
    python tools/plot_results.py --source <source>
"""

import argparse
import glob
import itertools
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import _paths
from tomb import config

WORLDS = ("antechamber", "labyrinth", "treasury")
N_STARTS = 5


def log_dir(source):
    return os.path.join(_paths.ROOT, "results", "experiments", source, "logs")


def done(source, world, strategy, start):
    pattern = f"{world}_{source}_{strategy}_s{start}_run_*.npz"
    return bool(glob.glob(os.path.join(log_dir(source), pattern)))


# ------------------------------------------------------------------ sim2d
def _sim2d_job(world, strategy, start, sim_time):
    import sim2d
    t0 = time.time()
    sim2d.run(world, strategy, sim_time, seed=start, quiet=True, start=start,
              log_dir=log_dir("sim2d"), figure=False)
    return time.time() - t0


def run_sim2d(jobs, sim_time, workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(_sim2d_job, *job, sim_time): job for job in jobs}
        for k, fut in enumerate(as_completed(futures), start=1):
            world, strategy, start = futures[fut]
            print(f"[{k:3d}/{len(jobs)}] {world} {strategy} start {start}: {fut.result():.0f}s wall", flush=True)


# ----------------------------------------------------------------- webots
def find_webots():
    candidates = [
        os.environ.get("WEBOTS_HOME") and os.path.join(os.environ["WEBOTS_HOME"], "msys64", "mingw64", "bin", "webots.exe"),
        os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Webots", "msys64", "mingw64", "bin", "webots.exe"),
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        shutil.which("webots"),
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    raise SystemExit("Webots not found: set WEBOTS_HOME or put webots on the PATH")


def run_webots(jobs, sim_time, wall_timeout):
    exe = find_webots()
    for k, (world, strategy, start) in enumerate(jobs, start=1):
        env = dict(os.environ)
        env["SCARAB_ARGS"] = (f"--strategy={strategy} --start={start} --max-time={sim_time} "
                              f"--quit --log-dir={log_dir('webots')}")
        world_file = os.path.join(_paths.ROOT, "worlds", f"{world}.wbt")
        t0 = time.time()
        proc = subprocess.Popen([exe, "--batch", "--mode=fast", "--stdout", "--stderr", world_file],
                                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            proc.wait(timeout=wall_timeout)
            status = "ok" if done("webots", world, strategy, start) else "NO LOG"
        except subprocess.TimeoutExpired:
            proc.kill()
            if sys.platform == "win32":   # Webots keeps a child process; make sure it is gone
                subprocess.run(["taskkill", "/F", "/IM", "webots-bin.exe"], capture_output=True)
            status = "TIMEOUT"
        print(f"[{k:3d}/{len(jobs)}] {world} {strategy} start {start}: {status} "
              f"({time.time() - t0:.0f}s wall)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=("sim2d", "webots"), required=True)
    ap.add_argument("--worlds", default="all")
    ap.add_argument("--strategies", default="all")
    ap.add_argument("--starts", default="all", help="comma list of start indices, or all")
    ap.add_argument("--time", type=float, default=600.0, help="simulated seconds per run")
    ap.add_argument("--jobs", type=int, default=max(1, min(6, (os.cpu_count() or 2) - 1)),
                    help="parallel sim2d runs")
    ap.add_argument("--wall-timeout", type=float, default=480.0, help="seconds before a Webots run is killed")
    ap.add_argument("--force", action="store_true", help="redo runs that already have a log")
    args = ap.parse_args()

    worlds = WORLDS if args.worlds == "all" else tuple(args.worlds.split(","))
    strategies = config.STRATEGIES if args.strategies == "all" else tuple(args.strategies.split(","))
    starts = range(N_STARTS) if args.starts == "all" else [int(s) for s in args.starts.split(",")]
    jobs = [j for j in itertools.product(worlds, strategies, starts)
            if args.force or not done(args.source, *j)]
    os.makedirs(log_dir(args.source), exist_ok=True)
    print(f"{len(jobs)} runs to do ({args.source}, {args.time:.0f} s each)", flush=True)
    t0 = time.time()
    if args.source == "sim2d":
        run_sim2d(jobs, args.time, args.jobs)
    else:
        run_webots(jobs, args.time, args.wall_timeout)
    print(f"finished in {(time.time() - t0) / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
