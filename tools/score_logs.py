"""Score every run log against the ground truth and print a results table.

    python tools/score_logs.py                 # all logs in results/logs
    python tools/score_logs.py --source webots # only Webots runs
    python tools/score_logs.py --markdown      # table ready to paste into the report
"""

import argparse
import glob
import os

import numpy as np

import _paths
from tomb import metrics
from tomb.mapping import OccupancyGrid

COLUMNS = ("world", "source", "strategy", "t95 (s)", "t99 (s)", "finish (s)", "path (m)",
           "explored (%)", "accuracy (%)", "plan (ms)", "A* (ms)")


def first_time(coverage, level):
    if len(coverage) == 0:
        return np.nan
    hit = coverage[coverage[:, 1] >= level, 0]
    return float(hit[0]) if len(hit) else np.nan


def score(path):
    d = np.load(path)
    world = str(d["world"])
    gt = metrics.load_ground_truth(world)
    grid = OccupancyGrid()
    grid.log_odds[:] = d["log_odds"]
    state = grid.state()
    cov = d["coverage"]
    get = lambda k: d[k] if k in d.files else np.zeros(0)   # noqa: E731
    return {
        "world": world,
        "source": str(d["source"]) if "source" in d.files else "?",
        "strategy": str(d["strategy"]) if "strategy" in d.files else "wander",
        "t95 (s)": first_time(cov, 0.95),
        "t99 (s)": first_time(cov, 0.99),
        "finish (s)": float(d["t_finish"]) if "t_finish" in d.files else np.nan,
        "path (m)": float(d["path_length"]) if "path_length" in d.files else np.nan,
        "explored (%)": 100 * metrics.explored_fraction(state, gt),
        "accuracy (%)": 100 * metrics.accuracy(state, gt),
        "plan (ms)": float(np.mean(get("plan_ms"))) if len(get("plan_ms")) else np.nan,
        "A* (ms)": float(np.mean(get("astar_ms"))) if len(get("astar_ms")) else np.nan,
    }


def fmt(v):
    if isinstance(v, str):
        return v
    return "-" if np.isnan(v) else f"{v:.0f}" if abs(v) >= 100 else f"{v:.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", help="sim2d or webots")
    ap.add_argument("--markdown", action="store_true")
    args = ap.parse_args()
    rows = [score(p) for p in sorted(glob.glob(os.path.join(_paths.ROOT, "results", "logs", "*_run_*.npz")))]
    if args.source:
        rows = [r for r in rows if r["source"] == args.source]
    order = {"wander": 0, "nearest": 1, "largest": 2, "utility": 3}
    rows.sort(key=lambda r: (r["world"], r["source"], order.get(r["strategy"], 9)))
    if args.markdown:
        print("| " + " | ".join(COLUMNS) + " |")
        print("|" + "---|" * len(COLUMNS))
        for r in rows:
            print("| " + " | ".join(fmt(r[c]) for c in COLUMNS) + " |")
    else:
        print("  ".join(f"{c:>12s}" for c in COLUMNS))
        for r in rows:
            print("  ".join(f"{fmt(r[c]):>12s}" for c in COLUMNS))


if __name__ == "__main__":
    main()
