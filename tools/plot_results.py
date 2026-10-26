"""Figures and tables for the report from the experiment logs.

    python tools/plot_results.py --source sim2d
    python tools/plot_results.py --source webots

Reads results/experiments/<source>/logs/*.npz and writes
    figs/coverage_vs_time.png    mapped share of the tomb over time, mean and min-max band
    figs/time_to_95.png          time until 95% mapped, mean bar + one dot per start pose
    figs/distance_to_95.png      metres driven until 95% mapped
    figs/maps_<tomb>.png         final map and trail of every strategy from start pose 0
    tables/summary.md / .csv     mean +- std per tomb and strategy
    tables/runs.csv              one row per run
"""

import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

import _paths  # noqa: E402
from tomb import config, metrics  # noqa: E402
from tomb.mapping import OccupancyGrid  # noqa: E402

WORLDS = ("antechamber", "labyrinth", "treasury")
TITLES = {"antechamber": "Antechamber", "labyrinth": "Labyrinth", "treasury": "Treasury"}
STRATEGIES = ("wander", "nearest", "largest", "utility")
# fixed colour per strategy in every figure (validated categorical slots 1-3;
# the wander baseline is neutral grey); dash patterns keep them apart in print
COLOUR = {"utility": "#2a78d6", "nearest": "#eb6834", "largest": "#1baf7a", "wander": "#898781"}
DASH = {"utility": "-", "nearest": "--", "largest": "-.", "wander": ":"}
LABEL = {"utility": "Utility (IG·e^-λd)", "nearest": "Nearest frontier",
         "largest": "Largest frontier", "wander": "Reactive wander"}
INK, INK2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
PAPYRUS, STONE, WALL = np.array([232, 212, 165]), np.array([60, 58, 66]), np.array([120, 40, 20])

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 10,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 0.6,
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "legend.frameon": False, "figure.facecolor": "white", "axes.facecolor": "white",
    "savefig.dpi": 200, "savefig.bbox": "tight",
})


# ------------------------------------------------------------------ data
def first_reach(cov, level, col=0):
    hit = cov[cov[:, 1] >= level]
    return float(hit[0, col]) if len(hit) else np.nan


def load_runs(source):
    runs = []
    gts = {w: metrics.load_ground_truth(w) for w in WORLDS}
    for path in sorted(glob.glob(os.path.join(_paths.ROOT, "results", "experiments", source, "logs", "*_run_*.npz"))):
        d = np.load(path)
        world = str(d["world"])
        grid = OccupancyGrid()
        grid.log_odds[:] = d["log_odds"]
        state = grid.state()
        cov = d["coverage"]
        runs.append({
            "world": world, "strategy": str(d["strategy"]), "start": int(d["start"]),
            "max_time": float(d["max_time"]),
            "t95": first_reach(cov, 0.95), "t99": first_reach(cov, 0.99),
            "dist95": first_reach(cov, 0.95, col=2),
            "finish": float(d["t_finish"]), "distance": float(d["path_length"]),
            "explored": 100 * metrics.explored_fraction(state, gts[world]),
            "accuracy": 100 * metrics.accuracy(state, gts[world]),
            "map_ms": float(np.mean(d["map_ms"])),
            "plan_ms": float(np.mean(d["plan_ms"])) if len(d["plan_ms"]) else np.nan,
            "astar_ms": float(np.mean(d["astar_ms"])) if len(d["astar_ms"]) else np.nan,
            "collisions": int(d["collisions"]),
            "coverage": cov, "trail": d["trail"], "log_odds": d["log_odds"],
        })
    return runs


def select(runs, world, strategy):
    return [r for r in runs if r["world"] == world and r["strategy"] == strategy]


# --------------------------------------------------------------- figures
def line_handles(strategies):
    return [Line2D([0], [0], color=COLOUR[s], lw=2, ls=DASH[s], label=LABEL[s]) for s in strategies]


def bar_handles(strategies):
    return [Patch(facecolor=COLOUR[s], alpha=0.85, label=LABEL[s]) for s in strategies]


def plot_coverage(runs, out):
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True)
    for ax, world in zip(axes, WORLDS):
        here = [r for r in runs if r["world"] == world]
        ends = np.array([r["t95"] for r in here])
        limit = max(r["max_time"] for r in here) if here else 600.0
        # show up to the slowest run reaching 95%; runs that never did run to the time limit
        tmax = limit if not np.isfinite(ends).all() or not len(ends) else min(limit, ends.max() * 1.15)
        grid_t = np.arange(0, tmax + 1, 1.0)
        for s in STRATEGIES:
            rs = select(runs, world, s)
            if not rs:
                continue
            # hold the last value after a run ended: the robot has stopped and the map is final
            curves = np.array([np.interp(grid_t, r["coverage"][:, 0], 100 * r["coverage"][:, 1]) for r in rs])
            ax.fill_between(grid_t, curves.min(axis=0), curves.max(axis=0), color=COLOUR[s], alpha=0.12, lw=0)
            ax.plot(grid_t, curves.mean(axis=0), color=COLOUR[s], ls=DASH[s], lw=1.8)
        ax.axhline(95, color=MUTED, lw=0.8, ls=(0, (2, 2)))
        ax.text(tmax * 0.015, 95.8, "95%", color=MUTED, ha="left", va="bottom", fontsize=8)
        area = metrics.load_ground_truth(world)["reachable"].sum() * config.MAP_RES ** 2
        ax.set_title(f"{TITLES[world]}  ({area:.0f} m²)", loc="left", fontsize=11)
        ax.set_xlim(0, tmax)
        ax.set_ylim(0, 102)
        ax.set_xlabel("Simulated time (s)")
    axes[0].set_ylabel("Tomb mapped (%)")
    fig.legend(handles=line_handles(STRATEGIES), loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.08))
    fig.text(0.5, -0.04, "Lines: mean over start poses. Shaded: range from fastest to slowest start.",
             ha="center", color=INK2, fontsize=8.5)
    fig.savefig(out)
    plt.close(fig)


def plot_bars(runs, key, ylabel, out, unit):
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    width = 0.2
    labels = []
    for i, world in enumerate(WORLDS):
        for j, s in enumerate(STRATEGIES):
            rs = select(runs, world, s)
            if not rs:
                continue
            x = i + (j - 1.5) * width
            vals = np.array([r[key] for r in rs])
            ok = vals[np.isfinite(vals)]
            jitter = np.linspace(-width * 0.25, width * 0.25, len(vals))
            top = 0.0
            if len(ok):
                sd = ok.std() if len(ok) > 1 else 0.0
                ax.bar(x, ok.mean(), width * 0.92, color=COLOUR[s], alpha=0.85, edgecolor="white", lw=1.5,
                       yerr=sd if sd else None, error_kw={"ecolor": INK2, "lw": 1, "capsize": 2})
                ax.scatter(x + jitter[np.isfinite(vals)], ok, s=12, color=INK, zorder=3, lw=0)
                top = max(ok.mean() + sd, ok.max())
            label = f"{ok.mean():.0f}" if len(ok) else ""
            if len(ok) < len(vals):          # say how many runs the bar is based on
                label += f"\n{len(ok)}/{len(vals)} reached"
            labels.append((x, top, label))
    ymax = max(t for _, t, _ in labels) * 1.25
    for x, top, label in labels:
        ax.text(x, top + ymax * 0.015, label, ha="center", va="bottom", fontsize=7.5, color=INK2, linespacing=1.0)
    ax.set_ylim(0, ymax)
    ax.set_xticks(range(len(WORLDS)), [TITLES[w] for w in WORLDS])
    ax.tick_params(axis="x", length=0, labelsize=10.5)
    ax.set_ylabel(ylabel)
    ax.legend(handles=bar_handles(STRATEGIES), loc="upper left", ncol=2, fontsize=8.5)
    fig.text(0.5, -0.03, f"Bar: mean over start poses ({unit}). Whisker: ±1 std. Dots: individual start poses. "
             "'k/n reached': only k of n runs mapped 95%\nwithin the time limit, and the bar averages those k.",
             ha="center", color=INK2, fontsize=8.5)
    fig.savefig(out)
    plt.close(fig)


def render_map(log_odds, gt):
    p = 1 - 1 / (1 + np.exp(log_odds))
    p = p[..., None]
    img = np.where(p < 0.5, PAPYRUS + (STONE - PAPYRUS) * (p / 0.5),
                   STONE + (WALL - STONE) * ((p - 0.5) / 0.5)).astype(np.uint8)
    inside = gt["reachable"] | gt["occ"]
    rows = np.nonzero(inside.any(axis=1))[0]
    cols = np.nonzero(inside.any(axis=0))[0]
    r0, r1, c0, c1 = rows[0] - 2, rows[-1] + 3, cols[0] - 2, cols[-1] + 3
    extent = (config.MAP_ORIGIN[0] + c0 * config.MAP_RES, config.MAP_ORIGIN[0] + c1 * config.MAP_RES,
              config.MAP_ORIGIN[1] + r0 * config.MAP_RES, config.MAP_ORIGIN[1] + r1 * config.MAP_RES)
    return img[r0:r1, c0:c1], extent


def plot_maps(runs, world, out, start=0):
    gt = metrics.load_ground_truth(world)
    present = [s for s in STRATEGIES if any(r["start"] == start for r in select(runs, world, s))]
    if not present:
        return
    fig, axes = plt.subplots(1, len(present), figsize=(3.1 * len(present), 3.1), squeeze=False)
    for ax, s in zip(axes[0], present):
        r = next(r for r in select(runs, world, s) if r["start"] == start)
        img, extent = render_map(r["log_odds"], gt)
        ax.imshow(img, origin="lower", extent=extent, interpolation="nearest")
        if len(r["trail"]) > 1:
            ax.plot(r["trail"][:, 0], r["trail"][:, 1], color=COLOUR[s], lw=1.2)
        sx, sy, syaw = gt["starts"][start]
        ax.plot(sx, sy, marker="o", ms=7, color="#dfaf3c", mec=INK, mew=0.8)
        t95 = f"{r['t95']:.0f} s" if np.isfinite(r["t95"]) else "not reached"
        ax.set_title(f"{LABEL[s]}\n95% at {t95}, {r['distance']:.0f} m driven", fontsize=9, loc="left")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for sp in ax.spines.values():
            sp.set_visible(False)
    fig.suptitle(f"{TITLES[world]}: final maps from start pose {start} (gold dot)", x=0.01, ha="left",
                 fontsize=11, color=INK)
    fig.savefig(out)
    plt.close(fig)


# ---------------------------------------------------------------- tables
def ms(vals, digits=0):
    v = np.array(vals, dtype=float)
    v = v[np.isfinite(v)]
    if not len(v):
        return "-"
    return f"{v.mean():.{digits}f} ± {v.std():.{digits}f}" if len(v) > 1 else f"{v.mean():.{digits}f}"


def write_tables(runs, out_dir, source):
    os.makedirs(out_dir, exist_ok=True)
    fields = ("world", "strategy", "start", "t95", "t99", "dist95", "finish", "distance", "explored",
              "accuracy", "map_ms", "plan_ms", "astar_ms", "collisions")
    with open(os.path.join(out_dir, "runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in sorted(runs, key=lambda r: (r["world"], STRATEGIES.index(r["strategy"]), r["start"])):
            w.writerow([r[k] if isinstance(r[k], (str, int)) else f"{r[k]:.2f}" for k in fields])

    head = ("Tomb", "Strategy", "Reached 95%", "Time to 95% (s)", "Distance to 95% (m)", "Time to 99% (s)",
            "Robot stops (s)", "Explored (%)", "Accuracy (%)", "Re-plan (ms)", "A* (ms)", "Collisions")
    rows = []
    for world in WORLDS:
        for s in STRATEGIES:
            rs = select(runs, world, s)
            if not rs:
                continue
            col = lambda k: [r[k] for r in rs]   # noqa: E731
            reached = sum(np.isfinite(r["t95"]) for r in rs)
            coll = [r["collisions"] for r in rs if r["collisions"] >= 0]
            rows.append((TITLES[world], s, f"{reached}/{len(rs)}", ms(col("t95")), ms(col("dist95")),
                         ms(col("t99")), ms(col("finish")), ms(col("explored"), 1), ms(col("accuracy"), 1),
                         ms(col("plan_ms")), ms(col("astar_ms"), 1), str(sum(coll)) if coll else "n/a"))
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(rows)

    lines = [f"## Exploration results ({source}, {len({r['start'] for r in runs})} start poses per tomb)", "",
             "Values are mean ± standard deviation over start poses. \"Reached 95%\" counts runs that mapped 95% "
             "of the reachable tomb within the time limit; time and distance averages use those runs only.", "",
             "| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]

    # who was fastest from each start pose, and speed-up of the frontier strategies
    lines += ["", "### Fastest strategy to 95% from each start pose", "",
              "| Tomb | " + " | ".join(f"start {i}" for i in range(5)) +
              " | wins (utility / nearest / largest / wander) |",
              "|" + "---|" * 7]
    for world in WORLDS:
        cells, wins = [], {s: 0 for s in STRATEGIES}
        for i in range(5):
            cand = [(r["t95"], r["strategy"]) for r in runs
                    if r["world"] == world and r["start"] == i and np.isfinite(r["t95"])]
            if cand:
                t, s = min(cand)
                wins[s] += 1
                cells.append(f"{s} ({t:.0f} s)")
            else:
                cells.append("-")
        lines.append(f"| {TITLES[world]} | " + " | ".join(cells) +
                     f" | {wins['utility']} / {wins['nearest']} / {wins['largest']} / {wins['wander']} |")

    lines += ["", "### Speed-up in mean time to 95% over reactive wander", "",
              "A run that never reached 95% counts as taking the whole time limit, so a speed-up marked ≥ "
              "is a lower bound.", "",
              "| Tomb | nearest | largest | utility |", "|---|---|---|---|"]

    def censored_mean(rs):
        t = np.array([r["t95"] if np.isfinite(r["t95"]) else r["max_time"] for r in rs])
        return (t.mean(), not all(np.isfinite(r["t95"]) for r in rs)) if len(t) else (np.nan, False)

    for world in WORLDS:
        base, base_cut = censored_mean(select(runs, world, "wander"))
        cells = []
        for s in ("nearest", "largest", "utility"):
            m, cut = censored_mean(select(runs, world, s))
            if not (np.isfinite(base) and np.isfinite(m)):
                cells.append("-")
            else:
                cells.append(f"{'≥ ' if base_cut and not cut else ''}{base / m:.1f}×")
        lines.append(f"| {TITLES[world]} | " + " | ".join(cells) + " |")
    with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=("sim2d", "webots"), required=True)
    args = ap.parse_args()
    runs = load_runs(args.source)
    if not runs:
        raise SystemExit(f"no logs in results/experiments/{args.source}/logs - run tools/run_experiments.py first")
    base = os.path.join(_paths.ROOT, "results", "experiments", args.source)
    figs = os.path.join(base, "figs")
    os.makedirs(figs, exist_ok=True)
    plot_coverage(runs, os.path.join(figs, "coverage_vs_time.png"))
    plot_bars(runs, "t95", "Time to map 95% (s)", os.path.join(figs, "time_to_95.png"), "simulated seconds")
    plot_bars(runs, "dist95", "Distance driven to 95% (m)", os.path.join(figs, "distance_to_95.png"), "metres")
    for world in WORLDS:
        plot_maps(runs, world, os.path.join(figs, f"maps_{world}.png"))
    print(write_tables(runs, os.path.join(base, "tables"), args.source))
    print(f"\n{len(runs)} runs -> {os.path.relpath(base, _paths.ROOT)}/figs and /tables")


if __name__ == "__main__":
    main()
