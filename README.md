# Scarab: Mapping the Pharaoh's Tomb

**CSAI4014 AISR, Project P12: Occupancy-Grid Mapping and Frontier Exploration**
Webots R2025a · Pioneer 3-DX + SICK LMS 291 lidar · Python 3.11 + NumPy

A gold-shelled Pioneer 3-DX nicknamed **Scarab** is sent into a sealed Egyptian
tomb that no one has mapped. Using only its lidar, it builds a log-odds occupancy grid
(drawn as a papyrus map on its display) and explores until the whole tomb is mapped.

| World | Story | Size | Explorable area |
|---|---|---|---|
| `antechamber` | Entrance hall: pillars, altar, obsidian guardians at the sealed door | 8 × 6 m | 44.5 m² |
| `labyrinth` | 6 × 6 maze of narrow passages; urns in the dead ends, golden death mask in the middle | 9.6 × 9.6 m | 83.8 m² |
| `treasury` | Crowded hall of sarcophagi and chests, plus the vault behind a narrow doorway | 12 × 10 m | 105.9 m² |

## Quick start

1. Install **Webots R2025a** (`webots-R2025a_setup.exe` from the cyberbotics/webots GitHub releases) and Python 3.11 with `pip install -r requirements.txt`.
2. Open `worlds/antechamber.wbt` in Webots and press play.
3. The Scarab starts exploring. The map display shows its live map and the share of the tomb that is mapped.

Keys (click the 3D view first): **M** toggles manual / autonomous, **arrow keys** drive, **S** saves a map snapshot.

Notes from testing on Windows:
- The first time a world opens, Webots downloads the Pioneer, lidar, sand and background models
  from GitHub (the worlds use full `https://raw.githubusercontent.com/...R2025a` links, because some
  installs ship without local model files). If the clock stays at 0:00, press play once. After that
  the models are cached.
- Some texture downloads can fail with "error code 399" over VPNs such as Cloudflare WARP. They are
  only warnings: the robot just shows without those textures.
- Webots must use the Python that has NumPy installed. If it can't find it, set
  *Tools → Preferences → Python command* to that interpreter.

Batch runs (no clicking; Webots closes when the mission ends):

```bash
SCARAB_ARGS="--strategy=nearest --max-time=600 --quit --shots" webots --batch --mode=fast worlds/treasury.wbt
```

`--shots` saves `results/figs/webots_<world>_<strategy>_view_start.jpg`, `..._view_end.jpg` and `..._map_end.png`.
Score every saved run with `python tools/score_logs.py` (add `--markdown` for a report-ready table).

## Without Webots: the 2D test simulator

`tools/sim2d.py` runs the same mission code against the ground-truth tomb, with the same kinematics, a
180-ray lidar and collision checks. It runs about 15× faster than real time and needs no GUI, so module
development and quick checks don't need Webots at all.

```bash
python tools/sim2d.py treasury --strategy utility --start 2 --verbose
```

Single runs write their map figure to `results/figs/` and their log (`.npz`: map, trail, coverage and
distance over time, timings) to `results/logs/`.

## Experiments

Every tomb is explored from 5 start poses (numbered on `ground_truth/<tomb>.png`): pose 0 is the world's own
start, and the other four are spread over the tomb by farthest-point sampling, with headings from a
fixed seed. Each run lasts at most 600 simulated seconds.

```bash
python tools/run_experiments.py --source sim2d     # 60 runs in parallel, ~5 min
python tools/run_experiments.py --source webots    # 60 runs in real Webots, ~45 min, resumable
python tools/plot_results.py --source sim2d        # figures + tables
python tools/plot_results.py --source webots
```

Outputs, in `results/experiments/<source>/`:

| File | Use in the report |
|---|---|
| `figs/coverage_vs_time.png` | main result: % of tomb mapped over time, mean and range over start poses |
| `figs/time_to_95.png` | time to map 95%, bar = mean, dots = individual starts |
| `figs/distance_to_95.png` | metres driven to map 95% (efficiency) |
| `figs/maps_<tomb>.png` | final map and route of each strategy from start pose 0 |
| `tables/summary.md` / `.csv` | the results table below, plus wins per start pose and speed-up over wander |
| `tables/runs.csv` | every run, for your own analysis |

### Results (sim2d, 5 start poses per tomb)

Values are mean ± standard deviation over start poses. "Reached 95%" counts runs that mapped 95% of the reachable tomb within the time limit; time and distance averages use those runs only.

| Tomb | Strategy | Reached 95% | Time to 95% (s) | Distance to 95% (m) | Time to 99% (s) | Robot stops (s) | Explored (%) | Accuracy (%) | Re-plan (ms) | A* (ms) | Collisions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Antechamber | wander | 5/5 | 36 ± 9 | 12 ± 3 | - | 36 ± 9 | 95.2 ± 0.2 | 96.6 ± 0.2 | - | - | 0 |
| Antechamber | nearest | 5/5 | 33 ± 6 | 9 ± 1 | 49 ± 9 | 52 ± 11 | 99.0 ± 0.4 | 96.5 ± 0.1 | 39 ± 3 | 8.1 ± 1.1 | 0 |
| Antechamber | largest | 5/5 | 30 ± 5 | 8 ± 1 | 55 ± 5 | 64 ± 10 | 99.1 ± 0.2 | 96.5 ± 0.1 | 42 ± 2 | 11.5 ± 1.9 | 0 |
| Antechamber | utility | 5/5 | 25 ± 1 | 6 ± 0 | 53 ± 9 | 55 ± 8 | 99.0 ± 0.2 | 96.5 ± 0.2 | 43 ± 5 | 10.8 ± 3.5 | 0 |
| Labyrinth | wander | 1/5 | 344 | 105 | - | 344 | 84.6 ± 7.5 | 94.7 ± 0.1 | - | - | 0 |
| Labyrinth | nearest | 5/5 | 167 ± 33 | 52 ± 10 | 173 ± 31 | 187 ± 30 | 99.8 ± 0.1 | 94.4 ± 0.1 | 56 ± 7 | 17.3 ± 3.0 | 0 |
| Labyrinth | largest | 5/5 | 195 ± 42 | 62 ± 14 | 232 ± 43 | 297 ± 30 | 99.8 ± 0.0 | 94.6 ± 0.1 | 67 ± 5 | 25.8 ± 1.9 | 0 |
| Labyrinth | utility | 5/5 | 156 ± 20 | 49 ± 7 | 180 ± 2 | 242 ± 16 | 99.8 ± 0.0 | 94.6 ± 0.1 | 75 ± 10 | 26.9 ± 6.7 | 0 |
| Treasury | wander | 5/5 | 237 ± 112 | 74 ± 36 | - | 237 ± 112 | 95.1 ± 0.1 | 96.8 ± 0.1 | - | - | 0 |
| Treasury | nearest | 5/5 | 118 ± 18 | 36 ± 5 | 165 ± 19 | 188 ± 25 | 99.7 ± 0.1 | 96.5 ± 0.1 | 66 ± 7 | 16.3 ± 3.9 | 0 |
| Treasury | largest | 5/5 | 139 ± 42 | 42 ± 13 | 230 ± 25 | 310 ± 38 | 99.7 ± 0.1 | 96.6 ± 0.0 | 89 ± 4 | 34.5 ± 2.8 | 0 |
| Treasury | utility | 5/5 | 88 ± 9 | 27 ± 3 | 140 ± 16 | 207 ± 24 | 99.6 ± 0.2 | 96.6 ± 0.1 | 83 ± 4 | 26.9 ± 4.3 | 0 |

#### Fastest strategy to 95% from each start pose

| Tomb | start 0 | start 1 | start 2 | start 3 | start 4 | wins (utility / nearest / largest / wander) |
|---|---|---|---|---|---|---|
| Antechamber | utility (24 s) | wander (24 s) | utility (26 s) | utility (23 s) | largest (24 s) | 3 / 0 / 1 / 1 |
| Labyrinth | largest (129 s) | utility (168 s) | utility (122 s) | utility (142 s) | nearest (158 s) | 3 / 1 / 1 / 0 |
| Treasury | largest (94 s) | largest (95 s) | utility (76 s) | utility (78 s) | utility (94 s) | 3 / 0 / 2 / 0 |

#### Speed-up in mean time to 95% over reactive wander

A run that never reached 95% counts as taking the whole time limit, so a speed-up marked ≥ is a lower bound.

| Tomb | nearest | largest | utility |
|---|---|---|---|
| Antechamber | 1.1× | 1.2× | 1.5× |
| Labyrinth | ≥ 3.3× | ≥ 2.8× | ≥ 3.5× |
| Treasury | 2.0× | 1.7× | 2.7× |

What the 2D results say:
- **Utility is the best strategy overall.** It has the lowest mean time *and* the shortest distance to 95% in all
  three tombs, the smallest spread between start poses in the antechamber and treasury, and it is fastest from
  9 of the 15 start poses.
- **Every frontier strategy beats reactive wandering**, by 2.0–2.7× in the treasury. In the labyrinth the
  wanderer mapped 95% from only 1 of 5 starts within 600 s.
- **Largest frontier is the least efficient frontier strategy**: it keeps crossing the tomb towards whichever
  cluster is currently biggest (the commitment rule limits this, but cannot remove it).
- Re-planning takes 40–90 ms in Python (once every 1.5 s) and the map update about 1 ms (every 64 ms), so
  everything runs in real time. No run collided.

### Results (webots, 5 start poses per tomb)

Values are mean ± standard deviation over start poses. "Reached 95%" counts runs that mapped 95% of the reachable tomb within the time limit; time and distance averages use those runs only.

| Tomb | Strategy | Reached 95% | Time to 95% (s) | Distance to 95% (m) | Time to 99% (s) | Robot stops (s) | Explored (%) | Accuracy (%) | Re-plan (ms) | A* (ms) | Collisions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Antechamber | wander | 5/5 | 28 ± 5 | 9 ± 2 | - | 28 ± 5 | 95.2 ± 0.1 | 98.4 ± 0.2 | - | - | n/a |
| Antechamber | nearest | 5/5 | 37 ± 3 | 9 ± 1 | 50 ± 2 | 54 ± 5 | 99.1 ± 0.3 | 98.3 ± 0.2 | 25 ± 2 | 4.5 ± 0.8 | n/a |
| Antechamber | largest | 5/5 | 27 ± 4 | 7 ± 1 | 54 ± 8 | 69 ± 6 | 99.3 ± 0.2 | 98.0 ± 0.2 | 25 ± 2 | 7.0 ± 1.4 | n/a |
| Antechamber | utility | 5/5 | 22 ± 2 | 6 ± 1 | 46 ± 7 | 65 ± 15 | 99.2 ± 0.6 | 98.1 ± 0.2 | 25 ± 2 | 7.2 ± 1.9 | n/a |
| Labyrinth | wander | 3/5 | 356 ± 126 | 106 ± 38 | - | 356 ± 126 | 83.4 ± 20.2 | 97.5 ± 0.2 | - | - | n/a |
| Labyrinth | nearest | 5/5 | 132 ± 14 | 41 ± 5 | 135 ± 16 | 139 ± 16 | 99.6 ± 0.1 | 98.0 ± 0.2 | 30 ± 4 | 6.7 ± 2.4 | n/a |
| Labyrinth | largest | 5/5 | 148 ± 28 | 48 ± 9 | 162 ± 24 | 165 ± 25 | 99.6 ± 0.0 | 98.0 ± 0.1 | 31 ± 4 | 7.7 ± 3.2 | n/a |
| Labyrinth | utility | 5/5 | 148 ± 11 | 48 ± 4 | 159 ± 13 | 172 ± 11 | 99.6 ± 0.0 | 97.9 ± 0.2 | 38 ± 3 | 11.7 ± 2.1 | n/a |
| Treasury | wander | 5/5 | 214 ± 162 | 65 ± 45 | - | 214 ± 162 | 95.1 ± 0.1 | 98.0 ± 0.0 | - | - | n/a |
| Treasury | nearest | 5/5 | 139 ± 27 | 41 ± 7 | 168 ± 23 | 176 ± 25 | 99.5 ± 0.1 | 98.1 ± 0.1 | 42 ± 6 | 7.5 ± 1.1 | n/a |
| Treasury | largest | 5/5 | 112 ± 12 | 33 ± 4 | 205 ± 36 | 265 ± 31 | 99.5 ± 0.1 | 97.9 ± 0.2 | 51 ± 4 | 16.7 ± 3.4 | n/a |
| Treasury | utility | 5/5 | 90 ± 17 | 28 ± 6 | 145 ± 11 | 189 ± 23 | 99.6 ± 0.1 | 98.0 ± 0.1 | 53 ± 8 | 14.2 ± 3.5 | n/a |

#### Fastest strategy to 95% from each start pose

| Tomb | start 0 | start 1 | start 2 | start 3 | start 4 | wins (utility / nearest / largest / wander) |
|---|---|---|---|---|---|---|
| Antechamber | utility (23 s) | wander (21 s) | utility (19 s) | utility (20 s) | largest (25 s) | 3 / 0 / 1 / 1 |
| Labyrinth | nearest (120 s) | utility (139 s) | largest (112 s) | largest (122 s) | utility (133 s) | 2 / 1 / 2 / 0 |
| Treasury | utility (89 s) | utility (88 s) | largest (103 s) | utility (79 s) | utility (72 s) | 4 / 0 / 1 / 0 |

#### Speed-up in mean time to 95% over reactive wander

A run that never reached 95% counts as taking the whole time limit, so a speed-up marked ≥ is a lower bound.

| Tomb | nearest | largest | utility |
|---|---|---|---|
| Antechamber | 0.7× | 1.0× | 1.2× |
| Labyrinth | ≥ 3.4× | ≥ 3.1× | ≥ 3.1× |
| Treasury | 1.5× | 1.9× | 2.4× |

Webots does not report collisions, so that column reads n/a. In the 2D simulator no run collided.

What the Webots results say, and how they compare with the 2D simulator:
- **Utility again wins most start poses (9 of 15).** It is clearly fastest in the open tombs: in the
  treasury 90 s against 112–139 s, and 2.4× faster than wandering. In the antechamber, 22 s against 27–37 s.
- **In the maze the frontier strategies are statistically tied.** Nearest averages 132 ± 14 s, against
  148 s for largest and utility. In a maze, finishing the corridor you are in matters more than how much a
  frontier promises. The 2D simulator had utility slightly ahead here, so treat the labyrinth ranking as
  "no clear winner".
- **Wandering is competitive only in the small open antechamber**, where it beats nearest (28 s vs 37 s). It
  fails in the maze, mapping 95% from only 3 of 5 starts within 600 s, and is unpredictable in the treasury
  (214 ± 162 s).
- **Webots maps are more accurate (about 98%) than 2D maps (about 96%)**. The likely reason is that the 2D
  simulator's lidar stops up to 1 cm short of each surface (it marches over a 1 cm grid), while Webots
  measures depth exactly from the rendered scene. This has not been tested separately.
- Re-planning takes 25–55 ms in Webots: faster than in 2D, because the batch ran one run at a time instead of six
  in parallel.


## Changing the tombs

Edit `tools/layouts.py` (the only source of truth), then regenerate:

```bash
python tools/world_gen.py
```

This rewrites `worlds/*.wbt`, the procedural textures in `worlds/textures/`, and the
ground-truth grids in `ground_truth/` (`.npz` plus a `.png` preview), so the map
score always matches the world.

## How the Scarab explores

Every 1.5 s the robot re-plans:

1. **Costmap**: obstacles from the map are grown by the robot radius + 5 cm (configuration space),
   and cells near obstacles cost extra so paths keep to the middle of corridors.
2. **Wavefront**: path distance from the robot to every reachable cell, in one array sweep.
3. **Frontiers**: free cells that touch unknown cells, grouped into clusters. Clusters narrower than the robot
   are ignored, and each cluster gets a reachable target cell near its centre.
4. **Information gain**: 48 rays cast from each target estimate how many m² of unknown tomb a scan there
   would reveal. Targets that would reveal less than 0.25 m² are skipped.
5. **Selection**, one of three strategies:
   - `nearest`: shortest path (Yamauchi, 1997)
   - `largest`: biggest frontier cluster
   - `utility`: U = IG · e^(−0.35·d) (González-Baños & Latombe, 2002)

   All three commit to a goal: it is tracked as its frontier recedes, kept for at least 8 s, and only
   swapped for one that scores 1.5× better (hysteresis). Without this, `largest` ping-pongs between
   frontiers on opposite sides of the maze and never finishes.
6. **A\***: 8-connected, octile heuristic, no corner cutting. If no path exists, the next candidate is tried.

Every control step (32 ms), **pure pursuit** (look-ahead 0.5 m) follows the path, turning on the spot when
the path points more than 55° away. The **safety layer** stops forward motion when the corridor the robot's
body would sweep through is blocked.

On arrival the robot **turns to face the frontier**: the SICK only sees the 180° in front, so arriving is not
the same as looking. A goal is blacklisted only if it is reached a second time and is still a frontier, or
if it is unreachable, blocked or late.

**Half-seen walls**: wall faces seen at grazing angles can stay "unknown" for ever. Unknown cells right next
to an occupied cell are therefore treated as wall, both for frontiers and for information-gain rays.
Without this rule, frontiers flicker along every wall and gain rays leak out of the tomb, which kept the
robot driving between two corners for minutes.

The robot decides it is finished when no worthwhile reachable frontier is left.

On the map display: blue = frontiers, teal = planned path, red cross = current goal, grey crosses = blacklisted goals, gold = trail.

Pick the strategy with `--strategy=wander|nearest|largest|utility` (in `controllerArgs`, `SCARAB_ARGS` or `sim2d.py --strategy`).

## Tests

```bash
python tests/test_exploration.py
```

These check the grid update, frontier detection (including half-seen walls), A* (detours, impossible
paths, diagonal squeezes), information gain, and wavefront distances (including that wavefront and A* agree on reachability).

## Repository layout

```
controllers/scarab/
  scarab.py                  Webots entry point (controllerArgs: --world --strategy --start --mode
                             --max-time --quit --shots --log-dir)
  tomb/config.py             every parameter (map, sensor model, robot, exploration, planning)
  tomb/hal.py                RobotIO interface + WebotsIO (motors, lidar, GPS, IMU, keyboard, display)
  tomb/mission.py            sense -> map -> decide -> act -> log loop
  tomb/mapping/              log-odds occupancy grid, vectorised inverse sensor model
  tomb/exploration/          frontiers.py (detection, clustering), selection.py (strategies, IG,
                             hysteresis, blacklist), explorer.py (the re-planning loop)
  tomb/planning/             costmap.py (C-space, clearance, wavefront), astar.py
  tomb/control/              pure_pursuit.py, wander.py (baseline, safety corridor check, teleop)
  tomb/metrics.py            explored fraction and accuracy against ground truth
  tomb/viz/hud.py            papyrus-map rendering for the robot display
tools/
  layouts.py                 tomb definitions
  world_gen.py               .wbt + textures + ground truth generator
  ground_truth.py            rasterises layouts into 1 cm and 5 cm grids
  textures.py                procedural sandstone / hieroglyph / door textures
  sim2d.py                   2D test simulator with the same RobotIO interface
  score_logs.py              scores ad-hoc run logs in results/logs against ground truth
  run_experiments.py         tomb x strategy x start-pose batches (sim2d in parallel, Webots in sequence)
  plot_results.py            report figures and tables from the experiment logs
tests/test_exploration.py    unit checks for mapping, frontiers and planning
```

## Still to build

| Module | File(s) | Owner |
|---|---|---|
| Mapping extensions: odometry-only pose, sensor-model tuning | `tomb/mapping/`, `tomb/localization/` | M3 |
| Extra experiments: λ / hysteresis / IG-threshold sweeps, sensor-range study, odometry-pose maps | `tools/run_experiments.py`, `tools/plot_results.py` | M5 |
| Recovery behaviours, controller tuning study | `tomb/control/` | M4 |
| Strategy parameter study (λ, hysteresis, IG threshold) | `tomb/exploration/` | M2 |
| Worlds, integration, ground truth | `tools/`, `tomb/mission.py` | M1 |
