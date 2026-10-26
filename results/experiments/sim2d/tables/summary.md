## Exploration results (sim2d, 5 start poses per tomb)

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

### Fastest strategy to 95% from each start pose

| Tomb | start 0 | start 1 | start 2 | start 3 | start 4 | wins (utility / nearest / largest / wander) |
|---|---|---|---|---|---|---|
| Antechamber | utility (24 s) | wander (24 s) | utility (26 s) | utility (23 s) | largest (24 s) | 3 / 0 / 1 / 1 |
| Labyrinth | largest (129 s) | utility (168 s) | utility (122 s) | utility (142 s) | nearest (158 s) | 3 / 1 / 1 / 0 |
| Treasury | largest (94 s) | largest (95 s) | utility (76 s) | utility (78 s) | utility (94 s) | 3 / 0 / 2 / 0 |

### Speed-up in mean time to 95% over reactive wander

A run that never reached 95% counts as taking the whole time limit, so a speed-up marked ≥ is a lower bound.

| Tomb | nearest | largest | utility |
|---|---|---|---|
| Antechamber | 1.1× | 1.2× | 1.5× |
| Labyrinth | ≥ 3.3× | ≥ 2.8× | ≥ 3.5× |
| Treasury | 2.0× | 1.7× | 2.7× |
