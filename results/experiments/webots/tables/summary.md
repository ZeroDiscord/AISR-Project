## Exploration results (webots, 5 start poses per tomb)

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

### Fastest strategy to 95% from each start pose

| Tomb | start 0 | start 1 | start 2 | start 3 | start 4 | wins (utility / nearest / largest / wander) |
|---|---|---|---|---|---|---|
| Antechamber | utility (23 s) | wander (21 s) | utility (19 s) | utility (20 s) | largest (25 s) | 3 / 0 / 1 / 1 |
| Labyrinth | nearest (120 s) | utility (139 s) | largest (112 s) | largest (122 s) | utility (133 s) | 2 / 1 / 2 / 0 |
| Treasury | utility (89 s) | utility (88 s) | largest (103 s) | utility (79 s) | utility (72 s) | 4 / 0 / 1 / 0 |

### Speed-up in mean time to 95% over reactive wander

A run that never reached 95% counts as taking the whole time limit, so a speed-up marked ≥ is a lower bound.

| Tomb | nearest | largest | utility |
|---|---|---|---|
| Antechamber | 0.7× | 1.0× | 1.2× |
| Labyrinth | ≥ 3.4× | ≥ 3.1× | ≥ 3.1× |
| Treasury | 1.5× | 1.9× | 2.4× |
