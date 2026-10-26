# How the team works on this repository

The grading needs **each member's technical work to be traceable to commits from their own GitHub
account** (rubric B1, Contribution section). Follow these rules and it will be.

## One-time setup (every member)

```bash
git clone https://github.com/<team-lead>/<repo>.git
cd <repo>
git config user.name  "Your Name"                  # as it should appear in the history
git config user.email "you@example.com"            # the email of YOUR GitHub account
pip install -r requirements.txt
python tests/test_exploration.py                   # all tests should print "ok"
```

Never commit from someone else's laptop or account, and never ask someone else to push your work for you.

## Who owns what

| Member | Module | Files you mainly touch | Report sections |
|---|---|---|---|
| M1 (lead) | Worlds, integration, ground truth | `tools/layouts.py`, `tools/world_gen.py`, `tools/ground_truth.py`, `controllers/scarab/tomb/mission.py` | Abstract, 1, 2, References |
| M2 | Frontier detection and selection | `controllers/scarab/tomb/exploration/` | 3, 4 |
| M3 | Occupancy-grid mapping, pose sources | `controllers/scarab/tomb/mapping/`, a new `tomb/localization/` | 5, 6 |
| M4 | Planning and path following | `controllers/scarab/tomb/planning/`, `controllers/scarab/tomb/control/` | 7, 8 |
| M5 | Experiments, plots, tables | `tools/run_experiments.py`, `tools/plot_results.py` | 9, 12, Appendix |

"Still to build" in the README lists the open work for each module.

## Everyday workflow

1. Start from an up-to-date `main`:
   ```bash
   git switch main
   git pull
   git switch -c m3/odometry-pose          # <member>/<short-topic>
   ```
2. Make small commits that each do one thing, with a message that says what and why:
   ```
   mapping: add wheel-odometry pose source

   Integrates encoder angles with the differential-drive model so maps can be
   built without GPS; used for the drift experiment in Section 9.
   ```
   Start the subject with the module (`mapping:`, `exploration:`, `planning:`, `control:`, `worlds:`,
   `experiments:`, `docs:`), so the history reads as a map of who did what.
3. Before pushing, check nothing broke:
   ```bash
   python tests/test_exploration.py
   python tools/sim2d.py antechamber --strategy utility       # should reach 95% in about 25 s
   ```
4. Push the branch and open a Pull Request on GitHub:
   ```bash
   git push -u origin m3/odometry-pose
   ```
   Ask **one teammate from a different module** to review it. Reviewing is co-learning, and the review
   comments are evidence of it. The reviewer approves, and the author merges.
5. After merging, delete the branch and start the next one from a fresh `main`.

## Rules that keep `main` working

- Do not push directly to `main` (the lead can protect the branch on GitHub: Settings → Branches).
- Change parameters in `controllers/scarab/tomb/config.py`, not inside the modules.
- Change tombs in `tools/layouts.py`, then run `python tools/world_gen.py` and commit the regenerated
  `worlds/` and `ground_truth/` files in the same commit.
- Do not commit `results/logs/` or `results/figs/` (ad-hoc runs; already in `.gitignore`).
  Experiment results go in `results/experiments/` through `tools/run_experiments.py` and
  `tools/plot_results.py`, and are committed together with the code that produced them.
- If two people need the same file, agree in the team chat first, or split the change into separate PRs.

## Making contributions visible

- In Section 10, list your commits/PRs (GitHub → Insights → Contributors shows them per member).
- In the video, demo the part you built.
- Every member must be able to explain every module in the viva. Read each other's PRs.
