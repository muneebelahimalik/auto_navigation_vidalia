# Running the simulations on your own computer

Everything below runs on the dev PC (Ubuntu 22.04, Python 3.10 is fine).  No
robot, LiDAR or farm-ng SDK needed.  PyTorch is only needed to *train* PPO/SAC;
looking at results and re-testing policies needs only NumPy + matplotlib.

## 1. One-time setup

```bash
git clone https://github.com/muneebelahimalik/auto_navigation_vidalia.git
cd auto_navigation_vidalia
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
# CPU-only PyTorch (≈200 MB instead of ≈3 GB of CUDA libraries — no GPU is needed):
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements-sim.txt
python3 -m pytest tests/ -q          # ~5 min; all pass except one that needs a field data file
```

## 2. Look at something (minutes, no training)

| What | Command | Output |
|---|---|---|
| Watch every controller drive the same simulated row (pursuit, MPC, ES, PPO, SAC) | `python3 scripts/watch_episode.py --hardest --gif --show` | `results/watch/episode_*.png/.gif` |
| Same, onion profile | `python3 scripts/watch_episode.py --profile onion --show` | |
| Re-test all trained policies on the 200 test episodes + learning curves | `python3 scripts/compare_trainers.py` | `results/trainers/summary_*.txt`, `learning_curves.png` |
| Onion growth stages through the ray-cast LiDAR simulator | `python3 scripts/eval_onion_growth.py --bed-end` | table per stage |
| Self-driving-style POV video (synthetic demo) | `python3 scripts/render_pov_video.py --demo --out results/pov` | GIF/MP4 |

`watch_episode.py --hardest` picks the steepest cross-slope test episode
that some controller can still drive.  Any test seed can be replayed with
`--seed 50000…50199`.

## 3. Re-run the training study (hours)

```bash
bash scripts/run_trainer_study.sh                  # ES + PPO + SAC, 3 seeds, both profiles (~1.5 h on 4 cores)
JOBS=8 bash scripts/run_trainer_study.sh           # more parallel jobs if you have the cores
FORCE=1 ALGOS="ppo" PROFILES="onion" bash scripts/run_trainer_study.sh   # redo one part
```

Existing policies are skipped unless `FORCE=1`, so delete or force to
reproduce from scratch.  Results land in `policies/trainers/` and
`results/trainers/`; methods and numbers are in
`docs/SIMULATION_METHODS.md` §5.5 / §9.4.  Your numbers should match within
the seed spread (different CPUs give slightly different floating point).

## 4. Sending onion field data

The onion simulators were calibrated from few runs (the steering profile from
the 2026-10-15 run only).  Real recorded runs make them better.  Most useful,
in order:

1. **Runs you already have on the brain** — `ls ~/auto_navigation_vidalia/runs/`
   (2026-10-15, 10-16, 10-29, 11-06 onion runs).
2. **New runs with `--record`** in the onion field, ideally a full bed
   including the bed end and headland.  For the richest data add
   `--save-scans 2` (every 2nd scan instead of every 10th):
   `python3 scripts/row_follow.py --auto --preset onion --controller pursuit --record --save-scans 2`
3. **A perception-only pass** (robot driven by joystick) over beds at a
   different growth stage also helps: `python3 scripts/row_follow.py --preset onion --record`.

Pack each run into one file and push it:

```bash
python3 scripts/pack_run.py runs/run_pursuit_20261112_101500      # -> field_data/<run>.npz, prints MB
git add field_data/ && git commit -m "Field data: onion run 2026-11-12" && git push
```

If a pack is over ~95 MB (GitHub's limit) the script says so — re-run with
`--every 2` or cut a window with `--from 0 --to 600` (seconds).  The pack holds
scans, telemetry, manifest and metrics; there is no GPS in it.

What the data is used for:
- replay the real scans through the current bed detector (`replay_scans.py
  --bed-rows 4`) and compare with the growth-stage simulator at the same stage;
- refit the onion steering simulator's noise model from more runs
  (`calibrate_sim_from_run.py`), then retrain/re-test the onion policies;
- replay every controller on the real logged estimates
  (`replay_controllers.py`).
