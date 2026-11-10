#!/usr/bin/env bash
# run_trainer_study.sh — the steering-policy training study for the paper.
#
# Trains the 385-parameter steering network with three algorithms, several
# seeds each, on both simulator profiles, with the reward weights of the
# deployed policies (c_u 0.8, c_du 8.0), then tests every exported NumPy policy
# on the held-out episodes against pure pursuit and MPC:
#   ES   scripts/train_rl.py   (NumPy, Evolution Strategies — the original trainer)
#   PPO  scripts/train_sb3.py  (Stable-Baselines3, PyTorch, Gymnasium env)
#   SAC  scripts/train_sb3.py
#
# Needs: pip install "gymnasium>=1.0" "stable-baselines3>=2.3" torch matplotlib
# (dev PC only — the robot runs the exported .npz with NumPy).
#
#   bash scripts/run_trainer_study.sh                 # everything
#   SEEDS="0 1 2" PROFILES="onion" JOBS=4 bash scripts/run_trainer_study.sh
set -euo pipefail
cd "$(dirname "$0")/.."

SEEDS=${SEEDS:-"0 1 2"}
PROFILES=${PROFILES:-"soybean onion"}
ALGOS=${ALGOS:-"es ppo sac"}             # add sac1m for the long SAC runs (~1.5 h each)
JOBS=${JOBS:-4}
ES_ITERS=${ES_ITERS:-300}
PPO_STEPS=${PPO_STEPS:-2000000}
SAC_STEPS=${SAC_STEPS:-300000}
SAC_LONG_STEPS=${SAC_LONG_STEPS:-1000000}   # ALGOS="sac1m": SAC with a longer budget
REWARD="--c-u 0.8 --c-du 8.0"
POL=policies/trainers
RES=results/trainers
mkdir -p "$POL" "$RES/logs"

cmds=()
for prof in $PROFILES; do
  for algo in $ALGOS; do
    for s in $SEEDS; do
      name="${algo}_${prof}_s${s}"
      [ -f "$POL/$name.npz" ] && [ -z "${FORCE:-}" ] && { echo "skip $name (exists; FORCE=1 to redo)"; continue; }
      case $algo in
        es)  c="python3 scripts/train_rl.py --profile $prof $REWARD --seed $s --iters $ES_ITERS --out $POL/$name.npz --curve-csv $RES/${name}_curve.csv" ;;
        ppo) c="python3 -u scripts/train_sb3.py --algo ppo --explore gsde --profile $prof $REWARD --seed $s --timesteps $PPO_STEPS --eval-every 100000 --out $POL/$name.npz --curve-csv $RES/${name}_curve.csv" ;;
        sac) c="python3 -u scripts/train_sb3.py --algo sac --profile $prof $REWARD --seed $s --timesteps $SAC_STEPS --eval-every 10000 --out $POL/$name.npz --curve-csv $RES/${name}_curve.csv" ;;
        sac1m) c="python3 -u scripts/train_sb3.py --algo sac --profile $prof $REWARD --seed $s --timesteps $SAC_LONG_STEPS --eval-every 20000 --out $POL/$name.npz --curve-csv $RES/${name}_curve.csv" ;;
      esac
      cmds+=("$c > $RES/logs/$name.log 2>&1")
    done
  done
done

echo "${#cmds[@]} training runs, $JOBS at a time"
printf '%s\n' "${cmds[@]}" | xargs -P "$JOBS" -I{} bash -c '{}'

python3 scripts/compare_trainers.py $(for p in $PROFILES; do echo --profile $p; done)
