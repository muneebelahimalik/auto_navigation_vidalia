# Simulation, learning and control methods — reference for the manuscript

Everything below is taken from the code in this repository (file
references given), not from memory.  The results in §9 were regenerated on
2026-11-10 with the code at the commit that adds this file (soybean numbers
identical to the earlier `results/controller_pareto.csv`).

## 1. Overview

Three simulation components were built in plain Python + NumPy (no
ROS/Gazebo or physics engine).  The steering simulator is also exposed as a
standard **Gymnasium** environment, and the steering policy is trained both
with our NumPy Evolution Strategies and with the **Stable-Baselines3** (PyTorch)
implementations of **PPO** and **SAC** (§5.5), so the learning results do not
rest on in-house RL code alone:

| Component | File | What it simulates | Used for |
|---|---|---|---|
| **Steering simulator** | `sim/row_follow_env.py` | Planar kinematics of the robot relative to the row centreline, with skid-steer slip, cross-slope drift, perception noise/bias and dropouts | Training the RL policy; held-out comparison of pure pursuit, RL and MPC |
| **Ray-cast LiDAR simulator** | `sim/onion_field.py` | Velodyne VLP-16 Puck Hi-Res scans of a raised-bed onion field at six growth stages | Developing and validating the bed-perception pipeline (no controller in the loop) |
| **Field replay** | `scripts/replay_controllers.py`, `scripts/replay_scans.py` | Real logged perception estimates / real point clouds re-fed to the controllers / detector | Open-loop sim-to-real check of each controller on identical real inputs |

Design choice: steering is the only thing a learned or optimised controller
is allowed to change.  The state machine, safety monitor, forward-speed law
and headland U-turn are shared and unchanged.  Pursuit, RL and MPC are
therefore compared on the same perception and the same speed law.

## 2. Software and hardware

| Item | Value |
|---|---|
| Language / numerics | Python + NumPy (simulators, ES, MPC, everything that runs on the robot) |
| RL environment API | Gymnasium 1.4.0 (`sim/gym_env.py`; passes `gymnasium.utils.env_checker.check_env`) |
| Deep-RL library | Stable-Baselines3 2.9.2 on PyTorch 2.15.0 (CPU), training only (`scripts/train_sb3.py`) |
| Development / evaluation machine | x86-64, Python 3.11.18, NumPy 2.4.6 (this document's numbers) |
| Robot computer | NVIDIA Jetson Xavier NX (6-core ARM), Ubuntu 20.04, Python 3.8.10, JetPack 5.1 |
| Robot | farm-ng Amiga (4-wheel skid-steer), farm-ng OS 2.0 SDK (gRPC) |
| LiDAR | Velodyne VLP-16 Puck Hi-Res (16 beams, ±10°, 1.33° spacing), 10 Hz, mounted 1.035 m high, 15.7° nose-down |
| Learned policy format | NumPy `.npz` (layer sizes + flat parameter vector), forward pass in µs on the Jetson |
| RL algorithms | Evolution Strategies (Salimans et al., 2017), own NumPy implementation (`scripts/train_rl.py`); PPO (Schulman et al., 2017) and SAC (Haarnoja et al., 2018) from Stable-Baselines3 (Raffin et al., 2021) |
| MPC algorithm | MPPI, Model-Predictive Path Integral control (Williams et al., 2017), own NumPy implementation |
| Unit/regression tests | pytest (`tests/test_rl.py`, `test_onion_rl.py`, `test_row_controller.py`, `test_onion_growth_stages.py`, …) |

## 3. Steering simulator (`sim/row_follow_env.py`)

### 3.1 State, dynamics and speed law

The state is the robot's pose relative to the row centreline:
- lateral offset *e* (m, + = row to the robot's right);
- heading error *θ* (rad).

Time step Δt = 0.1 s (one LiDAR scan); episodes are 200 steps (20 s).

The action is *a* ∈ [−1, 1]; the commanded yaw rate is ω_cmd = a·ω_max with ω_max = 0.40 rad/s.

```
θ_{k+1} = θ_k + (s · ω_cmd + n_ω) Δt              n_ω ~ N(0, 0.02²) rad/s
e_{k+1} = e_k + (v_k · sin θ_{k+1} + g) Δt
v_k     = max(v_min, v_max · c_k · max(0.25, 1 − |θ̂_k| / 1 rad))
```

- *s*: skid-steer slip, the achieved fraction of the commanded rate. Drawn once per episode, s ~ clip(N(0.75, 0.12²), 0.4, 1.0).
- *g*: cross-slope lateral drift, drawn once per episode, g ~ N(0, 0.035²) m/s.
- The speed law is the same function as the pure-pursuit controller's (`speed_from`), with v_max = 0.30 m/s and v_min = 0.08 m/s. A policy therefore changes only the steering.
- *c_k*: perception confidence (§3.2).

### 3.2 Perception model (observation)

```
ê = e + b_e + N(0, σ_e²)          θ̂ = θ + b_θ + N(0, σ_θ²)
σ_θ = σ_θ0 + k_g · |g|            (heading noise grows with the cross-slope)
```

- **Dropouts:** with probability p_drop per step the row fix is lost. Confidence drops from 0.92 to 0.20 and the previous observation is repeated (a stale fix).
- **Biases:** *b_e* and *b_θ* are optional AR(1) measurement biases: b ← ρ·b + σ_b·√(1−ρ²)·N(0,1). They model slow target wander (bed meander, shoulder irregularity).
- **Smoothing:** an optional EMA (α = 0.35) reproduces the detector's smoothing.
- **Initial state:** e₀ ~ U(−0.30, 0.30) m, θ₀ ~ U(−0.20, 0.20) rad.

The grade-correlated heading noise was calibrated from field telemetry: heading std was about 1.9° on flat ground and about 5.2° on graded ground.

### 3.3 Two disturbance profiles

| Parameter | Symbol | Soybean (default) | Onion (system-identified, §4) |
|---|---|---|---|
| max forward speed | v_max | 0.30 m/s | 0.20 m/s |
| slip mean / std | s | 0.75 / 0.12 | 0.75 / 0.12 |
| cross-slope drift std | σ_g | 0.035 m/s | 0.004 m/s |
| lateral white noise | σ_e | 0.02 m | 0.0691 m |
| heading white noise (flat) | σ_θ0 | 0.02 rad | 0.0192 rad |
| heading noise per unit drift | k_g | 1.0 rad/(m/s) | 0 |
| dropout probability | p_drop | 0.08 | 0.003 |
| lateral bias std / AR(1) ρ | σ_be / ρ_e | 0 / – | 0.0223 m / 0.9194 |
| heading bias std / AR(1) ρ | σ_bθ / ρ_θ | 0 / – | 0.0022 rad / 0.9631 |
| detector EMA | α | none | 0.35 |

### 3.4 Reward and termination

```
r_k = 0.5  +  2.0 · v_k cos θ_k Δt  −  ( c_e e² + c_θ θ² + c_u a² + c_du (a_k − a_{k−1})² )
```

- An episode **terminates** when |e| > 0.60 m (off the row), with a −5 penalty.
- **Alive bonus:** the constant 0.5 per step dominates the costs. Without it the agent reward-hacked by leaving the row early to stop paying per-step costs; this was observed and fixed during development.
- **Weights:** defaults are c_e = 2.0, c_θ = 0.5, c_u = 0.05, c_du = 0.30. They are swept in §5.4.

### 3.5 Observation given to the controllers

The observation is [ê, θ̂, c, a_{k−1}, ξ]:
- ξ is a leaky integral of the observed lateral offset, ξ ← clip(0.97 ξ + 0.1 ê, ±0.5) (`navigation/rl_policy.py::update_eint`). It lets a reactive policy cancel a steady drift. The same rule runs in the simulator and on the robot.
- The network input is scaled by [2.0, 2.5, 1.0, 1.0, 2.0].

## 4. System identification of the onion profile (`scripts/calibrate_sim_from_run.py`)

The onion profile was fitted to a real 150 m pure-pursuit run on a bare raised bed (2026-10-15, 0.20 m/s).

1. **Field statistics.** Only steady down-row FOLLOW scans are used. Start-up is skipped (100 scans), and the data is cut at the first row end, or where the bed relief fades below 0.6× its median for 15 scans. From that segment:
   - lateral estimate: std, std of the scan-to-scan change, autocorrelation at lags 1, 10 and 50;
   - heading estimate: std, std of the scan-to-scan change, autocorrelation at lag 10;
   - mean |ω| commanded.
2. **Simulated statistics.** The same pure-pursuit controller drives the simulator for 6 episodes × 1200 steps (first 50 steps discarded). Speed and dropout rate are taken from the run.
3. **Loss.** A weighted sum of squared relative errors over the 9 statistics. Weights: 2 for each std, 2 for lateral change std, 1 for lag-1/lag-10 autocorrelation, 0.5 for lag-50, 1 for heading change std, heading lag-10 and |ω|.
4. **Search.** 200 candidates: half drawn uniformly from the box (σ_e ∈ [0, 0.15], σ_θ ∈ [0, 0.06], σ_be ∈ [0, 0.10], ρ_e ∈ [0.90, 0.995], σ_bθ ∈ [0, 0.08], ρ_θ ∈ [0.95, 0.998]), then local Gaussian refinement around the best.
5. **Result.** Pure pursuit in the fitted simulator reproduces the run's closed-loop statistics within about 10 % (fitted values in §3.3).

## 5. Controllers

### 5.1 Pure pursuit with integral action (baseline; `navigation/row_controller.py`)

**Geometric law.** Look-ahead L = 2.0 m on the perceived centreline:
- target point (x_t, y_t), with y_t ≥ 0.5 m;
- curvature κ = 2x_t / (x_t² + y_t²);
- ω = −v·κ.

**Integral term.** A leaky integral of the offset is added to ω:
- update ξ ← 0.94 ξ + e Δt, only when confidence ≥ 0.45 **and** |Δe| ≤ 0.20 Δt (a persistent offset, not a transient);
- otherwise ξ only leaks;
- steering contribution ω_I = −0.15 ξ, clamped to ±0.15 rad/s, with anti-windup on the state.

**Outputs and fallback.** Output clamped to ±0.40 rad/s. Below confidence 0.35 the output is (0, 0) and the state machine reacts. Speed law as in §3.1.

### 5.2 Learned steering policy (RL)

| Item | Value |
|---|---|
| Network | MLP 5 → 16 → 16 → 1, tanh on all layers (bounded output), **385 parameters** |
| Initialisation | W ~ N(0, 0.3²)/√n_in, b = 0 (near-zero initial steering) |
| Output | a ∈ [−1, 1] → ω = a · 0.40 rad/s |
| Optimiser | Evolution Strategies (gradient-free) |
| Population | 40 (20 antithetic pairs ±σε) |
| Perturbation σ / step size | 0.10 / 0.05 |
| Fitness | mean return over 8 episodes; all candidates in one iteration share the same 8 seeds (common random numbers) |
| Utilities | centred ranks in [−0.5, 0.5] (robust to reward scale) |
| Gradient | ĝ = Σ_i (u_i⁺ − u_i⁻) ε_i / (2 · 20 · σ) |
| Iterations | 300 (script default) |
| Model selection | every 10 iterations on 64 held-out seeds (10 000–10 063), keep the best; never used for the gradient |
| Training seed | 0 |

**Deployment safety wrapper** (`navigation/rl_controller.py`):
- **Steering only:** the policy sets ω; forward speed comes from pure pursuit.
- **Fallback:** pure pursuit takes over when confidence is below 0.35, the fix is invalid, or no policy file is loaded.
- **Limits:** output clamped to ±0.40 rad/s; optional slew limit on the change per step (onion preset 0.12 rad/s per step).
- **Residual variant** (optional): total action = pursuit action + 0.5 × policy output.

### 5.3 MPPI model-predictive controller (`navigation/row_mpc_controller.py`)

Each step, K = 200 sequences of H = 20 actions (2 s) are sampled around the warm-started nominal plan:
- u ~ clip(u_nom + N(0, σ²), ±1), σ = 0.35.

Each sequence is rolled through the kinematic model of §3.1, using the observer estimates ŝ and d̂:

```
θ ← θ + a·ω_max·ŝ Δt ;   e ← e + (v sin θ + d̂) Δt
J = Σ_t  6.0 e² + 1.5 θ² + 0.05 a² + 0.30 (a_t − a_{t−1})²
w_k ∝ exp(−(J_k − min J)/λ),  λ = 0.6 ;   u_nom ← Σ_k w_k u_k
```

- The first action is applied; the plan is shifted one step for the next call (warm start).
- **Drift observer** (Luenberger): d̂ ← d̂ + 0.20 (Δe_meas − v sin θ Δt − d̂ Δt)/Δt, clamped to ±0.20 m/s.
- **Slip observer:** ŝ ← ŝ + 0.05 (clip(Δθ/(ω_cmd Δt), 0.2, 1.2) − ŝ), clamped to [0.3, 1.1].
- **Safety:** the same wrapper as RL (pursuit speed, pursuit fallback below 0.35, ±0.40 clamp).
- **Onion field setting** (`--preset onion`): σ = 0.20, λ = 0.3, drift gain 0.05. Raising the effort/jerk costs instead made it steer more on real estimates (§7).
- The held-out simulator tables (§9.1) use the default setting for both profiles.

### 5.4 Policies trained (`policies/`)

| File | Profile | c_e | c_θ | c_u | c_du | Notes |
|---|---|---|---|---|---|---|
| `follow_jerk{0.3,1.0,3.0,8.0}.npz` | soybean | 2.0 | 0.5 | 0.05 | 0.3 / 1 / 3 / 8 | jerk (smoothness) sweep |
| `follow_residual_jerk*.npz` | soybean | 2.0 | 0.5 | 0.05 | 0.3 / 1 / 3 / 8 | residual variant (scale 0.5) |
| `follow_gentle.npz`, `follow_gentle2.npz` | soybean | 2.0 | 0.5 | 0.4 / 0.8 | not recorded | effort sweep |
| **`follow_field.npz`** | soybean | 2.0 | 0.5 | **0.8** | **8.0** | deployed for soybean (`--controller rl`) |
| **`follow_onion.npz`** | onion | 2.0 | 0.5 | **0.8** | **8.0** | deployed for onion (`--preset onion --controller rl`), trained from scratch |

The deployed policy files store only the weights; their ES iteration count was not logged (script default 300). They are now backed by the training study (§5.5, §9.4): three fresh ES seeds per profile trained with the recorded command (`policies/trainers/es_*`, logs and curves in `results/trainers/`) reproduce them within the seed spread.

### 5.5 Training study: ES vs PPO vs SAC (standard libraries, several seeds)

To show the learned steering does not depend on one in-house optimiser, the
same 385-parameter network is trained with three algorithms, three training
seeds each (0, 1, 2), on both profiles, with the reward weights of the
deployed policies (c_e 2.0, c_θ 0.5, c_u 0.8, c_du 8.0).  One command runs the
whole study: `bash scripts/run_trainer_study.sh`.

**Environment** (`sim/gym_env.py::RowFollowGymEnv`): a thin Gymnasium wrapper
of §3 — identical dynamics, disturbances and reward.
- Observation: the 5 inputs of §3.5 scaled exactly as the robot's policy scales them (`encode_obs`), float32, bounds ±10 (a no-op clip).
- Action: Box [−1, 1] (ω = a · 0.40 rad/s).  `terminated` = left the row; `truncated` = 20 s elapsed.
- Each episode's disturbances come from an episode seed drawn from the Gymnasium RNG; `reset(options={"episode_seed": s})` replays exactly the episode `RowFollowEnv.reset(seed=s)` gives, so the test episodes are identical for every trainer.
- Checked by `gymnasium.utils.env_checker.check_env` and step-by-step against the core simulator (`tests/test_gym_sb3.py`).

**Actor** — identical for all three algorithms: 5 → 16 → 16 → 1, tanh hidden
units.  The critics (value / Q networks, 64 → 64 tanh) exist only during
training and are discarded.

| | ES (`train_rl.py`) | PPO (SB3) | SAC (SB3) |
|---|---|---|---|
| Type | gradient-free, population | on-policy actor-critic | off-policy max-entropy actor-critic |
| Key settings | population 40, σ 0.10, lr 0.05, 8 episodes/candidate | 8 parallel envs, n_steps 1024, batch 256, 10 epochs, γ 0.99, GAE λ 0.95, clip 0.2, lr 3·10⁻⁴, entropy coef 0; **gSDE exploration** (sample freq 4, initial log-std −2) | buffer 3·10⁵, 5 000 random warm-up steps, batch 256, γ 0.99, τ 0.005, lr 3·10⁻⁴, 1 gradient step per env step, automatic entropy tuning |
| Budget | 300 iterations (≈15 M simulator steps) | 2 M steps | 3·10⁵ steps; soybean also 10⁶ steps (≈1.4 h per seed) |
| Deterministic action | tanh(net) | clip(Gaussian mean, ±1) | tanh(Gaussian mean) |
| Exported `out_act` | `tanh` | `clip` | `tanh` |

Budgets were set so each run takes 6–30 min on one CPU core; SAC is
sample-efficient but ~25× slower per step (one gradient update per step).
All other settings are Stable-Baselines3 defaults.

**PPO exploration — a negative result worth reporting.**  With SB3's default
exploration (independent Gaussian noise, std 1), PPO got *worse* during
training (validation return 48 → −52 after 0.4 M steps, 3 seeds; curves in
`results/trainers/ppo_default_noise_failed/`).  Cause: under the deployed
reward the jerk term 8·(a_t − a_{t−1})² of uncorrelated unit-std noise costs
≈ 6 per step, far above the +0.5 per-step alive bonus, so while exploring,
leaving the row early looked better than following it.  PPO uses
generalized State-Dependent Exploration (gSDE; Raffin et al., 2022), which
SB3 provides for exactly this case (smooth, temporally correlated noise on
real robots), with initial std e⁻² ≈ 0.14 (`--explore gsde`, the default;
`--explore default` reproduces the failure).  ES has no such issue because
it perturbs the weights, not the actions.

**Model selection** is the same for all three: every 10 ES iterations / 100 k
PPO steps / 10 k SAC steps, the actor is exported to the robot's NumPy format
and evaluated on the 64 validation seeds 10 000–10 063; the best export is
kept.  These evaluations form the learning curves (`*_curve.csv`: simulator
steps used for training, wall time, validation return, cross-track, heading,
jerk, effort, success).

**Export to the robot** (`scripts/train_sb3.py::export_actor`): the PyTorch
actor's linear layers are copied into `navigation/rl_policy.py::MLPPolicy`
(`.npz`; field `out_act` records clip vs tanh, older files default to tanh).
The NumPy forward pass reproduces `model.predict(obs, deterministic=True)` to
< 10⁻⁵ (`test_sb3_actor_exports_exactly_to_numpy`).  The robot therefore runs
PPO/SAC policies through the same `RLController` safety wrapper, with no
PyTorch on the Jetson:
`python3 scripts/row_follow.py --auto --preset onion --controller rl --policy policies/trainers/ppo_onion_s0.npz`.

## 6. Evaluation protocol (simulator)

- **Test set:** 200 episodes on seeds 50 000–50 199, disjoint from the training seeds (random) and the model-selection seeds (10 000–10 063).
- **Paired comparison:** every controller sees the identical slip, drift, noise and dropout draws for each seed (`sim/evaluate.py::rollout`).
- **Metrics per episode,** computed on the true (hidden) state, not on the noisy observation:
  - cross-track RMSE;
  - heading RMSE;
  - control jerk = mean |a_k − a_{k−1}|;
  - control effort = mean |ω|;
  - success = never left the row (|e| ≤ 0.60 m for 20 s);
  - return.
- **Breakdown** by cross-slope severity |g| (bins 0–0.03, 0.03–0.06, > 0.06 m/s).
- **Export:** per-episode rows with `--csv` (columns: controller, seed, grade_drift, slip, xtrack_rmse_m, heading_rmse_deg, control_jerk, success, return).

## 7. Field replay (open-loop sim-to-real check)

`scripts/replay_controllers.py` feeds the logged per-scan estimates of a real run (lateral, heading, confidence) to every controller. Inputs are identical across controllers, and the robot path is not re-simulated.

- **What it measures:** each controller's response to real perception (steering effort, jerk, peak command), relative to pure pursuit.
- **Sanity check:** pursuit's replay reproduces its own logged commands (|ω| 0.0076 vs 0.0077 rad/s).
- **Why it matters:** this exposed the sim-to-real gap of the soybean policy on onion estimates (6.4× pursuit's effort). That led to the onion system identification (§4) and `follow_onion.npz` (3.1×).

## 8. Ray-cast LiDAR simulator of the onion field (`sim/onion_field.py`)

### 8.1 Scene

- **Ground:** periodic raised beds.
  - Pitch 1.83 m (72 in), flat top 1.05 m wide, linear shoulders 0.22 m wide, wheel furrows between.
  - Bed height depends on the growth stage (beds slump over the season).
  - Soil roughness: two-term sinusoid, amplitude 8 mm.
  - Optional bed end at y_end, beyond which the headland is flat with grass (0.22 m tall, 1 800 elements/m², radius 12 mm).
- **Crop:** 4 rows per bed at 0.2794 m (11 in), plants every 0.1016 m (4 in).
  - Each plant is a cloud of spherical leaf elements. Their lateral spread grows from base to top (leaves fan out), and their count sets the canopy's optical density.
  - Options: missing rows, random stand gaps, outer-row droop, lodged tops.
  - Weeds (optional): clumps of 25 elements, 3–25 cm tall.
- **Growth stages** (`STAGES`):

| Stage | Canopy height above bed (m) | Elements per plant | Leaf radius (mm) | Lateral spread base → top (m) | Bed height (m) |
|---|---|---|---|---|---|
| bare | 0 | 0 | – | – | 0.10 |
| seedling | 0.07 | 3 | 4 | 0.004 → 0.008 | 0.10 |
| young | 0.20 | 30 | 10 | 0.010 → 0.040 | 0.09 |
| mid | 0.35 | 70 | 15 | 0.015 → 0.080 | 0.08 |
| mature | 0.50 | 110 | 20 | 0.020 → 0.150 (+0.06 droop) | 0.07 |
| lodged | 0.28 | 120 | 20 | 0.080 → 0.200 (+0.15 droop) | 0.06 |

### 8.2 Sensor and ray casting

- **Beams:** the 16 VLP-16 Hi-Res angles. Azimuth ±62° (the forward sector the detector uses) in 0.2° steps (600 rpm).
- **Beam properties:** half-divergence 0.08°; range noise σ = 1.5 cm; max range 14 m.
- **Mount:** 1.035 m above the furrow, 15.7° nose-down (the calibrated mount). Robot yaw relative to the bed is a parameter.
- **Ground hits:** each beam marches the heightfield in 3 cm steps, then bisects 6 times.
- **Vegetation hits:** leaf elements are z-buffered onto the beam grid. An element of radius r at range R covers beams within asin(r/R) + divergence.
- **Return:** the nearest surface per beam. Porosity, partial occlusion of the soil, and furrows shadowed by the canopy therefore come from the geometry.
- **Output:** the same robot-frame, tilt-corrected, self-filtered (planar range ≥ 1.5 m) cloud the real pipeline produces. With it comes the ground truth: lateral offset of the nearest bed centre, and heading.
- **Robustness options:** residual tilt-calibration error, removal of azimuth sectors (packet loss), weaving yaw.

### 8.3 Protocol (`scripts/eval_onion_growth.py`)

- **What runs:** the real detector, configured exactly as `--preset onion`, on each stage.
- **Start conditions:** five per stage, (lateral, yaw) = (0, 0°), (+0.12 m, 0°), (−0.20 m, 0°), (+0.05 m, +4°), (−0.08 m, −6°).
- **Drive:** 60 scans at 2 cm per scan (0.2 m/s at 10 Hz); the first 20 scans are not scored.
- **Metrics:**
  - lateral error: mean, std and max;
  - heading error;
  - mean confidence and scans below the 0.35 FOLLOW threshold;
  - false row-end scans;
  - perception mode used.
- **`--bed-end`:** drives into a grass headland. Reports how far before the end confidence dropped, and whether grass was ever followed.
- **`--robust`:** weeds, 30 % stand gaps, 4 cm beds, 1.5° tilt error, sector dropouts, ±3° weaving, and U-turn re-entry up to 0.35 m / 12°.

## 9. Results (regenerated 2026-11-10) and reproduction

Raw outputs: `results/eval_soybean_20261110.{txt,csv}`,
`results/eval_onion_20261110.{txt,csv}`, `results/eval_onion_growth_20261110.txt`
(per-episode CSVs: 200 episodes × 3 controllers).

### 9.1 Steering controllers, held-out simulation (200 episodes, seeds 50 000–50 199)

**Soybean profile** (cross-slope drift, 8 % dropouts, 0.30 m/s):

| Controller | Cross-track RMSE (cm) | Heading RMSE (°) | Jerk | Effort \|ω\| (rad/s) | Success | Return |
|---|---|---|---|---|---|---|
| Pure pursuit + integral | 17.9 | 7.8 | 0.028 | 0.023 | 98.0 % | 88.95 |
| RL (`follow_field`) | **10.3** | 9.2 | 0.046 | 0.041 | 99.5 % | **98.81** |
| MPC (MPPI, default) | 14.2 | **6.1** | 0.054 | 0.036 | 99.5 % | 95.94 |

By cross-slope severity (cross-track RMSE; success):

| \|g\| (m/s) | n | Pursuit | RL | MPC |
|---|---|---|---|---|
| 0–0.03 | 131 | 12.4 cm; 100 % | 7.0 cm; 100 % | 9.3 cm; 100 % |
| 0.03–0.06 | 55 | 24.9 cm; 100 % | 13.5 cm; 100 % | 20.4 cm; 100 % |
| > 0.06 | 14 | 41.5 cm; 71 % | 27.8 cm; 93 % | 35.6 cm; 93 % |

**Onion profile** (system-identified, flat, 0.20 m/s):

| Controller | Cross-track RMSE (cm) | Heading RMSE (°) | Jerk | Effort \|ω\| (rad/s) | Success | Return |
|---|---|---|---|---|---|---|
| Pure pursuit + integral | 9.7 | 4.3 | **0.010** | **0.016** | 100 % | 101.31 |
| RL (`follow_onion`) | 8.0 | 4.4 | 0.016 | 0.022 | 100 % | 102.66 |
| MPC (MPPI, default) | **7.7** | **3.7** | 0.057 | 0.046 | 100 % | 102.79 |

The full jerk / residual sweep (soybean, 200 episodes) is in
`results/controller_pareto.csv` (accuracy–smoothness Pareto front).

### 9.2 Real-estimate replay (open-loop, §7)

On the real 2026-10-15 onion estimates, steering effort relative to pure pursuit:
- soybean policy `follow_field`: 6.4×;
- onion policy `follow_onion`: 3.1× (peak 0.12 vs 0.21 rad/s, jerk ≈ pursuit);
- default MPPI: 5.3×; onion-tuned MPPI: 3.5×.

These figures come from the 2026-10-16 analysis (commit `f1209ae`). Re-run
`replay_controllers.py` on the final field runs before quoting them.

### 9.3 Bed perception at every growth stage (ray-cast, real detector)

Lateral error after settling, over the 5 start conditions (bias ± std, worst |error|):

| Stage | Perception mode | Lateral bias (cm) | Lateral std (cm) | Worst \|error\| (cm) | Heading worst (°) | Confidence | Bed end detected before the end |
|---|---|---|---|---|---|---|---|
| bare | bed shape | −1.2 … 0.0 | ≤ 0.6 | 1.7 | 0.2 | 1.00 | 3.6 m |
| seedling | bed shape | −0.6 … 0.0 | ≤ 0.4 | 1.1 | 0.1 | 1.00 | 3.5 m |
| young | crop rows (comb) | −0.6 … +0.2 | ≤ 0.2 | 0.9 | 0.1 | 1.00 | 2.3 m |
| mid | bed shape | −1.1 … +0.3 | ≤ 1.1 | 3.0 | 0.2 | 1.00 | 2.4 m |
| mature | bed shape | −1.1 … +1.4 | ≤ 1.9 | 4.9 | 0.7 | 1.00 | 2.7 m |
| lodged | bed shape | −1.0 … +1.2 | ≤ 2.0 | 5.1 | 0.6 | 1.00 | 2.7 m |

- No false row ends, and no scans below the FOLLOW threshold, at any stage.
- After the bed end the grass headland was never followed: maximum confidence 0.32–0.33, below the 0.35 threshold.

### 9.4 Training study: ES vs PPO vs SAC (§5.5), 200 test episodes

Test seeds 50 000–50 199 (as §9.1).  Trained policies: mean ± SD over 3
training seeds; each is the exported NumPy policy the robot would load.  MPC
uses the robot's setting for each profile (onion: σ 0.20, λ 0.3, drift gain
0.05 — so its onion row differs from §9.1, which used the default).
Raw: `results/trainers/test_{soybean,onion}.csv`, `summary_*.txt`; learning
curves `results/trainers/*_curve.csv` → `learning_curves.png/.svg`; training
logs `results/trainers/logs/`.

**Soybean profile**

| Controller | n | Cross-track RMSE (cm) | Heading RMSE (°) | Jerk | Effort \|ω\| (rad/s) | Success (%) | Training steps |
|---|---|---|---|---|---|---|---|
| Pure pursuit + integral | – | 17.9 | 7.8 | 0.029 | 0.024 | 98.0 | – |
| MPC (MPPI) | – | 14.2 | **6.1** | 0.054 | 0.036 | 99.5 | – |
| ES (NumPy) | 3 | **10.5 ± 0.6** | 9.1 ± 0.6 | 0.043 ± 0.001 | 0.041 ± 0.003 | 99.5 ± 0.0 | 18.4 M |
| PPO (SB3, gSDE) | 3 | 11.1 ± 0.9 | 9.3 ± 0.6 | 0.023 ± 0.001 | 0.044 ± 0.007 | 99.5 ± 0.0 | 2.0 M |
| SAC (SB3) | 3 | 18.1 ± 0.5 | 11.8 ± 1.3 | 0.008 ± 0.001 | 0.039 ± 0.005 | 95.8 ± 2.1 | 0.3 M |
| SAC (SB3), longer budget | 3 | 18.3 ± 1.2 | 11.1 ± 2.2 | **0.007 ± 0.001** | 0.034 ± 0.008 | 95.8 ± 2.8 | 1.0 M |
| deployed `follow_field.npz` (ES) | 1 | 10.3 | 9.2 | 0.046 | 0.041 | 99.5 | not logged |

**Onion profile**

| Controller | n | Cross-track RMSE (cm) | Heading RMSE (°) | Jerk | Effort \|ω\| (rad/s) | Success (%) | Training steps |
|---|---|---|---|---|---|---|---|
| Pure pursuit + integral | – | 9.7 | 4.4 | 0.010 | **0.016** | 100 | – |
| MPC (MPPI, onion setting) | – | 8.1 | **3.5** | 0.031 | 0.028 | 100 | – |
| ES (NumPy) | 3 | **7.7 ± 0.2** | 4.2 ± 0.1 | 0.018 ± 0.002 | 0.023 ± 0.001 | 100 | 19.1 M |
| PPO (SB3, gSDE) | 3 | 8.6 ± 0.4 | 4.4 ± 0.2 | 0.007 ± 0.001 | 0.022 ± 0.003 | 100 | 2.0 M |
| SAC (SB3) | 3 | 8.7 ± 0.3 | 4.5 ± 0.7 | **0.005 ± 0.001** | 0.021 ± 0.004 | 100 | 0.3 M |
| deployed `follow_onion.npz` (ES) | 1 | 8.0 | 4.4 | 0.016 | 0.022 | 100 | not logged |

What this shows:
- **The learned-steering result does not depend on the in-house optimiser.** PPO, from a standard library, reaches the same tracking as ES within about one seed SD (soybean 11.1 vs 10.5 cm), with half the jerk, using 9× fewer simulator steps.
- **The deployed policies are reproducible:** `follow_field` and `follow_onion` lie inside the spread of three fresh ES seeds trained with the recorded command.
- **Every learned controller beats pure pursuit on cross-track error** in both profiles. On the onion profile, all learned policies are within 1 cm of each other and of MPC.
- **SAC is the one trainer that does not beat pure pursuit on the soybean profile, and more training does not change that.** At 3·10⁵ steps its validation return was still rising, so three more seeds were trained for 10⁶ steps (`ALGOS=sac1m`). The return plateaued below pursuit, at best 74–76 on validation against pursuit's 75 (best checkpoints at 0.3, 0.5 and 0.96 M steps). The test result was unchanged: 18.3 ± 1.2 cm vs 18.1 ± 0.5 cm, test return 83.8 ± 1.4 vs pursuit 85.9 and PPO 92.6 ± 0.9. SAC settles on a different trade-off, with the lowest jerk of any controller (0.007) but no better tracking than pursuit. The test does not show why. A plausible cause is its entropy bonus and squashed-Gaussian policy, which favour small, smooth actions under this reward's heavy jerk cost. On the milder onion profile SAC matches the others (8.7 cm).
- **Note on the SAC 1 M logs:** the stdout logs in `results/trainers/logs/sac1m_*.log` were rebuilt from the curve CSVs, because the original files were overwritten during training (as each log's header says). The curve CSVs and policies were written separately and are complete. Each exported policy reproduces the best validation return in its curve exactly.
- Trade-off: ES tracks slightly tighter; the SB3 policies are smoother (lower jerk). Which to deploy is the accuracy–smoothness choice of §5.4.

### 9.5 Reproduce



```bash
# held-out controller comparison, 200 episodes each profile
python3 scripts/eval_controller.py --policy policies/follow_field.npz --mpc --episodes 200 --csv results/eval_soybean.csv
python3 scripts/eval_controller.py --policy policies/follow_onion.npz --mpc --profile onion --episodes 200 --csv results/eval_onion.csv
# growth-stage perception (ray-cast)
python3 scripts/eval_onion_growth.py --bed-end
python3 scripts/eval_onion_growth.py --robust
# re-train the deployed policies with a recorded command (recommended before submission)
python3 scripts/train_rl.py --iters 300 --c-u 0.8 --c-du 8.0 --seed 0 --out policies/follow_field.npz
python3 scripts/train_rl.py --iters 300 --c-u 0.8 --c-du 8.0 --seed 0 --profile onion --out policies/follow_onion.npz
# training study: ES + PPO + SAC, 3 seeds, both profiles, then the test tables + figure
pip install -r requirements-sim.txt          # dev PC only
bash scripts/run_trainer_study.sh            # ~1.5 h on 4 cores
python3 scripts/compare_trainers.py          # re-run tables/figure from the saved policies
# sim calibration from a field run
python3 scripts/calibrate_sim_from_run.py runs/<run>/telemetry.jsonl
# open-loop replay of real estimates through all controllers
python3 scripts/replay_controllers.py runs/<run>/telemetry.jsonl --mpc-onion --policy policies/follow_onion.npz
```

## 10. Limitations to state in the paper

- **Steering simulator:** first-order kinematics only. There is no tire, terrain-contact or actuator-lag model; slip is a constant per-episode scale. All RL and MPC numbers are simulation results until confirmed in the field.
- **Calibration data:** the onion profile was fitted to a single bare-bed run, so other growth stages may have different perception noise.
- **Ray-cast simulator:** idealised plant geometry (spherical leaf elements, regular spacing) and no wind or leaf motion. Validation against field scans so far is qualitative: point-pattern and relief magnitudes. Report it as a design tool, not as ground truth.
- **Seeds:** ES training uses a single seed per policy. For the paper, train with several seeds (for example 5) and report the spread.
