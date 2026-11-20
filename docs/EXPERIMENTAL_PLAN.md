# Field experimental plan — steering-controller comparison in Vidalia onion beds

Purpose: compare three steering controllers — **pure pursuit (PP)**, **MPC
(MPPI)** and **RL (PPO policy)** — for autonomous bed following with headland
U-turns, in **transplanted** and **direct-seeded** onion fields, with
replication, randomisation and an independent ground truth, so the results
can be published.  Everything except the steering law is held fixed.

Schedules: `scripts/make_run_schedule.py`.  Data sheets: `docs/field_sheets/`.
Simulation companion (predictions to compare against):
`docs/SIMULATION_METHODS.md`.

---

## 1. Questions and hypotheses

| # | Question | Hypothesis (H1) | Test |
|---|---|---|---|
| Q1 | Do the controllers differ in **tracking accuracy** (true cross-track error)? | MPC and RL ≤ PP (sim: PP 9.4, MPC 8.3, RL 8.6 cm) | Mixed model on bed-pass RMSE; pairwise contrasts; **equivalence** test, margin ±2 cm |
| Q2 | Do they differ in **smoothness** (steering effort, jerk)? | PP ≈ RL < MPC (field so far: MPC jerk ≈ 4.8× PP) | Mixed model on log(jerk), log(effort) |
| Q3 | Does **crop establishment** (transplanted vs direct-seeded) change accuracy or the controller ranking? | Direct-seeded (smaller, sparser plants early) relies more on bed shape → larger error | Controller × establishment interaction |
| Q4 | Does **growth stage** change it? | Error grows with canopy (sim: mature/lodged worst) | Controller × stage interaction |
| Q5 | Are **headland turns and re-entry** equally reliable? | No difference (turn is controller-independent; re-entry steering is not) | GLMM (binomial) on turn success; landing offset |
| Q6 | Is the robot's **self-estimate** of its offset unbiased? | Self-estimate ≈ truth (bias < 2 cm) | Agreement analysis (Bland–Altman) at stations |
| Q7 | Do **simulation rankings** predict field rankings? | Yes for smoothness; tracking within CI | Compare sim predictions with field estimates |

Primary outcome: **true cross-track RMSE per bed pass** (Q1).  Everything else
is secondary and labelled as such.

---

## 2. Factors

| Factor | Levels | Type | How it is handled |
|---|---|---|---|
| **Controller** | PP, MPC (noise 0.10, temp 0.1), RL (`ppo_onion_s1`) | Fixed, **treatment** | Randomised within session (Graeco-Latin square, §4) |
| **Crop establishment** | Transplanted (TP), direct-seeded (DS) | Fixed | Between fields — see confounding note below |
| **Growth stage** | e.g. early / mid / late (defined by **measured** plant height, not date) | Fixed | One field visit per stage |
| Field | ≥ 1 per establishment (ideally 2) | Random, nested in establishment | Block |
| Session | 3 per visit | Random | Block; each controller once per session |
| Bed set | 3 sets of 3 adjacent beds per visit | Random, nested in field | Each controller once per bed set |
| Bed | 9 per visit | Random, nested in bed set | Repeated measures |
| Run position | 1st / 2nd / 3rd in the session | Nuisance | Balanced by the square |
| Pass direction | Out / back (alternates in serpentine) | Nuisance covariate | Same for all controllers; in model |
| First-turn direction | Left / right | Nuisance | Balanced over two visits (§4) |

**Confounding warning.** If there is only **one** transplanted and **one**
direct-seeded field, "establishment" and "field" are the same thing: any
difference could be soil, slope, bed shape or the field itself.  Two fields
per establishment separate them; with one each, report establishment as a
**descriptive** comparison, not a tested effect.

---

## 3. Units, replication and pseudo-replication

| Level | What it is | Counts as a replicate for… |
|---|---|---|
| **Run** | One 3-bed serpentine (bed, U-turn, bed, U-turn, bed) by one controller | Reliability, turns per run, interventions |
| **Bed pass** | One traverse of one bed | **Primary unit** for tracking (Q1–Q4) |
| Station | Tape measurement every 10 m along a bed | **Sub-samples** of a bed pass → summarised to one RMSE per pass |
| Scan | One LiDAR scan (~7 per second) | **Never** a replicate (thousands per pass, strongly autocorrelated) |
| Turn | One headland U-turn | Unit for Q5 (2 per run) |

Using scans or stations as n would be **pseudo-replication**: hundreds of
"replicates" that are really one bed pass.

### Sample size (from this season's data)

Bed-to-bed spread of self-reported RMSE: PP 5.6–8.4 cm, MPC 5.4–9.5 cm →
SD ≈ 1.5 cm.  For a paired comparison (same bed set, two controllers), with
paired SD σ_d ≈ 1.5–2 cm, detecting a **1.5 cm** difference (α = 0.05, power
0.8) needs n ≈ 7.85 (σ_d/δ)² ≈ **8–14 bed passes per controller per
condition**.  One visit (§4) gives **9 bed passes per controller**; two visits
per condition give 18.  Re-estimate σ_d from the first visit's **tape**
measurements and adjust.

---

## 4. Design within one field visit — Graeco-Latin square

Choose **9 adjacent, fully planted beds** of equal length, grouped into three
**bed sets** of 3 (A, B, C).  Three **sessions** (e.g. day 1 morning,
afternoon; day 2 morning).  In each session every controller drives **its own
bed set**, so tracks of different controllers never share a furrow that day.

| | Bed set A | Bed set B | Bed set C |
|---|---|---|---|
| **Session 1** | Ctrl 1 · run 1st | Ctrl 2 · run 2nd | Ctrl 3 · run 3rd |
| **Session 2** | Ctrl 2 · run 3rd | Ctrl 3 · run 1st | Ctrl 1 · run 2nd |
| **Session 3** | Ctrl 3 · run 2nd | Ctrl 1 · run 3rd | Ctrl 2 · run 1st |

Over the visit each controller drives **every bed set once**, runs **in every
session once**, and takes **every run position once**, so bed, time-of-day and
order effects are balanced.  Which controller is "Ctrl 1", which physical
beds are "A", and the session order are **randomised** per visit with a
recorded seed:

```bash
python3 scripts/make_run_schedule.py --field TP1 --visit 1 --seed 101 --csv docs/field_sheets/TP1_v1_schedule.csv
```

The script also prints each run's exact command.  **Turn direction:** in a
3-bed serpentine the first U-turn is left when sweeping left
(`--beds-left 2`) and right when sweeping right (`--beds-right 2`).  The
script alternates the start side so every controller gets 2:1 left/right
first turns per visit and **3:3 over two visits**.

### Per run (≈ 1.5 h)

| Step | Time |
|---|---|
| Robot to the start stake of the bed set, centred and aligned; check `row=0/3` on the first status line | 5 min |
| Drive 3 beds (~165 m each at 0.19 m/s) + 2 U-turns | ~45–50 min |
| Ground truth: tape at every station of the 3 beds (2 people) | ~25–30 min |
| Copy files, note conditions | 5 min |

One visit ≈ 9 runs ≈ **13–14 h ≈ 2 field days**.  Measuring every **20 m**
instead of 10 m halves the tape time; keep 10 m if possible for the first
visit to estimate variance.

### Total effort

| Scope | Visits | Runs | Bed passes / controller | Field days |
|---|---|---|---|---|
| **Minimum publishable** — 1 TP + 1 DS field × 2 stages | 4 | 36 | 36 (9 per condition) | ~8 |
| **Recommended** — 2 TP + 2 DS fields × 2 stages | 8 | 72 | 72 (18 per condition) | ~16 |
| Full — 2 + 2 fields × 3 stages | 12 | 108 | 108 | ~24 |

If time is short, keep the square intact and drop a stage or a field — never
drop a controller from a session.

---

## 5. What is held fixed (record in every manifest)

| Item | Value | Check |
|---|---|---|
| Software | **One frozen git commit for the whole season** (tag it, e.g. `field-2026-12`) | `manifest.json` → `git_commit`, `git_dirty: false` |
| Preset / speed | `--preset onion --speed 0.20`, `--headland-arc-cap 4.5`, `--no-slam`, `--save-scans 2` | Schedule commands |
| Controller settings | MPC noise 0.10 / temp 0.1 / drift gain 0.05; RL `ppo_onion_s1` + slew 0.12 | Startup lines |
| LiDAR calibration | 1.035 m / 15.7° / 0.9° | Re-check with `diag_pitch_rings.py` if the mount is touched |
| Loop health | ≥ 7 scans/s, 0 % half-scans | `bash scripts/field_speed_test.sh` once per field day |
| Network | `sudo bash scripts/setup_lidar_network.sh` after every power-up | LiDAR validate passes |
| Battery | Start each session above an agreed level | Note on run sheet |
| Operator | Same operator role, same e-stop rules | Run sheet |

---

## 6. Ground truth (independent of the robot)

**Primary: tire-track offset at stations.**
1. Before the visit: flag every bed of the 9 at **stations every 10 m**, from
   5 m after the bed start to 5 m before the bed end (equal bed lengths → the
   same station positions on every bed).
2. Right after each run, at each station on its 3 beds:
   `dL` = outer-left onion row → centre of left tire track;
   `dR` = outer-right onion row → centre of right tire track.
   **True offset = (dL − dR) / 2** (sign: + = robot right of bed centre).
   Nominal when centred: dL = dR ≈ 0.54 m (wheels 0.96 m, outer rows 0.42 m
   from centre).
3. Per bed pass: RMSE, mean (bias), max |offset| over the stations.

**Track overlap between sessions.** A bed set is driven once per session, so
by session 3 its furrows hold three sets of tracks.  Pick one before the
visit:
- smooth the furrows (rake / drag) after each session's measurements; or
- mark the path instead of the tracks: a downward chalk or marking-paint
  nozzle at the robot centre, a different colour per controller; or
- (best for a large field) use **fresh beds every session** — 27 beds per
  visit; bed set then becomes fully nested.

**Measurement error.** On the first visit, two people measure the same 20
stations independently; report the repeatability (expected ≤ 1 cm).

**Optional:** verified RTK track (only after a static test: rover still for
10 min, scatter < 2 cm; plus a taped baseline), mainly for U-turns and
re-entry.  The 2026-11-06 logs had no RTK fix and are not usable.

**Linking stations to the robot's log.** Bed lengths are equal, so station
position along the bed = `row_dist / final row_dist × bed length` from
`telemetry.jsonl` (until measured wheel speed works, odometry is commanded
speed, so normalise per bed).  This gives the robot's self-estimate at each
station for Q6.

---

## 7. Responses

| Response | Unit | Source | Level |
|---|---|---|---|
| **True cross-track RMSE** (primary) | cm | Tape (§6) | Bed pass |
| True bias, max \|offset\| | cm | Tape | Bed pass |
| Self-reported cross-track RMSE, heading RMSE | cm, ° | `telemetry.jsonl` / `per_row.csv` | Bed pass |
| Self-estimate error (self − truth) | cm | Tape + telemetry | Station → bed pass |
| Steering effort \|ω\|, jerk | rad/s | Telemetry | Bed pass |
| Speed mean / SD | m/s | Telemetry | Bed pass |
| Interventions (operator take-over, e-stop, manual restart) | count / 100 m | Run sheet + telemetry | Run |
| Crop damage: plants run over or wheel on bed | count / bed | Walk the bed after the run | Bed pass |
| U-turn success (lands on the correct next bed, no help) | yes/no | Run sheet, `turns.csv` | Turn |
| U-turn landing offset | cm | Tape at the first two stations of the next bed | Turn |
| U-turn time, arc length, final rotation | s, m, ° | `turns.csv` | Turn |
| Bed-end stop error (stop point − real bed end) | m | Tape | Bed pass |
| Loop health (scans/s, half-scans) | — | `summary.json` | Run |

Steady-state tracking excludes the first and last 10 m of each bed;
entry (first 10 m after a turn) and bed-end behaviour are reported
separately.

---

## 8. Covariates to record (each run)

Plant height at 5 stations per bed set, leaf count / stage, plant spacing
gaps; bed-top relief (the robot logs `bed_contrast_m`); soil moisture
(visual class or probe), surface (crusted / loose / wet), weeds (class);
weather (wind, sun/cloud, rain in last 24 h), time of day; battery level;
`manifest.json` commit; any hardware change.

---

## 9. Safety, interventions and stopping rules

- E-stop within reach at all times; one person walks behind during U-turns.
- **Intervention** = any operator action that changes the robot's motion
  (e-stop, joystick, manual restart).  Log time, place (bed, metres), reason.
- If a run stops (e.g. after a U-turn): note it as an intervention, drive the
  robot to the next bed start with the joystick, and finish with the
  single-bed command for that controller.  The run still counts.
- Abort the session (not the controller) for rain, a mechanical fault or a
  LiDAR/network failure; resume with the **same** schedule row later.
- Never change software, presets or calibration inside a visit.  If a fix is
  unavoidable, start a new visit (new seed) and record the commit.

---

## 10. Analysis plan (fixed before the data)

**Primary model (Q1–Q4):** linear mixed model on bed-pass true RMSE (log
scale if residuals are skewed):

```
log(RMSE) ~ Controller * Establishment * Stage + PassDirection + RunPosition
            + (1 | Field) + (1 | Field:Visit:Session) + (1 | Field:Bed)
```

- Estimated marginal means with 95 % CIs; pairwise controller contrasts with
  Holm correction.
- **Equivalence** (TOST, margin ±2 cm on the original scale): "no meaningful
  difference" is a valid, publishable result.
- With one field per establishment, drop `(1 | Field)` and report
  establishment descriptively (§2).

**Secondary:** same model for log(jerk), log(effort), heading RMSE.
Interventions: Poisson GLMM with log(distance) offset.  Turn success:
binomial GLMM with turn direction and controller.  Self-estimate vs truth:
Bland–Altman (bias, 95 % limits of agreement) per controller and stage.
Sim-to-real (Q7): plot sim prediction vs field mean ± CI per controller.

**Reporting:** n at every level, all runs including failed ones (no
exclusions except documented equipment failures), software commit, raw data
deposited (`pack_run.py` packs + tape sheets).

---

## 11. Data management

| Item | Where |
|---|---|
| Schedule (with seed) | `docs/field_sheets/<field>_v<visit>_schedule.csv` |
| Run sheet | `docs/field_sheets/run_log_template.csv` (one row per run) |
| Tape measurements | `docs/field_sheets/ground_truth_template.csv` (one row per station) |
| Robot data | `runs/run_<controller>_<time>/` → `pack_run.py` → `field_data/` |
| Photos | One per bed set per visit (canopy), plus any failure |

---

## 12. Threats to validity

| Threat | Mitigation |
|---|---|
| Establishment confounded with field | ≥ 2 fields per establishment, or descriptive only |
| Day / light / soil drying trends | Sessions as blocks; run position balanced |
| Order and learning effects (operator, soil compaction) | Graeco-Latin square |
| Overlapping tracks | Furrow smoothing, colour marking, or fresh beds (§6) |
| Self-reported metrics biased | Tape ground truth is the primary outcome |
| One RL policy, one MPC setting | State them; generalising to "RL" or "MPC" is out of scope |
| Software changes mid-season | Frozen tagged commit |
| Wheel odometry on commanded speed | Station alignment normalised per bed; fix pending (`diag_canbus_topics.py`) |
| Known re-entry weaknesses (straight post-turn creep, angled exit leg) | Count as interventions; report |

---

## 13. Before the first visit (checklist)

- [ ] Re-entry fix (`9975632`) driven once on all three controllers.
- [ ] Freeze and tag the season's commit.
- [ ] Wheel-speed topic found (`scripts/diag_canbus_topics.py`), or the
      odometry limitation is accepted and stated.
- [ ] Tape protocol rehearsed on one bed; repeatability measured.
- [ ] Track-overlap option chosen (§6).
- [ ] Schedules generated with recorded seeds for every planned visit.
- [ ] Data sheets printed or on a tablet.
