# Change log and rollback guide — since the 2026-10-29 field visit

Purpose: keep track of every change made after the last field visit where
the perception loop ran at full speed (~7.9 scans/s), so that each one can be
switched off or rolled back if the field says so.

## Field reference points

| Date | Code | Loop rate | Notes |
|---|---|---|---|
| 2026-10-29 | `7e6cbc4` generation (before the SLAM rework) | **7.9 scans/s** | 180 m onion row; SLAM map collapsed to 7.9 m |
| 2026-11-06 | `43968a1` | **4.0 scans/s** | 3 beds + 2 U-turns, 446 m; 4 of 5 saved scans missing a whole left or right half of the forward view; self-reported lateral RMSE 9.6 cm (was 4–6 cm); SLAM heading wrong (turns read 563° / −443°) |

## Every change, newest first

| Commit | Date | Change | Affects driving? | Field status | Switch off without a code change |
|---|---|---|---|---|---|
| `090a151` | 2026-11-10 | **LiDAR reader thread**: every packet is read in a background thread; navigation takes the newest *complete* scan, so a slow loop skips whole scans instead of getting half-scans | Yes (input data) | Lab test only: fake LiDAR + slow consumer → 0 of 8 scans partial (old path: 7 of 8) | `--lidar-reader inline` |
| `090a151` | 2026-11-10 | **numpy on one CPU core** (`OPENBLAS_NUM_THREADS=1` etc., set at the top of `row_follow.py`) — suspected cause of the 2× slowdown since the SLAM rework | Speed only | Not field-tested | prefix the command with `OPENBLAS_NUM_THREADS=6 OMP_NUM_THREADS=6` |
| `090a151` | 2026-11-10 | **SLAM rate limit** `--slam-hz 2` (mapping only) | Speed only | Not field-tested | `--slam-hz 0` |
| `090a151` | 2026-11-10 | **Half-scan guard** (bed mode): a scan with one side of the swath empty is skipped, the estimate held (≤ 5 in a row; after that it is used with confidence capped at 0.30, so a broken reader stops the robot instead of driving 26 cm off) | Yes (perception) | Sim: 4 of 5 halved → lateral max 16 cm without, < 1.1 cm with, all stages; real scans classified correctly | `--no-half-scan-guard` |
| `090a151` | 2026-11-10 | **Scan-health logging**: telemetry `pkts`, `scan_age_ms`, `lidar_skipped`, `scan_partial`, `fwd_cov`, `half_scan`, `loop_ms`; `summary.json` → `scan_health`; one line in the printed summary | No | — | — |
| `43968a1` | 2026-11-06 | Bed end: follow bare bed past the last onion (far-half absolute 2 cm relief floor) | Yes (row end) | Ran in the 3-bed run; rows 2–3 were 172 m (earlier same day 154 m). **Confirm it turned at the real bed end** | none (code) — roll back to `b1d002a` |
| `b1d002a` | 2026-11-06 | `--headland-arc-cap` (default 3 = unchanged) | Only if set | Turn 1 on 11-06 needed 10.2 m of wheel arc (> old 8.6 m limit) | omit the flag |
| `31c3293` | 2026-11-06 | `scripts/turn_gps_check.py` (offline analysis) | No | Used on 11-06 | — |
| `2937106` | 2026-11-06 | Telemetry `mono` + manifest clock stamps (logging) | No | OK | — |
| `261ec2c` | 2026-11-06 | `scripts/gnss_logger.py` (separate read-only process) | No (separate program) | Ran on 11-06; RTK never FIXED | don't start it |
| `ec5f300` | 2026-10-30 | **SLAM rework**: wheel odometry fed to SLAM, along-track limit, BLAS distance maths (5–6× cheaper on x86) | Mapping only, but **prime suspect for the 2× slowdown** (multi-core BLAS) | First field use 11-06: loop 4 scans/s; map heading still wrong | `--no-slam` (or the one-core setting above) |
| `7e6cbc4` | 2026-10-29 | Detect a leftover run holding the LiDAR port | No | Worked 11-06 | — |
| `cbaf03b` | 2026-10-29 | LiDAR startup check times out with a diagnosis | No | OK | — |
| `c3c5050` | 2026-10-29 | Raised-mount calibration 1.035 m / 15.7° / 0.9° | Yes | Used since; yaw ±1° uncertain | `--lidar-height/--lidar-tilt/--lidar-yaw` |

## Getting back to an earlier state

**1. Flags only (no code change) — "behave like 2026-11-06":**

```bash
OPENBLAS_NUM_THREADS=6 OMP_NUM_THREADS=6 python3 scripts/row_follow.py --auto --preset onion \
    --lidar-reader inline --slam-hz 0 --no-half-scan-guard  … (your usual flags)
```

**2. Whole code version (on the Brain):**

```bash
cd ~/auto_navigation_vidalia
git fetch origin
git checkout 43968a1      # exactly the code that ran on 2026-11-06
git checkout 7e6cbc4      # before the SLAM rework (≈ the 2026-10-29 generation;
                          # loses the bed-end fix, arc cap, GPS logger, scan-health)
git checkout 090a151      # the 2026-11-10 fixes (= main until the next change)
git checkout main         # back to the newest
```

Optional, so the names are easy to remember (tags could not be pushed from
the development session, so create them on the Brain once):

```bash
git tag field-2026-11-06 43968a1
git tag pre-slam-rework  7e6cbc4
```

## Field result 2026-11-13 — speed test (`runs/speedtest_20261113_194101/`)

Robot parked on an onion bed, perception only, 40 s per configuration.
(The first attempt logged no scans because the Brain had lost its LiDAR
address after a reboot. Fix: `sudo bash scripts/setup_lidar_network.sh` after
every power-up.)

| Configuration | scans/s | loop ms | incomplete % | half-scans % |
|---|---|---|---|---|
| 1 new defaults (thread reader, 1-core numpy, SLAM 2 Hz) | 3.6 | 225 | 1 | 1 |
| 2 old inline reader | 8.7 | 124 | **50** | **35** |
| 3 numpy on all cores | 2.9 | 328 | 2 | 1 |
| 4 SLAM unlimited | 4.0 | 203 | 1 | 1 |
| **5 no SLAM** | **7.7** | **119** | **1** | **1** |
| 6 as on Nov 6 | 4.7 | 197 | 72 | 52 |

Conclusions:
- **The SLAM thread is what halves the loop.** Throttled to 2 Hz it still
  costs ~100 ms per loop on the Jetson, presumably GIL contention with the
  driving loop. Without it, the loop runs at 7.7 scans/s.
- **Numpy on all cores makes it worse** (2.9 vs 3.6 scans/s), so the one-core
  setting stays.
- **The reader thread fixes the half-scans** (1 % vs 35–52 %). The old inline
  reader is fast only because it hands over broken scans.
- Decision for driving: **`--no-slam`** with the new defaults. Keep
  `--save-scans 2` so the map can be rebuilt offline from the saved scans.
  To do: move SLAM out of the driving process (separate process or offline).

## Next field visit — what to run, in order

1. `bash scripts/field_speed_test.sh` (robot parked on a bed, ~5 min, does
   not move).  It runs six configurations, from "all new" to "as on Nov 6", and
   prints one table: scans/s, loop time, % incomplete scans, % half-scans.
   Send the `runs/speedtest_<time>/` folder.
2. If configuration 1 is near 8+ scans/s with ~0 % incomplete: drive one bed
   with the new defaults and `--record`; check the `scan health` line in the
   summary.
3. If not: keep the best configuration from the table and send the folder.
4. Copy the GPS files from `gnss/` (the 11-06 one is still on the Brain).
