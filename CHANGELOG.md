# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/);
versions are git tags on `main`.

## [Unreleased] — headland U-turn reliability (not yet field-validated)

Work toward the row-to-row turn milestone (built on the v0.1.0 baseline).

### Changed
- **LiDAR yaw re-calibrated 71° → 66°** after a mount disturbance (lens
  cleaning rotated the sensor ~5°). Measured with the new `diag_birdseye.py`
  forward-object locator (a straight-ahead bucket read +5.4° at yaw 71, 0.0° at
  yaw 66). Pitch independently re-confirmed at 21.5°. `--lidar-yaw` default
  updated in `row_follow.py` / `diag_birdseye.py`.
- **`diag_birdseye.py` forward-object locator** added: reports the azimuth of
  elevated forward objects so the mount yaw can be re-verified to ~1° any time
  the LiDAR is touched (`--range 3`, read the straight-ahead azimuth).

### Added
- **Filter (IMU/GPS) heading for the pivots.** `navigation/filter_heading.py`
  subscribes to the filter service (`FilterState`, port 20001) and exposes the
  fused absolute heading. `HeadlandTurn` now closes the two 90° pivots on this
  heading when it is fresh and converged — robust to the wheel slip that an
  in-place skid-steer pivot causes — and falls back to wheel-odometry heading
  otherwise. The source is latched per turn and shown in the status line
  (`R-UTURN:TURN_A[filter]`/`[wheel]`).
- **`--headland-shift` (default 1.52 m):** the centre-to-centre SHIFT distance
  to the next strip the robot straddles, made distinct from `--row-spacing`
  (0.76 m, the in-strip soybean-row separation used by the detector).

### Fixed
- **Row end never reached the headland turn**, then **spurious turns on slopes.**
  Crop running out trips both the row-end signal and the low-confidence miss
  counter at once. First fix routed the miss to `ROW_END` when a real row had
  been driven and the band ahead was empty — but a 4-frame trigger then misread
  a brief crop **dropout on a slope** (crop flickering 0↔700) as a row end and
  started a headland turn mid-row. Final logic (`navigation/state_logic.py
  follow_loss_action`): a real row end requires a *long continuous* crop absence
  (`row_end_frames`, raised 8→15) — the miss counter resets the instant crop
  reappears, so an intermittent slope dropout can never accumulate it; a
  non-row-end loss re-acquires after `follow_miss_thresh`; otherwise wait.
  Unit-tested incl. the slope-dropout regression (`action(4, True) == "WAIT"`).

### Notes
- Logic is unit-tested (filter-vs-wheel selection, latching, distinct shift
  distance, row-end-vs-loss decision); the full turn still needs field
  validation (pivot accuracy, re-acquisition of the next row). 98 tests pass.

## [v0.1.0] — 2026-06-23 — First field-validated row-follow baseline

First version that autonomously follows a soybean centre-residue row in the
field: drives the strip centred, with stable heading, and survives real
(non-flat) terrain. Tagged as a known-good checkpoint to build on.

### Fixed — perception/geometry root causes
- **LiDAR correction order.** Apply yaw **then** tilt (was tilt→yaw). The two
  do not commute at the 71° mount yaw; the wrong order left a >1 m ground ramp
  that biased the heading PCA and drove the robot off-row. Regression-locked.
- **Pitch calibration.** Field-swept the true robot-frame pitch to **21.5°**
  (the eyeballed 15° was body lean) via `diag_birdseye.py --tilt-sweep`. Set as
  the default with yaw **71°** so field runs need no flags. `LIDAR_MOUNT_HEIGHT`
  confirmed correct at 0.75 m (tape).
- **ROS 2 bridge TF.** Publish `base_link→velodyne` as identity rotation — the
  cloud is already yaw/tilt-corrected upstream, so the old −15° pitch in the TF
  re-tilted an already-flat cloud in the live 3-D view.

### Fixed — connectivity
- **Canbus host.** `service_config.json` pointed at the Tailscale FQDN while the
  process runs on the brain itself, so `/twist` looped out through Tailscale and
  intermittently timed out. Use `localhost` for the on-brain canbus/filter
  services. Added a retrying handshake with a longer first-call timeout.

### Added — field robustness (terrain-adaptive crop band)
- **Grade detrend.** Estimate the forward ground slope per scan (low percentile
  of z per range-bin, slope-only → robust to furrow/soil bimodality) and remove
  it, so crop stays inside the height band on sloped fields. No-op on flat /
  canopy-only data.
- **Level shift on dips.** When a real ground+canopy column sits below the
  flat-calibrated `h=0` (terrain dip), lower the crop band onto the local ground
  so the (correctly elevated) canopy is not clipped from below. Gated on height
  spread; only ever lowers the band.
- Live `grade=±N°` / `drop=−N.Nm` shown in the status line.

### Added — tooling
- `diag_birdseye.py`: `--tilt-sweep LO:HI:STEP` to calibrate pitch, and a
  detector-ROI height profile printout for diagnosing marginal spots.
- `--no-ground-detrend` escape hatch on `row_follow.py`.

### Notes / not yet validated
- Headland U-turn (`--headland`) and multi-row serpentine exist but are **not
  yet field-validated** — next milestone.
- 89 unit tests pass (`python3 -m pytest tests/ -q`).
