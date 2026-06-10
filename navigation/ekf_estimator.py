#!/usr/bin/env python3
"""
ekf_estimator.py — Extended Kalman Filter for fusing LiDAR and camera row estimates.

State vector: x = [lateral_offset (m), heading_error (rad)]
  - lateral_offset : signed distance from robot centreline to row centre (m)
  - heading_error  : angle from robot heading to row direction (rad)

Process model: constant state with additive Gaussian noise (Brownian motion).
  x_k = x_{k-1} + w,   w ~ N(0, Q)

Measurement model: direct observation (H = I₂).
  z = x + v,   v ~ N(0, R)

Measurement noise R is scaled inversely by detection confidence:
  R_lidar = R_lidar_base / max(conf², ε)
  R_cam   = R_cam_base   / max(conf², ε)

This ensures that a high-confidence LiDAR scan dominates a low-confidence
camera reading, and both are correctly marginalised when their confidences
drop simultaneously (e.g. at row ends).

Usage::

    ekf = RowEKF()
    ekf.reset()

    # Each scan:
    ekf.predict()
    ekf.update_lidar(lidar_est)
    ekf.update_camera(vis_est)       # pass None if cameras disabled
    fused = ekf.to_estimate(lidar_est)   # RowEstimate with fused geometry
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from navigation.row_perception import RowEstimate

# State dimension
_N = 2

# Minimum squared confidence for noise inversion (avoids division by zero).
_CONF_EPS = 1e-4


class RowEKF:
    """2-state Kalman filter that fuses LiDAR and camera row geometry estimates.

    Parameters
    ----------
    q_sigma_m : float
        Process noise standard deviation for lateral offset (m / scan).
    q_sigma_rad : float
        Process noise standard deviation for heading error (rad / scan).
    lidar_sigma_m : float
        Base LiDAR measurement noise for lateral offset at confidence=1 (m).
    lidar_sigma_rad : float
        Base LiDAR measurement noise for heading at confidence=1 (rad).
    cam_sigma_m : float
        Base camera measurement noise for lateral offset at confidence=1 (m).
    cam_sigma_rad : float
        Base camera measurement noise for heading at confidence=1 (rad).
    """

    def __init__(
        self,
        q_sigma_m: float = 0.015,
        q_sigma_rad: float = 0.008,
        lidar_sigma_m: float = 0.04,
        lidar_sigma_rad: float = 0.025,
        cam_sigma_m: float = 0.12,
        cam_sigma_rad: float = 0.06,
    ) -> None:
        self._Q = np.diag([q_sigma_m ** 2, q_sigma_rad ** 2])
        self._R_lidar_base = np.diag([lidar_sigma_m ** 2, lidar_sigma_rad ** 2])
        self._R_cam_base = np.diag([cam_sigma_m ** 2, cam_sigma_rad ** 2])

        # State and covariance — initialised lazily on first update.
        self._x: np.ndarray = np.zeros(_N)
        self._P: np.ndarray = np.diag([0.20, 0.10])
        self._initialised: bool = False
        self._real_updates: int = 0

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Re-initialise the filter (call when entering ACQUIRE)."""
        self._x = np.zeros(_N)
        self._P = np.diag([0.20, 0.10])
        self._initialised = False
        self._real_updates = 0

    # ------------------------------------------------------------------
    def predict(self, linear_vel: float = 0.0, angular_vel: float = 0.0, dt: float = 0.1) -> None:
        """Prediction step: propagate state with commanded velocity, then grow covariance.

        Velocity-informed model (Euler integration), in the perception sign
        convention (heading_error +ve = row angled to robot's right;
        lateral_offset +ve = row centre to the robot's right; angular_vel
        +ve = CCW/left turn):

          d(lateral_offset)/dt  ≈ +linear_vel × sin(heading_error)
            Driving forward while the row is angled to the right (θ>0)
            makes the row centreline pull further to the right of the robot
            path: x_row(t) = x₀ + v·t·tanθ.
          d(heading_error)/dt   ≈ +angular_vel
            Turning left (CCW, ω>0) rotates the row direction toward the
            robot's right in the robot frame, increasing heading_error;
            equivalently the controller's right-turn command (ω<0) drives
            a positive heading_error back to zero.

        Falls back to pure Brownian-motion prediction (P += Q only) when
        called with defaults (linear_vel=0, angular_vel=0, dt=0.1).
        """
        if self._initialised and dt > 0.0 and (linear_vel != 0.0 or angular_vel != 0.0):
            self._x[0] += linear_vel * math.sin(self._x[1]) * dt
            self._x[1] += angular_vel * dt
        self._P = self._P + self._Q

    # ------------------------------------------------------------------
    def update_lidar(self, est: RowEstimate) -> None:
        """LiDAR measurement update.

        Skipped when the estimate is invalid (n=0, empty scan) or confidence is
        low.  The `valid` flag is False whenever RowDetector._decay() ran without
        a real scan — confidence in that case is a decayed EMA phantom, not a
        sensor measurement.  Feeding phantoms into the filter is circular and
        inflates the update count without adding real information.
        """
        if est is None or not est.valid or est.confidence < 0.10:
            return
        z = np.array([est.lateral_offset, est.heading_error])
        conf2 = max(est.confidence ** 2, _CONF_EPS)
        R = self._R_lidar_base / conf2
        self._kf_update(z, R, initialise_from=est)

    # ------------------------------------------------------------------
    def update_camera(self, vis_est: Optional[object]) -> None:
        """Camera measurement update.  Safe to pass None when cameras disabled.

        Innovation gate: if the camera's lateral estimate disagrees with the
        current filter state by more than 3σ + 0.15 m the measurement is
        rejected as an outlier (wrong row, shadow, specular reflection).
        This prevents a single bad camera frame from dragging the fused
        estimate sideways while the LiDAR is between scans.
        """
        if vis_est is None or getattr(vis_est, "confidence", 0.0) < 0.05:
            return
        z = np.array([vis_est.lateral_offset, vis_est.heading_error])
        if self._initialised:
            gate = 3.0 * self.std_lateral + 0.15
            if abs(z[0] - self._x[0]) > gate:
                return  # outlier — skip this measurement
            hdg_gate = 3.0 * self.std_heading + math.radians(15.0)
            if abs(z[1] - self._x[1]) > hdg_gate:
                return  # heading outlier (PCA flip on round blob) — skip
        conf2 = max(vis_est.confidence ** 2, _CONF_EPS)
        R = self._R_cam_base / conf2
        self._kf_update(z, R)

    # ------------------------------------------------------------------
    def to_estimate(self, lidar_est: RowEstimate) -> RowEstimate:
        """Build a RowEstimate from the current filter state.

        Confidence is the max of the raw LiDAR confidence and an EKF-derived
        floor based on lateral uncertainty:
          ekf_conf = max(0, 1 − σ_lateral / 0.20)
          → 1.0 at σ=0 m, 0.5 at σ=0.10 m, 0.0 at σ≥0.20 m

        This prevents the controller from slowing down on alternating empty
        VLP-16 scans (where LiDAR confidence halves via EMA decay) when the
        EKF is well-converged (σ ≈ 0.04 m → ekf_conf ≈ 0.80).
        """
        if self._initialised:
            ekf_conf = max(0.0, 1.0 - self.std_lateral / 0.20)
            confidence = max(float(lidar_est.confidence), ekf_conf)
        else:
            confidence = float(lidar_est.confidence)
        return RowEstimate(
            heading_error=float(self._x[1]),
            lateral_offset=float(self._x[0]),
            confidence=confidence,
            row_end_confidence=float(lidar_est.row_end_confidence),
            n_points=int(lidar_est.n_points),
            valid=lidar_est.valid,
        )

    # ------------------------------------------------------------------
    # Accessors for diagnostics
    # ------------------------------------------------------------------

    @property
    def lateral_offset(self) -> float:
        return float(self._x[0])

    @property
    def heading_error(self) -> float:
        return float(self._x[1])

    @property
    def std_lateral(self) -> float:
        return float(math.sqrt(max(0.0, float(self._P[0, 0]))))

    @property
    def std_heading(self) -> float:
        return float(math.sqrt(max(0.0, float(self._P[1, 1]))))

    @property
    def converged(self) -> bool:
        """True once the filter has absorbed ≥2 real measurements and std_lateral < 0.08 m.

        With the VLP-16 alternating full/empty scan pattern, the first real scan drops
        std_lateral from 0.45 m to ~0.057 m; the second brings it to ~0.041 m.  After
        that, predict-only (empty) steps raise std by ≤0.015 m per scan so convergence
        persists through multiple consecutive empty scans.  This lets ACQUIRE complete
        reliably in ~0.3 s regardless of the alternating pattern.
        """
        return self._initialised and self._real_updates >= 2 and self.std_lateral < 0.08

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _kf_update(
        self,
        z: np.ndarray,
        R: np.ndarray,
        initialise_from: Optional[RowEstimate] = None,
    ) -> None:
        """Standard linear Kalman update with H = I."""
        self._real_updates += 1
        if not self._initialised:
            # Seed the filter with the first valid measurement.
            self._x = z.copy()
            self._P = R.copy()
            self._initialised = True
            return

        # Innovation
        innov = z - self._x
        S = self._P + R
        try:
            K = self._P @ np.linalg.solve(S, np.eye(_N))
        except np.linalg.LinAlgError:
            K = self._P @ np.linalg.pinv(S)

        self._x = self._x + K @ innov
        self._P = (np.eye(_N) - K) @ self._P

        # Ensure P stays symmetric and positive-definite.
        self._P = 0.5 * (self._P + self._P.T)
        min_var = 1e-6
        self._P[0, 0] = max(self._P[0, 0], min_var)
        self._P[1, 1] = max(self._P[1, 1], min_var)
