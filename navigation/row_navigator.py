#!/usr/bin/env python3
"""
row_navigator.py — Autonomous centre-row navigation state machine.

Ties together live LiDAR perception, obstacle safety, and pure-pursuit
control into a self-contained autonomy loop that needs NO pre-built map:

    ACQUIRE  -> lock onto the centre crop row while stationary
    FOLLOW   -> pure-pursuit drive along the row
    ROW_END  -> crop ahead has run out at the headland
    HEADLAND -> open-loop turn onto the next bed (optional, gated)
    OBSTACLE_WAIT -> something is in the path or a wheel track; hold

Safety overrides everything: any occupied zone forces OBSTACLE_WAIT and a
zero velocity command immediately.

Motion is gated by ``auto``.  In perception-only mode the full state
machine still runs and the command it *would* send is shown on the status
line, but no velocity is ever written to the canbus — the recommended way
to validate detection before letting the robot move.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import struct
import time
from typing import Optional

import numpy as np

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import tilt_correct_pts, yaw_correct_pts
from navigation.headland import HeadlandTurn
from navigation.state_logic import (
    follow_loss_is_row_end, follow_loss_action, approach_action,
    acquire_rowend_escape, post_turn_loss_action)
from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowDetector, RowEstimate
from navigation.row_safety import SafetyMonitor

# Optional enhanced modules — imported lazily to keep baseline import-clean.
try:
    from camera.depth_to_points import DepthToPoints as _DepthToPoints
except ImportError:
    _DepthToPoints = None  # type: ignore

try:
    from navigation.ekf_estimator import RowEKF as _RowEKF
except ImportError:
    _RowEKF = None  # type: ignore

try:
    from navigation.odometry import WheelOdometry as _WheelOdometry
except ImportError:
    _WheelOdometry = None  # type: ignore


def _write_shm_bgr(img: np.ndarray, path: str) -> None:
    """Write BGR numpy HxWx3 uint8 image to shm as: int32 H, int32 W, raw bytes."""
    try:
        h, w = img.shape[:2]
        raw = struct.pack("<ii", h, w) + img.astype(np.uint8).tobytes()
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(raw)
        os.rename(tmp, path)
    except OSError:
        pass


class _S:
    ACQUIRE = "ACQUIRE"
    FOLLOW = "FOLLOW"
    ROW_END = "ROW_END"
    OBSTACLE_WAIT = "OBSTACLE_WAIT"
    HEADLAND = "HEADLAND"
    APPROACH = "APPROACH"      # post-turn: drive into the next row until it locks
    DONE = "DONE"


# States that make up the headland manoeuvre — paused (not aborted) when the
# safety monitor trips, and resumed in place.  APPROACH is the final leg that
# drives the robot forward into the next row until perception re-acquires it.
_HEADLAND_STATES = (_S.HEADLAND, _S.APPROACH)


class RowNavigator:
    """Autonomous crop-row follower driven by a stream of VLP-16 scans."""

    def __init__(
        self,
        canbus: Optional[CanbusInterface],
        detector: RowDetector,
        safety: SafetyMonitor,
        controller: PurePursuitController,
        *,
        auto: bool = False,
        rows: int = 1,
        headland: bool = False,
        slam=None,
        self_radius: float = 1.5,
        acquire_conf: float = 0.35,
        acquire_frames: int = 5,
        row_end_conf: float = 0.70,
        row_end_frames: int = 15,
        row_end_min_dist: float = 1.5,
        obstacle_clear_secs: float = 1.5,
        row_end_green: float = 0.04,
        row_spacing: float = 0.76,
        headland_shift: float = 1.52,
        headland_exit_dist: float = 1.0,
        first_turn_sign: float = 1.0,
        headland_speed: float = 0.15,
        headland_turn_rate: float = 0.35,
        headland_radius: float = 0.0,
        approach_speed: float = 0.12,
        approach_max_dist: float = 3.0,
        approach_acquire_frames: int = 3,
        post_turn_max_dist: float = 5.0,
        heading_source=None,
        align_heading: bool = False,
        align_thresh: float = 0.14,
        align_rate: float = 0.20,
        max_align_frames: int = 20,
        left_cam=None,
        right_cam=None,
        vis_detector=None,
        depth_left=None,
        depth_right=None,
        depth_pts_left=None,
        depth_pts_right=None,
        ekf=None,
        odometry=None,
        tilt_rad: float = 0.0,
        yaw_rad: float = 0.0,
        cam_block_frames: int = 3,
        cam_self_radius: float = 2.0,
        ros2_bridge: bool = False,
        follow_miss_thresh: int = 4,
        acq_miss_thresh: int = 2,
        scan_timeout: float = 0.5,
        cam_stale_secs: float = 1.5,
        cam_fusion: str = "estimate",
    ) -> None:
        self.canbus = canbus
        self.detector = detector
        self.safety = safety
        self.controller = controller
        self.auto = auto
        self.rows = max(1, rows)
        self.headland = headland
        self.slam = slam
        self.self_radius = self_radius
        self.cam_self_radius = cam_self_radius
        self.left_cam = left_cam
        self.right_cam = right_cam
        self.vis_detector = vis_detector
        self.depth_left = depth_left
        self.depth_right = depth_right
        self.depth_pts_left = depth_pts_left
        self.depth_pts_right = depth_pts_right
        self.ekf = ekf
        self.odometry = odometry
        self.tilt_rad = tilt_rad
        self.yaw_rad = yaw_rad
        self.cam_block_frames = cam_block_frames
        self.ros2_bridge = ros2_bridge
        self.follow_miss_thresh = follow_miss_thresh
        self.acq_miss_thresh = acq_miss_thresh
        self.scan_timeout = scan_timeout
        self.cam_stale_secs = cam_stale_secs
        self.cam_fusion = cam_fusion if cam_fusion in ("estimate", "point") else "estimate"

        self.acquire_conf = acquire_conf
        self.acquire_frames = acquire_frames
        self.row_end_conf = row_end_conf
        self.row_end_frames = row_end_frames
        self.row_end_min_dist = row_end_min_dist
        self.obstacle_clear_secs = obstacle_clear_secs
        self.row_end_green = row_end_green
        self.row_spacing = row_spacing
        self.headland_exit_dist = headland_exit_dist
        self.approach_speed = approach_speed
        self.approach_max_dist = approach_max_dist
        self.approach_acquire_frames = approach_acquire_frames
        self.post_turn_max_dist = post_turn_max_dist
        self._approach_dist = 0.0
        self.first_turn_sign = 1.0 if first_turn_sign >= 0 else -1.0
        self.headland_speed = headland_speed
        self.headland_turn_rate = headland_turn_rate

        # Closed-loop headland U-turn driver (odometry feedback).  Built only
        # when an odometry source is available; otherwise headland turns are
        # disabled and ROW_END goes straight to DONE.
        # SHIFT distance is the centre-to-centre distance to the NEXT strip the
        # robot straddles (headland_shift, ~1.52 m) — distinct from the
        # detector's row_spacing (0.76 m, the in-strip soybean-row separation).
        self._headland_turn = None
        if self.odometry is not None:
            self._headland_turn = HeadlandTurn(
                self.odometry,
                row_spacing=headland_shift,
                exit_dist=headland_exit_dist,
                speed=headland_speed,
                turn_rate=headland_turn_rate,
                turn_radius=(headland_radius if headland_radius > 0.0 else None),
                heading_source=heading_source,
            )
        self.align_heading = align_heading
        self.align_thresh = align_thresh
        self.align_rate = align_rate
        self.max_align_frames = max_align_frames

        self.state = _S.ACQUIRE
        self.cmd = (0.0, 0.0)
        self._stop = False
        self._acq_count = 0
        self._row_end_count = 0
        self._row_dist = 0.0
        self._rows_done = 0
        self._obstacle_clear_t: Optional[float] = None
        self._cam_block_count: int = 0
        self._cam_cloud_buf: list = []   # rolling 3-frame camera depth buffer
        self._follow_miss_count: int = 0  # consecutive low-conf scans while in FOLLOW
        self._acq_miss_count: int = 0     # consecutive low-conf scans while in ACQUIRE
        self._came_from_follow: bool = False  # ACQUIRE entered from FOLLOW (row-end escape)
        self._acq_rowend_count: int = 0   # consecutive empty-crop scans while in ACQUIRE
        self._post_turn: bool = False     # settling onto the next row after a headland turn
        self._post_turn_settle_dist: float = 2.0  # row_dist in FOLLOW that ends post-turn settling
        self._post_turn_dist: float = 0.0  # cumulative travel since the turn (settling guard)
        self._headland_abort_count: int = 0  # consecutive solid-crop frames during EXIT
        self.headland_abort_frames: int = 3  # frames of re-found crop to abort a turn
        self._last_green: Optional[float] = None  # latest camera green fraction (row-end cross-check)
        self._turn_sign = 1.0             # last headland turn direction (for status)
        self._t_prev = time.monotonic()

    # ------------------------------------------------------------------
    def _fuse_estimates(self, lidar_est: RowEstimate, vis_est) -> RowEstimate:
        """Weighted blend of LiDAR and visual estimates; camera capped at 50% of LiDAR weight."""
        lidar_conf = lidar_est.confidence
        vis_conf = vis_est.confidence if vis_est is not None else 0.0

        if vis_conf <= 0.0 or lidar_conf <= 0.0:
            return lidar_est

        # Camera weight capped at half the LiDAR weight.
        cam_w = min(vis_conf, lidar_conf * 0.5)
        total_w = lidar_conf + cam_w
        lidar_w = lidar_conf / total_w
        cam_w_norm = cam_w / total_w

        fused_offset = lidar_w * lidar_est.lateral_offset + cam_w_norm * vis_est.lateral_offset
        fused_heading = (lidar_w * lidar_est.heading_error
                         + cam_w_norm * vis_est.heading_error)
        # Confidence boost is scaled by cross-sensor agreement: two sensors
        # that AGREE corroborate each other; a camera that disagrees with the
        # LiDAR by 0.30 m should not make the fused estimate MORE confident.
        agreement = max(0.0, 1.0 - abs(vis_est.lateral_offset
                                       - lidar_est.lateral_offset) / 0.30)
        fused_conf = min(1.0, lidar_conf + vis_conf * 0.3 * agreement)

        return RowEstimate(
            heading_error=fused_heading,
            lateral_offset=fused_offset,
            confidence=fused_conf,
            row_end_confidence=lidar_est.row_end_confidence,
            n_points=lidar_est.n_points,
            valid=lidar_est.valid,
        )

    # ------------------------------------------------------------------
    async def run(self, lidar: LidarDriver) -> None:
        """Consume the LiDAR scan stream and drive the state machine.

        A scan-stall watchdog guards the loop: if no scan arrives within
        ``scan_timeout`` seconds (cable pulled, power loss, network drop) the
        navigator actively commands zero velocity while it waits, instead of
        silently blocking with the last non-zero twist still in effect.
        """
        self._t_prev = time.monotonic()
        stream = lidar.scan_stream_np()
        next_scan = None       # pending __anext__ task (survives watchdog timeouts)
        stall_warned = False
        while not self._stop:
            if next_scan is None:
                next_scan = asyncio.ensure_future(stream.__anext__())
            done, _pending = await asyncio.wait({next_scan}, timeout=self.scan_timeout)
            if not done:
                if not stall_warned:
                    stall_warned = True
                    print(f"\n[navigator] LiDAR stall — no scan for "
                          f"{self.scan_timeout:.1f}s; holding zero velocity")
                self.cmd = (0.0, 0.0)
                await self._drive(0.0, 0.0)
                continue
            task, next_scan = next_scan, None
            try:
                pts = task.result()
            except StopAsyncIteration:
                break
            if stall_warned:
                stall_warned = False
                print("\n[navigator] LiDAR stream recovered")

            now = time.monotonic()
            dt = min(max(now - self._t_prev, 1e-3), 0.5)
            self._t_prev = now

            # Drop the robot's own frame/crossbar: the centre-mounted LiDAR
            # sees its mounting structure as a dense return at ~0.5 m.
            # Without this it permanently trips the safety zones and the
            # robot never leaves OBSTACLE_WAIT.
            if len(pts):
                rng = np.hypot(pts[:, 0], pts[:, 1])
                pts = pts[rng >= self.self_radius]
                # Order matters: yaw FIRST, then tilt.  The yaw aligns the cloud
                # to the robot frame so the subsequent tilt rotates about the
                # robot's left-right (X) axis — i.e. corrects a true nose-down
                # PITCH that flattens the forward ground ramp.  Applying tilt
                # first would rotate about the un-yawed sensor X axis (71° off),
                # leaving a >1 m residual ground ramp (see
                # test_correction_order_matters / scratchpad verify_order.py).
                # Must match the order in scripts/diag_birdseye.py.
                if self.yaw_rad != 0.0:
                    pts = yaw_correct_pts(pts, self.yaw_rad)
                if self.tilt_rad != 0.0:
                    pts = tilt_correct_pts(pts, self.tilt_rad)

            # --- Camera integration (runs BEFORE the LiDAR fit so that
            # point-level fusion can feed camera ground points into it) ---
            vis_est = None
            frame_l = frame_r = dep_l = dep_r = None
            if self.vis_detector is not None or self.depth_pts_left is not None or self.depth_pts_right is not None:
                frame_l = self.left_cam.get_latest() if self.left_cam is not None else None
                frame_r = self.right_cam.get_latest() if self.right_cam is not None else None
                # Staleness gate: a camera whose stream died keeps returning
                # its last frame forever.  Fusing a frozen frame holds the
                # visual lateral estimate and green fraction constant — the
                # green-fraction row-end veto in particular would then block
                # ROW_END indefinitely.  Treat old frames as "no camera".
                if (frame_l is not None
                        and now - frame_l.timestamp > self.cam_stale_secs):
                    frame_l = None
                if (frame_r is not None
                        and now - frame_r.timestamp > self.cam_stale_secs):
                    frame_r = None
                dep_l = frame_l.depth if frame_l is not None else None
                dep_r = frame_r.depth if frame_r is not None else None
                rgb_l = frame_l.rgb if frame_l is not None else None
                rgb_r = frame_r.rgb if frame_r is not None else None

                if self.vis_detector is not None:
                    vis_est = self.vis_detector.update(rgb_l, dep_l, rgb_r, dep_r)

            # --- Row fit: LiDAR-only, or pooled LiDAR+camera point fit ---
            # cam_fusion == "point": the tracker's metric ground points join
            # the LiDAR crop points in ONE weighted fit (single histogram /
            # peak pairing / PCA over all evidence).  Camera points fill
            # empty VLP-16 crop-ROI scans and the < 1.5 m self-filter blind
            # zone; their total mass is capped so a healthy LiDAR scan
            # always dominates.  The camera estimate is then NOT fused again
            # at the estimate level (that would double-count it).
            aux_xy = None
            if self.cam_fusion == "point" and self.vis_detector is not None:
                aux_xy = getattr(self.vis_detector, "last_ground_points", None)
            est = self.detector.update(pts, aux_xy=aux_xy)

            # --- 3-D camera depth cloud: fill LiDAR blind zone ---
            # Projects depth images to 3D, merges with LiDAR cloud so that
            # SafetyMonitor.check() sees obstacles the LiDAR misses at < 1.5 m.
            cam_cloud = np.empty((0, 3), dtype=np.float32)  # current frame, for bridge
            if self.depth_pts_left is not None or self.depth_pts_right is not None:
                cam_frames = []
                if self.depth_pts_left is not None and dep_l is not None:
                    cl = self.depth_pts_left.project(dep_l)
                    if len(cl):
                        cam_frames.append(cl)
                if self.depth_pts_right is not None and dep_r is not None:
                    cr = self.depth_pts_right.project(dep_r)
                    if len(cr):
                        cam_frames.append(cr)
                if cam_frames:
                    cam_cloud = np.vstack(cam_frames)

            # Camera cloud self-filter: remove points within the robot's own
            # body envelope before the safety check.  Side-mounted cameras look
            # inward at 35° and see the robot's own gantry / delta arm overhead
            # at y ≈ 0.5–1.2 m, h ≈ 0.8–1.2 m — exactly matching the forward
            # and tire-zone obstacle criteria.  Applying the same planar-range
            # filter as LiDAR ensures the camera cloud only extends coverage
            # beyond the self-filter radius and never reports the robot itself.
            if len(cam_cloud) and self.cam_self_radius > 0:
                _rng = np.hypot(cam_cloud[:, 0], cam_cloud[:, 1])
                cam_cloud = cam_cloud[_rng >= self.cam_self_radius]

            # Debounce: accumulate camera cloud over a rolling 3-frame window.
            # A single noisy depth frame can't trigger OBSTACLE_WAIT on its own
            # — it must produce ≥ min_points across 3 consecutive scans.
            # Mirrors cam_block_frames=3 from the legacy strip-based detector.
            if len(cam_cloud):
                self._cam_cloud_buf.append(cam_cloud)
                if len(self._cam_cloud_buf) > 3:
                    self._cam_cloud_buf.pop(0)
            elif self._cam_cloud_buf:
                self._cam_cloud_buf.pop(0)

            cam_stable = (np.vstack(self._cam_cloud_buf)
                          if self._cam_cloud_buf else np.empty((0, 3), dtype=np.float32))
            safety_pts = pts
            if len(cam_stable):
                safety_pts = np.vstack([pts, cam_stable]) if len(pts) else cam_stable

            safety = self.safety.check(safety_pts)

            # --- Legacy strip-based camera obstacle detection ---
            # Only runs when old DepthObstacleDetector objects are provided AND
            # the new 3D path is NOT active (backward compatibility).
            if (self.depth_pts_left is None and self.depth_pts_right is None
                    and self.vis_detector is not None):
                cam_raw_blocked = False
                cam_raw_reason = ""
                if self.depth_left is not None:
                    ds_l = self.depth_left.check(dep_l, "left")
                    if ds_l.blocked:
                        cam_raw_blocked = True
                        cam_raw_reason = ds_l.reason()
                if self.depth_right is not None:
                    ds_r = self.depth_right.check(dep_r, "right")
                    if ds_r.blocked:
                        cam_raw_blocked = True
                        reason_r = ds_r.reason()
                        cam_raw_reason = (cam_raw_reason + "," + reason_r
                                          if cam_raw_reason else reason_r)
                if cam_raw_blocked:
                    self._cam_block_count += 1
                else:
                    self._cam_block_count = 0
                if self._cam_block_count >= self.cam_block_frames:
                    safety.cam_blocked = True
                    safety.cam_reason = cam_raw_reason

            # Camera green fraction — corroborates LiDAR row-end so a single
            # sparse VLP-16 scan mid-row cannot trigger a false ROW_END while
            # the crop is still clearly visible to the cameras.
            self._last_green = (float(getattr(vis_est, "green_fraction", 0.0))
                                if vis_est is not None else None)

            # --- Sensor fusion: EKF or weighted average ---
            # With point-level fusion the camera evidence is already inside
            # `est`; feeding vis_est in again here would double-count it.
            if self.ekf is not None:
                prev_v, prev_w = self.cmd   # commanded last scan — best motion estimate
                self.ekf.predict(linear_vel=prev_v, angular_vel=prev_w, dt=dt)
                self.ekf.update_lidar(est)
                if vis_est is not None and self.cam_fusion != "point":
                    self.ekf.update_camera(vis_est)
                est = self.ekf.to_estimate(est)
            elif vis_est is not None and self.cam_fusion != "point":
                est = self._fuse_estimates(est, vis_est)

            linear, angular = self._step(est, safety, dt)
            self.cmd = (linear, angular)

            # Centralised odometry integration: one tick per scan with the
            # command just issued.  FOLLOW accrues row distance from the
            # measured (or commanded-fallback) odometry delta; HEADLAND turns
            # read odometry directly for their closed-loop feedback.
            if self.odometry is not None:
                d_before = self.odometry.distance
                self.odometry.tick(linear, angular, dt)
                step = max(0.0, self.odometry.distance - d_before)
            else:
                step = max(0.0, linear * dt)
            if self.state == _S.FOLLOW:
                self._row_dist += step
            elif self.state == _S.APPROACH:
                self._approach_dist += step
            # Cumulative travel since the U-turn until a stable down-row FOLLOW
            # is established — counts BOTH the APPROACH creep and any short,
            # un-settled FOLLOW segments.  Bounds how far the robot may search
            # for the next row before giving up, so it can never drive off the
            # end of the field hunting for a row that isn't there.
            if self._post_turn and self.state in (_S.FOLLOW, _S.APPROACH):
                self._post_turn_dist += step

            await self._drive(linear, angular)
            self._print_status(est, safety, linear, angular)
            if self.ros2_bridge:
                img_l = getattr(frame_l, "rgb", None) if frame_l is not None else None
                img_r = getattr(frame_r, "rgb", None) if frame_r is not None else None
                self._write_bridge(pts, safety_pts, cam_cloud, est, safety, linear, angular, img_l, img_r)

            if self.state == _S.DONE:
                break

        if next_scan is not None:
            next_scan.cancel()
        await self.stop()

    # ------------------------------------------------------------------
    def _step(self, est, safety, dt: float) -> tuple[float, float]:
        """Advance the state machine one scan; return (linear, angular)."""
        st = self.state

        # --- Safety override — only interrupt active motion states ---
        # ACQUIRE: robot is always stationary (cmd=0 always); entering OBSTACLE_WAIT
        # while stationary gains nothing and creates spurious halt cycles from
        # intermittent depth noise or near-field camera returns.
        # HEADLAND states: pause-in-place rather than abandon the manoeuvre.
        if st in (_S.FOLLOW,) or st in _HEADLAND_STATES:
            if safety.blocked:
                if st in _HEADLAND_STATES:
                    return 0.0, 0.0
                self._enter(_S.OBSTACLE_WAIT)
                self._obstacle_clear_t = None
                return 0.0, 0.0

        if st == _S.ACQUIRE:
            return self._step_acquire(est)
        if st == _S.FOLLOW:
            return self._step_follow(est, dt)
        if st == _S.OBSTACLE_WAIT:
            return self._step_obstacle(safety, est)
        if st == _S.ROW_END:
            return self._step_row_end()
        if st == _S.HEADLAND:
            return self._step_headland(dt, est)
        if st == _S.APPROACH:
            return self._step_approach(est)
        return 0.0, 0.0   # DONE

    # ------------------------------------------------------------------
    def _row_end_reached(self, est) -> bool:
        """True when the crop has run out ahead.

        Requires the LiDAR crop-band to be sparse (high row_end_confidence) and,
        when cameras are active, the green fraction to also be low — so a single
        sparse VLP-16 scan mid-row cannot trigger a false ROW_END while soybean
        foliage is still clearly in view.
        """
        if self._row_dist < self.row_end_min_dist:
            return False
        if est.row_end_confidence < self.row_end_conf:
            return False
        if self._last_green is not None and self._last_green >= self.row_end_green:
            return False
        return True

    # ------------------------------------------------------------------
    def _step_acquire(self, est) -> tuple[float, float]:
        # Row-end escape: if we dropped to ACQUIRE from FOLLOW (we were on a
        # row) and the crop band ahead is now genuinely empty for a sustained
        # period, the row has ENDED — go to ROW_END (→ headland) instead of
        # hunting forever for a row that isn't there.  Without this, a row end
        # with residual sparse clutter (which keeps the FOLLOW-exit row-end
        # check from tripping) leaves the robot stuck in ACQUIRE at the field
        # edge.  Uses the same continuous-absence requirement as a real row end.
        if self._came_from_follow:
            if est.row_end_confidence >= self.row_end_conf:
                self._acq_rowend_count += 1
            else:
                self._acq_rowend_count = 0
            if acquire_rowend_escape(self._came_from_follow,
                                     self._acq_rowend_count,
                                     row_end_frames=self.row_end_frames):
                self._enter(_S.ROW_END)
                return 0.0, 0.0

        if self.ekf is not None:
            # EKF-aware: once the filter absorbs ≥2 real measurements its
            # std_lateral drops to ~0.04 m and stays below 0.08 m through
            # predict-only (empty-scan) steps.  Raw confidence oscillates
            # 0.70→0.35 on alternating full/empty VLP-16 scans, making the
            # old consecutive-frame counter reset indefinitely.
            frame_ok = self.ekf.converged
        else:
            frame_ok = est.confidence >= self.acquire_conf

        # Debounce: same principle as follow_miss_thresh. A single empty scan
        # decays EMA confidence below acquire_conf and would reset the 5-frame
        # counter that was at acq=4/5. Require acq_miss_thresh consecutive bad
        # scans before resetting, so one intermittent empty scan is absorbed
        # without losing accumulated progress.
        if frame_ok:
            self._acq_miss_count = 0
            self._acq_count += 1
        else:
            self._acq_miss_count += 1
            if self._acq_miss_count >= self.acq_miss_thresh:
                self._acq_count = 0
                self._acq_miss_count = 0
        if self._acq_count >= self.acquire_frames:
            align_frames = self._acq_count - self.acquire_frames
            # Pre-align: rotate in-place to reduce heading error before FOLLOW.
            # Timeout after max_align_frames — if the heading doesn't converge
            # (e.g. curved row, EKF measurement pulling heading back each scan),
            # enter FOLLOW anyway rather than blocking indefinitely.
            if (self.align_heading
                    and abs(est.heading_error) > self.align_thresh
                    and align_frames < self.max_align_frames):
                rate = self.align_rate
                w = max(-rate, min(rate, -est.heading_error * 2.0))
                return 0.0, w
            if align_frames >= self.max_align_frames and abs(est.heading_error) > self.align_thresh:
                print(f"\n[navigator] heading pre-align timeout "
                      f"(hdg={math.degrees(est.heading_error):+.1f}°) — entering FOLLOW")
            self._enter(_S.FOLLOW)
            self._row_dist = 0.0
            self._row_end_count = 0
        return 0.0, 0.0

    def _step_follow(self, est, dt: float) -> tuple[float, float]:
        # Post-turn settling: once we have driven a solid distance down the new
        # row, the lock is established — stop treating early losses specially.
        if self._post_turn and self._row_dist >= self._post_turn_settle_dist:
            self._post_turn = False

        if self._row_end_reached(est):
            self._row_end_count += 1
        else:
            self._row_end_count = 0
        if self._row_end_count >= self.row_end_frames:
            self._enter(_S.ROW_END)
            return 0.0, 0.0

        # Debounce low-confidence scans: the VLP-16 routinely produces 1–3
        # empty crop-ROI scans per second due to beam-angle variation between
        # 10 Hz rotations.  A single empty scan decays EMA confidence by ×0.75
        # per _decay(); dropping to ACQUIRE immediately causes rapid
        # FOLLOW→ACQUIRE→FOLLOW cycling (seen in warehouse cardboard tests).
        #
        # Policy: tolerate up to follow_miss_thresh consecutive sub-threshold
        # scans by stopping motion (v=0, ω=0) and waiting; only abort to
        # ACQUIRE when the gap is long enough to indicate genuine row loss.
        # EKF path: same counter but with a lower effective threshold when the
        # filter uncertainty is small.
        min_conf = self.controller.min_confidence
        if self.ekf is not None and self.ekf.std_lateral < 0.08:
            min_conf = max(0.12, min_conf - 0.20)

        if est.confidence < min_conf:
            self._follow_miss_count += 1
            # Crop running out trips BOTH the row-end signal and the miss
            # counter at once, so the loss must be classified.  A REAL row end
            # needs a long CONTINUOUS crop absence (row_end_frames); a brief
            # dropout on a slope keeps reappearing and resets the counter, so it
            # can never reach that and fake a headland turn.  A non-row-end loss
            # (crop still partly present) re-acquires after follow_miss_thresh.
            is_end = follow_loss_is_row_end(
                self._row_dist, est.row_end_confidence,
                row_end_min_dist=self.row_end_min_dist,
                row_end_conf=self.row_end_conf)
            action = follow_loss_action(
                self._follow_miss_count, is_end,
                row_end_frames=self.row_end_frames,
                follow_miss_thresh=self.follow_miss_thresh)
            # Still settling onto the row just after a headland turn?  A
            # marginal early loss should keep creeping forward (APPROACH), not
            # stall in a stationary ACQUIRE that can never improve a sparse,
            # half-in-ROI view of the new row.
            action = post_turn_loss_action(action, self._post_turn, is_end)
            if action == "ROW_END":
                self._follow_miss_count = 0
                self._enter(_S.ROW_END)
                return 0.0, 0.0
            if action == "APPROACH":
                self._follow_miss_count = 0
                self._acq_count = 0
                self._enter(_S.APPROACH)
                return 0.0, 0.0
            if action == "ACQUIRE":
                self._follow_miss_count = 0
                self._enter(_S.ACQUIRE)
                self._acq_count = 0
                return 0.0, 0.0
            # WAIT: pause in FOLLOW (v=0), wait for the row to reappear.
            return 0.0, 0.0
        self._follow_miss_count = 0

        # Row distance is integrated centrally in run() from odometry.
        return self.controller.compute(est)

    def _step_obstacle(self, safety, est) -> tuple[float, float]:
        # If the crop has ended while waiting for the obstacle to clear,
        # transition to ROW_END.  This handles the common end-of-row case
        # where the row terminates at a wall: the obstacle will never clear,
        # but the crop has already disappeared — the row IS done.
        if self._row_end_reached(est):
            self._row_end_count += 1
        else:
            self._row_end_count = 0
        if self._row_end_count >= self.row_end_frames:
            self._enter(_S.ROW_END)
            return 0.0, 0.0

        if safety.blocked:
            self._obstacle_clear_t = None
        else:
            if self._obstacle_clear_t is None:
                self._obstacle_clear_t = time.monotonic()
            elif time.monotonic() - self._obstacle_clear_t >= self.obstacle_clear_secs:
                self._enter(_S.ACQUIRE)
                self._acq_count = 0
        return 0.0, 0.0

    def _step_row_end(self) -> tuple[float, float]:
        self._rows_done += 1
        if (self.headland and self._rows_done < self.rows
                and self._headland_turn is not None):
            # Serpentine coverage: alternate turn direction each row, starting
            # from first_turn_sign (+1 = right).  rows_done is now 1 after the
            # first row → odd → first_turn_sign; 2 → even → opposite; …
            self._turn_sign = self.first_turn_sign * (
                1.0 if self._rows_done % 2 == 1 else -1.0)
            self._headland_turn.begin(self._turn_sign)
            self._headland_abort_count = 0
            self._enter(_S.HEADLAND)
        else:
            self._enter(_S.DONE)
        return 0.0, 0.0

    def _step_headland(self, dt: float, est=None) -> tuple[float, float]:
        """Closed-loop headland U-turn (odometry feedback via HeadlandTurn)."""
        if self._headland_turn is None:
            self._enter(_S.DONE)
            return 0.0, 0.0

        # Blind-spot guard: the VLP-16 is blind inside ~1.5 m, so a brief sparse
        # patch (or the last plants sitting in the near zone) can read as a row
        # end.  The turn opens with a straight EXIT leg — drive that little bit
        # of extra distance forward and, if solid crop comes back into the ROI,
        # the row had NOT actually ended: abort the turn and resume FOLLOW.
        # Only during EXIT (before any rotation), so an abort never leaves the
        # robot half-turned.
        if (est is not None and self._headland_turn.phase == "EXIT"
                and est.valid and est.confidence >= self.acquire_conf):
            self._headland_abort_count += 1
            if self._headland_abort_count >= self.headland_abort_frames:
                print(f"\n[navigator] row end was a LiDAR blind-spot gap — crop "
                      f"reappeared during EXIT; resuming FOLLOW.")
                self._headland_abort_count = 0
                self._rows_done = max(0, self._rows_done - 1)  # undo the row-end count
                self._enter(_S.FOLLOW)
                self._row_end_count = 0
                self._follow_miss_count = 0
                return self.controller.compute(est)
        else:
            self._headland_abort_count = 0

        linear, angular = self._headland_turn.step(dt)
        if self._headland_turn.done:
            # The turn ends at the headland margin with no crop immediately
            # ahead, so a stationary ACQUIRE here would hang forever (field
            # failure: robot stuck after the U-turn).  Instead drive forward
            # into the next row (APPROACH) until perception re-acquires it.
            self._acq_count = 0
            self._approach_dist = 0.0
            # Settling window: the next-row lock is marginal (partly in-ROI)
            # until the robot has driven a solid distance down it.  While this
            # is set, an early FOLLOW loss keeps creeping forward (back to
            # APPROACH) instead of stalling in a stationary ACQUIRE.
            self._post_turn = True
            self._post_turn_dist = 0.0
            self._enter(_S.APPROACH)
            return 0.0, 0.0
        return linear, angular

    def _step_approach(self, est) -> tuple[float, float]:
        """Drive forward into the next row until perception locks on.

        After the U-turn the robot sits at the headland margin pointing down
        the next row but with no crop yet in the ROI.  Creep straight forward
        (the closed-loop turn left it roughly aligned) and hand off to FOLLOW
        as soon as the row is solidly detected; FOLLOW's pure-pursuit then
        corrects any residual lateral error.  Bounded by approach_max_dist so a
        genuinely missing next row (field edge / overshoot) stops the robot
        rather than driving on forever.
        """
        # Cumulative settling guard: if the robot has travelled the whole
        # post-turn budget (APPROACH creep + any short FOLLOW segments) without
        # establishing a stable down-row FOLLOW, the next row isn't there —
        # stop rather than keep driving off the end of the field.
        if self._post_turn and self._post_turn_dist >= self.post_turn_max_dist:
            print(f"\n[navigator] post-turn: no stable row within "
                  f"{self.post_turn_max_dist:.1f} m of the U-turn — stopping "
                  f"(field edge / turn overshoot).")
            self._enter(_S.DONE)
            return 0.0, 0.0
        if est.confidence >= self.acquire_conf and est.valid:
            self._acq_count += 1
        else:
            self._acq_count = 0
        action = approach_action(
            self._acq_count, self._approach_dist,
            approach_acquire_frames=self.approach_acquire_frames,
            approach_max_dist=self.approach_max_dist)
        if action == "FOLLOW":
            self._enter(_S.FOLLOW)
            self._row_dist = 0.0
            self._row_end_count = 0
            self._follow_miss_count = 0
            return 0.0, 0.0
        if action == "STOP":
            print(f"\n[navigator] APPROACH: no next row within "
                  f"{self.approach_max_dist:.1f} m — stopping (field edge?).")
            self._enter(_S.DONE)
            return 0.0, 0.0
        return self.approach_speed, 0.0

    # ------------------------------------------------------------------
    def _enter(self, state: str) -> None:
        if state != self.state:
            prev_state = self.state
            if state == _S.FOLLOW:
                self._follow_miss_count = 0
                est_hdg = ""
                if self.ekf is not None:
                    est_hdg = f" (hdg={math.degrees(self.ekf._x[1]):.1f}°)" if hasattr(self.ekf, '_x') else ""
                print(f"\n[navigator] {prev_state} -> {state}{est_hdg}")
            else:
                print(f"\n[navigator] {prev_state} -> {state}")
            if state == _S.ACQUIRE:
                # Remember whether we arrived here by LOSING a row we were
                # following (vs starting up / post-turn): only then may ACQUIRE
                # escape to ROW_END on a sustained empty crop band.
                self._came_from_follow = (prev_state == _S.FOLLOW)
                self._acq_rowend_count = 0
            if state == _S.ACQUIRE and self.ekf is not None:
                # Don't reset EKF when resuming from OBSTACLE_WAIT — the robot
                # was stationary, so the EKF state (heading, lateral) is still
                # valid.  Resetting here caused the next sparse scan to corrupt
                # the heading estimate (high post-reset uncertainty accepted a
                # noisy +31° measurement, sending the robot into a spiral).
                if prev_state != _S.OBSTACLE_WAIT:
                    self.ekf.reset()
            if state == _S.APPROACH and prev_state == _S.HEADLAND:
                # New row: the smoothed estimates accumulated while the ROI
                # swept across the headland are garbage; without a reset their
                # outlier gates fight the first detections of the next row.
                if self.ekf is not None:
                    self.ekf.reset()
                if hasattr(self.detector, "reset"):
                    self.detector.reset()
                if self.vis_detector is not None and hasattr(self.vis_detector, "reset"):
                    self.vis_detector.reset()
        self.state = state

    async def _drive(self, linear: float, angular: float) -> None:
        """Send the command to the canbus, unless in perception-only mode."""
        if self.canbus is None or not self.auto:
            return
        try:
            await self.canbus.send_twist(linear, angular)
        except asyncio.TimeoutError:
            pass  # canbus not responding yet; will retry next cycle
        except Exception as exc:
            print(f"\n[navigator] twist send failed: {exc}")

    async def stop(self) -> None:
        """Halt the robot and end the run loop."""
        self._stop = True
        if self.canbus is not None:
            try:
                await self.canbus.stop()
            except Exception:
                pass

    # ------------------------------------------------------------------
    def _write_bridge(
        self,
        lidar_pts: np.ndarray,
        fused_pts: np.ndarray,
        cam_pts: np.ndarray,
        est,
        safety,
        linear: float,
        angular: float,
        img_left: Optional[np.ndarray] = None,
        img_right: Optional[np.ndarray] = None,
    ) -> None:
        """Write scan + nav status to /dev/shm/ for the ROS2 bridge container.

        lidar_pts : LiDAR-only cloud (self-filtered, tilt-corrected).
        fused_pts : LiDAR + camera merged — what SafetyMonitor used.
        cam_pts   : camera-only 3D cloud — shown as orange /camera_points.
        img_left  : left OAK-D BGR image (HxWx3 uint8) or None.
        img_right : right OAK-D BGR image (HxWx3 uint8) or None.
        """
        try:
            def _write_pts(arr: np.ndarray, path: str) -> None:
                f32 = arr.astype(np.float32)
                raw = struct.pack("<i", len(f32)) + (f32.tobytes() if len(f32) else b"")
                tmp = path + ".tmp"
                with open(tmp, "wb") as fp:
                    fp.write(raw)
                os.rename(tmp, path)

            _write_pts(lidar_pts, "/dev/shm/vidalia_lidar_pts.bin")
            _write_pts(fused_pts, "/dev/shm/vidalia_pts.bin")
            _write_pts(cam_pts,   "/dev/shm/vidalia_cam_pts.bin")

            # --- camera images (BGR HxWx3 uint8) ---
            if img_left is not None and img_left.ndim == 3:
                _write_shm_bgr(img_left,  "/dev/shm/vidalia_img_left.bin")
            if img_right is not None and img_right.ndim == 3:
                _write_shm_bgr(img_right, "/dev/shm/vidalia_img_right.bin")

            # --- navigation status (JSON) ---
            status = {
                "state":              self.state,
                "heading_error":      float(est.heading_error),
                "lateral_offset":     float(est.lateral_offset),
                "confidence":         float(est.confidence),
                "row_end_confidence": float(est.row_end_confidence),
                "n_points":           int(est.n_points),
                "linear_vel":         float(linear),
                "angular_vel":        float(angular),
                "forward_blocked":    bool(safety.forward_blocked),
                "left_tire_blocked":  bool(safety.left_tire_blocked),
                "right_tire_blocked": bool(safety.right_tire_blocked),
                "cam_blocked":        bool(safety.cam_blocked),
                "nearest_forward":    float(safety.nearest_forward)
                                      if safety.nearest_forward < 99.0 else 99.0,
                "rows_done":          int(self._rows_done),
                "rows_total":         int(self.rows),
                "row_dist":           float(self._row_dist),
                "acq_count":          int(self._acq_count),
                "acquire_frames":     int(self.acquire_frames),
                "ekf_std_lat":        float(self.ekf.std_lateral) if self.ekf is not None else 0.0,
                "ekf_std_hdg":        float(self.ekf.std_heading) if self.ekf is not None else 0.0,
                "ekf_converged":      bool(self.ekf.converged)    if self.ekf is not None else False,
                "ts":                 time.monotonic(),
            }
            tmp2 = "/dev/shm/vidalia_status_tmp.json"
            with open(tmp2, "w") as f:
                json.dump(status, f)
            os.rename(tmp2, "/dev/shm/vidalia_status.json")
        except OSError:
            pass  # /dev/shm not available or full — bridge is optional

    def _print_status(self, est, safety, linear: float, angular: float) -> None:
        cam_active = self.vis_detector is not None
        depth3d = self.depth_pts_left is not None or self.depth_pts_right is not None
        ekf_active = self.ekf is not None
        suffix = ""
        if cam_active:
            suffix += "+CAMpt" if self.cam_fusion == "point" else "+CAM"
        if depth3d:
            suffix += "+3D"
        if ekf_active:
            suffix += "+EKF"
        mode = ("AUTO" if self.auto else "PERC") + suffix
        deg = math.degrees(est.heading_error)
        ekf_str = ""
        if ekf_active and self.ekf is not None:
            ekf_str = f" σ={self.ekf.std_lateral:.3f}"
        # Show the live ground grade being detrended (only when non-zero) so a
        # field operator can confirm the terrain-adaptive band is engaging.
        grade_str = ""
        _gs = getattr(self.detector, "last_ground_slope", 0.0)
        if _gs:
            grade_str += f" grade={math.degrees(math.atan(_gs)):+.0f}°"
        _sh = getattr(self.detector, "last_ground_shift", 0.0)
        if _sh:
            grade_str += f" drop={_sh:+.2f}m"
        if self.state == _S.ACQUIRE:
            align_frames = self._acq_count - self.acquire_frames
            if align_frames > 0:
                acq_str = (f" ALIGN {align_frames}/{self.max_align_frames}"
                           f"({math.degrees(est.heading_error):+.1f}°)")
            else:
                acq_str = f" acq={self._acq_count}/{self.acquire_frames}"
        elif self.state == _S.HEADLAND and self._headland_turn is not None:
            turn_dir = "R" if self._headland_turn.turn_sign >= 0 else "L"
            acq_str = (f" {turn_dir}-UTURN:{self._headland_turn.phase}"
                       f"[{self._headland_turn.heading_source_name}]")
        elif self.state == _S.APPROACH:
            tag = "SETTLE" if self._post_turn else "ENTER"
            acq_str = (f" {tag} {self._approach_dist:.1f}/{self.approach_max_dist:.1f}m"
                       f" acq={self._acq_count}/{self.approach_acquire_frames}")
        else:
            acq_str = ""
        line = (
            f"\r[{mode}] {self.state:11s} | "
            f"hdg={deg:+5.1f}° off={est.lateral_offset:+5.2f}m "
            f"conf={est.confidence:.2f} end={est.row_end_confidence:.2f} "
            f"n={est.n_points:4d}{ekf_str}{grade_str}{acq_str} | "
            f"row={self._rows_done}/{self.rows} d={self._row_dist:4.1f}m | "
            f"safe={safety.reason():14s} | "
            f"cmd v={linear:+.2f} w={angular:+.2f}   "
        )
        print(line, end="", flush=True)
