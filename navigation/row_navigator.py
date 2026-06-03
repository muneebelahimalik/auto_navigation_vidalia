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

import json
import math
import os
import struct
import time
from typing import Optional

import numpy as np

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import tilt_correct_pts
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
    HL_ADVANCE = "HL_ADVANCE"
    HL_TURN1 = "HL_TURN1"
    HL_SHIFT = "HL_SHIFT"
    HL_TURN2 = "HL_TURN2"
    DONE = "DONE"


_HEADLAND_STATES = (_S.HL_ADVANCE, _S.HL_TURN1, _S.HL_SHIFT, _S.HL_TURN2)


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
        acquire_conf: float = 0.45,
        acquire_frames: int = 5,
        row_end_conf: float = 0.70,
        row_end_frames: int = 5,
        row_end_min_dist: float = 2.0,
        obstacle_clear_secs: float = 1.5,
        buffer_dist: float = 1.5,
        bed_shift: float = 1.5,
        headland_speed: float = 0.15,
        headland_turn_rate: float = 0.30,
        align_heading: bool = True,
        align_thresh: float = 0.14,
        align_rate: float = 0.20,
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
        cam_block_frames: int = 3,
        ros2_bridge: bool = False,
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
        self.cam_block_frames = cam_block_frames
        self.ros2_bridge = ros2_bridge

        self.acquire_conf = acquire_conf
        self.acquire_frames = acquire_frames
        self.row_end_conf = row_end_conf
        self.row_end_frames = row_end_frames
        self.row_end_min_dist = row_end_min_dist
        self.obstacle_clear_secs = obstacle_clear_secs
        self.buffer_dist = buffer_dist
        self.bed_shift = bed_shift
        self.headland_speed = headland_speed
        self.headland_turn_rate = headland_turn_rate
        self.align_heading = align_heading
        self.align_thresh = align_thresh
        self.align_rate = align_rate

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
        self._hl_progress = 0.0
        self._turn_sign = 1.0
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
        fused_conf = min(1.0, lidar_conf + vis_conf * 0.3)

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
        """Consume the LiDAR scan stream and drive the state machine."""
        self._t_prev = time.monotonic()
        async for pts in lidar.scan_stream_np():
            if self._stop:
                break

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
                if self.tilt_rad != 0.0:
                    pts = tilt_correct_pts(pts, self.tilt_rad)

            est = self.detector.update(pts)

            # --- Camera integration ---
            vis_est = None
            frame_l = frame_r = dep_l = dep_r = None
            if self.vis_detector is not None or self.depth_pts_left is not None or self.depth_pts_right is not None:
                frame_l = self.left_cam.get_latest() if self.left_cam is not None else None
                frame_r = self.right_cam.get_latest() if self.right_cam is not None else None
                dep_l = frame_l.depth if frame_l is not None else None
                dep_r = frame_r.depth if frame_r is not None else None
                rgb_l = frame_l.rgb if frame_l is not None else None
                rgb_r = frame_r.rgb if frame_r is not None else None

                if self.vis_detector is not None:
                    vis_est = self.vis_detector.update(rgb_l, dep_l, rgb_r, dep_r)

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

            # --- Sensor fusion: EKF or weighted average ---
            if self.ekf is not None:
                prev_v, prev_w = self.cmd   # commanded last scan — best motion estimate
                self.ekf.predict(linear_vel=prev_v, angular_vel=prev_w, dt=dt)
                self.ekf.update_lidar(est)
                if vis_est is not None:
                    self.ekf.update_camera(vis_est)
                est = self.ekf.to_estimate(est)
            elif vis_est is not None:
                est = self._fuse_estimates(est, vis_est)

            linear, angular = self._step(est, safety, dt)
            self.cmd = (linear, angular)
            await self._drive(linear, angular)
            self._print_status(est, safety, linear, angular)
            if self.ros2_bridge:
                img_l = getattr(frame_l, "rgb", None) if frame_l is not None else None
                img_r = getattr(frame_r, "rgb", None) if frame_r is not None else None
                self._write_bridge(pts, safety_pts, cam_cloud, est, safety, linear, angular, img_l, img_r)

            if self.state == _S.DONE:
                break

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
            return self._step_obstacle(safety)
        if st == _S.ROW_END:
            return self._step_row_end()
        if st in _HEADLAND_STATES:
            return self._step_headland(dt)
        return 0.0, 0.0   # DONE

    # ------------------------------------------------------------------
    def _step_acquire(self, est) -> tuple[float, float]:
        if self.ekf is not None:
            # EKF-aware: once the filter absorbs ≥2 real measurements its
            # std_lateral drops to ~0.04 m and stays below 0.08 m through
            # predict-only (empty-scan) steps.  Raw confidence oscillates
            # 0.70→0.35 on alternating full/empty VLP-16 scans, making the
            # old consecutive-frame counter reset indefinitely.
            frame_ok = self.ekf.converged
        else:
            frame_ok = est.confidence >= self.acquire_conf
        self._acq_count = self._acq_count + 1 if frame_ok else 0
        if self._acq_count >= self.acquire_frames:
            # Pre-align: rotate in-place to reduce heading error before FOLLOW
            # so the robot doesn't lurch sideways at the start of each row.
            # -heading_error * P maps: negative hdg → positive angular (right turn).
            if self.align_heading and abs(est.heading_error) > self.align_thresh:
                rate = self.align_rate
                w = max(-rate, min(rate, -est.heading_error * 2.0))
                return 0.0, w
            self._enter(_S.FOLLOW)
            self._row_dist = 0.0
            self._row_end_count = 0
        return 0.0, 0.0

    def _step_follow(self, est, dt: float) -> tuple[float, float]:
        if (self._row_dist >= self.row_end_min_dist
                and est.row_end_confidence >= self.row_end_conf):
            self._row_end_count += 1
        else:
            self._row_end_count = 0
        if self._row_end_count >= self.row_end_frames:
            self._enter(_S.ROW_END)
            return 0.0, 0.0

        # When the EKF is tracking well (small lateral uncertainty), tolerate
        # brief LiDAR detection gaps without dropping back to ACQUIRE. VLP-16
        # scans vary widely in point density (12k–28k) scan-to-scan; a single
        # empty crop-band scan halves EMA confidence and without this floor the
        # robot exits FOLLOW after just 1–2 bad scans.
        # Threshold 0.08 m: steady-state EKF lateral std ≈ 0.055 m (well below).
        min_conf = self.controller.min_confidence
        if self.ekf is not None and self.ekf.std_lateral < 0.08:
            min_conf = max(0.12, min_conf - 0.20)

        if est.confidence < min_conf:
            self._enter(_S.ACQUIRE)
            self._acq_count = 0
            return 0.0, 0.0

        linear, angular = self.controller.compute(est)
        if self.odometry is not None:
            self._row_dist += abs(self.odometry.tick(linear, angular, dt))
        else:
            self._row_dist += linear * dt
        return linear, angular

    def _step_obstacle(self, safety) -> tuple[float, float]:
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
        if self.headland and self._rows_done < self.rows:
            self._turn_sign = 1.0 if self._rows_done % 2 == 1 else -1.0
            self._hl_progress = 0.0
            self._enter(_S.HL_ADVANCE)
        else:
            self._enter(_S.DONE)
        return 0.0, 0.0

    def _step_headland(self, dt: float) -> tuple[float, float]:
        """Open-loop headland maneuver (dead-reckoned from commanded speed)."""
        creep = self.headland_speed
        turn = self.headland_turn_rate * self._turn_sign
        if self.state == _S.HL_ADVANCE:
            self._hl_progress += creep * dt
            if self._hl_progress >= self.buffer_dist:
                self._hl_progress = 0.0
                self._enter(_S.HL_TURN1)
            return creep, 0.0
        if self.state == _S.HL_TURN1:
            self._hl_progress += abs(turn) * dt
            if self._hl_progress >= math.pi / 2.0:
                self._hl_progress = 0.0
                self._enter(_S.HL_SHIFT)
            return 0.0, turn
        if self.state == _S.HL_SHIFT:
            self._hl_progress += creep * dt
            if self._hl_progress >= self.bed_shift:
                self._hl_progress = 0.0
                self._enter(_S.HL_TURN2)
            return creep, 0.0
        # HL_TURN2
        self._hl_progress += abs(turn) * dt
        if self._hl_progress >= math.pi / 2.0:
            self._acq_count = 0
            self._enter(_S.ACQUIRE)
        return 0.0, turn

    # ------------------------------------------------------------------
    def _enter(self, state: str) -> None:
        if state != self.state:
            print(f"\n[navigator] {self.state} -> {state}")
            if state == _S.ACQUIRE and self.ekf is not None:
                self.ekf.reset()
        self.state = state

    async def _drive(self, linear: float, angular: float) -> None:
        """Send the command to the canbus, unless in perception-only mode."""
        if self.canbus is None or not self.auto:
            return
        try:
            await self.canbus.send_twist(linear, angular)
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
            suffix += "+CAM"
        if depth3d:
            suffix += "+3D"
        if ekf_active:
            suffix += "+EKF"
        mode = ("AUTO" if self.auto else "PERC") + suffix
        deg = math.degrees(est.heading_error)
        ekf_str = ""
        if ekf_active and self.ekf is not None:
            ekf_str = f" σ={self.ekf.std_lateral:.3f}"
        acq_str = f" acq={self._acq_count}/{self.acquire_frames}" if self.state == _S.ACQUIRE else ""
        line = (
            f"\r[{mode}] {self.state:11s} | "
            f"hdg={deg:+5.1f}° off={est.lateral_offset:+5.2f}m "
            f"conf={est.confidence:.2f} end={est.row_end_confidence:.2f} "
            f"n={est.n_points:4d}{ekf_str}{acq_str} | "
            f"row={self._rows_done}/{self.rows} d={self._row_dist:4.1f}m | "
            f"safe={safety.reason():14s} | "
            f"cmd v={linear:+.2f} w={angular:+.2f}   "
        )
        print(line, end="", flush=True)
