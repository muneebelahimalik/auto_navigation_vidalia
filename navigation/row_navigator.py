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

import math
import time
from typing import Optional

import numpy as np

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowDetector, RowEstimate
from navigation.row_safety import SafetyMonitor


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
        left_cam=None,
        right_cam=None,
        vis_detector=None,
        depth_left=None,
        depth_right=None,
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

        self.state = _S.ACQUIRE
        self.cmd = (0.0, 0.0)
        self._stop = False
        self._acq_count = 0
        self._row_end_count = 0
        self._row_dist = 0.0
        self._rows_done = 0
        self._obstacle_clear_t: Optional[float] = None
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
        fused_conf = min(1.0, lidar_conf + vis_conf * 0.3)

        return RowEstimate(
            heading_error=lidar_est.heading_error,
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

            est = self.detector.update(pts)
            safety = self.safety.check(pts)

            if self.vis_detector is not None:
                frame_l = self.left_cam.get_latest() if self.left_cam is not None else None
                frame_r = self.right_cam.get_latest() if self.right_cam is not None else None
                rgb_l = frame_l.rgb if frame_l is not None else None
                dep_l = frame_l.depth if frame_l is not None else None
                rgb_r = frame_r.rgb if frame_r is not None else None
                dep_r = frame_r.depth if frame_r is not None else None
                vis_est = self.vis_detector.update(rgb_l, dep_l, rgb_r, dep_r)
                est = self._fuse_estimates(est, vis_est)

                if self.depth_left is not None:
                    ds_l = self.depth_left.check(dep_l, "left")
                    if ds_l.blocked:
                        safety.cam_blocked = True
                        safety.cam_reason = ds_l.reason()

                if self.depth_right is not None:
                    ds_r = self.depth_right.check(dep_r, "right")
                    if ds_r.blocked:
                        safety.cam_blocked = True
                        reason_r = ds_r.reason()
                        if safety.cam_reason:
                            safety.cam_reason = safety.cam_reason + "," + reason_r
                        else:
                            safety.cam_reason = reason_r

            linear, angular = self._step(est, safety, dt)
            self.cmd = (linear, angular)
            await self._drive(linear, angular)
            self._print_status(est, safety, linear, angular)

            if self.state == _S.DONE:
                break

        await self.stop()

    # ------------------------------------------------------------------
    def _step(self, est, safety, dt: float) -> tuple[float, float]:
        """Advance the state machine one scan; return (linear, angular)."""
        st = self.state

        # --- safety override: highest priority during active motion ---
        if st in (_S.ACQUIRE, _S.FOLLOW) or st in _HEADLAND_STATES:
            if safety.blocked and st != _S.OBSTACLE_WAIT:
                if st in _HEADLAND_STATES:
                    # pause the maneuver in place rather than abandon it
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
        self._acq_count = self._acq_count + 1 if est.confidence >= self.acquire_conf else 0
        if self._acq_count >= self.acquire_frames:
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

        if est.confidence < self.controller.min_confidence:
            self._enter(_S.ACQUIRE)
            self._acq_count = 0
            return 0.0, 0.0

        linear, angular = self.controller.compute(est)
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
    def _print_status(self, est, safety, linear: float, angular: float) -> None:
        cam_active = self.vis_detector is not None
        mode = ("AUTO" if self.auto else "PERC") + ("+CAM" if cam_active else "")
        deg = math.degrees(est.heading_error)
        line = (
            f"\r[{mode}] {self.state:11s} | "
            f"hdg={deg:+5.1f}° off={est.lateral_offset:+5.2f}m "
            f"conf={est.confidence:.2f} end={est.row_end_confidence:.2f} "
            f"n={est.n_points:4d} | "
            f"row={self._rows_done}/{self.rows} d={self._row_dist:4.1f}m | "
            f"safe={safety.reason():14s} | "
            f"cmd v={linear:+.2f} w={angular:+.2f}   "
        )
        print(line, end="", flush=True)
