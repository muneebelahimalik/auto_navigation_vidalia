#!/usr/bin/env python3
"""
row_navigator_cam.py — Camera-only autonomous row-following state machine.

Identical state machine to RowNavigator but all perception comes from the
OAK-D cameras — no Velodyne LiDAR required.

  Lateral offset + heading  ← HSV green centroid + PCA in RGB frames
  Obstacle safety           ← depth frame centre-strip (both cameras)
  Row-end detection         ← green_fraction drops below threshold

State machine:
    ACQUIRE       → wait for enough green and confident estimate
    FOLLOW        → pure-pursuit drive along detected row
    ROW_END       → green vanishes at headland; stop or turn
    OBSTACLE_WAIT → depth camera blocked; hold until clear
    HL_*          → open-loop headland manoeuvre (optional)
    DONE          → all rows complete
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Optional

from canbus.canbus_interface import CanbusInterface
from camera.depth_obstacle import DepthObstacleDetector
from camera.row_detector_visual import VisualRowDetector, VisualRowEstimate
from navigation.headland import HeadlandTurn
from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowEstimate


class _S:
    ACQUIRE = "ACQUIRE"
    FOLLOW = "FOLLOW"
    ROW_END = "ROW_END"
    OBSTACLE_WAIT = "OBSTACLE_WAIT"
    HEADLAND = "HEADLAND"
    DONE = "DONE"


_HEADLAND_STATES = (_S.HEADLAND,)


def _vis_to_row(vis: VisualRowEstimate) -> RowEstimate:
    """Adapt VisualRowEstimate to RowEstimate for the pure-pursuit controller."""
    return RowEstimate(
        heading_error=vis.heading_error,
        lateral_offset=vis.lateral_offset,
        confidence=vis.confidence,
        row_end_confidence=0.0,
        n_points=0,
        valid=vis.confidence > 0.0,
    )


class CamRowNavigator:
    """Autonomous crop-row follower driven entirely by OAK-D camera frames."""

    def __init__(
        self,
        canbus: Optional[CanbusInterface],
        left_cam,
        right_cam,
        vis_detector: VisualRowDetector,
        depth_left: Optional[DepthObstacleDetector],
        depth_right: Optional[DepthObstacleDetector],
        controller: PurePursuitController,
        *,
        auto: bool = False,
        rows: int = 1,
        headland: bool = False,
        acquire_conf: float = 0.20,
        acquire_green: float = 0.08,
        acquire_frames: int = 5,
        row_end_green: float = 0.04,
        row_end_frames: int = 10,
        row_end_min_dist: float = 2.0,
        obstacle_clear_secs: float = 1.5,
        row_spacing: float = 0.76,
        headland_exit_dist: float = 1.0,
        first_turn_sign: float = 1.0,
        headland_speed: float = 0.12,
        headland_turn_rate: float = 0.35,
        poll_hz: float = 20.0,
        cam_block_frames: int = 3,
        odometry=None,
    ) -> None:
        self.canbus = canbus
        self.left_cam = left_cam
        self.right_cam = right_cam
        self.vis_detector = vis_detector
        self.depth_left = depth_left
        self.depth_right = depth_right
        self.controller = controller
        self.auto = auto
        self.rows = max(1, rows)
        self.headland = headland

        self.acquire_conf = acquire_conf
        self.acquire_green = acquire_green
        self.acquire_frames = acquire_frames
        self.row_end_green = row_end_green
        self.row_end_frames = row_end_frames
        self.row_end_min_dist = row_end_min_dist
        self.obstacle_clear_secs = obstacle_clear_secs
        self.row_spacing = row_spacing
        self.headland_exit_dist = headland_exit_dist
        self.first_turn_sign = 1.0 if first_turn_sign >= 0 else -1.0
        self.headland_speed = headland_speed
        self.headland_turn_rate = headland_turn_rate
        self._poll_interval = 1.0 / max(poll_hz, 1.0)
        self.odometry = odometry

        self.state = _S.ACQUIRE
        self.cmd = (0.0, 0.0)
        self._stop = False
        self._acq_count = 0
        self._row_end_count = 0
        self._row_dist = 0.0
        self._rows_done = 0
        self._obstacle_clear_t: Optional[float] = None
        self.cam_block_frames = cam_block_frames
        self._cam_block_count: int = 0
        self._turn_sign = 1.0
        self._t_prev = time.monotonic()

        # Closed-loop headland U-turn (odometry feedback).  The camera-only
        # stack typically has no canbus wheel telemetry, so WheelOdometry
        # dead-reckons from commanded velocity — still far more accurate than
        # the previous fixed-time open-loop manoeuvre.
        self._headland_turn = None
        if self.odometry is not None:
            self._headland_turn = HeadlandTurn(
                self.odometry,
                row_spacing=row_spacing,
                exit_dist=headland_exit_dist,
                speed=headland_speed,
                turn_rate=headland_turn_rate,
            )

    # ------------------------------------------------------------------
    async def run(self) -> None:
        """Poll cameras at poll_hz; drive the state machine each cycle."""
        self._t_prev = time.monotonic()
        while not self._stop:
            now = time.monotonic()
            dt = min(max(now - self._t_prev, 1e-3), 0.5)
            self._t_prev = now

            frame_l = self.left_cam.get_latest() if self.left_cam is not None else None
            frame_r = self.right_cam.get_latest() if self.right_cam is not None else None
            rgb_l = frame_l.rgb if frame_l is not None else None
            dep_l = frame_l.depth if frame_l is not None else None
            rgb_r = frame_r.rgb if frame_r is not None else None
            dep_r = frame_r.depth if frame_r is not None else None

            vis = self.vis_detector.update(rgb_l, dep_l, rgb_r, dep_r)

            blocked, cam_reason = self._depth_check(dep_l, dep_r)

            linear, angular = self._step(vis, blocked, dt)
            self.cmd = (linear, angular)

            # Centralised odometry integration (one tick per cycle).  FOLLOW
            # accrues row distance; HEADLAND reads odometry for turn feedback.
            if self.odometry is not None:
                d_before = self.odometry.distance
                self.odometry.tick(linear, angular, dt)
                if self.state == _S.FOLLOW:
                    self._row_dist += max(0.0, self.odometry.distance - d_before)
            elif self.state == _S.FOLLOW:
                self._row_dist += max(0.0, linear * dt)

            await self._drive(linear, angular)
            self._print_status(vis, blocked, cam_reason, linear, angular)

            if self.state == _S.DONE:
                break

            await asyncio.sleep(self._poll_interval)

        await self.stop()

    # ------------------------------------------------------------------
    def _depth_check(self, dep_l, dep_r):
        """Check both depth cameras; return (blocked, reason_str) with debounce."""
        raw_blocked = False
        reason = ""
        if self.depth_left is not None:
            ds = self.depth_left.check(dep_l, "left")
            if ds.blocked:
                raw_blocked = True
                reason = ds.reason()
        if self.depth_right is not None:
            ds = self.depth_right.check(dep_r, "right")
            if ds.blocked:
                raw_blocked = True
                r = ds.reason()
                reason = (reason + "," + r) if reason else r

        if raw_blocked:
            self._cam_block_count += 1
        else:
            self._cam_block_count = 0
        blocked = self._cam_block_count >= self.cam_block_frames
        return blocked, reason

    # ------------------------------------------------------------------
    def _step(self, vis: VisualRowEstimate, blocked: bool, dt: float):
        st = self.state

        if st in (_S.ACQUIRE, _S.FOLLOW) or st in _HEADLAND_STATES:
            if blocked and st != _S.OBSTACLE_WAIT:
                if st in _HEADLAND_STATES:
                    return 0.0, 0.0
                self._enter(_S.OBSTACLE_WAIT)
                self._obstacle_clear_t = None
                return 0.0, 0.0

        if st == _S.ACQUIRE:
            return self._step_acquire(vis)
        if st == _S.FOLLOW:
            return self._step_follow(vis, dt)
        if st == _S.OBSTACLE_WAIT:
            return self._step_obstacle(blocked)
        if st == _S.ROW_END:
            return self._step_row_end()
        if st == _S.HEADLAND:
            return self._step_headland(dt)
        return 0.0, 0.0  # DONE

    def _step_acquire(self, vis: VisualRowEstimate):
        sees_row = (vis.confidence >= self.acquire_conf
                    and vis.green_fraction >= self.acquire_green)
        self._acq_count = self._acq_count + 1 if sees_row else 0
        if self._acq_count >= self.acquire_frames:
            self._enter(_S.FOLLOW)
            self._row_dist = 0.0
            self._row_end_count = 0
        return 0.0, 0.0

    def _step_follow(self, vis: VisualRowEstimate, dt: float):
        if (self._row_dist >= self.row_end_min_dist
                and vis.green_fraction < self.row_end_green):
            self._row_end_count += 1
        else:
            self._row_end_count = 0
        if self._row_end_count >= self.row_end_frames:
            self._enter(_S.ROW_END)
            return 0.0, 0.0

        if vis.confidence < self.controller.min_confidence:
            self._enter(_S.ACQUIRE)
            self._acq_count = 0
            return 0.0, 0.0

        # Row distance is integrated centrally in run() from odometry.
        return self.controller.compute(_vis_to_row(vis))

    def _step_obstacle(self, blocked: bool):
        if blocked:
            self._obstacle_clear_t = None
        else:
            if self._obstacle_clear_t is None:
                self._obstacle_clear_t = time.monotonic()
            elif time.monotonic() - self._obstacle_clear_t >= self.obstacle_clear_secs:
                self._enter(_S.ACQUIRE)
                self._acq_count = 0
        return 0.0, 0.0

    def _step_row_end(self):
        self._rows_done += 1
        if (self.headland and self._rows_done < self.rows
                and self._headland_turn is not None):
            # Serpentine coverage: alternate turn direction each row, starting
            # from first_turn_sign (+1 = right).
            self._turn_sign = self.first_turn_sign * (
                1.0 if self._rows_done % 2 == 1 else -1.0)
            self._headland_turn.begin(self._turn_sign)
            self._enter(_S.HEADLAND)
        else:
            self._enter(_S.DONE)
        return 0.0, 0.0

    def _step_headland(self, dt: float):
        """Closed-loop headland U-turn (odometry feedback via HeadlandTurn)."""
        if self._headland_turn is None:
            self._enter(_S.DONE)
            return 0.0, 0.0
        linear, angular = self._headland_turn.step(dt)
        if self._headland_turn.done:
            self._acq_count = 0
            self._row_dist = 0.0
            self._enter(_S.ACQUIRE)
            return 0.0, 0.0
        return linear, angular

    # ------------------------------------------------------------------
    def _enter(self, state: str) -> None:
        if state != self.state:
            print(f"\n[cam_nav] {self.state} -> {state}")
        self.state = state

    async def _drive(self, linear: float, angular: float) -> None:
        if self.canbus is None or not self.auto:
            return
        try:
            await self.canbus.send_twist(linear, angular)
        except Exception as exc:
            print(f"\n[cam_nav] twist send failed: {exc}")

    async def stop(self) -> None:
        self._stop = True
        if self.canbus is not None:
            try:
                await self.canbus.stop()
            except Exception:
                pass

    def _print_status(
        self,
        vis: VisualRowEstimate,
        blocked: bool,
        cam_reason: str,
        linear: float,
        angular: float,
    ) -> None:
        mode = "AUTO" if self.auto else "PERC"
        deg = math.degrees(vis.heading_error)
        safe_str = ("BLK:" + cam_reason) if blocked else "clear"
        line = (
            f"\r[CAM-{mode}] {self.state:11s} | "
            f"hdg={deg:+5.1f}° off={vis.lateral_offset:+5.2f}m "
            f"conf={vis.confidence:.2f} grn={vis.green_fraction:.3f} | "
            f"row={self._rows_done}/{self.rows} d={self._row_dist:4.1f}m | "
            f"safe={safe_str:18s} | "
            f"cmd v={linear:+.2f} w={angular:+.2f}   "
        )
        print(line, end="", flush=True)
