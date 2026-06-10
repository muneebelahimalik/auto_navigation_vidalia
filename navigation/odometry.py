#!/usr/bin/env python3
"""
odometry.py — Wheel odometry for the Amiga robot.

Subscribes to AmigaTpdo1 (measured_speed, measured_angular_rate) from the
farm-ng canbus service and integrates a 2D pose estimate (x, y, theta) and
cumulative row distance.

If the canbus subscription is unavailable (service offline, SDK missing, or
path mismatch), the module falls back to commanded-velocity integration via
tick_cmd().  The caller never needs to check availability explicitly.

Usage::

    odo = WheelOdometry(canbus_config)
    asyncio.ensure_future(odo.run())   # background task

    # Each LiDAR scan:
    dist_delta = odo.tick(linear_cmd, angular_cmd, dt)
    row_dist += dist_delta
"""
from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Optional

log = logging.getLogger(__name__)

# Subscription paths tried in order on the canbus EventService.
# farm-ng OS 2.0 (Barley) publishes AmigaTpdo1 at "/state" by default.
_CANBUS_PATHS = ["/state", "/canbus", "/amiga"]

# Timeout (s) before we give up on each subscription attempt.
_SUBSCRIBE_TIMEOUT = 5.0

# Maximum stale age for measured values; fall back to commanded after this.
_STALE_SECS = 0.5


class WheelOdometry:
    """Integrate wheel speeds to a 2-D pose and cumulative distance.

    All public attributes are safe to read from any coroutine — they are
    updated atomically (Python GIL makes single-float assignment atomic).

    Attributes
    ----------
    x, y : float
        Dead-reckoned robot position in metres (world frame, relative to start).
    theta : float
        Robot heading in radians (relative to start, CCW positive).
    distance : float
        Cumulative distance travelled (m).
    available : bool
        True after the first measured-speed reading arrives.
    """

    def __init__(self, canbus_config=None) -> None:
        self._config = canbus_config
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.distance: float = 0.0
        self.available: bool = False

        self._meas_speed: float = 0.0
        self._meas_angular: float = 0.0
        self._meas_t: float = 0.0
        self._running: bool = False

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def tick(self, cmd_speed: float, cmd_angular: float, dt: float) -> float:
        """Integrate one time step.  Returns distance delta (m) for this step.

        Uses measured wheel velocity if fresh; falls back to commanded otherwise.
        """
        now = time.monotonic()
        if self.available and (now - self._meas_t) < _STALE_SECS:
            speed = self._meas_speed
            angular = self._meas_angular
        else:
            speed = cmd_speed
            angular = cmd_angular
        return self._integrate(speed, angular, dt)

    def reset_distance(self) -> None:
        """Zero the cumulative distance counter (call when starting a new row)."""
        self.distance = 0.0

    def reset_pose(self) -> None:
        """Reset pose to origin."""
        self.x = self.y = self.theta = self.distance = 0.0

    # ------------------------------------------------------------------
    # Background subscription task
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Background task: subscribe to AmigaTpdo1 and update measured speeds."""
        if self._config is None:
            return
        self._running = True
        try:
            from farm_ng.core.event_client import EventClient
            from farm_ng.core.event_service_pb2 import SubscribeRequest
            from farm_ng.core.uri_pb2 import Uri
        except ImportError:
            log.debug("[odometry] farm-ng SDK unavailable — using commanded velocity")
            return

        for path in _CANBUS_PATHS:
            try:
                await asyncio.wait_for(
                    self._subscribe(path),
                    timeout=_SUBSCRIBE_TIMEOUT,
                )
                return  # successful subscription finished (CancelledError raises out)
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                return
            except Exception as exc:
                log.debug("[odometry] path %s failed: %s", path, exc)
                continue

        log.debug("[odometry] no working canbus subscription path — using commanded velocity")

    async def _subscribe(self, path: str) -> None:
        from farm_ng.core.event_client import EventClient
        from farm_ng.core.event_service_pb2 import SubscribeRequest
        from farm_ng.core.uri_pb2 import Uri

        req = SubscribeRequest(uri=Uri(path=path), every_n=1)
        client = EventClient(self._config)
        first = True
        async for _event, msg in client.subscribe(req, decode=True):
            if not self._running:
                return
            if first:
                first = False
                # Probe the message for the fields we need.
                if not (hasattr(msg, "measured_speed") and hasattr(msg, "measured_angular_rate")):
                    # Wrong message type on this path.
                    raise ValueError(f"path {path}: unexpected message type {type(msg)}")
                log.debug("[odometry] subscribed AmigaTpdo1 at %s", path)
                if not self.available:
                    print(f"\n[odometry] wheel speed available at {path}")
            self._meas_speed = float(msg.measured_speed)
            self._meas_angular = float(msg.measured_angular_rate)
            self._meas_t = time.monotonic()
            self.available = True

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _integrate(self, speed: float, angular: float, dt: float) -> float:
        """Midpoint integration of differential-drive kinematics. Returns dist delta.

        Position is advanced along the heading at the middle of the step
        (2nd-order accurate) instead of the end-of-step heading, which biases
        the pose inward during sustained turns — exactly the regime the
        closed-loop headland U-turn depends on.
        """
        dist = speed * dt
        self.distance += abs(dist)
        theta_mid = self.theta + 0.5 * angular * dt
        self.theta += angular * dt
        self.x += speed * math.cos(theta_mid) * dt
        self.y += speed * math.sin(theta_mid) * dt
        return dist
