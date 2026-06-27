#!/usr/bin/env python3
"""
filter_heading.py — Absolute heading from the Amiga filter service (GPS+IMU).

The headland U-turn pivots 90° in place.  On a 4-wheel skid-steer that scrubs
and slips, so heading integrated from wheel encoders (AmigaTpdo1
measured_angular_rate, see odometry.py) drifts — a 90° pivot can finish 10–20°
off.  The filter service (FilterState, port 20001) publishes a GPS+IMU-fused
absolute heading that does NOT depend on wheel contact, so closing the pivots
on it makes the turn robust to slip.

This subscriber mirrors WheelOdometry.run(): it tries the filter service's
"/state" path, exposes the latest heading, and never raises into the caller.
``HeadlandTurn`` prefers this source when it is fresh and converged and falls
back to wheel-odometry heading otherwise, so a non-converged filter can never
strand a turn.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time

log = logging.getLogger(__name__)

_FILTER_PATHS = ["/state", "/filter"]
_SUBSCRIBE_TIMEOUT = 5.0
_STALE_SECS = 0.5


class FilterHeading:
    """Latest absolute heading (rad) from the filter service.

    Attributes
    ----------
    heading : float
        Fused heading in radians (sign convention is irrelevant to the U-turn,
        which only uses the *change* over a pivot).
    available : bool
        True once any FilterState message has been decoded.
    converged : bool
        Mirror of FilterState.has_converged — the pivot only trusts the filter
        heading when this is True.
    """

    def __init__(self, filter_config=None) -> None:
        self._config = filter_config
        self.heading: float = 0.0
        self.available: bool = False
        self.converged: bool = False
        self._t: float = 0.0
        self._running: bool = False

    @property
    def fresh(self) -> bool:
        """True if a message arrived within the staleness window."""
        return self.available and (time.monotonic() - self._t) < _STALE_SECS

    @property
    def usable(self) -> bool:
        """True when the heading is safe to close a pivot loop on."""
        return self.fresh and self.converged

    # ------------------------------------------------------------------
    async def run(self) -> None:
        """Background task: subscribe to FilterState and track heading."""
        if self._config is None:
            return
        self._running = True
        try:
            from farm_ng.core.event_client import EventClient  # noqa: F401
        except ImportError:
            log.debug("[filter_heading] farm-ng SDK unavailable")
            return

        for path in _FILTER_PATHS:
            try:
                if await self._subscribe(path):
                    return
            except asyncio.CancelledError:
                return
            except Exception as exc:  # noqa: BLE001
                log.debug("[filter_heading] path %s failed: %s", path, exc)
                continue
        log.debug("[filter_heading] no working filter subscription path")

    async def _subscribe(self, path: str) -> bool:
        """Subscribe and consume FilterState forever.

        Returns True once at least one usable message has been received (so
        ``run()`` stops trying other paths), False if this path delivered
        nothing within the timeout (try the next path).

        Only the wait for the FIRST message is bounded by a timeout; once the
        stream is flowing it is consumed indefinitely.  The previous version
        wrapped this entire infinite loop in ``asyncio.wait_for`` — which
        cancelled the subscription after the timeout even while messages were
        arriving, so the heading went permanently stale and the U-turn always
        fell back to wheel odometry (``rot=…[wheel]``).
        """
        from farm_ng.core.event_client import EventClient
        from farm_ng.core.event_service_pb2 import SubscribeRequest
        from farm_ng.core.uri_pb2 import Uri

        req = SubscribeRequest(uri=Uri(path=path), every_n=1)
        client = EventClient(self._config)            # keep a reference alive
        stream = client.subscribe(req, decode=True).__aiter__()

        # Bound only the FIRST message; the filter service may be slow to start.
        try:
            _event, msg = await asyncio.wait_for(
                stream.__anext__(), timeout=_SUBSCRIBE_TIMEOUT)
        except asyncio.TimeoutError:
            return False
        if not hasattr(msg, "heading"):
            raise ValueError(f"path {path}: no heading field on {type(msg)}")
        print(f"\n[filter_heading] absolute heading available at {path}")
        self._update(msg)

        async for _event, msg in stream:             # consume forever
            if not self._running:
                return True
            self._update(msg)
        return True

    def _update(self, msg) -> None:
        self.heading = float(msg.heading)
        # FilterState.has_converged gates trust; default True if absent.
        self.converged = bool(getattr(msg, "has_converged", True))
        self._t = time.monotonic()
        self.available = True
