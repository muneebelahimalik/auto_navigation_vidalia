#!/usr/bin/env python3
"""
canbus_interface.py — Send Twist2d velocity commands to the Amiga via gRPC.

Uses the farm-ng OS 2.0 (Barley) EventClient pattern:
    await client.request_reply("/twist", Twist2d(...))

Farm-ng API references:
    from farm_ng.canbus.canbus_pb2 import Twist2d
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig

See also:
    https://amiga.farm-ng.com/docs/brain/sdk-barley-migration
"""

from __future__ import annotations

import asyncio

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig


class CanbusInterface:
    """
    Async interface for sending velocity commands to the Amiga robot.

    Usage::

        canbus = CanbusInterface(configs["canbus"])
        await canbus.send_twist(0.5, 0.0)   # 0.5 m/s forward
        await canbus.stop()
    """

    def __init__(
        self,
        config: EventServiceConfig,
        max_linear: float = 1.5,
        max_angular: float = 1.0,
    ) -> None:
        """
        Args:
            config:      EventServiceConfig for the canbus service (port 6001).
            max_linear:  Maximum forward/reverse speed in m/s (clamped).
            max_angular: Maximum turn rate in rad/s (clamped).
        """
        self._config = config
        self._client = EventClient(config)
        self._max_linear = max_linear
        self._max_angular = max_angular

    async def send_twist(self, linear_x: float, angular_z: float) -> None:
        """Send a velocity command to the Amiga robot.

        Args:
            linear_x:  Forward velocity in m/s.  Positive = forward.
            angular_z: Turn rate in rad/s.  Positive = counter-clockwise.
        """
        twist = Twist2d()
        twist.linear_velocity_x = max(
            -self._max_linear, min(self._max_linear, linear_x)
        )
        twist.angular_velocity = max(
            -self._max_angular, min(self._max_angular, angular_z)
        )
        await self._client.request_reply("/twist", twist)

    async def stop(self) -> None:
        """Send a zero-velocity command to bring the robot to a halt."""
        await self.send_twist(0.0, 0.0)
