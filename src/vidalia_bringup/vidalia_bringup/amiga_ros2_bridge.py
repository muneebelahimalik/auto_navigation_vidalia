#!/usr/bin/env python3
"""
amiga_ros2_bridge.py — native ROS 2 / farm-ng gRPC bridge.

Compatible with Amiga OS 2.0 (Barley) and farm-ng-core/farm-ng-amiga >= 2.0.0.
Uses the EventClient subscribe + request_reply pattern as shown in the
official farm-ng virtual-joystick / pose_generator examples.

INSTALLATION (run once, before building the ROS 2 workspace):
    pip3 install farm-ng-amiga farm-ng-core
    # farm-ng must be in the SAME Python that ROS 2 uses (system Python,
    # NOT a venv).  Verify with:  python3 -c "import farm_ng; print('ok')"

Published topics
----------------
/amiga/vel   (geometry_msgs/TwistStamped)
    Measured wheel velocity from the canbus service (AmigaTpdo1).
    Consumed by amiga_odometry → /wheel_odom for SLAM.

/amiga/pose  (nav_msgs/Odometry, frame_id="world")
    Filter state (GPS + IMU).  Only published when has_converged=true
    (or when publish_unconverged_filter:=true for lab debugging).
    The x/y coordinates are in the filter's world frame (UTM-based).

Subscribed topics
-----------------
/cmd_vel  (geometry_msgs/Twist)
    Velocity commands (from Nav2 / teleop) forwarded to the Amiga as
    Twist2d via canbus request_reply("/twist", ...) at the canbus rate.

ROS 2 parameters
----------------
host                      (str,   default 'camphor-clone.tail0be07.ts.net')
canbus_port               (int,   default 6001)
filter_port               (int,   default 20001)
max_linear                (float, default 1.5)   m/s clamp
max_angular               (float, default 1.0)   rad/s clamp
publish_unconverged_filter (bool,  default false)

OS 2.0 API notes
----------------
• AmigaTpdo1 is now in farm_ng.canbus.packet (not canbus_pb2).
• Canbus messages must be decoded manually:
    subscribe(..., decode=False)  →  payload_to_protobuf(event, payload)
    →  AmigaTpdo1.from_proto(message.amiga_tpdo1)
• Velocity commands use Twist2d via EventClient.request_reply("/twist", twist).
  No raw gRPC stub needed.
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# farm-ng SDK imports — OS 2.0 (Barley) layout
# ---------------------------------------------------------------------------
try:
    from farm_ng.canbus.packet import AmigaTpdo1
    from farm_ng.canbus.canbus_pb2 import Twist2d
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    from google.protobuf import json_format
    _SDK_OK = True
except ImportError as _sdk_err:
    _SDK_OK = False
    _sdk_err_msg = str(_sdk_err)

# payload_to_protobuf location varies slightly across SDK patch versions
_payload_to_proto = None
if _SDK_OK:
    try:
        from farm_ng.core.events_file_reader import payload_to_protobuf as _p
        _payload_to_proto = _p
    except ImportError:
        pass
    if _payload_to_proto is None:
        try:
            from farm_ng.core.event_client import payload_to_protobuf as _p
            _payload_to_proto = _p
        except ImportError:
            pass

# farm_ng_core_pybind is optional (used for filter state pose)
try:
    from farm_ng_core_pybind import Pose3F64
    _PYBIND_OK = True
except ImportError:
    _PYBIND_OK = False


def _make_config(name: str, host: str, port: int, path: str, query: str) -> 'EventServiceConfig':
    """Build EventServiceConfig matching the service_config.json JSON format."""
    return json_format.ParseDict(
        {
            'name': name,
            'host': host,
            'port': port,
            'subscriptions': [
                {'uri': {'path': path, 'query': query}, 'every_n': 1}
            ],
        },
        EventServiceConfig(),
    )


class AmigaRos2Bridge(Node):

    def __init__(self):
        super().__init__('amiga_ros2_bridge')

        if not _SDK_OK:
            self.get_logger().fatal(
                f'farm-ng Python SDK not found ({_sdk_err_msg}).\n'
                'Install in the system Python used by ROS 2 (NOT a venv):\n'
                '    pip3 install farm-ng-amiga farm-ng-core\n'
                'Then rebuild:  colcon build --symlink-install'
            )
            raise RuntimeError('farm_ng SDK not installed in ROS 2 Python environment')

        if _payload_to_proto is None:
            self.get_logger().warn(
                'payload_to_protobuf not found in farm_ng SDK — '
                'canbus messages may not decode correctly.'
            )

        # ---- parameters ----
        self.declare_parameter('host', 'camphor-clone.tail0be07.ts.net')
        self.declare_parameter('canbus_port', 6001)
        self.declare_parameter('filter_port', 20001)
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('publish_unconverged_filter', False)

        self._host = self.get_parameter('host').value
        self._canbus_port = int(self.get_parameter('canbus_port').value)
        self._filter_port = int(self.get_parameter('filter_port').value)
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
        self._pub_unconverged = self.get_parameter('publish_unconverged_filter').value

        # ---- publishers ----
        self._vel_pub = self.create_publisher(TwistStamped, '/amiga/vel', 10)
        self._pose_pub = self.create_publisher(Odometry, '/amiga/pose', 10)

        # ---- cmd_vel: latest command stored for injection into canbus loop ----
        self._last_cmd: Optional[Twist] = None
        self._cmd_lock = threading.Lock()
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.get_logger().info(
            f'amiga_ros2_bridge: connecting to {self._host} '
            f'(canbus:{self._canbus_port}, filter:{self._filter_port})'
        )

        # ---- asyncio in background daemon thread ----
        self._loop = asyncio.new_event_loop()
        self._bg = threading.Thread(target=self._run_loop, daemon=True)
        self._bg.start()

    # ------------------------------------------------------------------
    # ROS 2 subscriber (main thread)
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._last_cmd = msg

    def _get_cmd(self) -> Twist:
        """Return latest /cmd_vel command (thread-safe), or zero Twist."""
        with self._cmd_lock:
            return self._last_cmd if self._last_cmd is not None else Twist()

    # ------------------------------------------------------------------
    # Asyncio loop (background thread)
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        tasks = [
            asyncio.create_task(self._run_canbus()),
            asyncio.create_task(self._run_filter()),
        ]
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for t in pending:
            t.cancel()
        for t in done:
            if not t.cancelled() and t.exception():
                exc = t.exception()
                self.get_logger().error(f'async task raised: {exc!r}')
                # RuntimeError from first-message timeout: print the full
                # diagnostic message so it's visible in the ROS 2 log.
                if isinstance(exc, RuntimeError):
                    self.get_logger().fatal(str(exc))

    # ------------------------------------------------------------------
    # Canbus: read AmigaTpdo1 → /amiga/vel  AND  send Twist2d commands
    #
    # Pattern mirrors farm-ng filter_client.py / pose_generator:
    #   subscribe(config.subscriptions[0], decode=False)
    #   → payload_to_protobuf(event, payload) → AmigaTpdo1.from_proto
    #   → request_reply("/twist", Twist2d(...)) within the same loop
    # ------------------------------------------------------------------

    # First-message timeout: if the canbus service never sends a message
    # (wrong port, wrong path, service not running) this makes the node
    # raise a clear error immediately instead of hanging forever.
    _CANBUS_FIRST_MSG_TIMEOUT = 20.0   # seconds

    @staticmethod
    async def _anext_with_timeout(aiter, timeout: float):
        """Await the next item from an async iterator with a timeout.
        Returns the item, or raises asyncio.TimeoutError on timeout."""
        return await asyncio.wait_for(aiter.__anext__(), timeout=timeout)

    async def _run_canbus(self) -> None:
        config = _make_config(
            'canbus', self._host, self._canbus_port,
            '/state', 'service_name=canbus',
        )

        while rclpy.ok():
            try:
                self.get_logger().info(
                    f'Connecting to canbus service at '
                    f'{self._host}:{self._canbus_port} …'
                )
                client = EventClient(config)

                # Use config.subscriptions[0] — same pattern as filter_client.py.
                gen = client.subscribe(config.subscriptions[0], decode=False)

                # Wait up to _CANBUS_FIRST_MSG_TIMEOUT seconds for the first
                # message; give a clear error rather than hanging silently.
                try:
                    event, payload = await self._anext_with_timeout(
                        gen, self._CANBUS_FIRST_MSG_TIMEOUT
                    )
                except asyncio.TimeoutError:
                    raise RuntimeError(
                        f'canbus service at {self._host}:{self._canbus_port} '
                        f'did not send any message within '
                        f'{self._CANBUS_FIRST_MSG_TIMEOUT:.0f} s.\n'
                        f'  • Check the port — run:  '
                        f'python3 scripts/test_canbus.py --scan\n'
                        f'  • Verify Tailscale is active on both devices\n'
                        f'  • Verify the Amiga canbus service is running'
                    )

                self.get_logger().info(
                    'Canbus service connected — receiving AmigaTpdo1 messages.'
                )

                # Process first message, then continue the stream.
                while True:
                    # ---- decode state ----
                    if _payload_to_proto is not None:
                        message = _payload_to_proto(event, payload)
                        tpdo1 = AmigaTpdo1.from_proto(message.amiga_tpdo1)
                    else:
                        tpdo1 = AmigaTpdo1.from_proto(payload)

                    # ---- publish /amiga/vel ----
                    vel_msg = TwistStamped()
                    vel_msg.header.stamp = self.get_clock().now().to_msg()
                    vel_msg.header.frame_id = 'base_link'
                    vel_msg.twist.linear.x = float(tpdo1.meas_speed)
                    vel_msg.twist.angular.z = float(tpdo1.meas_ang_rate)
                    self._vel_pub.publish(vel_msg)

                    # ---- forward velocity command to Amiga ----
                    cmd = self._get_cmd()
                    twist = Twist2d()
                    twist.linear_velocity_x = max(
                        -self._max_lin, min(self._max_lin, cmd.linear.x)
                    )
                    twist.angular_velocity = max(
                        -self._max_ang, min(self._max_ang, cmd.angular.z)
                    )
                    try:
                        await client.request_reply('/twist', twist)
                    except Exception as cmd_exc:
                        self.get_logger().debug(
                            f'request_reply /twist failed: {cmd_exc!r}'
                        )

                    # Advance to next message
                    try:
                        event, payload = await gen.__anext__()
                    except StopAsyncIteration:
                        self.get_logger().warn('Canbus stream ended — reconnecting…')
                        break

            except RuntimeError:
                raise   # propagate first-message-timeout error to _async_main
            except Exception as exc:
                self.get_logger().warn(
                    f'canbus error: {exc!r}  — retrying in 3 s'
                )
                await asyncio.sleep(3.0)

    # ------------------------------------------------------------------
    # Filter: read FilterState → /amiga/pose  (GPS, when converged)
    # ------------------------------------------------------------------

    async def _run_filter(self) -> None:
        config = _make_config(
            'filter', self._host, self._filter_port,
            '/state', 'service_name=filter',
        )

        while rclpy.ok():
            try:
                self.get_logger().info('Connecting to filter service…')
                async for event, message in EventClient(config).subscribe(
                    config.subscriptions[0], decode=True
                ):
                    if not message.has_converged and not self._pub_unconverged:
                        continue

                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = 'world'
                    odom.child_frame_id = 'robot'

                    if _PYBIND_OK:
                        pose = Pose3F64.from_proto(message.pose)
                        odom.pose.pose.position.x = pose.translation[0]
                        odom.pose.pose.position.y = pose.translation[1]
                        odom.pose.pose.position.z = (
                            pose.translation[2] if len(pose.translation) > 2 else 0.0
                        )
                    else:
                        # proto-direct fallback (field names may vary)
                        t = getattr(message.pose, 'translation', None)
                        if t is not None:
                            odom.pose.pose.position.x = getattr(t, 'x', 0.0)
                            odom.pose.pose.position.y = getattr(t, 'y', 0.0)

                    h = message.heading
                    odom.pose.pose.orientation.z = math.sin(h * 0.5)
                    odom.pose.pose.orientation.w = math.cos(h * 0.5)

                    unc = message.uncertainty_diagonal
                    if unc and len(unc.data) >= 3:
                        odom.pose.covariance[0] = unc.data[0] ** 2
                        odom.pose.covariance[7] = unc.data[1] ** 2
                        odom.pose.covariance[35] = unc.data[2] ** 2

                    self._pose_pub.publish(odom)

            except Exception as exc:
                self.get_logger().warn(
                    f'filter error: {exc!r}  — retrying in 3 s'
                )
                await asyncio.sleep(3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = AmigaRos2Bridge()
        rclpy.spin(node)
    except RuntimeError as exc:
        rclpy.logging.get_logger('amiga_ros2_bridge').fatal(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
