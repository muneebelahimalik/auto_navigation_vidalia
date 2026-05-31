from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[oak_driver] opencv not installed — image decoding disabled")

try:
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    _FARMNG_OK = True
except ImportError:
    _FARMNG_OK = False
    print("[oak_driver] farm-ng SDK not installed — OAK-D cameras disabled")

# amiga_service gRPC endpoint that hosts all oak sub-services.
_OAK_HOST = "localhost"
_OAK_PORT = 50010

# OAK-D approximate stereo calibration at 640 px width.
# Used to convert uint8 disparity → uint16 depth in millimetres.
# depth_mm = BASELINE_MM * FOCAL_PX / disparity_px
_BASELINE_MM = 75.0
_FOCAL_PX = 452.0


@dataclass
class CameraFrame:
    """One captured frame pair from an OAK-D device."""
    rgb: np.ndarray    # HxWx3 BGR uint8
    depth: np.ndarray  # HxW uint16 mm (converted from disparity)
    side: str
    timestamp: float


def _decode_image(image_data: bytes) -> Optional[np.ndarray]:
    """Decode farm-ng image_data bytes to a numpy array via cv2."""
    if not image_data:
        return None
    buf = np.frombuffer(image_data, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def _disp_to_depth_mm(disp: np.ndarray) -> np.ndarray:
    """Convert disparity (uint8 px) to depth (uint16 mm) using OAK-D calibration."""
    d = disp.astype(np.float32)
    if d.ndim == 3:
        d = d[..., 0]  # take first channel if accidentally decoded as 3-ch
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = np.where(d > 0, (_BASELINE_MM * _FOCAL_PX) / d, 0.0)
    return np.clip(depth, 0, 65535).astype(np.uint16)


class OakDriver:
    """Async driver for one OAK-D camera via farm-ng amiga_service EventClient.

    Subscribes to /rgb and /disparity streams from amiga_service (port 50010).
    The device_id parameter selects the oak sub-service (e.g. 'oak0', 'oak1').
    Empty device_id defaults to 'oak1' for the left camera, 'oak0' for the right
    (oak0 is physically mounted on the right side of the Amiga).
    """

    def __init__(self, side: str, device_id: str = "", fps: int = 10) -> None:
        self.side = side
        self.service_name = device_id if device_id else ("oak1" if side == "left" else "oak0")
        self.fps = fps
        self._latest: Optional[CameraFrame] = None
        self._running = False
        self._rgb: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._rgb_ts: float = 0.0
        self._depth_ts: float = 0.0
        self._not_found_logged = False
        self._intrinsics_logged = False

    def get_latest(self) -> Optional[CameraFrame]:
        return self._latest

    async def run(self) -> None:
        if not _FARMNG_OK or not _CV2_OK:
            return
        self._running = True
        # Suppress repeated gRPC NOT_FOUND warnings that the farm-ng EventClient
        # retry loop emits when a named sub-service (e.g. oak1) is not registered
        # on amiga_service.  We print a one-time message from our own handler.
        logging.getLogger("oak/client").setLevel(logging.ERROR)
        try:
            await asyncio.gather(
                self._subscribe_rgb(),
                self._subscribe_disparity(),
            )
        except asyncio.CancelledError:
            self._running = False

    async def stop(self) -> None:
        self._running = False

    def _client(self) -> "EventClient":
        cfg = EventServiceConfig(name="oak", host=_OAK_HOST, port=_OAK_PORT)
        return EventClient(cfg)

    def _is_not_found(self, exc: Exception) -> bool:
        """True if exc is a gRPC NOT_FOUND ('no matching topics') error."""
        s = str(exc)
        return "NOT_FOUND" in s or "no matching topics" in s

    async def _subscribe_rgb(self) -> None:
        req = SubscribeRequest(
            uri=Uri(path="/rgb", query=f"service_name={self.service_name}"),
            every_n=1,
        )
        try:
            async for _event, msg in self._client().subscribe(req, decode=True):
                if not self._running:
                    break
                img = _decode_image(msg.image_data)
                if img is not None:
                    if not self._intrinsics_logged:
                        self._intrinsics_logged = True
                        h, w = img.shape[:2]
                        # depth_to_points defaults assume 640×400.
                        # Log actual size so the operator can verify or supply
                        # calibrated fx/fy/cx/cy if the resolution differs.
                        match = "OK" if (w == 640 and h == 400) else "MISMATCH — pass fx/fy/cx/cy to DepthToPoints"
                        print(
                            f"[oak_driver:{self.side}] first RGB frame: {w}×{h} "
                            f"— intrinsic assumption {match}"
                        )
                    self._rgb = img
                    self._rgb_ts = time.monotonic()
                    self._merge()
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            if self._running:
                if self._is_not_found(exc):
                    if not self._not_found_logged:
                        self._not_found_logged = True
                        print(f"\n[oak_driver:{self.side}] service '{self.service_name}' not available — camera offline")
                    return  # don't retry
                print(f"\n[oak_driver:{self.side}] RGB stream error: {exc}")

    async def _subscribe_disparity(self) -> None:
        req = SubscribeRequest(
            uri=Uri(path="/disparity", query=f"service_name={self.service_name}"),
            every_n=1,
        )
        try:
            async for _event, msg in self._client().subscribe(req, decode=True):
                if not self._running:
                    break
                img = _decode_image(msg.image_data)
                if img is not None:
                    self._depth = _disp_to_depth_mm(img)
                    self._depth_ts = time.monotonic()
                    self._merge()
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            if self._running:
                if self._is_not_found(exc):
                    if not self._not_found_logged:
                        self._not_found_logged = True
                        print(f"\n[oak_driver:{self.side}] service '{self.service_name}' not available — camera offline")
                    return  # don't retry
                print(f"\n[oak_driver:{self.side}] disparity stream error: {exc}")

    def _merge(self) -> None:
        """Publish a CameraFrame when both RGB and depth are available and recent."""
        if self._rgb is None or self._depth is None:
            return
        if abs(self._rgb_ts - self._depth_ts) > 2.0:
            return
        self._latest = CameraFrame(
            rgb=self._rgb,
            depth=self._depth,
            side=self.side,
            timestamp=time.monotonic(),
        )
