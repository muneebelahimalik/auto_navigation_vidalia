from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import depthai as dai
    _DEPTHAI_OK = True
except ImportError:
    _DEPTHAI_OK = False
    print("[oak_driver] depthai not installed — OAK-D cameras disabled")


@dataclass
class CameraFrame:
    """One captured frame pair from an OAK-D device."""
    rgb: np.ndarray    # HxWx3 BGR uint8
    depth: np.ndarray  # HxW uint16 mm
    side: str
    timestamp: float


class OakDriver:
    """Async driver for one OAK-D camera (left or right side)."""

    def __init__(self, side: str, device_id: str = "", fps: int = 10) -> None:
        self.side = side
        self.device_id = device_id
        self.fps = fps
        self._latest: Optional[CameraFrame] = None
        self._running = False

    def get_latest(self) -> Optional[CameraFrame]:
        return self._latest

    async def run(self) -> None:
        if not _DEPTHAI_OK:
            return

        loop = asyncio.get_event_loop()
        try:
            device = await loop.run_in_executor(None, self._open_device)
        except Exception as exc:
            print(f"[oak_driver:{self.side}] failed to open device: {exc}")
            return

        self._running = True
        try:
            rgb_q = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            depth_q = device.getOutputQueue("depth", maxSize=1, blocking=False)
            while self._running:
                rgb_msg = rgb_q.tryGet()
                depth_msg = depth_q.tryGet()
                if rgb_msg is not None and depth_msg is not None:
                    rgb = rgb_msg.getCvFrame()
                    depth = depth_msg.getFrame()
                    self._latest = CameraFrame(
                        rgb=rgb,
                        depth=depth,
                        side=self.side,
                        timestamp=time.monotonic(),
                    )
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            pass
        finally:
            device.close()

    async def stop(self) -> None:
        self._running = False

    def _open_device(self):
        pipeline = self._build_pipeline()
        if self.device_id:
            device_info = dai.DeviceInfo(self.device_id)
            return dai.Device(pipeline, device_info)
        return dai.Device(pipeline)

    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(640, 400)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(self.fps)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        mono_left.setFps(self.fps)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        mono_right.setFps(self.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setOutputSize(640, 400)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline
