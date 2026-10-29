"""--lidar-height: one override must reach every module that measures height."""
import numpy as np

import lidar.obstacle_filter as of
import navigation.row_perception as rp
import navigation.row_safety as rs


def test_set_mount_height_rebinds_all_importers():
    old = of.LIDAR_MOUNT_HEIGHT
    try:
        prev = of.set_mount_height(1.234)
        assert prev == old
        assert of.LIDAR_MOUNT_HEIGHT == 1.234
        assert rp.LIDAR_MOUNT_HEIGHT == 1.234
        assert rs.LIDAR_MOUNT_HEIGHT == 1.234
    finally:
        of.set_mount_height(old)
    assert rp.LIDAR_MOUNT_HEIGHT == old and rs.LIDAR_MOUNT_HEIGHT == old


def test_safety_uses_overridden_height():
    """A post 0.9 m tall, 2 m ahead: an obstacle only at the right mount height."""
    old = of.LIDAR_MOUNT_HEIGHT
    ys = np.linspace(1.9, 2.1, 10)
    try:
        for H, expect in ((0.99, True), (0.40, False)):
            of.set_mount_height(H)
            pts = np.column_stack([np.zeros(10), ys, np.full(10, 0.90 - 0.99)])
            st = rs.SafetyMonitor().check(pts)
            assert st.forward_blocked is expect, H
    finally:
        of.set_mount_height(old)


def test_lidar_validate_times_out_instead_of_hanging():
    """A previous run still bound to UDP 2368 (SO_REUSEPORT) receives the whole
    stream — startup must fail with a diagnosis, not wait forever."""
    import asyncio
    import time

    class SilentLidar:
        async def scan_stream(self):
            await asyncio.sleep(3600)
            yield []

    t0 = time.monotonic()
    ok = asyncio.run(of.validate_lidar_startup(SilentLidar(), timeout_s=0.3))
    assert ok is False and time.monotonic() - t0 < 2.0
