"""--slam-hz: the background mapper processes at most N scans per second, and
0 keeps the unlimited behaviour (up to 2026-11-06)."""
import time

import numpy as np

from slam.slam_runner import SlamRunner


class _Engine:
    def __init__(self):
        self.calls = 0

    def process_scan(self, pts, odom_delta=None):
        self.calls += 1


def _feed(max_hz, seconds=1.0):
    eng = _Engine()
    r = SlamRunner(eng, max_hz=max_hz, idle_sleep=0.001)
    r.start()
    pts = np.ones((10, 3))
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        r.submit(pts, (0.0, 0.0, 0.0))
        time.sleep(0.01)                 # 100 scans/s offered
    r.stop()
    return eng.calls


def test_slam_rate_limited():
    assert _feed(2.0) <= 3


def test_slam_unlimited_when_zero():
    assert _feed(0.0) >= 30
