#!/usr/bin/env python3
"""
scan_recorder.py — Stream corrected LiDAR scans to disk during a run.

So a --record run keeps the raw point-cloud time series (not just one
representative scan), you can render the perception figures — or animations —
from ANY moment offline, later.

Writes into ``<run>/scans/``:
    scan_00000.npy, scan_00001.npy, …   Nx3 corrected robot-frame clouds
    index.csv                            file, t, state, lateral, heading_deg, conf, n

The heavy work (numpy save) runs in a **background daemon thread** fed by an
O(1) ``submit`` from the control loop, so recording can never slow or destabilise
driving.  Errors are swallowed; if the writer can't keep up it drops scans
(counted) rather than blocking.  Saving is downsampled by ``every`` (default
every 10th scan ≈ 1 Hz); ``every=1`` saves every scan (full fidelity).
"""

from __future__ import annotations

import csv
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np


class ScanRecorder:
    def __init__(self, out_dir, *, every: int = 10, queue_max: int = 64) -> None:
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.every = max(1, int(every))
        self._q: deque = deque(maxlen=queue_max)
        self._lock = threading.Lock()
        self._stop = False
        self._seen = 0          # scans submitted (before downsample)
        self._sub = 0           # scans accepted for saving
        self.saved = 0
        self.dropped = 0
        self._t0 = time.monotonic()
        self._index = open(self.dir / "index.csv", "w", newline="")
        self._w = csv.writer(self._index)
        self._w.writerow(["file", "t", "state", "lateral", "heading_deg", "conf", "n"])
        self._thread = threading.Thread(target=self._loop, daemon=True, name="scanrec")
        self._thread.start()

    # ------------------------------------------------------------------
    def submit(self, pts, meta: dict) -> None:
        """Hand a corrected Nx3 scan + metadata to the writer.  Non-blocking."""
        self._seen += 1
        if pts is None or len(pts) == 0 or (self._seen % self.every):
            return
        with self._lock:
            if len(self._q) >= self._q.maxlen:
                self.dropped += 1
                return
            idx = self._sub
            self._sub += 1
            self._q.append((idx, np.asarray(pts, dtype=np.float32).copy(), dict(meta)))

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop or self._q:
            item = None
            with self._lock:
                if self._q:
                    item = self._q.popleft()
            if item is None:
                time.sleep(0.01)
                continue
            idx, pts, meta = item
            try:
                fn = f"scan_{idx:05d}.npy"
                np.save(self.dir / fn, pts)
                self._w.writerow([
                    fn, round(meta.get("t", time.monotonic() - self._t0), 3),
                    meta.get("state", ""), round(float(meta.get("lateral", 0.0)), 4),
                    round(float(meta.get("heading_deg", 0.0)), 3),
                    round(float(meta.get("conf", 0.0)), 3), int(meta.get("n", 0))])
                self.saved += 1
            except Exception:
                pass

    # ------------------------------------------------------------------
    def stop(self, timeout: float = 5.0) -> None:
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)
        try:
            self._index.flush()
            self._index.close()
        except Exception:
            pass
