#!/usr/bin/env python3
"""
telemetry.py — One-line-per-scan run logger for field analysis.

Writes a JSON Lines file (one JSON object per LiDAR scan) capturing the full
navigation state: perception (confidence, crop points, lateral/heading error,
row-end confidence, terrain grade/drop), the command output, the safety zones,
and the headland-turn state.  This is the data behind the "unified dashboard"
— load it straight into pandas for figures::

    import pandas as pd
    df = pd.read_json("logs/run_<ts>.jsonl", lines=True)

Design: logging must NEVER affect driving.  ``log()`` swallows every error and
the writer flushes periodically; if the file cannot be opened the logger
disables itself silently.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional, TextIO


class TelemetryLogger:
    def __init__(self, path: str, *, flush_every: int = 20) -> None:
        self.path = str(path)
        self._flush_every = flush_every
        self._n = 0
        self._t0 = time.monotonic()
        self._f: Optional[TextIO] = None
        try:
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
            self._f = open(self.path, "w", buffering=1)
        except OSError:
            self._f = None   # disabled — never raises into the caller

    @property
    def enabled(self) -> bool:
        return self._f is not None

    @property
    def count(self) -> int:
        return self._n

    # ------------------------------------------------------------------
    def log(self, record: dict) -> None:
        """Append one record. Never raises."""
        if self._f is None:
            return
        try:
            record.setdefault("t", round(time.monotonic() - self._t0, 4))
            self._f.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._n += 1
            if self._n % self._flush_every == 0:
                self._f.flush()
        except Exception:
            pass

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self._f is not None:
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass
            self._f = None
