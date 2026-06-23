#!/usr/bin/env python3
"""
state_logic.py — Pure (dependency-free) decision helpers for the navigator
state machine.

Kept separate from row_navigator.py so the branching logic can be unit-tested
without importing the farm-ng SDK (which row_navigator pulls in via the canbus
interface and is unavailable off-robot).
"""
from __future__ import annotations


def follow_loss_is_row_end(
    row_dist: float,
    row_end_confidence: float,
    *,
    row_end_min_dist: float,
    row_end_conf: float,
) -> bool:
    """True when a sustained low-confidence loss in FOLLOW is the row END.

    Crop running out at the headland trips BOTH the row-end signal (the crop
    band ahead empties) and the low-confidence miss counter (confidence
    collapses) at the same instant.  The miss counter (follow_miss_thresh, 4)
    is shorter than the row-end confirmation (row_end_frames, 8), so a naive
    state machine always reaches the ACQUIRE branch first and the headland turn
    never starts.

    When the miss counter trips, this distinguishes the two causes:
      * row END  → we have driven a real row (``row_dist`` past
        ``row_end_min_dist``) AND the crop band ahead is genuinely empty
        (``row_end_confidence`` at/above ``row_end_conf``);
      * mid-row loss → anything else (short lock, or crop still partly present),
        which should re-ACQUIRE rather than turn.
    """
    return row_dist >= row_end_min_dist and row_end_confidence >= row_end_conf
