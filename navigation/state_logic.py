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


def follow_loss_action(
    miss_count: int,
    is_end: bool,
    *,
    row_end_frames: int,
    follow_miss_thresh: int,
) -> str:
    """Decide what a low-confidence FOLLOW scan should do.

    ``miss_count`` is how many CONSECUTIVE sub-threshold scans have occurred
    (it resets to 0 the instant crop reappears, so an *intermittent* dropout —
    e.g. crop flickering out on a slope — never accumulates it).  ``is_end`` is
    ``follow_loss_is_row_end(...)``.

    Returns one of:
      * ``"ROW_END"`` — a real row end: the geometry looks like an end AND the
        crop has been *continuously* absent for ``row_end_frames`` scans.  The
        long continuous requirement is what stops a brief slope dropout (where
        the crop keeps reappearing and resets ``miss_count``) from faking a row
        end and triggering a spurious headland turn.
      * ``"ACQUIRE"`` — not a row end (crop still partly present) and the lock
        has been poor for ``follow_miss_thresh`` scans: re-acquire.
      * ``"WAIT"`` — pause in FOLLOW (v=0) and keep waiting; not yet conclusive.
    """
    if is_end:
        return "ROW_END" if miss_count >= row_end_frames else "WAIT"
    if miss_count >= follow_miss_thresh:
        return "ACQUIRE"
    return "WAIT"


def approach_action(
    acquire_count: int,
    approach_dist: float,
    *,
    approach_acquire_frames: int,
    approach_max_dist: float,
) -> str:
    """Decide what the post-turn APPROACH leg should do.

    After a headland U-turn the robot drives forward into the next row until
    perception re-acquires it (a stationary ACQUIRE there hangs forever because
    no crop is in the ROI yet).

      * ``"FOLLOW"`` — the next row is solidly detected (``acquire_count`` confident
        scans): hand control to the row follower.
      * ``"STOP"``   — driven ``approach_max_dist`` without finding a row (field
        edge or the turn overshot): stop rather than drive on blindly.
      * ``"DRIVE"``  — keep creeping forward and searching.
    """
    if acquire_count >= approach_acquire_frames:
        return "FOLLOW"
    if approach_dist >= approach_max_dist:
        return "STOP"
    return "DRIVE"


def turn_reacquired(
    valid: bool,
    confidence: float,
    heading_error: float,
    lateral_offset: float,
    *,
    reacquire_conf: float,
    align_thresh: float,
    offset_thresh: float,
) -> bool:
    """True when, mid-U-turn, the next row is detected lined up AHEAD.

    The headland turn keeps arcing until this fires (then hands to FOLLOW),
    instead of trusting the wheel-odometry heading to know when 180° is reached
    — on a skid-steer that heading over-reports rotation badly, so fixed-angle
    turns under-rotated and ended pointing across the rows.

    A genuine "lined up on the next strip" lock is confident AND aligned (small
    heading error) AND roughly centred (small lateral offset) — all three, so a
    brief glimpse of crop crossing the field of view mid-rotation does not count.
    The caller additionally requires this to hold for several consecutive scans
    and only after a minimum arc has been driven.
    """
    return (
        valid
        and confidence >= reacquire_conf
        and abs(heading_error) <= align_thresh
        and abs(lateral_offset) <= offset_thresh
    )


def post_turn_loss_action(
    action: str,
    post_turn: bool,
    is_end: bool,
) -> str:
    """Re-route a FOLLOW loss that happens while still settling onto the NEXT
    row right after a headland turn.

    The U-turn → APPROACH → FOLLOW handoff lands the robot on the next row with
    a marginal, partly-in-ROI view (field: n≈70 vs ≈700 on a well-centred row,
    confidence flickering around the 0.35 threshold).  A normal FOLLOW loss
    there drops to a *stationary* ACQUIRE — but a stationary sensor on a sparse,
    half-visible row cannot improve its view, so it hangs at the row start
    (field failure: "turned … then went to follow and acquire" and stuck).

    While ``post_turn`` is set (from the end of the turn until the next-row lock
    has settled a solid distance into FOLLOW) and the loss is NOT a real row end,
    convert an ``"ACQUIRE"`` decision into ``"APPROACH"`` so the robot keeps
    creeping straight forward and re-locks as the row fills the ROI.  The
    creep is still bounded by ``approach_max_dist`` (via ``approach_action``), so
    a genuinely missing row still stops rather than driving on forever.

    Pass-through for every other case (mid-field losses, row ends, WAIT).
    """
    if action == "ACQUIRE" and post_turn and not is_end:
        return "APPROACH"
    return action


def acquire_rowend_escape(
    came_from_follow: bool,
    rowend_count: int,
    *,
    row_end_frames: int,
) -> bool:
    """True when a stalled ACQUIRE should escape to ROW_END (the headland).

    The FOLLOW-exit row-end check can be fooled into dropping to ACQUIRE at a
    real row end — e.g. residual sparse clutter (a few straggler/weed returns)
    keeps ``row_end_confidence`` below threshold, or a short row leaves
    ``row_dist`` under ``row_end_min_dist``.  ACQUIRE is otherwise a dead end
    (stationary, hunting forever for a row that isn't there), so the robot hangs
    at the field edge.

    When ACQUIRE was entered *from FOLLOW* (``came_from_follow`` — we were
    actively on a row, not at startup or post-turn) and the crop band ahead has
    been genuinely empty for ``row_end_frames`` consecutive scans
    (``rowend_count``), the row has ended → escape to ROW_END.
    """
    return came_from_follow and rowend_count >= row_end_frames

