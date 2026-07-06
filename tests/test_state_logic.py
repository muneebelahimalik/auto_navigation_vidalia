"""Unit tests for navigation/state_logic.py — pure navigator branch logic."""
from navigation.state_logic import (
    follow_loss_is_row_end, follow_loss_action, approach_action,
    acquire_rowend_escape, post_turn_loss_action, turn_reacquired)
import math

APPROACH_FRAMES = 3
APPROACH_MAX = 3.0


def appr(acq_count, dist):
    return approach_action(
        acq_count, dist,
        approach_acquire_frames=APPROACH_FRAMES, approach_max_dist=APPROACH_MAX)

ROW_END_FRAMES = 15
FOLLOW_MISS_THRESH = 4


def action(miss_count, is_end):
    return follow_loss_action(
        miss_count, is_end,
        row_end_frames=ROW_END_FRAMES, follow_miss_thresh=FOLLOW_MISS_THRESH)

MIN_DIST = 1.5
END_CONF = 0.70


def is_end(row_dist, row_end_conf):
    return follow_loss_is_row_end(
        row_dist, row_end_conf,
        row_end_min_dist=MIN_DIST, row_end_conf=END_CONF)


def test_real_row_end_routes_to_row_end():
    """Drove a full row (6.6 m) and the crop band ahead is empty -> ROW_END.
    Regression for the field bug where the row end was misread as a lost lock
    and the headland turn never started."""
    assert is_end(6.6, 1.0) is True


def test_short_lock_loss_is_not_row_end():
    """Lost the lock before driving a real row -> re-ACQUIRE, do not turn."""
    assert is_end(0.4, 1.0) is False


def test_partial_crop_present_is_not_row_end():
    """Driven far but crop band still partly populated (low row-end conf) ->
    a mid-row gap, not the headland -> ACQUIRE, do not turn."""
    assert is_end(6.6, 0.40) is False


def test_thresholds_are_inclusive():
    """At exactly the distance and confidence thresholds it counts as a row end."""
    assert is_end(MIN_DIST, END_CONF) is True


def test_just_below_thresholds_is_not_row_end():
    assert is_end(MIN_DIST - 0.01, 1.0) is False
    assert is_end(6.6, END_CONF - 0.01) is False


# ---------------------------------------------------------------------------
# follow_loss_action — what a low-confidence FOLLOW scan should do
# ---------------------------------------------------------------------------

def test_slope_dropout_does_not_turn():
    """Regression: a brief crop dropout on a slope (looks like a row end via
    distance+end-conf, but only a few consecutive misses) must NOT turn — the
    field bug where the robot took a headland turn mid-row on a slope."""
    assert action(1, True) == "WAIT"
    assert action(4, True) == "WAIT"      # 4 was the old (buggy) trigger
    assert action(14, True) == "WAIT"


def test_sustained_absence_is_row_end():
    """A long CONTINUOUS absence past a real row IS the row end → turn."""
    assert action(ROW_END_FRAMES, True) == "ROW_END"
    assert action(ROW_END_FRAMES + 5, True) == "ROW_END"


def test_non_row_end_loss_reacquires():
    """Crop still partly present (not a row end) and a sustained poor lock →
    re-acquire, never turn."""
    assert action(FOLLOW_MISS_THRESH, False) == "ACQUIRE"
    assert action(20, False) == "ACQUIRE"


def test_short_non_row_end_loss_waits():
    """A 1–3 scan dropout that isn't a row end just pauses."""
    assert action(1, False) == "WAIT"
    assert action(FOLLOW_MISS_THRESH - 1, False) == "WAIT"


# ---------------------------------------------------------------------------
# approach_action — post-turn drive-into-next-row leg
# ---------------------------------------------------------------------------

def test_approach_drives_until_row_found():
    """No row yet, within range → keep creeping forward (the fix for the robot
    hanging stationary in the headland after the U-turn)."""
    assert appr(0, 0.0) == "DRIVE"
    assert appr(2, 1.5) == "DRIVE"          # 2 confident frames, not yet 3


def test_approach_hands_off_to_follow_when_locked():
    assert appr(APPROACH_FRAMES, 1.0) == "FOLLOW"
    assert appr(APPROACH_FRAMES + 2, 0.5) == "FOLLOW"


def test_approach_stops_at_field_edge():
    """Drove the full search distance without finding a row → stop, don't drive
    on blindly (field edge or the turn overshot)."""
    assert appr(0, APPROACH_MAX) == "STOP"
    assert appr(1, APPROACH_MAX + 0.5) == "STOP"


def test_approach_follow_beats_stop_at_max_dist():
    """If the row locks on the same scan the max distance is reached, follow."""
    assert appr(APPROACH_FRAMES, APPROACH_MAX) == "FOLLOW"


# ---------------------------------------------------------------------------
# turn_reacquired — end the U-turn when the next row is lined up ahead
# ---------------------------------------------------------------------------

CONF = 0.55
ALIGN = math.radians(11.5)
OFF = 0.40


def reacq(valid, conf, hdg, off):
    return turn_reacquired(valid, conf, hdg, off,
                           reacquire_conf=CONF, align_thresh=ALIGN, offset_thresh=OFF)


def test_reacquire_fires_on_confident_aligned_centred_row():
    """The next row is dead ahead, aligned and centred → end the turn."""
    assert reacq(True, 0.80, math.radians(2.0), 0.05) is True
    assert reacq(True, CONF, ALIGN, OFF) is True          # exactly at thresholds


def test_reacquire_rejects_low_confidence():
    """A faint/cross-row glimpse mid-rotation must not end the turn."""
    assert reacq(True, 0.40, math.radians(2.0), 0.05) is False


def test_reacquire_rejects_misaligned_row():
    """Confident but the robot is still pointing well off the row → keep arcing."""
    assert reacq(True, 0.90, math.radians(25.0), 0.05) is False


def test_reacquire_rejects_off_centre_row():
    """Confident and aligned but the strip is far to the side (not the one we
    should straddle) → keep arcing."""
    assert reacq(True, 0.90, math.radians(2.0), 0.80) is False


def test_reacquire_rejects_invalid_estimate():
    assert reacq(False, 0.90, 0.0, 0.0) is False


# ---------------------------------------------------------------------------
# post_turn_loss_action — keep creeping onto the next row after a U-turn
# ---------------------------------------------------------------------------

def test_post_turn_marginal_loss_keeps_creeping():
    """Field bug: after the U-turn the next-row lock is marginal (n≈70,
    conf flickering), FOLLOW drops to a stationary ACQUIRE and the robot hangs.
    While settling post-turn, a non-row-end ACQUIRE becomes APPROACH so the
    robot keeps creeping forward and re-locks as the row fills the ROI."""
    assert post_turn_loss_action("ACQUIRE", True, False) == "APPROACH"


def test_post_turn_does_not_hijack_row_end():
    """A real row end during settling still routes to the row end, never creep."""
    assert post_turn_loss_action("ROW_END", True, True) == "ROW_END"
    # An ACQUIRE that IS classified a row end is left alone too.
    assert post_turn_loss_action("ACQUIRE", True, True) == "ACQUIRE"


def test_post_turn_passes_through_when_settled():
    """Once settled (post_turn cleared), a mid-field loss drops to ACQUIRE as
    normal — no perpetual creeping."""
    assert post_turn_loss_action("ACQUIRE", False, False) == "ACQUIRE"


def test_post_turn_leaves_wait_alone():
    """WAIT (brief dropout) is untouched in every case."""
    assert post_turn_loss_action("WAIT", True, False) == "WAIT"
    assert post_turn_loss_action("WAIT", False, False) == "WAIT"


# ---------------------------------------------------------------------------
# acquire_rowend_escape — unstick ACQUIRE at a row end
# ---------------------------------------------------------------------------

ROW_END_FRAMES_ESC = 15


def escape(came_from_follow, count):
    return acquire_rowend_escape(
        came_from_follow, count, row_end_frames=ROW_END_FRAMES_ESC)


def test_acquire_escapes_to_rowend_after_following():
    """Stuck in ACQUIRE after FOLLOW with the crop band empty long enough → the
    row ended, escape to ROW_END (the field bug: residual clutter sent FOLLOW
    to ACQUIRE and it hung forever at the row end)."""
    assert escape(True, ROW_END_FRAMES_ESC) is True
    assert escape(True, ROW_END_FRAMES_ESC + 10) is True


def test_acquire_no_escape_at_startup():
    """ACQUIRE not entered from FOLLOW (startup / post-turn) must NOT escape —
    there is no row behind us to have 'ended'."""
    assert escape(False, 100) is False


def test_acquire_no_escape_before_sustained():
    """A brief empty patch in ACQUIRE is not yet a row end."""
    assert escape(True, ROW_END_FRAMES_ESC - 1) is False
    assert escape(True, 0) is False


# ---------------------------------------------------------------------------
# rowend_count_update — the counter must survive row_end_confidence FLICKER
# (the field hang: robot stuck in ACQUIRE at the row end for 16 s because the
# residual-clutter flicker kept resetting the confirmation counter to 0)
# ---------------------------------------------------------------------------

from navigation.state_logic import rowend_count_update

ACQ_CONF = 0.35
REACQ = 0.70


def _run_counter(scans):
    """Replay (confidence, row_end_confidence) scans; return frames-to-escape or
    None if it never reached ROW_END_FRAMES_ESC."""
    c = 0
    for i, (conf, endc) in enumerate(scans):
        c = rowend_count_update(c, conf, endc,
                                acquire_conf=ACQ_CONF, row_end_conf=REACQ)
        if c >= ROW_END_FRAMES_ESC:
            return i + 1
    return None


def test_rowend_counter_survives_flicker_field_case():
    """The field row-end signature: crop confidence pinned below acquire_conf,
    row_end_confidence oscillating above/below its threshold every couple of
    scans.  The counter must still accumulate to the escape (the OLD reset-on-dip
    logic never would → the robot hung)."""
    # conf never re-acquires (all < 0.35); end flickers high/low
    scans = [(0.30, 0.85), (0.31, 0.60), (0.29, 0.80), (0.33, 0.45),
             (0.28, 0.90), (0.32, 0.20)] * 6           # 36 scans
    assert _run_counter(scans) is not None              # DOES escape now
    # the old naive logic (reset to 0 on any end-conf dip) would never reach 15
    old = 0
    for conf, endc in scans:
        old = old + 1 if endc >= REACQ else 0
        assert old < ROW_END_FRAMES_ESC                 # confirms the bug it fixes


def test_rowend_counter_resets_only_on_confident_reacquire():
    """A genuinely confident re-acquire (real row back in front) cancels the
    escape; a mere row_end_confidence dip does not."""
    assert rowend_count_update(14, 0.10, 0.30,
                               acquire_conf=ACQ_CONF, row_end_conf=REACQ) == 14  # ambiguous → hold
    assert rowend_count_update(14, 0.80, 0.30,
                               acquire_conf=ACQ_CONF, row_end_conf=REACQ) == 0   # real row → reset
    assert rowend_count_update(14, 0.20, 0.90,
                               acquire_conf=ACQ_CONF, row_end_conf=REACQ) == 15  # empty → increment


def test_rowend_counter_no_false_escape_on_recovering_row():
    """If the crop keeps coming back confidently (a transient mid-row dropout,
    not a row end), the counter is repeatedly reset and never escapes."""
    scans = [(0.80, 0.10), (0.20, 0.90), (0.75, 0.10), (0.25, 0.85)] * 10
    assert _run_counter(scans) is None
