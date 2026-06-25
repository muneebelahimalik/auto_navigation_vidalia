"""Unit tests for navigation/state_logic.py — pure navigator branch logic."""
from navigation.state_logic import (
    follow_loss_is_row_end, follow_loss_action, approach_action)

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
