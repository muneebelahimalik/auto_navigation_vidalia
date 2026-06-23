"""Unit tests for navigation/state_logic.py — pure navigator branch logic."""
from navigation.state_logic import follow_loss_is_row_end

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
