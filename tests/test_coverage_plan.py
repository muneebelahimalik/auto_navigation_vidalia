"""Serpentine side selection from the operator's field layout.

Field 2026-10-16: started on the second bed from the right (rightmost bed
unplanted), the robot took the default RIGHT first turn into the unplanted
side.  With ``--beds-left/--beds-right`` the sweep goes toward the beds."""
from navigation.coverage_plan import plan_serpentine


def test_no_layout_keeps_old_defaults():
    p = plan_serpentine(None, None)
    assert (p.turn_dir, p.rows, p.headland) == ("right", 1, False)
    p = plan_serpentine(None, None, rows=4, turn_dir="left", headland=True)
    assert (p.turn_dir, p.rows, p.headland) == ("left", 4, True)


def test_field_case_sweeps_toward_the_planted_beds():
    # second-from-right start, rightmost bed unplanted → 0 to the right
    p = plan_serpentine(5, 0)
    assert p.turn_dir == "left" and p.rows == 6 and p.headland


def test_rows_capped_at_the_field_edge():
    p = plan_serpentine(0, 2, rows=6)
    assert p.turn_dir == "right" and p.rows == 3
    assert any("capped" in n for n in p.notes)


def test_explicit_rows_below_capacity_kept():
    p = plan_serpentine(4, 1, rows=2)
    assert p.turn_dir == "left" and p.rows == 2 and p.headland


def test_explicit_turn_dir_wins_and_caps_rows():
    p = plan_serpentine(3, 1, turn_dir="right")
    assert p.turn_dir == "right" and p.rows == 2


def test_single_bed_needs_no_turn():
    p = plan_serpentine(0, 0)
    assert p.rows == 1 and not p.headland


def test_tie_defaults_right_and_warns():
    p = plan_serpentine(2, 2)
    assert p.turn_dir == "right" and p.rows == 3
    assert any("second run" in n for n in p.notes)
