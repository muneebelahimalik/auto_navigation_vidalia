"""make_run_schedule.py: the Graeco-Latin schedule is balanced."""
from collections import Counter

from scripts.make_run_schedule import schedule


def test_every_controller_once_per_session_bed_set_and_position():
    for seed in range(20):
        rows = schedule("TP1", 1, seed)
        assert len(rows) == 9
        for key in ("session", "bed_set", "run_pos"):
            pairs = Counter((r["controller"], r[key]) for r in rows)
            assert len(pairs) == 9 and set(pairs.values()) == {1}, (seed, key)
        for s in (1, 2, 3):                       # a session uses three different sets
            assert len({r["bed_set"] for r in rows if r["session"] == s}) == 3


def test_first_turn_direction_balances_over_two_visits():
    for seed in range(10):
        a = schedule("TP1", 1, seed)
        b = schedule("TP1", 2, seed + 1000)
        for c in ("PP", "MPC", "RL"):
            turns = [r["first_turn"] for r in a + b if r["controller"] == c]
            assert Counter(turns) == {"L": 3, "R": 3}, (seed, c, turns)
        assert all(("--beds-left 2" in r["command"]) == (r["first_turn"] == "L") for r in a + b)
