"""
coverage_plan.py — which way to sweep a field in a serpentine.

A serpentine whose FIRST U-turn is to the left keeps moving the robot left
(turn left → drive back → turn right, which in the ground frame is again one
bed further left …), so the first-turn direction fixes the SIDE of the field
that gets covered.  Starting on an interior bed, the robot cannot know which
side still has beds to work — the 2026-10-16 onion run started on the second
bed from the right (the rightmost bed was unplanted), took the default RIGHT
turn into the unplanted side and stopped.  The LiDAR cannot settle this today
either: the onion seedlings are invisible to the VLP-16, so a planted and an
unplanted bed have the same relief.

So the operator states the layout once, before starting:

    --beds-left N  --beds-right M      beds still to cover on each side of the
                                       start bed (not counting the start bed)

and ``plan_serpentine`` turns that into the first-turn direction (toward the
side with more beds) and the number of rows the sweep can actually cover
(1 + beds on that side), so the robot never runs a U-turn off the field edge.
An explicit --turn-dir still wins (the row count is then capped by the beds on
THAT side).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class SerpentinePlan:
    turn_dir: str                 # "left" / "right" — direction of the FIRST U-turn
    rows: int                     # beds/rows to drive in total (incl. the start bed)
    headland: bool                # U-turns needed (rows > 1)
    notes: List[str] = field(default_factory=list)


def plan_serpentine(
    beds_left: Optional[int],
    beds_right: Optional[int],
    *,
    rows: Optional[int] = None,
    turn_dir: Optional[str] = None,
    headland: bool = False,
) -> SerpentinePlan:
    """Resolve the serpentine from the operator's field layout.

    ``rows`` / ``turn_dir`` are the explicit CLI values (None = not given).
    Without --beds-left/--beds-right this only fills the old defaults
    (1 row, first turn right), so existing commands behave exactly as before.
    """
    notes: List[str] = []
    if beds_left is None and beds_right is None:
        return SerpentinePlan(turn_dir or "right", rows if rows is not None else 1,
                              headland, notes)

    L = max(0, int(beds_left or 0))
    R = max(0, int(beds_right or 0))
    if turn_dir is not None:
        side = turn_dir                      # explicit operator choice always wins
    elif L > R:
        side = "left"
    elif R > L:
        side = "right"
    else:
        side = "right"
        if L > 0:
            notes.append(f"equal beds on both sides ({L}) — sweeping right; "
                         f"the other side needs a second run")
    capacity = 1 + (L if side == "left" else R)
    if rows is None:
        n = capacity
    else:
        n = min(int(rows), capacity)
        if rows > capacity:
            notes.append(f"--rows {rows} capped to {capacity}: only "
                         f"{capacity - 1} bed(s) to the {side} of the start bed")
    other = R if side == "left" else L
    if other > 0 and L != R:
        notes.append(f"{other} bed(s) to the {'right' if side == 'left' else 'left'} "
                     f"are NOT covered by this sweep")
    use_headland = headland or n > 1
    if n > 1 and not headland:
        notes.append("headland U-turns enabled (needed to cover more than one bed)")
    return SerpentinePlan(side, n, use_headland, notes)
