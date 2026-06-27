"""Unit tests for navigation/telemetry.py — the per-scan run logger."""
import json

from navigation.telemetry import TelemetryLogger


def test_logger_writes_one_json_line_per_record(tmp_path):
    p = tmp_path / "run.jsonl"
    log = TelemetryLogger(str(p), flush_every=2)
    assert log.enabled
    for i in range(5):
        log.log({"state": "FOLLOW", "conf": 0.9, "n": 500 + i})
    log.close()
    lines = p.read_text().strip().splitlines()
    assert len(lines) == 5
    rec = json.loads(lines[0])
    assert rec["state"] == "FOLLOW"
    assert rec["n"] == 500
    assert "t" in rec                       # elapsed time auto-added
    assert log.count == 5


def test_logger_never_raises_on_bad_path(tmp_path):
    # A directory path that cannot be a file → logger disables, swallows logs.
    bad = tmp_path  # an existing directory
    log = TelemetryLogger(str(bad))
    assert log.enabled is False
    log.log({"state": "X"})                 # must not raise
    log.close()
    assert log.count == 0


def test_logger_records_are_pandas_loadable(tmp_path):
    pd = __import__("importlib").util.find_spec("pandas")
    p = tmp_path / "run.jsonl"
    log = TelemetryLogger(str(p))
    for i in range(3):
        log.log({"state": "ACQUIRE", "conf": 0.1 * i, "lateral": -0.05 * i})
    log.close()
    # Validate as JSONL regardless of pandas availability.
    recs = [json.loads(ln) for ln in p.read_text().splitlines()]
    assert [r["state"] for r in recs] == ["ACQUIRE", "ACQUIRE", "ACQUIRE"]
    if pd is not None:
        import pandas as pandas_mod
        df = pandas_mod.read_json(str(p), lines=True)
        assert list(df["conf"]) == [0.0, 0.1, 0.2]
