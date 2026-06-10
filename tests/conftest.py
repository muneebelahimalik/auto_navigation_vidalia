"""Pytest config — make the workspace root importable.

These tests are pure-software (numpy only): no LiDAR, cameras, canbus or
farm-ng SDK required, so they run on any dev machine:

    pip install numpy pytest
    pytest tests/ -v
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
