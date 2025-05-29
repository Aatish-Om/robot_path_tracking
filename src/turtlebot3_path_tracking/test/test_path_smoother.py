# test_path_smoother.py
import pytest
from turtlebot3_path_tracking.path_tracker.path_smoother import smooth_path


def test_smooth_zigzag_path():
    waypoints = [(0, 0), (1, 1), (2, 0), (3, 1)]
    smoothed = smooth_path(waypoints)
    assert len(smoothed) == 100
    assert isinstance(smoothed[0], tuple)


def test_smooth_single_point():
    waypoints = [(0, 0)]
    with pytest.raises(ValueError):
        smooth_path(waypoints)


def test_smooth_empty_path():
    waypoints = []
    with pytest.raises(ValueError):
        smooth_path(waypoints)
