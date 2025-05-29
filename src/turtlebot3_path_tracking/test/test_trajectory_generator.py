# test_trajectory_generator.py
import pytest
from turtlebot3_path_tracking.path_tracker.trajectory_generator import generate_trajectory, generate_dense_trajectory_from_raw_path
def test_dense_trajectory_length():
    waypoints = [(0, 0), (1, 0), (2, 0)]
    traj = generate_dense_trajectory_from_raw_path(
        waypoints, points_per_segment=5)
    assert len(traj) == 11  # 5+5+1


def test_trajectory_time_increasing():
    path = [(0, 0), (1, 0), (2, 0)]
    traj = generate_trajectory(path)
    times = [t for _, _, t in traj]
    assert all(t2 >= t1 for t1, t2 in zip(times, times[1:]))


def test_empty_waypoint_error():
    with pytest.raises(ValueError):
        generate_trajectory([])
