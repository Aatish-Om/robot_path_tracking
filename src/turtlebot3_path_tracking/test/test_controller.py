# test_controller.py
from turtlebot3_path_tracking.path_tracker.controller import pure_pursuit_controller


def test_pure_pursuit_straight_line():
    path = [(x, 0, x * 0.1) for x in range(10)]  # Straight path
    pose = (0, 0, 0)  # Facing along X-axis
    v, w = pure_pursuit_controller(pose, path)
    assert v > 0
    assert abs(w) < 0.1  # Shouldn’t rotate much


def test_pure_pursuit_goal_reached():
    path = [(0, 0, 0)]
    pose = (0, 0, 0)
    v, w = pure_pursuit_controller(pose, path)
    assert v == 0.0
    assert w == 0.0


def test_pure_pursuit_turning():
    path = [(0, 0, 0), (1, 1, 1)]
    pose = (0, 0, 3.14)  # Facing opposite
    v, w = pure_pursuit_controller(pose, path)
    assert v > 0
    assert abs(w) > 0.5  # Should rotate significantly
