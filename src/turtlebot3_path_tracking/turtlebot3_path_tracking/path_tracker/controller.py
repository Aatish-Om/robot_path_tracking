"""
controller.py

Trajectory tracking controller using the Pure Pursuit algorithm.

This module contains:
- A function to compute the nearest point on a trajectory.
- A Pure Pursuit controller to compute linear and angular velocity commands
  based on the robot’s current pose and a target path.

Author: Aatish Om
"""

import numpy as np


def nearest_point_index(path, x, y):
    """
    Find the index of the point in the path closest to the given (x, y) position.

    Args:
        path (list of tuples): The path as a list of (x, y, t) points.
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.

    Returns:
        int: Index of the closest point in the path.
    """
    return np.argmin([np.hypot(x - px, y - py) for px, py, _ in path])


def pure_pursuit_controller(current_pose, path, lookahead=0.3):
    """
    Compute velocity commands using the Pure Pursuit method.

    Args:
        current_pose (tuple): Current robot pose as (x, y, yaw).
        path (list of tuples): Trajectory as list of (x, y, t).
        lookahead (float): Minimum distance to look ahead on the path.

    Returns:
        tuple: (v, w) where v is linear velocity and w is angular velocity.
    """
    x, y, theta = current_pose

    # Find the closest point on the trajectory
    index = nearest_point_index(path, x, y)

    # Search for a lookahead point on the path
    for i in range(index, len(path)):
        px, py, _ = path[i]
        if np.hypot(px - x, py - y) >= lookahead:
            goal = (px, py)
            break
    else:
        return 0.0, 0.0  # Stop if no further point is found

    # Compute control action
    dx = goal[0] - x
    dy = goal[1] - y
    angle_to_goal = np.arctan2(dy, dx)
    angle_error = angle_to_goal - theta

    # Normalize angular error to [-pi, pi]
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

    # Compute velocities
    linear_velocity = 0.1
    angular_velocity = 1.5 * angle_error

    return linear_velocity, angular_velocity

