"""
trajectory_generator.py

This module provides functions to generate time-parameterized trajectories
from both smoothed and raw waypoint paths.

Author: Aatish Om
"""


def generate_trajectory(smooth_path, velocity=0.1):
    """
    Generate a time-stamped trajectory from a smoothed path.

    Args:
        smooth_path: List of (x, y) tuples representing the smoothed path.
        velocity: Constant velocity used to determine time steps.

    Returns:
        List of (x, y, t) tuples representing the trajectory.
    """
    if len(smooth_path) < 2:
        raise ValueError("Trajectory generation requires at least two path points.")

    trajectory = []
    time = 0.0

    for i in range(len(smooth_path) - 1):
        x0, y0 = smooth_path[i]
        x1, y1 = smooth_path[i + 1]
        dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        dt = dist / velocity
        time += dt
        trajectory.append((x0, y0, time))

    # Add the final point
    trajectory.append((*smooth_path[-1], time))
    return trajectory


def generate_dense_trajectory_from_raw_path(waypoints, velocity=0.1, points_per_segment=10):
    """
    Generate a dense trajectory by linearly interpolating between raw waypoints.
    Useful for visualizing unsmoothed paths.

    Args:
        waypoints: List of (x, y) tuples representing original waypoints.
        velocity: Constant velocity used to determine time steps.
        points_per_segment: Number of interpolated points between each segment.

    Returns:
        List of (x, y, t) tuples representing the dense trajectory.
    """
    trajectory = []
    time = 0.0

    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        segment_length = (dx ** 2 + dy ** 2) ** 0.5
        dt = segment_length / velocity

        for j in range(points_per_segment):
            ratio = j / points_per_segment
            xi = x0 + ratio * dx
            yi = y0 + ratio * dy
            t = time + ratio * dt
            trajectory.append((xi, yi, t))

        time += dt

    # Add the final waypoint
    trajectory.append((*waypoints[-1], time))
    return trajectory

