"""
path_smoother.py

Generates a smooth 2D path from a sequence of discrete waypoints using cubic spline interpolation.
This helps improve trajectory continuity for better control and tracking.

Author: Aatish Om
"""

import numpy as np
from scipy.interpolate import CubicSpline


def smooth_path(waypoints, num_points=100):
    """
    Smooth a list of 2D waypoints using cubic spline interpolation.

    Args:
        waypoints (list of tuple): List of (x, y) tuples representing discrete waypoints.
        num_points (int): Number of points to sample along the smoothed path.

    Returns:
        list of tuple: List of (x, y) points forming a smooth path.

    Raises:
        ValueError: If fewer than two waypoints are provided.
    """
    if len(waypoints) < 2:
        raise ValueError("Need at least two waypoints to smooth a path.")

    waypoints = np.array(waypoints)

    # Compute cumulative distances between waypoints
    distances = np.cumsum(
        np.r_[0, np.linalg.norm(np.diff(waypoints, axis=0), axis=1)]
    )

    # Fit cubic splines for x and y coordinates
    cs_x = CubicSpline(distances, waypoints[:, 0])
    cs_y = CubicSpline(distances, waypoints[:, 1])

    # Sample the path at uniform intervals along the arc-length
    sampled_distances = np.linspace(0, distances[-1], num_points)
    smooth_path = [(cs_x(s), cs_y(s)) for s in sampled_distances]

    return smooth_path

