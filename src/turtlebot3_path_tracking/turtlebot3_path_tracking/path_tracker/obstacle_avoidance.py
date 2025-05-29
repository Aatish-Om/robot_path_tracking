"""
obstacle_avoidance.py

Provides obstacle detection and avoidance logic for a differential drive robot.
Includes functions to:
- Detect obstacles in front using LaserScan data.
- Choose a rerouting direction (left or right).
- Generate a bypass waypoint for smooth trajectory adjustment.

Author: Aatish Om
"""

import numpy as np


def should_avoid_obstacle(laser_ranges, threshold=1.0):
    """
    Determine if an obstacle is present in the robot's front sector.

    Args:
        laser_ranges (list of float): List of distance readings from LaserScan.
        threshold (float): Minimum distance (in meters) to trigger avoidance.

    Returns:
        bool: True if obstacle detected within threshold in the front sector.
    """
    if not laser_ranges or len(laser_ranges) < 20:
        return False

    n = len(laser_ranges)
    mid = n // 2
    arc_width = n // 12  # approx. 60-degree arc total (30 on each side)

    # Extract a front-facing sector of the scan
    front_sector = laser_ranges[mid - arc_width: mid + arc_width]
    front_sector = [r for r in front_sector if 0.05 < r < 3.5 and np.isfinite(r)]

    if not front_sector:
        return False

    return min(front_sector) < threshold


def choose_turn_direction(laser_ranges):
    """
    Choose the turn direction (left or right) based on available clearance.

    Args:
        laser_ranges (list of float): List of distance readings from LaserScan.

    Returns:
        str: 'left' if the left side has more clearance, else 'right'.
    """
    n = len(laser_ranges)
    third = n // 3

    left_sector = laser_ranges[2 * third:]
    right_sector = laser_ranges[:third]

    left_clearance = sum(d for d in left_sector if d > 0.01 and np.isfinite(d))
    right_clearance = sum(d for d in right_sector if d > 0.01 and np.isfinite(d))

    return 'left' if left_clearance > right_clearance else 'right'


def get_bypass_waypoint(current_pose, laser_ranges, forward_offset=1.0, lateral_offset=0.8):
    """
    Compute a bypass waypoint to avoid a detected obstacle.

    Args:
        current_pose (tuple): Current robot pose (x, y, yaw).
        laser_ranges (list of float): LaserScan distance readings.
        forward_offset (float): Distance to move forward.
        lateral_offset (float): Distance to shift sideways (left/right).

    Returns:
        tuple: (x, y) coordinates of the bypass waypoint.
    """
    x, y, yaw = current_pose
    direction = choose_turn_direction(laser_ranges)

    # Calculate forward movement vector
    forward_dx = forward_offset * np.cos(yaw)
    forward_dy = forward_offset * np.sin(yaw)

    # Calculate lateral movement vector
    lateral_dx = lateral_offset * np.cos(yaw + np.pi / 2)
    lateral_dy = lateral_offset * np.sin(yaw + np.pi / 2)

    if direction == 'left':
        bypass_x = x + forward_dx + lateral_dx
        bypass_y = y + forward_dy + lateral_dy
    else:
        bypass_x = x + forward_dx - lateral_dx
        bypass_y = y + forward_dy - lateral_dy

    return (bypass_x, bypass_y)

