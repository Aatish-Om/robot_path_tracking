#!/usr/bin/env python3
"""
main_tracker.py

ROS 2 Node for path following and obstacle-aware rerouting using TurtleBot3.

This script:
- Subscribes to Odometry and LaserScan data.
- Tracks a predefined path using a Pure Pursuit controller.
- Smooths the path using Cubic Splines if enabled.
- Monitors for obstacles and dynamically generates a bypass trajectory when needed.

Author: Aatish Om
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import numpy as np

from turtlebot3_path_tracking.path_tracker.path_smoother import smooth_path
from turtlebot3_path_tracking.path_tracker.trajectory_generator import (
    generate_trajectory,
    generate_dense_trajectory_from_raw_path
)
from turtlebot3_path_tracking.path_tracker.controller import pure_pursuit_controller
from turtlebot3_path_tracking.path_tracker.obstacle_avoidance import (
    should_avoid_obstacle,
    get_bypass_waypoint
)

# Predefined 2D waypoints
WAYPOINTS = [(0, 0), (1, 0), (2, 2), (3, 0), (4, 2)]


class PathFollower(Node):
    """
    ROS2 Node that controls the robot to follow a trajectory,
    and performs obstacle avoidance via smooth path rerouting.
    """

    def __init__(self):
        super().__init__('path_follower')

        # Parameter to choose between smoothed or raw path
        self.declare_parameter('use_smoothing', True)
        use_smoothing = self.get_parameter('use_smoothing').value

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # Internal states
        self.laser_ranges = []
        self.current_pose = None
        self.avoiding = False
        self.avoid_trajectory = None
        self.avoid_trigger_time = None

        # Build initial path (smoothed or raw)
        if use_smoothing:
            self.get_logger().info('[PathFollower] Using smoothed path.')
            path = smooth_path(WAYPOINTS)
            self.original_trajectory = generate_trajectory(path)
        else:
            self.get_logger().info('[PathFollower] Using original discrete path.')
            self.original_trajectory = generate_dense_trajectory_from_raw_path(WAYPOINTS)

        # Active trajectory to follow
        self.trajectory = self.original_trajectory

    def laser_callback(self, msg):
        """Callback to store LaserScan data."""
        self.laser_ranges = msg.ranges

    def odom_callback(self, msg):
        """Main control logic: receives odometry and performs trajectory tracking."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

        if not self.current_pose or not self.laser_ranges:
            return

        twist = Twist()

        # Check for obstacle and reroute if necessary
        if should_avoid_obstacle(self.laser_ranges) and not self.avoiding:
            self.get_logger().warn('[PathFollower] Obstacle detected. Attempting to generate reroute path...')
            bypass_wp = get_bypass_waypoint(
                self.current_pose,
                self.laser_ranges,
                forward_offset=1.0,
                lateral_offset=0.8
            )

            if bypass_wp:
                current_xy = self.current_pose[:2]
                goal_xy = self.original_trajectory[-1][:2]

                # Construct a temporary bypass path
                mid1 = ((current_xy[0] + bypass_wp[0]) / 2, (current_xy[1] + bypass_wp[1]) / 2)
                mid2 = ((bypass_wp[0] + goal_xy[0]) / 2, (bypass_wp[1] + goal_xy[1]) / 2)
                temp_path = [current_xy, mid1, bypass_wp, mid2, goal_xy]

                smoothed = smooth_path(temp_path)
                self.avoid_trajectory = generate_trajectory(smoothed, velocity=0.05)  # Reduced velocity for safer turns
                self.avoid_trigger_time = self.get_clock().now()
                self.avoiding = True

                self.get_logger().info('[PathFollower] Rerouting trajectory generated successfully.')
            else:
                self.get_logger().warn('[PathFollower] Failed to compute bypass waypoint.')

        # Delay before fully switching to the reroute trajectory
        if self.avoiding and self.avoid_trigger_time:
            elapsed = (self.get_clock().now() - self.avoid_trigger_time).nanoseconds / 1e9
            if elapsed < 1.0:
                twist.linear.x = 0.03
                twist.angular.z = 0.2
                self.publisher_.publish(twist)
                return
            else:
                self.trajectory = self.avoid_trajectory

        # Compute control commands using Pure Pursuit
        v, w = pure_pursuit_controller(self.current_pose, self.trajectory)
        twist.linear.x = v
        twist.angular.z = w

        # Check if reroute is complete
        if self.avoiding:
            goal_x, goal_y, _ = self.trajectory[-1]
            dist_to_goal = np.hypot(self.current_pose[0] - goal_x, self.current_pose[1] - goal_y)
            if dist_to_goal < 0.3:
                self.trajectory = self.original_trajectory
                self.avoiding = False
                self.avoid_trajectory = None
                self.avoid_trigger_time = None
                self.get_logger().info('[PathFollower] Reroute complete. Resuming original path.')

        self.publisher_.publish(twist)


def main(args=None):
    """Entry point for the PathFollower node."""
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

