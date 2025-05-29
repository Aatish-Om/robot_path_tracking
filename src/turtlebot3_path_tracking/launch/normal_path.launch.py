"""
normal_path.launch.py

Author: Aatish Om

This launch file starts the main_tracker node for path tracking without using
any path smoothing. It sets the 'use_smoothing' parameter to False.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_path_tracking',
            executable='main_tracker',
            name='path_follower',
            parameters=[{'use_smoothing': False}]
        )
    ])

