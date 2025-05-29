"""
smoothed_path.launch.py

This launch file starts the main_tracker node for path tracking with
path smoothing enabled. It sets the 'use_smoothing' parameter to True.

Author: Aatish Om
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_path_tracking',
            executable='main_tracker',
            name='path_follower',
            parameters=[{'use_smoothing': True}]
        )
    ])

