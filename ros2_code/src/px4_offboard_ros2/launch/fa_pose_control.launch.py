#!/usr/bin/env python

__author__ = 'Tareq Alqutami'
__contact__ = 'tareqaziz2010@gmail.com'

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard_ros2',
            namespace='px4_offboard_ros2',
            executable='fa_pose_control',
            name='pose_controller'
        ),
        Node(
            package='px4_offboard_ros2',
            namespace='px4_offboard_ros2',
            executable='pose_interactive_marker',
            name='pose_interactive_marker'
        ),
    ])
    