import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,'rviz'), glob('rviz/*rviz')),
        (os.path.join('share', package_name,'plotjuggler_layouts'), glob('plotjuggler_layouts/*xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tareq alqutami',
    maintainer_email='tareqaziz2010@gmail.com',
    description='PX4 offboard control using ROS2',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_sig_generator = px4_offboard_ros2.control_sig_generator_node:main',
            'visualizer = px4_offboard_ros2.visualizer_node:main',
            'pose_interactive_marker = px4_offboard_ros2.pose_interactive_marker_node:main',
            'position_control = px4_offboard_ros2.pose_control_node:main',
            'fa_pose_control = px4_offboard_ros2.fa_pose_control_node:main',
            'fa_vel_control = px4_offboard_ros2.fa_vel_control_node:main',
            'vel_control = px4_offboard_ros2.vel_control_node:main',
        ],
    },
)
