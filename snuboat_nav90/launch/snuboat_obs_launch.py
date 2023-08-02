#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snuboat_nav90',
            executable='lidar',
            name='LidarConverter'
        ),
        Node(
            package='snuboat_nav90',
            executable='obs_slam3d',
            name='ObstacleAvoidance'
        ),
        Node(
            package='snuboat_nav90',
            executable='pwm_cvt',
            name='PWMConverter'
        ),
    ])
