#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[ "0", "0", "0", "0", "0", "0", "base_link", "laser" ],
            name='tf2_base_laser'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[ "0", "0", "0", "0", "0", "0", "map", "odom" ],
            name='tf2_map_odom'
        ),
        
        
        Node(
            package='snuboat_apf',
            executable='waypoint_processor',
            name='waypoint_processor',
            output='screen'
        ),
        Node(
            package='snuboat_apf',
            executable='apf_processor',
            name='apf_processor'
        ),
        Node(
            package='snuboat_apf',
            executable='dynamics_processor',
            name='dynamics_processor'
        )
    ])
