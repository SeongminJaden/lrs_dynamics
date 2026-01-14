#!/usr/bin/env python3
"""
HPOP Visualization Launch File
Launches RViz2 with HPOP control panel and displays
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hpop_bringup')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(bringup_dir, 'rviz', 'hpop_visualization.rviz')
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'hpop_visualization.rviz'),
            description='Path to RViz2 config file'
        ),

        # RViz2 with HPOP plugins
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
