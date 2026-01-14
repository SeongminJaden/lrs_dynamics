#!/usr/bin/env python3
"""
Satellite Simulation Launch File
Launches Gazebo with GGM05C gravity plugin and HPOP bridge node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_hpop_gazebo = get_package_share_directory('hpop_gazebo')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world',
        default=os.path.join(pkg_hpop_gazebo, 'worlds', 'orbital_environment.world'))

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_hpop_gazebo, 'worlds', 'orbital_environment.world'),
            description='World file to load'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'true',
            }.items(),
        ),

        # Gazebo-HPOP Bridge Node
        Node(
            package='hpop_gazebo',
            executable='gazebo_hpop_bridge_node',
            name='gazebo_hpop_bridge',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_prefix': 'satellite',
                'scale_factor': 0.001,
                'publish_rate': 10.0,
            }],
            output='screen'
        ),

        # TF Static broadcaster for Earth frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='earth_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'earth'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
