#!/usr/bin/env python3
"""
HPOP System Full Launch File
Launches all HPOP nodes including propagator, TLE service, analysis, and visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hpop_bringup')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    # Load parameters
    propagator_params = os.path.join(bringup_dir, 'config', 'propagator.yaml')
    satellites_config = os.path.join(bringup_dir, 'config', 'satellites.yaml')
    ground_stations_config = os.path.join(bringup_dir, 'config', 'ground_stations.yaml')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz2 visualization'
        ),

        # TF Static broadcaster for Earth frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='earth_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'earth'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Orbit Propagator Node
        # Note: Will be implemented in Phase 7 with full ROS2 node integration
        # Node(
        #     package='hpop_core',
        #     executable='orbit_propagator_node',
        #     name='orbit_propagator',
        #     parameters=[propagator_params, {'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),

        # TLE Service Node
        # Note: Will be implemented in Phase 7
        # Node(
        #     package='hpop_tle',
        #     executable='tle_service_node',
        #     name='tle_service',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),

        # Include RViz2 visualization launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'hpop_visualization.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
