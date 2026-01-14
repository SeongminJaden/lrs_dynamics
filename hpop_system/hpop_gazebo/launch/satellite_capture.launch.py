#!/usr/bin/env python3
"""
Launch file for satellite capture simulation with OpenMANIPULATOR-P
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Package directories
    pkg_hpop_gazebo = get_package_share_directory('hpop_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world',
        default=os.path.join(pkg_hpop_gazebo, 'worlds', 'satellite_capture.world'))
    gui = LaunchConfiguration('gui', default='true')
    paused = LaunchConfiguration('paused', default='false')

    # Process xacro for chaser satellite
    xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'chaser_satellite.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'pause': paused
        }.items()
    )

    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn chaser satellite
    spawn_chaser = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_chaser_satellite',
        arguments=[
            '-entity', 'chaser_satellite',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Arm trajectory controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Static TF for world to base_link (for visualization)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # Capture status monitor (optional)
    capture_monitor = Node(
        package='hpop_gazebo',
        executable='gazebo_hpop_bridge_node',
        name='capture_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('world',
            default_value=os.path.join(pkg_hpop_gazebo, 'worlds', 'satellite_capture.world')),

        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_chaser,
        static_tf,
        # Note: Controllers require ros2_control to be properly set up
        # joint_state_broadcaster_spawner,
        # arm_controller_spawner,
    ])
