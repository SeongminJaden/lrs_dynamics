#!/usr/bin/env python3
"""
Launch file for Gazebo simulation only
- Gazebo world
- Spawn Chaser satellite
- Spawn Target satellite

Run hpop_mission.launch.py after this is fully loaded
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    chaser_xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'chaser_satellite.urdf.xacro')
    chaser_robot_description = Command(['xacro ', chaser_xacro_file])

    # Process xacro for target satellite
    target_xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'target_satellite.urdf.xacro')
    target_robot_description = Command(['xacro ', target_xacro_file])

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

    # Chaser Robot state publisher (publishes to default /robot_description for gazebo_ros2_control)
    chaser_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': chaser_robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': ''
        }]
    )

    # TF Structure for space simulation:
    # world (ECI inertial frame)
    #   └── earth (Earth center)
    #         ├── chaser/dummy_root -> base_link -> arm...
    #         └── target/target_dummy_root -> target_base_link...

    # Static transform: world -> earth (Earth at center of inertial frame)
    world_to_earth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_earth_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'earth']
    )

    # =============================================
    # Floating Origin System (Chaser-centered)
    # =============================================
    # Real orbit parameters:
    #   ISS altitude: ~420km, Earth radius: 6371km
    #   Orbit radius: 6791km = 6,791,000m
    #   Chaser-Target separation: 1500km
    #
    # Gazebo uses Floating Origin:
    #   - Chaser at origin (0,0,0)
    #   - Target at relative position (~165km ahead)
    #   - This avoids float precision issues
    #
    # TF publishes real ECI coordinates for RViz visualization

    # Chaser position - at origin for Gazebo floating origin
    earth_to_chaser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='earth_to_chaser_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'earth', '--child-frame-id', 'dummy_root']
    )

    # Target position - 2m ahead matching Gazebo spawn
    earth_to_target_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='earth_to_target_tf',
        arguments=['--x', '2', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'earth', '--child-frame-id', 'target/target_dummy_root']
    )

    # Target Robot state publisher
    target_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_robot_state_publisher',
        namespace='target',
        output='screen',
        parameters=[{
            'robot_description': target_robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'target/'
        }]
    )

    # =============================================
    # Gazebo Floating Origin (Chaser at origin)
    # =============================================
    # For physics simulation, use small coordinates to avoid precision issues
    # Chaser at origin, Target at relative position for docking simulation
    #
    # Real distance: 1500km, but for docking sim we start closer
    # Initial separation: 2m (for robot arm reach testing)

    # Spawn chaser satellite at origin
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

    # Spawn target satellite 2m ahead of chaser (for docking simulation)
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_target_satellite',
        arguments=[
            '-entity', 'target_satellite',
            '-topic', '/target/robot_description',
            '-x', '2.0', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # Group spawn actions with a delay to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=2.0,  # Wait for Gazebo to fully initialize
        actions=[
            GroupAction([
                spawn_chaser,
                spawn_target,
            ])
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('world',
            default_value=os.path.join(pkg_hpop_gazebo, 'worlds', 'satellite_capture.world')),

        gazebo_server,
        gazebo_client,
        chaser_robot_state_publisher,
        target_robot_state_publisher,
        # TF hierarchy: world -> earth -> satellites
        world_to_earth_tf,
        earth_to_chaser_tf,
        earth_to_target_tf,
        delayed_spawn,
    ])
