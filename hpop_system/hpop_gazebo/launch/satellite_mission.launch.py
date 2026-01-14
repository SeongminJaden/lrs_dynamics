#!/usr/bin/env python3
"""
Integrated Launch File for Satellite Rendezvous and Docking Mission

Includes:
- Gazebo simulation with Chaser and Target satellites
- Robot arm controllers
- HPOP orbit propagation
- Rendezvous controller
- RViz visualization

Usage:
  ros2 launch hpop_gazebo satellite_mission.launch.py

Then in another terminal:
  ros2 service call /hpop/start_propagation std_srvs/srv/Trigger
  ros2 service call /rendezvous/start std_srvs/srv/Trigger
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, GroupAction, RegisterEventHandler, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
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
    rviz = LaunchConfiguration('rviz', default='true')

    # Robot descriptions (wrapped in ParameterValue for ROS2 Humble compatibility)
    chaser_xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'chaser_satellite.urdf.xacro')
    chaser_robot_description = ParameterValue(
        Command(['xacro ', chaser_xacro_file]),
        value_type=str
    )

    target_xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'target_satellite.urdf.xacro')
    target_robot_description = ParameterValue(
        Command(['xacro ', target_xacro_file]),
        value_type=str
    )

    # SRDF for MoveIt2
    srdf_file = os.path.join(pkg_hpop_gazebo, 'config', 'chaser_satellite.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # RViz config
    rviz_config = os.path.join(pkg_hpop_gazebo, 'config', 'docking_view.rviz')

    # ==================== GAZEBO ====================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'pause': 'false'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # ==================== ROBOT STATE PUBLISHERS ====================
    chaser_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': chaser_robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

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

    # ==================== TF ====================
    # Only static TF: world -> earth (identity transform)
    # All other TFs are published dynamically by rendezvous_controller based on HPOP positions
    world_to_earth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_earth_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'earth']
    )

    # Note: earth -> chaser and earth -> target TFs are published by rendezvous_controller
    # based on HPOP orbit propagation data

    # ==================== SPAWN SATELLITES ====================
    spawn_chaser = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_chaser_satellite',
        arguments=['-entity', 'chaser_satellite', '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_target_satellite',
        arguments=['-entity', 'target_satellite', '-topic', '/target/robot_description',
                   '-x', '2.0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # ==================== JOINT STATE PUBLISHER ====================
    # Using joint_state_publisher for arm visualization (gazebo_ros2_control disabled temporarily)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ==================== MISSION NODES ====================
    orbit_demo_node = Node(
        package='hpop_core',
        executable='orbit_demo_node',
        name='orbit_demo_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rendezvous_controller = Node(
        package='hpop_gazebo',
        executable='rendezvous_controller',
        name='rendezvous_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approach_distance': 0.5,
            'capture_distance': 0.35,
            'approach_velocity': 0.1,
            'robot_description': chaser_robot_description,
            'robot_description_semantic': robot_description_semantic,
        }]
    )

    # ==================== RVIZ ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    # ==================== DELAYED ACTIONS ====================
    # Spawn satellites after Gazebo is ready (2 seconds)
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[GroupAction([spawn_chaser, spawn_target])]
    )

    # Start joint state publisher after spawn (5 seconds)
    delayed_joint_state = TimerAction(
        period=5.0,
        actions=[GroupAction([joint_state_publisher])]
    )

    # Start mission nodes (7 seconds)
    delayed_mission = TimerAction(
        period=7.0,
        actions=[GroupAction([orbit_demo_node, rendezvous_controller])]
    )

    # Start RViz (10 seconds)
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[GroupAction([rviz_node])]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world',
            default_value=os.path.join(pkg_hpop_gazebo, 'worlds', 'satellite_capture.world')),

        # Gazebo
        gazebo_server,
        gazebo_client,

        # Robot State Publishers
        chaser_robot_state_publisher,
        target_robot_state_publisher,

        # TF (only world->earth, others are dynamic from rendezvous_controller)
        world_to_earth_tf,

        # Delayed actions (sequential startup)
        delayed_spawn,
        delayed_joint_state,
        delayed_mission,
        delayed_rviz,
    ])
