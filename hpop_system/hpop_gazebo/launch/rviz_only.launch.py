#!/usr/bin/env python3
"""
RViz-only launch for testing without Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_hpop_gazebo = get_package_share_directory('hpop_gazebo')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot descriptions
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
    
    # RViz configs
    docking_rviz = os.path.join(pkg_hpop_gazebo, 'config', 'docking_view.rviz')
    lvlh_rviz = os.path.join(pkg_hpop_gazebo, 'config', 'lvlh_view.rviz')
    
    # Robot State Publishers
    chaser_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': chaser_robot_description,
            'use_sim_time': use_sim_time,
        }]
    )
    
    target_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_robot_state_publisher',
        namespace='target',
        parameters=[{
            'robot_description': target_robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'target/'
        }]
    )
    
    # Joint state publisher for arm visualization
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # Static TF
    world_to_earth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_earth_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'earth']
    )
    
    # HPOP orbit demo
    orbit_demo = Node(
        package='hpop_core',
        executable='orbit_demo_node',
        name='orbit_demo_node',
        output='screen',
    )
    
    # Rendezvous controller
    rendezvous_ctrl = Node(
        package='hpop_gazebo',
        executable='rendezvous_controller',
        name='rendezvous_controller',
        output='screen',
    )
    
    # RViz - Docking view
    rviz_docking = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_docking',
        arguments=['-d', docking_rviz],
    )
    
    # RViz - LVLH view
    rviz_lvlh = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_lvlh',
        arguments=['-d', lvlh_rviz],
    )
    
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[GroupAction([orbit_demo, rendezvous_ctrl])]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        chaser_rsp,
        target_rsp,
        joint_state_publisher,
        world_to_earth_tf,
        delayed_nodes,
        rviz_docking,
        # rviz_lvlh,  # Uncomment for LVLH view
    ])
