#!/usr/bin/env python3
"""
Launch file for HPOP mission control system
- Robot arm controllers
- HPOP orbit propagation
- Rendezvous controller
- MoveIt2 move_group
- RViz2 visualization

Run this AFTER gazebo_satellites.launch.py is fully loaded
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    """Load a YAML file from a package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading {absolute_file_path}: {e}")
        return {}


def generate_launch_description():
    pkg_hpop_gazebo = get_package_share_directory('hpop_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    # RViz config file
    rviz_config = os.path.join(pkg_hpop_gazebo, 'config', 'satellite_capture.rviz')

    # Robot description (URDF) for MoveIt2
    xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'chaser_satellite.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Load SRDF for MoveIt2
    srdf_file = os.path.join(pkg_hpop_gazebo, 'config', 'chaser_satellite.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Load MoveIt configurations
    kinematics_yaml = load_yaml('hpop_gazebo', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('hpop_gazebo', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('hpop_gazebo', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('hpop_gazebo', 'config/moveit_controllers.yaml')

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

    # HPOP Orbit Demo Node
    # Control via services:
    #   - ros2 service call /hpop/start_propagation std_srvs/srv/Trigger
    #   - ros2 service call /hpop/stop_propagation std_srvs/srv/Trigger
    #   - ros2 service call /hpop/reset_propagation std_srvs/srv/Trigger
    orbit_demo_node = Node(
        package='hpop_core',
        executable='orbit_demo_node',
        name='orbit_demo_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # MoveIt2 move_group node configuration
    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': ompl_planning_yaml,
        'planning_pipelines': ['ompl'],
        'use_sim_time': use_sim_time,
    }
    moveit_config.update(joint_limits_yaml if joint_limits_yaml else {})

    # MoveIt2 Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config,
            moveit_controllers_yaml if moveit_controllers_yaml else {},
            {'trajectory_execution.allowed_execution_duration_scaling': 1.2},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            {'trajectory_execution.allowed_start_tolerance': 0.01},
        ]
    )

    # Rendezvous Controller Node
    # Control via services:
    #   - ros2 service call /rendezvous/start std_srvs/srv/Trigger
    #   - ros2 service call /rendezvous/abort std_srvs/srv/Trigger
    #   - ros2 service call /rendezvous/capture std_srvs/srv/Trigger
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
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics_yaml,
        }]
    )

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    # Spawn arm controller after joint_state_broadcaster
    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='Launch RViz2 visualization'),

        # Controllers
        joint_state_broadcaster_spawner,
        delayed_arm_controller,

        # MoveIt2
        move_group_node,

        # Mission nodes
        orbit_demo_node,
        rendezvous_controller,

        # Visualization
        rviz2_node,
    ])
