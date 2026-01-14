#!/usr/bin/env python3
"""
MoveIt2 Launch file for Chaser Satellite Robot Arm
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
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
    # Package directory
    pkg_hpop_gazebo = get_package_share_directory('hpop_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (URDF)
    xacro_file = os.path.join(pkg_hpop_gazebo, 'urdf', 'chaser_satellite.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # SRDF
    srdf_file = os.path.join(pkg_hpop_gazebo, 'config', 'chaser_satellite.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Load configurations
    kinematics_yaml = load_yaml('hpop_gazebo', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('hpop_gazebo', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('hpop_gazebo', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('hpop_gazebo', 'config/moveit_controllers.yaml')

    # MoveIt2 parameters
    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': ompl_planning_yaml,
        'planning_pipelines': ['ompl'],
        'use_sim_time': use_sim_time,
    }

    # Merge joint limits
    moveit_config.update(joint_limits_yaml)

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config,
            moveit_controllers_yaml,
            {'trajectory_execution.allowed_execution_duration_scaling': 1.2},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            {'trajectory_execution.allowed_start_tolerance': 0.01},
        ],
        remappings=[
            ('/robot_description', '/chaser/robot_description')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group_node,
    ])
