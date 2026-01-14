from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('hpop_core')

    # Declare launch arguments
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Propagation update rate in Hz'
    )

    integrator_arg = DeclareLaunchArgument(
        'integrator',
        default_value='RKF78',
        description='Integrator type: RK4 or RKF78'
    )

    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='60.0',
        description='Integration step size in seconds'
    )

    enable_j2_arg = DeclareLaunchArgument(
        'enable_j2',
        default_value='true',
        description='Enable J2 perturbation'
    )

    # Orbit propagator node
    propagator_node = Node(
        package='hpop_core',
        executable='orbit_propagator_node',
        name='orbit_propagator',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'integrator': LaunchConfiguration('integrator'),
            'step_size': LaunchConfiguration('step_size'),
            'enable_j2': LaunchConfiguration('enable_j2'),
            'enable_drag': False,
            'enable_srp': False,
            'enable_third_body': False,
        }],
        remappings=[
            ('/hpop/states', '/hpop/states'),
            ('/hpop/status', '/hpop/status'),
        ]
    )

    return LaunchDescription([
        update_rate_arg,
        integrator_arg,
        step_size_arg,
        enable_j2_arg,
        propagator_node,
    ])
