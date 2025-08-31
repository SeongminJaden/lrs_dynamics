from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rsds')
    world_path = os.path.join(pkg_share, 'worlds', 'leo_test.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])

