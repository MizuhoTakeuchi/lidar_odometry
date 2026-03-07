import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_odometry')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
