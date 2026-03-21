import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_odometry'),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
