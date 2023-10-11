from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('training_perception')

    usv_arg = DeclareLaunchArgument('usv', default_value='simulation')
    usv_config = LaunchConfiguration('usv', default='simulation')

    lidar_filter_param_file = (pkg_share, '/config/', usv_config, '/lidar_filter.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='training_perception',
            executable='lidar_filter',
            parameters=[lidar_filter_param_file],
            remappings=[('input', 'wamv/sensors/lidars/lidar_wamv_sensor/points')]
        )
    ])