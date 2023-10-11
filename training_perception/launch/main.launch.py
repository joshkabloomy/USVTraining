from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('training_perception')

    usv_arg = DeclareLaunchArgument('usv', default_value='simulation')
    usv_config = LaunchConfiguration('usv', default='simulation')

    return LaunchDescription([
        usv_arg,
    ])