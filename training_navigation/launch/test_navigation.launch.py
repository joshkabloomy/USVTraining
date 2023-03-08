from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('training_navigation')

    localization = get_package_share_directory('training_localization')
    perception = get_package_share_directory('training_perception')

    usv_arg = DeclareLaunchArgument('usv', default_value='simulation')
    usv_config = LaunchConfiguration('usv', default='simulation')

    navigation_params_file = (pkg_share, '/config/', usv_config, '/navigation.yaml')

    return LaunchDescription([
        usv_arg, 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),

        Node(
            package='training_navigation',
            executable='test_navigation',
            parameters=[navigation_params_file]
        )
    ])