from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('training_perception')

    usv_arg = DeclareLaunchArgument('usv', default_value='simulation')
    usv_config = LaunchConfiguration('usv', default='simulation')

    euclidean_clustering_params_file = (pkg_share, '/config/', usv_config, 
        '/euclidean_clustering.yaml')

    return LaunchDescription([
        usv_arg,

        DeclareLaunchArgument(
            'euclidean_clustering_params_file',
            default_value=euclidean_clustering_params_file
        ),

        Node(
            package='euclidean_cluster_nodes',
            executable='euclidean_cluster_node_exe',
            parameters=[LaunchConfiguration('euclidean_clustering_params_file')],
            remappings=[('/points_in', '/wamv/sensors/lidars/lidar_wamv/points')]
        )
    ])