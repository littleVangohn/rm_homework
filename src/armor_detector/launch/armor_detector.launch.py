from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('armor_detector')
    default_config = os.path.join(package_share, 'config', 'armor_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Armor detector parameter file'),
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            parameters=[LaunchConfiguration('config')]),
    ])
