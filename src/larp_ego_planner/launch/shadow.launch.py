"""Attach EGO to existing static map/body inputs; publish diagnostic candidates."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('larp_ego_planner')) / 'config/ego.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('ego_config', default_value=default),
        Node(package='larp_ego_planner', executable='ego_shadow_node',
             namespace='autonomy/ego', output='screen',
             parameters=[LaunchConfiguration('ego_config')]),
    ])
