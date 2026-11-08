"""Read-only localization, depth mapping and planning visualization."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('larp_perception')) / 'launch/perception.launch.py'))),
        Node(package='larp_autonomy', executable='planner_visualization', output='screen'),
    ])
