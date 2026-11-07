from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='YAML file including measured mounting quaternion'),
        Node(package='larp_px4_bridge', executable='vins_px4_bridge',
             parameters=[LaunchConfiguration('params_file')], output='screen')])
