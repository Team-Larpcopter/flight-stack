"""Fly the hard-coded DroneHacks route against an already running comp SITL."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from flight_executive.dronehacks_course import CRUISE_HEIGHT


def generate_launch_description():
    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('cruise_height', default_value=str(CRUISE_HEIGHT)),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('max_tracking_error', default_value='0.4'),
        DeclareLaunchArgument('yaw_mode', default_value='tangent',
                              description='tangent (face along path) or fixed'),
        DeclareLaunchArgument('max_yaw_rate', default_value='1.0'),
        DeclareLaunchArgument('px4_topic_prefix', default_value='/fmu'),
        Node(
            package='flight_executive', executable='flight_executive_node',
            output='screen', parameters=[{
                'simulation_only': True,
                'course': 'dronehacks',
                'takeoff_height': number('cruise_height'),
                'max_speed': number('max_speed'),
                'max_tracking_error': number('max_tracking_error'),
                'yaw_mode': LaunchConfiguration('yaw_mode'),
                'max_yaw_rate': number('max_yaw_rate'),
                'px4_topic_prefix': LaunchConfiguration('px4_topic_prefix'),
            }]),
    ])
