"""Fly the hard-coded DroneHacks course against an already running comp SITL.

Starts the flight executive (takeoff, HOLD, TRACKING, safety checks) and the
flight_trajectory course node, which streams the hard-coded route on the
executive's trajectory input once the executive holds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Matches flight_trajectory.dronehacks_course.CRUISE_HEIGHT (tunnel axis height).
CRUISE_HEIGHT = '0.75'


def generate_launch_description():
    def arg(name):
        return LaunchConfiguration(name)

    def number(name):
        return ParameterValue(arg(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('cruise_height', default_value=CRUISE_HEIGHT),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('yaw_mode', default_value='tangent',
                              description='tangent (face along path) or fixed'),
        DeclareLaunchArgument('max_yaw_rate', default_value='1.0'),
        DeclareLaunchArgument('max_tracking_error', default_value='0.4'),
        DeclareLaunchArgument('hold_timeout', default_value='5.0'),
        DeclareLaunchArgument('landing_timeout', default_value='45.0'),
        DeclareLaunchArgument('trajectory_topic', default_value='/larper/trajectory_in'),
        DeclareLaunchArgument('px4_topic_prefix', default_value='/fmu'),
        Node(
            package='flight_executive', executable='flight_executive_node',
            output='screen', parameters=[{
                'simulation_only': True,
                'takeoff_height': number('cruise_height'),
                'hold_timeout': number('hold_timeout'),
                'landing_timeout': number('landing_timeout'),
                'px4_topic_prefix': arg('px4_topic_prefix'),
                'trajectory_topic': arg('trajectory_topic'),
            }]),
        Node(
            package='flight_trajectory', executable='course_trajectory_node',
            output='screen', parameters=[{
                'simulation_only': True,
                'course': 'dronehacks',
                'cruise_height': number('cruise_height'),
                'max_speed': number('max_speed'),
                'yaw_mode': arg('yaw_mode'),
                'max_yaw_rate': number('max_yaw_rate'),
                'max_tracking_error': number('max_tracking_error'),
                'trajectory_topic': arg('trajectory_topic'),
                'px4_topic_prefix': arg('px4_topic_prefix'),
            }]),
    ])
