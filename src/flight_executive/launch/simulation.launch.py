"""Start automatic takeoff against an already running PX4 SITL instance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('takeoff_height', default_value='1.0'),
        DeclareLaunchArgument('px4_topic_prefix', default_value='/fmu'),
        Node(
            package='flight_executive', executable='flight_executive_node',
            output='screen', parameters=[{
                'simulation_only': True,
                'takeoff_height': ParameterValue(
                    LaunchConfiguration('takeoff_height'), value_type=float),
                'px4_topic_prefix': LaunchConfiguration('px4_topic_prefix'),
            }]),
    ])
