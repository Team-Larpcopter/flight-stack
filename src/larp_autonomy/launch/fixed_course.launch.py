"""Pilot-triggered 1.0 m fixed course; no depth, mapping or online planning."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from flight_trajectory.fixed_course import load_waypoints

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from larp_autonomy.runtime_config import load_config


def nodes(context):
    mission_file = LaunchConfiguration('mission_file').perform(context)
    cfg = load_config(mission_file, 'hardware')
    route_file = Path(mission_file).parent / 'fixed_course.yaml'
    if not route_file.is_file():
        route_file = Path(get_package_share_directory('flight_trajectory')) / 'config/fixed_course.yaml'
    takeoff_height = load_waypoints(route_file)[0][2]
    return [
        Node(package='flight_executive', executable='flight_executive_node',
             output='screen', parameters=[dict(
                 simulation_only=False, hardware_enabled=True,
                 takeoff_trigger='position', takeoff_height=takeoff_height, hold_timeout=2.,
                 target_system=cfg.get('target_system', 1),
                 target_component=cfg.get('target_component', 1),
                 trajectory_topic='/fixed_course/trajectory_in',
                 # Fixed-course flight intentionally bypasses the executive fence; map limits stay unchanged.
                 require_autonomy_ready=False, enforce_boundary=False,
                 align_px4_to_map_on_ground=True,
                 map_boundary_lower=cfg['hard_boundary_lower'],
                 map_boundary_upper=cfg['hard_boundary_upper'],
                 map_heading_ned=cfg['map_heading_ned'],
                 map_origin_ned=cfg['map_origin_ned'])]),
        Node(package='flight_trajectory', executable='course_trajectory_node',
             output='screen', parameters=[dict(
                 simulation_only=False, hardware_enabled=True, course='fixed_course',
                 fixed_course_file=str(route_file),
                 trajectory_topic='/fixed_course/trajectory_in')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_file'), OpaqueFunction(function=nodes),
    ])
