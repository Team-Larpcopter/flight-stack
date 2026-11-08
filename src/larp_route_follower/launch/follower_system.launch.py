"""Whole route-follower stack in one ``ros2 launch`` (no tmux).

Starts every mission component through this package's mission_runtime.launch.py:
the planner is the route follower, every other component is larp_autonomy's
unchanged component. As in the tmux stack, any component exiting shuts the
launch down. ``components`` selects a subset, e.g. ``components:=depth,map,planner``.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from larp_autonomy.runtime_config import COMPONENTS

# Same order and set as the tmux panes (planner once: 'super' is its legacy alias).
DEFAULT = ('drivers', 'foxglove', 'vins', 'dds', 'depth', 'map', 'planner', 'executive')


def components(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    selected = [c.strip() for c in value('components').split(',') if c.strip()] or list(DEFAULT)
    unknown = [c for c in selected if c not in COMPONENTS]
    if unknown:
        raise ValueError(f'Unknown runtime components: {unknown}')
    runtime = PythonLaunchDescriptionSource(str(Path(share('larp_route_follower')) / 'launch' / 'mission_runtime.launch.py'))
    shared = {k: value(k) for k in ('mode', 'mission_file', 'follower_horizon_m', 'follower_replan_period_s',
                                    'follower_route_file')}
    return [IncludeLaunchDescription(runtime, launch_arguments={'component': c, **shared}.items()) for c in selected]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_file'),
        DeclareLaunchArgument('mode', default_value='shadow'),
        DeclareLaunchArgument('components', default_value=','.join(DEFAULT)),
        DeclareLaunchArgument('follower_horizon_m', default_value='1.5'),
        DeclareLaunchArgument('follower_replan_period_s', default_value='0.4'),
        DeclareLaunchArgument('follower_route_file', default_value=''),
        OpaqueFunction(function=components)])
