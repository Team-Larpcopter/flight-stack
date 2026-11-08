"""Route-follower mission runtime: larp_autonomy/mission_runtime.launch.py with only the planner swapped.

Every component except ``planner``/``super`` (and the ``map`` status label) is
delegated unchanged to larp_autonomy's launch file. The planner component uses
the same mission configuration, parameters and topic remappings as the SUPER/EGO
``depth_map_node``, but runs ``larp_route_follower/route_follower_node``.
Run one planner stack at a time: both publish the same topics.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from larp_autonomy.runtime_config import COMPONENTS, load_config

# Same /navigation -> /autonomy remappings as larp_autonomy's planner component.
REMAPS = [('/navigation/' + n, '/autonomy/' + n) for n in (
    'odometry', 'vio_healthy', 'goal', 'route', 'map_snapshot', 'map_state', 'rejected_map', 'committed_path')]
REMAPS.append(('/navigation/trajectory', '/autonomy/committed_trajectory'))


def planner_parameters(cfg, nav):
    """Identical to larp_autonomy's planner parameters, with the follower backend."""
    params = {k: cfg[k] for k in ('rig_file', 'hard_boundary_lower', 'hard_boundary_upper',
                                  'hard_boundary_reference', 'takeoff_height')}
    params.update(config_file=cfg.get('navigation_config', str(nav / 'navigation_fine.yaml')),
                  cad_prior_file=cfg.get('cad_prior_file', str(nav / 'course_cad_prior_fine.txt')),
                  min_depth_m=.2, max_depth_m=float('inf'), ignore_top_depth_rows=10, snapshot_include_free=False,
                  max_depth_age_s=cfg['trajectory_max_age_s'],
                  declared_launch_radius_m=float(cfg.get('declared_launch_radius_m', .52)),
                  declared_launch_min_z_m=float(cfg.get('declared_launch_min_z_m', .1)),
                  declared_launch_max_z_m=float(cfg.get('declared_launch_max_z_m', 1.5)),
                  launch_clearance_reference=cfg.get(
                      'launch_clearance_reference',
                      'Operator-declared clear course launch cylinder at map origin: radius 0.52 m, z=0.1 to 1.5 m'),
                  require_hard_boundary=True, static_cad_only=False, route_backend='super_astar',
                  planner_backend='follower',
                  flight_envelope_lower=[v + .4 for v in cfg['hard_boundary_lower']],
                  flight_envelope_upper=[v - .4 for v in cfg['hard_boundary_upper']])
    return params


def planner_remaps():
    return [(source, '/autonomy/map_update_ready' if source == '/navigation/vio_healthy' else target)
            for source, target in REMAPS]


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    part, mode, mission = value('component'), value('mode'), value('mission_file')
    if part not in COMPONENTS:
        raise ValueError('Unknown runtime component')
    if part not in ('planner', 'super', 'map'):
        source = PythonLaunchDescriptionSource(str(Path(share('larp_autonomy')) / 'launch' / 'mission_runtime.launch.py'))
        return [IncludeLaunchDescription(source, launch_arguments={
            'component': part, 'mode': mode, 'mission_file': mission, 'planner': 'follower'}.items())]
    if part == 'map':
        result = [Node(package='larp_autonomy', executable='map_status', output='screen',
                       parameters=[{'planner': 'follower'}])]
    else:
        cfg = load_config(mission, mode)
        params = planner_parameters(cfg, Path(share('larp_navigation')) / 'config')
        params.update(execution_gated=mode=='hardware' or LaunchConfiguration('preplan_only', default='false').perform(context)=='true', follower_horizon_m=float(value('follower_horizon_m')),
                      follower_replan_period_s=float(value('follower_replan_period_s')),
                      follower_route_file=value('follower_route_file') or cfg.get('follower_route_file', str(Path(share('larp_route_follower')) / 'config' / 'course_master_route.xyz')))
        result = [Node(package='larp_route_follower', executable='route_follower_node', output='screen',
                       parameters=[params], remappings=planner_remaps())]
    return [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Component exited'))])),
            *result]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('component'), DeclareLaunchArgument('mission_file'),
        DeclareLaunchArgument('mode', default_value='shadow'),
        DeclareLaunchArgument('preplan_only', default_value='false'),
        DeclareLaunchArgument('follower_horizon_m', default_value='1.5'),
        DeclareLaunchArgument('follower_replan_period_s', default_value='0.4'),
        DeclareLaunchArgument('follower_route_file', default_value=''),
        OpaqueFunction(function=nodes)])
