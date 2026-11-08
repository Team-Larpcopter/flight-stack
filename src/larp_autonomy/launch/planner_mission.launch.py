"""Fixed-map planning with explicit shadow, simulation, or hardware execution.

Does not launch sensors or external-vision injection. Hardware mode enables the
executive's vehicle command path; default shadow mode never starts it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ('shadow', 'simulation', 'hardware'):
        raise ValueError('mode must be shadow, simulation, or hardware')
    backend = value('route_backend')
    if backend != 'super_astar':
        raise ValueError('route_backend must be super_astar')
    active = mode != 'shadow'
    aligned = value('alignment_verified').lower() == 'true'
    if active and (not aligned or not value('mission_id').strip()):
        raise ValueError('Active execution requires alignment_verified and mission_id')
    clock = mode == 'simulation'
    heading = float(value('map_heading_ned'))
    origin = [float(value('map_origin_ned_' + axis)) for axis in 'xyz']
    remaps = [('/navigation/' + n, '/autonomy/' + n) for n in (
        'odometry', 'vio_healthy', 'goal', 'route', 'map_snapshot', 'map_state', 'rejected_map', 'committed_path')]
    remaps.append(('/navigation/trajectory', '/autonomy/committed_trajectory'))
    result = [
        Node(package='larp_navigation', executable='body_odometry_node', output='screen',
             parameters=[{'rig_file': value('rig_file'), 'use_sim_time': clock,
                          'initial_position_m': [float(value('initial_map_' + a)) for a in 'xyz'],
                          'initial_yaw_rad': float(value('initial_map_yaw'))}],
             remappings=[('/odomimu', '/autonomy/odomimu'), ('/poseimu', '/autonomy/poseimu'), *remaps]),
        Node(package='larp_navigation', executable='depth_map_node', output='screen',
             parameters=[value('boundary_config'), {'rig_file': value('rig_file'),
                         'config_file': value('navigation_config'), 'cad_prior_file': value('cad_prior_file'),
                         'require_hard_boundary': True, 'static_cad_only': True, 'route_backend': backend, 'use_sim_time': clock}],
             remappings=remaps),
        Node(package='larp_autonomy', executable='trajectory_bridge', output='screen',
             parameters=[{'simulation_only': mode == 'simulation', 'hardware_enabled': mode == 'hardware',
                          'alignment_verified': aligned, 'mission_id': value('mission_id'),
                          'map_heading_ned': heading, 'map_origin_ned': origin, 'use_sim_time': clock}]),
    ]
    if active:
        result.append(Node(package='flight_executive', executable='flight_executive_node', output='screen',
                           parameters=[{'simulation_only': mode == 'simulation',
                                        'hardware_enabled': mode == 'hardware', 'use_sim_time': clock,
                                        'takeoff_height': float(value('takeoff_height')),
                                        'hold_timeout': float(value('hold_timeout'))}]))
    return result


def generate_launch_description():
    required = ('rig_file', 'navigation_config', 'cad_prior_file', 'boundary_config')
    defaults = {'mode': 'shadow', 'route_backend': 'super_astar', 'alignment_verified': 'false', 'mission_id': '',
                'map_heading_ned': '0.0', 'map_origin_ned_x': '0.0', 'map_origin_ned_y': '0.0',
                'map_origin_ned_z': '0.0', 'initial_map_x': '0.0', 'initial_map_y': '0.0',
                'initial_map_z': '0.0', 'initial_map_yaw': '0.0',
                'takeoff_height': '0.75', 'hold_timeout': '5.0'}
    return LaunchDescription([*[DeclareLaunchArgument(k) for k in required],
                              *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=nodes)])
