"""Native sensors, VIO, CAD planning, XRCE agent and optional PX4 execution."""
from pathlib import Path
import math
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ('shadow', 'hardware'):
        raise ValueError('mode must be shadow or hardware')
    active = mode == 'hardware'
    if active and (value('alignment_verified').lower() != 'true' or not value('mission_id').strip()):
        raise ValueError('Hardware execution requires alignment_verified:=true and mission_id')
    args = {name: value(name) for name in (
        'rig_file', 'navigation_config', 'airfar_config', 'cad_prior_file', 'boundary_config',
        'initial_map_x', 'initial_map_y', 'initial_map_z', 'initial_map_yaw',
        'map_heading_ned', 'map_origin_ned_x', 'map_origin_ned_y', 'map_origin_ned_z',
        'alignment_verified', 'mission_id', 'takeoff_height', 'hold_timeout')}
    files = ['rig_file', 'navigation_config', 'airfar_config', 'cad_prior_file', 'boundary_config', 'estimator_config']
    if active:
        files.append('bridge_params')
    for name in files:
        if not Path(value(name)).is_file():
            raise ValueError('Missing input file: ' + name + '=' + value(name))
    numeric = [float(value(k)) for k in ('goal_x', 'goal_y', 'goal_z', 'goal_yaw',
               'initial_map_x', 'initial_map_y', 'initial_map_z', 'initial_map_yaw',
               'map_heading_ned', 'map_origin_ned_x', 'map_origin_ned_y', 'map_origin_ned_z')]
    if not all(math.isfinite(v) for v in numeric):
        raise ValueError('Goal and frame alignment values must be finite')
    transport = value('agent_transport')
    if transport == 'udp4':
        port = int(value('agent_port'))
        if not 1 <= port <= 65535:
            raise ValueError('Invalid agent UDP port')
        command = ['MicroXRCEAgent', 'udp4', '-p', str(port)]
    elif transport == 'serial':
        device, baud = value('agent_device'), int(value('agent_baud'))
        if not device.startswith('/dev/') or baud <= 0:
            raise ValueError('Invalid agent serial device/baud')
        command = ['MicroXRCEAgent', 'serial', '--dev', device, '-b', str(baud)]
    else:
        raise ValueError('agent_transport must be udp4 or serial')
    # Any exited process stops the complete group instead of leaving orphan
    # command publishers running. PX4's configured offboard-loss failsafe applies.
    result = [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(
        event=Shutdown(reason='A full-stack process exited'))])),
        ExecuteProcess(cmd=command, output='screen')]
    if value('start_localization').lower() == 'true':
        directory = tempfile.mkdtemp(prefix='flight-stack-camera-')
        result.extend([
            Node(package='larp_hardware', executable='oak_native_node', output='screen',
                 parameters=[{'connect_device': True, 'publish_images': True, 'align_imu': True,
                              'factory_calibration_out': directory + '/factory.json',
                              'camera_fps': 30.0, 'accel_hz': 500, 'gyro_hz': 400,
                              'max_exposure_us': 5000, 'capture_seconds': 0.0}]),
            Node(package='ov_srvins', executable='run_subscribe_msckf', namespace='autonomy',
                 output='screen', parameters=[{'config_path': value('estimator_config'), 'verbosity': 'INFO'}],
                 remappings=[('/sim/processed/vio', '/autonomy/processed_vio')])])
    if active:
        result.append(Node(package='larp_px4_bridge', executable='vins_px4_bridge',
                           parameters=[value('bridge_params'), {'input_topic': '/autonomy/odomimu',
                           'output_topic': '/fmu/in/vehicle_visual_odometry'}], output='screen'))
    args['mode'] = mode
    result.extend([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('larp_autonomy')) / 'launch/planner_mission.launch.py')),
            launch_arguments=args.items()),
        Node(package='larp_autonomy', executable='mission_goal', output='screen',
             parameters=[dict(zip(('x', 'y', 'z', 'yaw'), numeric[:4]))]),
        Node(package='larp_autonomy', executable='planner_visualization', output='screen')])
    return result


def generate_launch_description():
    nav = Path(get_package_share_directory('larp_navigation')) / 'config'
    estimator = Path(get_package_share_directory('larp_localization')) / 'config/oak_d_s2/estimator_config.yaml'
    required = ('rig_file', 'boundary_config', 'goal_x', 'goal_y', 'goal_z')
    defaults = {'mode': 'shadow', 'alignment_verified': 'false', 'mission_id': '',
        'bridge_params': '', 'navigation_config': str(nav / 'navigation.yaml'),
        'airfar_config': str(nav / 'airfar_geometry.yaml'), 'cad_prior_file': str(nav / 'course_cad_prior.txt'),
        'estimator_config': str(estimator), 'start_localization': 'true',
        'agent_transport': 'udp4', 'agent_port': '8888', 'agent_device': '/dev/px4', 'agent_baud': '921600',
        'goal_yaw': '0.0', 'initial_map_x': '0.0', 'initial_map_y': '0.0', 'initial_map_z': '0.0',
        'initial_map_yaw': '0.0', 'map_heading_ned': '0.0', 'map_origin_ned_x': '0.0',
        'map_origin_ned_y': '0.0', 'map_origin_ned_z': '0.0', 'takeoff_height': '0.75', 'hold_timeout': '5.0'}
    return LaunchDescription([*[DeclareLaunchArgument(k) for k in required],
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()], OpaqueFunction(function=nodes)])
