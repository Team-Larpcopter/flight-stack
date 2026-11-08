"""Live native OAK -> sqrtVINS -> fixed-map planning -> shadow executive input.

Launching this connects the camera. It never starts the executive or sends PX4
commands. Attach-only localization is available when the camera/VIO already run.
"""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    args = {name: value(name) for name in (
        'rig_file', 'navigation_config', 'airfar_config', 'cad_prior_file', 'boundary_config',
        'initial_map_x', 'initial_map_y', 'initial_map_z', 'initial_map_yaw',
        'map_heading_ned', 'map_origin_ned_x', 'map_origin_ned_y', 'map_origin_ned_z')}
    for name in ('rig_file', 'navigation_config', 'airfar_config', 'cad_prior_file', 'boundary_config'):
        if not Path(args[name]).is_file():
            raise ValueError('Missing profiling input: ' + name + '=' + args[name])
    result = []
    if value('start_localization').lower() == 'true':
        if not Path(value('estimator_config')).is_file():
            raise ValueError('Missing estimator_config')
        directory = tempfile.mkdtemp(prefix='planner-camera-')
        result += [
            Node(package='larp_hardware', executable='oak_native_node', output='screen',
                 parameters=[{'connect_device': True, 'publish_images': True, 'align_imu': True,
                              'factory_calibration_out': directory + '/factory.json',
                              'camera_fps': 30.0, 'accel_hz': 500, 'gyro_hz': 400,
                              'max_exposure_us': 5000, 'capture_seconds': 0.0}]),
            Node(package='ov_srvins', executable='run_subscribe_msckf', namespace='autonomy',
                 output='screen', parameters=[{'config_path': value('estimator_config'),
                                               'verbosity': 'WARNING'}],
                 remappings=[('/sim/processed/vio', '/autonomy/processed_vio')]),
        ]
    result.append(Node(package='larp_autonomy', executable='planner_visualization', output='screen'))
    args['mode'] = 'shadow'
    result.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('larp_autonomy')) / 'launch/planner_mission.launch.py')),
        launch_arguments=args.items()))
    return result


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('larp_localization')) /
                         'config/oak_d_s2/estimator_config.yaml')
    required = ('rig_file', 'navigation_config', 'airfar_config', 'cad_prior_file', 'boundary_config')
    defaults = {'start_localization': 'true', 'estimator_config': default_config,
                'initial_map_x': '0.0', 'initial_map_y': '0.0', 'initial_map_z': '0.0',
                'initial_map_yaw': '0.0', 'map_heading_ned': '0.0',
                'map_origin_ned_x': '0.0', 'map_origin_ned_y': '0.0', 'map_origin_ned_z': '0.0'}
    return LaunchDescription([*[DeclareLaunchArgument(k) for k in required],
                              *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=nodes)])
