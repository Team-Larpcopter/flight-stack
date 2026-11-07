"""Explicitly launched sensor observer. Never starts a driver or PX4 publisher.

Do not run while hardware access is paused. A captured rig/config directory is
required; factory geometry is not replaced with generic camera calibration.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    directory=Path(LaunchConfiguration('calibration_dir').perform(context))
    for file in ('rig.json','estimator_config.yaml','kalibr_imucam_chain.yaml','kalibr_imu_chain.yaml'):
        if not (directory/file).is_file():raise RuntimeError('Missing captured calibration: '+str(directory/file))
    return [
        Node(package='larp_autonomy',executable='stereo_frontend',output='screen'),
        Node(package='ov_srvins',executable='run_subscribe_msckf',namespace='autonomy',
             parameters=[{'config_path':str(directory/'estimator_config.yaml'),'verbosity':'WARNING','use_sim_time':False}],
             remappings=[('/sim/processed/vio','/autonomy/processed_vio')],output='screen'),
        Node(package='larp_navigation',executable='body_odometry_node',
             parameters=[{'rig_file':str(directory/'rig.json'),'use_sim_time':False}],
             remappings=[('/odomimu','/autonomy/odomimu'),('/poseimu','/autonomy/poseimu'),
                         ('/navigation/odometry','/autonomy/odometry'),('/navigation/vio_healthy','/autonomy/vio_healthy')],output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration_dir'),OpaqueFunction(function=nodes)])
