"""Fixed CAD -> Air-FAR route -> SUPER polynomial -> shadow executive input.

Observer must already be running in the CAD coordinate frame. Boundary config
is an explicit ROS parameter file containing surveyed hard_boundary_lower,
hard_boundary_upper and hard_boundary_reference. No depth acquisition or PX4
command publisher is started here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg=LaunchConfiguration
    remaps=[('/navigation/'+name,'/autonomy/'+name) for name in (
        'odometry','vio_healthy','goal','route','map_snapshot','rejected_map','committed_path')]
    remaps.append(('/navigation/trajectory','/autonomy/committed_trajectory'))
    return LaunchDescription([
        DeclareLaunchArgument('rig_file'),DeclareLaunchArgument('navigation_config'),
        DeclareLaunchArgument('airfar_config'),
        DeclareLaunchArgument('cad_prior_file'),DeclareLaunchArgument('boundary_config'),
        Node(package='larp_navigation',executable='depth_map_node',output='screen',
             parameters=[arg('boundary_config'),{'rig_file':arg('rig_file'),'config_file':arg('navigation_config'),
                          'cad_prior_file':arg('cad_prior_file'),
                          'require_hard_boundary':True,'static_cad_only':True,'use_sim_time':False}],remappings=remaps),
        Node(package='larp_navigation',executable='airfar_route_node',name='airfar_planner',output='screen',
             parameters=[arg('airfar_config'),{'navigation_config_file':arg('navigation_config'),
                         'snapshot_mode':True,'use_sim_time':False}],remappings=remaps),
        Node(package='larp_autonomy',executable='trajectory_bridge',output='screen',
             parameters=[{'simulation_only':False,'use_sim_time':False}]),
    ])
