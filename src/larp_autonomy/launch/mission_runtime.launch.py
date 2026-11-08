"""One component per pane; ROG and SUPER share one authoritative map process."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from larp_autonomy.runtime_config import COMPONENTS, load_config


def nodes(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    part, mode=value('component'),value('mode')
    if part not in COMPONENTS: raise ValueError('Unknown runtime component')
    cfg=load_config(value('mission_file'),mode)
    active=mode=='hardware'
    nav=Path(share('larp_navigation'))/'config'
    remaps=[('/navigation/'+n,'/autonomy/'+n) for n in (
        'odometry','vio_healthy','goal','route','map_snapshot','map_state','rejected_map','committed_path')]
    remaps.append(('/navigation/trajectory','/autonomy/committed_trajectory'))
    def node(pkg,exe,params=None,**kwargs):
        return Node(package=pkg,executable=exe,output='screen',parameters=params or [],**kwargs)
    def include(pkg,file,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(pkg))/'launch'/file)),launch_arguments=args.items())
    if part=='drivers':
        result=[node('larp_hardware','oak_native_node',[dict(connect_device=True,publish_images=True,
            publish_rectified=True,align_imu=True,camera_fps=30.,accel_hz=500,gyro_hz=400,
            max_exposure_us=5000,capture_seconds=0.)])]
    elif part=='foxglove':
        result=[include('larp_perception','perception.launch.py',{'port':'8766','send_buffer_limit':'8388608','flight_monitor_only':'true'}),
                node('larp_autonomy','planner_visualization')]
    elif part=='vins':
        config=cfg.get('estimator_config',str(Path(share('larp_localization'))/'config/oak_d_s2/estimator_config.yaml'))
        result=[node('ov_srvins','run_subscribe_msckf',[{'config_path':config,'verbosity':'INFO'}],namespace='autonomy',
                     remappings=[('/sim/processed/vio','/autonomy/processed_vio')]),
                node('larp_navigation','body_odometry_node',[{k:cfg[k] for k in ('rig_file','initial_position_m','initial_yaw_rad')}],
                     remappings=[('/odomimu','/autonomy/odomimu'),('/poseimu','/autonomy/poseimu'),*remaps])]
        if active: result.append(node('larp_px4_bridge','vins_px4_bridge',[cfg['bridge_params'],
            {'input_topic':'/autonomy/odomimu','output_topic':'/fmu/in/vehicle_visual_odometry'}]))
    elif part=='dds':
        transport=os.environ.get('PX4_TRANSPORT','udp4')
        if transport=='udp4': command=['MicroXRCEAgent','udp4','-p',os.environ.get('PX4_PORT','8888')]
        elif transport=='serial': command=['MicroXRCEAgent','serial','--dev','/dev/px4','-b',os.environ.get('PX4_BAUD','921600')]
        else: raise ValueError('PX4_TRANSPORT must be udp4 or serial')
        result=[ExecuteProcess(cmd=command,output='screen')]
    elif part=='depth':
        result=[include('larp_liteanystereo','las_depth.launch.py',{'publish_as_primary':'true','min_depth_m':'0.3','max_depth_m':'3.0','viz_enable':'false'})]
    elif part=='map':
        result=[node('larp_autonomy','map_status')]
    elif part=='super':
        params={k:cfg[k] for k in ('rig_file','hard_boundary_lower','hard_boundary_upper','hard_boundary_reference','takeoff_height')}
        params.update(config_file=cfg.get('navigation_config',str(nav/'navigation_fine.yaml')),
            cad_prior_file=cfg.get('cad_prior_file',str(nav/'course_cad_prior_fine.txt')),
            min_depth_m=.3,max_depth_m=3.,snapshot_include_free=False,
            require_hard_boundary=True,static_cad_only=False,route_backend='super_astar',
            flight_envelope_lower=[v+.4 for v in cfg['hard_boundary_lower']],
            flight_envelope_upper=[v-.4 for v in cfg['hard_boundary_upper']])
        result=[node('larp_navigation','depth_map_node',[params],remappings=remaps)]
    else:
        params=dict(simulation_only=False,hardware_enabled=active,alignment_verified=cfg['alignment_verified'],
            mission_id=cfg['mission_id'],map_heading_ned=cfg['map_heading_ned'],map_origin_ned=cfg['map_origin_ned'])
        result=[node('larp_autonomy','trajectory_bridge',[params]),
            node('larp_autonomy','mission_goal',[dict(zip(('x','y','z','yaw'),cfg['goal']),wait_for_hold=active)])]
        if active:
            result.append(node('flight_executive','flight_executive_node',[dict(
                simulation_only=False,hardware_enabled=True,takeoff_trigger='offboard',
                takeoff_height=cfg['takeoff_height'],require_autonomy_ready=True,enforce_boundary=True,
                map_boundary_lower=cfg['hard_boundary_lower'],map_boundary_upper=cfg['hard_boundary_upper'],
                map_heading_ned=cfg['map_heading_ned'],map_origin_ned=cfg['map_origin_ned'])]))
    return [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Component exited'))])),*result]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('component'),DeclareLaunchArgument('mission_file'),
        DeclareLaunchArgument('mode',default_value='shadow'),OpaqueFunction(function=nodes)])
