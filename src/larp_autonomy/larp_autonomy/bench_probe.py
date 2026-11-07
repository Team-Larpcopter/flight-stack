"""Bounded, subscription-only hardware evidence. Never publishes flight commands."""
import argparse,json,time
from pathlib import Path
import numpy as np


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String,Int64,Bool
    from px4_msgs.msg import VehicleOdometry,VehicleLocalPosition,VehicleStatus,EstimatorStatusFlags,TrajectorySetpoint as Px4Setpoint
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);ap.add_argument('--seconds',type=float,default=30.);ap.add_argument('--warmup',type=float,default=5.)
    a=ap.parse_args();rclpy.init();node=rclpy.create_node('autonomy_bench_probe');records={};imu=[];states=[];health=[];frames={};poses=[]
    start=time.monotonic()
    def receive(m,topic):
        wall=time.monotonic();now=node.get_clock().now().nanoseconds*1e-9
        if isinstance(m,Int64):stamp=m.data*1e-9
        elif hasattr(m,'header'):stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        else:stamp=getattr(m,'timestamp_sample',m.timestamp)*1e-6
        if wall-start>=a.warmup:records.setdefault(topic,[]).append((wall-start,stamp,now-stamp))
        if isinstance(m,Image):frames[topic]={'width':m.width,'height':m.height,'encoding':m.encoding,'frame':m.header.frame_id}
        if isinstance(m,Imu):
            x=m.linear_acceleration;w=m.angular_velocity
            if wall-start>=a.warmup:imu.append([x.x,x.y,x.z,w.x,w.y,w.z])
        if isinstance(m,Odometry) and topic=='/autonomy/odometry':
            q=m.pose.pose.position;poses.append([wall-start,q.x,q.y,q.z])
    for topic,typ in [('/oak/left/image_undistorted',Image),('/oak/right/image_undistorted',Image),('/oak/imu/data',Imu),
      ('/oak/stereo/image_raw',Image),('/autonomy/stereo/left/image',Image),('/autonomy/stereo/right/image',Image),
      ('/native_oak/left/image_undistorted',Image),('/native_oak/right/image_undistorted',Image),('/autonomy/imu/data',Imu),
      ('/fmu/in/vehicle_visual_odometry',VehicleOdometry),('/fmu/out/vehicle_odometry',VehicleOdometry),
      ('/fmu/out/vehicle_local_position',VehicleLocalPosition),('/fmu/in/trajectory_setpoint',Px4Setpoint),
      ('/autonomy/odometry',Odometry),('/autonomy/poseimu',PoseWithCovarianceStamped),('/autonomy/processed_vio',Int64)]:
        node.create_subscription(typ,topic,lambda m,t=topic:receive(m,t),qos_profile_sensor_data)
    node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',lambda m:states.append({'wall_s':time.monotonic()-start,'arming_state':m.arming_state,'nav_state':m.nav_state,'failsafe':m.failsafe}),qos_profile_sensor_data)
    node.create_subscription(EstimatorStatusFlags,'/fmu/out/estimator_status_flags',lambda m:states.append({'wall_s':time.monotonic()-start,'ev_pos':m.cs_ev_pos,'ev_vel':m.cs_ev_vel,'ev_yaw':m.cs_ev_yaw,'ev_hgt':m.cs_ev_hgt}),qos_profile_sensor_data)
    node.create_subscription(String,'/skybeta/state',lambda m:states.append({'wall_s':time.monotonic()-start,'data':m.data}),10)
    node.create_subscription(Bool,'/autonomy/vio_healthy',lambda m:health.append([time.monotonic()-start,m.data]),10)
    try:
        while time.monotonic()-start<a.seconds+a.warmup:rclpy.spin_once(node,timeout_sec=.05)
    finally:node.destroy_node();rclpy.try_shutdown()
    summary={}
    for topic,rows in records.items():
        x=np.asarray(rows);dt=np.diff(x[:,0]);capture_dt=np.diff(x[:,1]);ages=x[:,2]*1000
        summary[topic]={'samples':len(x),'receive_rate_hz':float((len(x)-1)/(x[-1,0]-x[0,0])) if len(x)>1 else 0.,
          'capture_age_ms_p50_p95_p99_max':np.percentile(ages,[50,95,99,100]).tolist() if np.all(np.abs(ages)<10000) else None,
          'timestamp_comparable_to_ros_clock':bool(np.all(np.abs(ages)<10000)),
          'negative_capture_age_count':int(sum(ages < -5)),
          'nonincreasing_capture_count':int(sum(capture_dt<=0)),
          'receive_max_gap_ms':float(max(dt)*1000) if len(dt) else None,
          'capture_max_gap_ms':float(max(capture_dt)*1000) if len(capture_dt) else None}
    result={'scope':__doc__,'flight_qualified':False,'steady_observation_s':a.seconds,'warmup_s':a.warmup,'topics':summary,'frames':frames,'hardware_states':states[-20:],
            'vio_health_samples':len(health),'vio_unhealthy_after_warmup':sum(not h for t,h in health if t>=a.warmup),
            'qualification_limits':['No independent moving-pose truth','No flight commands tested','IMU characterization and airframe mounting require independent validation']}
    if imu:
        x=np.asarray(imu);result['imu']={'accel_mean':x[:,:3].mean(axis=0).tolist(),'accel_norm_mean':float(np.linalg.norm(x[:,:3],axis=1).mean()),
          'accel_std':x[:,:3].std(axis=0).tolist(),'gyro_mean':x[:,3:].mean(axis=0).tolist(),'gyro_std':x[:,3:].std(axis=0).tolist()}
    if poses:
        x=np.array(poses);result['pose_displacement_m']={'final':float(np.linalg.norm(x[-1,1:]-x[0,1:])),'max':float(np.max(np.linalg.norm(x[:,1:]-x[0,1:],axis=1))),
          'note':'Displacement is not error unless the sensor was independently stationary'}
    p=Path(a.out);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
