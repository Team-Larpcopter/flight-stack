"""Isolated synthetic startup: no vehicle connection or command publisher."""
import os
import subprocess
import sys
import time
import pytest

if os.environ.get('ROS_DOMAIN_ID')!='188' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    pytest.skip('Requires isolated ROS domain 188',allow_module_level=True)
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleStatus,VehicleLandDetected


def test_origin_precedes_public_odometry_and_map_waits_for_px4_alignment():
    rclpy.init();node=rclpy.create_node('synthetic_origin_inputs')
    raw=node.create_publisher(Odometry,'/autonomy/unregistered_odometry',10)
    health=node.create_publisher(Bool,'/autonomy/unregistered_vio_healthy',10)
    status=node.create_publisher(VehicleStatus,'/fmu/out/vehicle_status_v1',qos_profile_sensor_data)
    landed=node.create_publisher(VehicleLandDetected,'/fmu/out/vehicle_land_detected',qos_profile_sensor_data)
    align=node.create_publisher(PoseStamped,'/autonomy/map_to_px4',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    poses=[];readiness=[]
    node.create_subscription(Odometry,'/autonomy/odometry',poses.append,10)
    node.create_subscription(Bool,'/autonomy/map_update_ready',lambda m:readiness.append(m.data),10)
    process=subprocess.Popen([sys.executable,'-c','from larp_autonomy.map_origin import main;main()',
                              '--ros-args','-p','hardware_enabled:=true'])
    def feed(duration,healthy=True,x=2.):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            m=Odometry();m.header.frame_id='vio_odom';m.child_frame_id='base_link'
            m.header.stamp=node.get_clock().now().to_msg()
            m.pose.pose.position.x=x;m.pose.pose.position.y=-1.;m.pose.pose.position.z=-.07
            m.pose.pose.orientation.w=1.
            raw.publish(m);health.publish(Bool(data=healthy))
            s=VehicleStatus();s.system_id=1;s.component_id=1
            s.arming_state=VehicleStatus.ARMING_STATE_DISARMED;s.pre_flight_checks_pass=False
            status.publish(s);landed.publish(VehicleLandDetected(landed=True))
            rclpy.spin_once(node,timeout_sec=.01);time.sleep(.01)
    try:
        feed(1.5,healthy=False)
        assert not any(readiness)
        feed(2.)
        assert poses,'Stable origin did not release odometry'
        p=poses[-1].pose.pose.position
        assert (p.x,p.y,p.z)==pytest.approx((0.,0.,0.))
        assert not any(readiness),'Map must not mutate before PX4 alignment'
        a=PoseStamped();a.header.frame_id='local_ned';a.pose.orientation.w=1.
        # Old retained alignment must not release a newly registered origin.
        align.publish(a);feed(.3)
        assert not any(readiness)
        a.header.stamp=node.get_clock().now().to_msg();align.publish(a)
        feed(.5,x=2.1)
        assert any(readiness)
        p=poses[-1].pose.pose.position
        assert (p.x,p.y,p.z)==pytest.approx((.1,0.,0.))
        feed(.4,healthy=False,x=float('nan'))
        assert any(m.pose.pose.position.x!=m.pose.pose.position.x for m in poses)
        feed(.4,x=2.2)
        assert poses[-1].pose.pose.position.x==pytest.approx(.2)

    finally:
        process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
