"""Exercise ground re-registration through the real trajectory bridge."""
import os
import subprocess
import sys
import time
import pytest

if os.environ.get('ROS_DOMAIN_ID') != '79' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    pytest.skip('Requires isolated ROS domain 79', allow_module_level=True)

rclpy = pytest.importorskip('rclpy')
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from dronehacks_msgs.msg import CommittedTrajectory, NavigationGoal, PolynomialPiece
from flight_executive_msgs.msg import TrajectorySetpoint


def test_refreshed_ground_transform_reaches_planner_output():
    rclpy.init()
    node=rclpy.create_node('alignment_refresh_test')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    phase=node.create_publisher(String,'/flight_executive/state',latched)
    align=node.create_publisher(PoseStamped,'/autonomy/map_to_px4',latched)
    goals=node.create_publisher(NavigationGoal,'/autonomy/goal',latched)
    curves=node.create_publisher(CommittedTrajectory,'/autonomy/committed_trajectory',4)
    received=[]
    sub=node.create_subscription(TrajectorySetpoint,'/larper/trajectory_in',received.append,10)
    proc=subprocess.Popen([sys.executable,'-c','from larp_autonomy.trajectory_bridge import main; main()',
        '--ros-args','-p','simulation_only:=true','-p','alignment_verified:=true',
        '-p','align_px4_to_map_on_ground:=true','-p','mission_id:=refresh-test'])
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    try:
        phase.publish(String(data='TAKEOFF'))
        msg=PoseStamped();msg.header.frame_id='local_ned';msg.pose.orientation.w=1.
        msg.pose.position.x=1.4
        align.publish(msg)
        spin(2.)
        msg.pose.position.x=0.
        align.publish(msg)
        spin(.5)
        goal=NavigationGoal();goal.header.frame_id='map';goal.goal_id=goal.localization_epoch=1
        goal.pose.position.x=1.;goal.pose.position.y=2.;goal.pose.position.z=3.
        goals.publish(goal);phase.publish(String(data='HOLD'))
        end=time.monotonic()+3.
        while time.monotonic()<end and not received:
            curve=CommittedTrajectory();curve.header.frame_id='map'
            curve.header.stamp=node.get_clock().now().to_msg();curve.start_time=curve.header.stamp
            curve.sequence=node.get_clock().now().nanoseconds
            curve.goal_id=curve.localization_epoch=1;curve.status=curve.VALID
            p=PolynomialPiece();p.degree=1;p.duration=10.;p.coefficients=[0.,1.,0.,2.,0.,3.]
            y=PolynomialPiece();y.degree=1;y.duration=10.;y.coefficients=[0.,0.]
            curve.position=[p];curve.yaw=[y];curves.publish(curve);spin(.1)
        assert received, 'Replacement ground alignment permanently blocked planner output'
        p=received[-1].position
        assert (p.x,p.y,p.z)==pytest.approx((1.,-2.,-3.))
        # An active registration/epoch change and restarted sequence must recover.
        phase.publish(String(data='TRACKING'))
        spin(.2)
        msg.pose.position.x=2.
        align.publish(msg)
        goal.localization_epoch=2
        goals.publish(goal)
        spin(.3)
        received.clear()
        curve.localization_epoch=2
        curve.sequence=1
        end=time.monotonic()+3.
        while time.monotonic()<end and not received:
            curves.publish(curve);spin(.1)
        assert received, 'Active alignment or epoch change latched command output off'
        assert received[-1].position.x==pytest.approx(3.)
        # A sequence restart within the same epoch must not block subsequent curves.
        curve.sequence=0
        curve.position[0].coefficients=[0.,4.,0.,2.,0.,3.]
        received.clear()
        end=time.monotonic()+3.
        while time.monotonic()<end and not any(abs(m.position.x-6.)<1e-6 for m in received):
            curves.publish(curve);spin(.1)
        assert any(abs(m.position.x-6.)<1e-6 for m in received)
    finally:
        proc.terminate();proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
