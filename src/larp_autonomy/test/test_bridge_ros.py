"""Synthetic wire smoke; requires an isolated ROS domain, no executive or PX4."""
import os
import subprocess
import sys
import time
import pytest

if os.environ.get('ROS_DOMAIN_ID') != '79' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    pytest.skip('Run in isolated ROS domain 79 with ROS_LOCALHOST_ONLY=1', allow_module_level=True)

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from dronehacks_msgs.msg import CommittedTrajectory, NavigationGoal, PolynomialPiece
from flight_executive_msgs.msg import TrajectorySetpoint
from std_msgs.msg import Bool, String


def test_certified_super_sample_reaches_executive_input_after_hold():
    rclpy.init()
    node = Node('synthetic_planner_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = node.create_publisher(String, '/flight_executive/state', latched)
    goal_pub = node.create_publisher(NavigationGoal, '/autonomy/goal', latched)
    health = node.create_publisher(Bool, '/autonomy/vio_healthy', qos_profile_sensor_data)
    curve_pub = node.create_publisher(CommittedTrajectory, '/autonomy/committed_trajectory', 4)
    received = []
    sub = node.create_subscription(TrajectorySetpoint, '/larper/trajectory_in', received.append, 10)
    process = subprocess.Popen([sys.executable, '-c',
        'from larp_autonomy.trajectory_bridge import main; main()', '--ros-args',
        '-p', 'simulation_only:=true', '-p', 'alignment_verified:=true',
        '-p', 'mission_id:=synthetic-flight', '-p', 'trajectory_max_age_s:=0.6'])
    try:
        goal = NavigationGoal()
        goal.header.frame_id = 'map'
        goal.goal_id = 1
        goal.localization_epoch = 1
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 1., 2., 3.
        goal_pub.publish(goal)
        def exercise(phase, duration):
            state.publish(String(data=phase))
            end = time.monotonic() + duration
            while time.monotonic() < end:
                health.publish(Bool(data=True))
                curve = CommittedTrajectory()
                curve.header.frame_id = 'map'
                # Account for capture/inference/search latency without rewriting age.
                capture_ns = node.get_clock().now().nanoseconds - 400_000_000
                curve.header.stamp.sec = capture_ns // 10**9
                curve.header.stamp.nanosec = capture_ns % 10**9
                # A restarted mapper starts a new timestamp-based sequence.
                curve.sequence = node.get_clock().now().nanoseconds
                curve.start_time.sec = curve.header.stamp.sec - 2
                curve.goal_id = curve.localization_epoch = 1
                curve.status = curve.VALID
                position = PolynomialPiece()
                position.degree, position.duration = 1, 1.
                position.coefficients = [0., 1., 0., 2., 0., 3.]
                yaw = PolynomialPiece()
                yaw.degree, yaw.duration, yaw.coefficients = 1, 1., [0., 0.]
                curve.position, curve.yaw = [position], [yaw]
                curve_pub.publish(curve)
                rclpy.spin_once(node, timeout_sec=.05)
                time.sleep(.02)
        exercise('TAKEOFF', 2.)
        assert not received
        exercise('HOLD', 2.)
        assert received, 'No executive input despite fresh healthy certified trajectory'
        sample = received[-1]
        assert sample.header.frame_id == 'local_ned'
        assert (sample.position.x, sample.position.y, sample.position.z) == (1., -2., -3.)
        assert sample.trajectory_id == 'synthetic-flight'
        assert sample.is_final and sample.certificate_required
        deadline=sample.valid_until.sec+sample.valid_until.nanosec*1e-9
        sent=sample.header.stamp.sec+sample.header.stamp.nanosec*1e-9
        assert 0 < deadline-sent < .25, 'Capture time must bound remaining lease'
        # A healthy stream without new map certificates must stop producing commands.
        until=time.monotonic()+.8
        while time.monotonic()<until:
            health.publish(Bool(data=True))
            rclpy.spin_once(node,timeout_sec=.02)
        count=len(received)
        until=time.monotonic()+.2
        while time.monotonic()<until:
            health.publish(Bool(data=True))
            rclpy.spin_once(node,timeout_sec=.02)
        assert len(received)==count
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
