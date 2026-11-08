"""Regression: a pose correction or bad sample cannot latch off VIO output."""
import os
import subprocess
import time
import sys

import pytest

if os.environ.get('ROS_DOMAIN_ID') != '79' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    pytest.skip('Requires isolated ROS domain 79', allow_module_level=True)

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data


def test_jump_covariance_and_invalid_sample_do_not_latch_output(tmp_path):
    rig = tmp_path / 'rig.yaml'
    rig.write_text('T_body_imu: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]\n')
    rclpy.init()
    node = rclpy.create_node('body_output_regression')
    pub = node.create_publisher(Odometry, '/odomimu', qos_profile_sensor_data)
    received = []
    px4_received = []
    px4_sub = node.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", px4_received.append, qos_profile_sensor_data)
    sub = node.create_subscription(Odometry, '/navigation/odometry', received.append, 20)
    executable = os.environ['BODY_ODOMETRY_EXECUTABLE']
    proc = subprocess.Popen([executable, '--ros-args', '-p', f'rig_file:={rig}'])
    bridge = subprocess.Popen([sys.executable, '-c',
        'from larp_px4_bridge.node import main;main()', '--ros-args',
        '-p', 'imu_to_body_frd_quaternion_wxyz:=[0.0,1.0,0.0,0.0]',
        '-p', 'input_topic:=/navigation/odometry', '-p', 'world_frame:=map',
        '-p', 'imu_frame:=base_link', '-p', 'max_sample_age_s:=0.0'])
    def send(x, stamp, covariance=.01):
        m = Odometry()
        m.header.frame_id = 'global'; m.child_frame_id = 'imu'
        m.header.stamp.sec = stamp
        m.pose.pose.orientation.w = 1.
        m.pose.pose.position.x = x
        for i in range(6):
            m.pose.covariance[7*i] = covariance
            m.twist.covariance[7*i] = .01
        pub.publish(m)
    def wait(predicate, seconds=5.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate(): return
        assert predicate()
    try:
        wait(lambda: pub.get_subscription_count() > 0)
        wait(lambda: node.count_subscribers("/navigation/odometry") >= 2)
        send(0., 100)
        wait(lambda: bool(received))
        wait(lambda: bool(px4_received))
        send(10., 101)  # Exceeds the old jump threshold.
        wait(lambda: any(m.pose.pose.position.x == 10. for m in received))
        send(11., 102, covariance=10.)  # Health must not gate publication.
        wait(lambda: any(m.pose.pose.position.x == 11. for m in received))
        send(float('nan'), 103)
        wait(lambda: any(m.pose.pose.position.x != m.pose.pose.position.x for m in received))
        wait(lambda: any(m.position[0] != m.position[0] for m in px4_received))
        send(12., 1)  # A bad sample and backward clock must not poison future output.
        wait(lambda: any(m.pose.pose.position.x == 12. for m in received))
        wait(lambda: any(m.position[0] == 12. for m in px4_received))
    finally:
        bridge.terminate(); bridge.wait(timeout=5)
        proc.terminate(); proc.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()
