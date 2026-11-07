"""Run with the built Humble overlay sourced. Uses isolated synthetic topics."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node('vins_bridge_smoke')
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    prefix = f'/bridge_test_{os.getpid()}'
    publisher = node.create_publisher(Odometry, prefix + '/input', qos)
    received = []
    subscription = node.create_subscription(
        VehicleOdometry, prefix + '/output', received.append, qos)
    process = None
    sent = set()
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'bridge.yaml'
            config.write_text(
                'vins_px4_bridge:\n  ros__parameters:\n'
                '    imu_to_body_frd_quaternion_wxyz: [0.0, 1.0, 0.0, 0.0]\n'
                f'    input_topic: {prefix}/input\n'
                f'    output_topic: {prefix}/output\n'
                '    publish_rate_hz: 50.0\n'
                '    max_sample_age_s: 0.2\n')
            process = subprocess.Popen(
                ['ros2', 'launch', 'larp_px4_bridge', 'bridge.launch.py',
                 f'params_file:={config}'], start_new_session=True)
            deadline = time.monotonic() + 15
            while publisher.get_subscription_count() == 0 or node.count_publishers(prefix + '/output') == 0:
                assert process.poll() is None, 'Bridge exited during discovery'
                assert time.monotonic() < deadline, 'Bridge topic discovery timed out'
                rclpy.spin_once(node, timeout_sec=0.05)
            deadline = time.monotonic() + 2
            next_sample = time.monotonic()
            while time.monotonic() < deadline:
                if time.monotonic() >= next_sample:
                    message = Odometry()
                    stamp = node.get_clock().now()
                    message.header.stamp = stamp.to_msg()
                    message.header.frame_id = 'global'
                    message.child_frame_id = 'imu'
                    message.pose.pose.position.x = 1.0
                    message.pose.pose.position.y = 2.0
                    message.pose.pose.position.z = 3.0
                    message.pose.pose.orientation.w = 1.0
                    message.twist.twist.linear.x = 4.0
                    message.twist.twist.linear.y = 5.0
                    message.twist.twist.linear.z = 6.0
                    for index in (0, 7, 14, 21, 28, 35):
                        message.pose.covariance[index] = 1.0
                        message.twist.covariance[index] = 1.0
                    sent.add(stamp.nanoseconds // 1000)
                    publisher.publish(message)
                    next_sample = time.monotonic() + 0.005
                rclpy.spin_once(node, timeout_sec=0.001)
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            count = len(received)
            assert 40 <= count <= 120, f'Unexpected output count {count}'
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            assert len(received) == count, 'Bridge repeated samples after input stopped'
            previous = 0
            for message in received:
                for actual, expected in zip(message.position, (1, -2, -3)):
                    assert math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(message.velocity, (4, -5, -6)):
                    assert math.isclose(actual, expected, abs_tol=1e-6)
                assert math.isclose(abs(message.q[0]), 1, abs_tol=1e-6)
                assert all(abs(value) < 1e-6 for value in message.q[1:])
                assert message.pose_frame == VehicleOdometry.POSE_FRAME_FRD
                assert message.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD
                assert message.timestamp_sample in sent, 'Sample timestamp was changed'
                assert previous < message.timestamp_sample <= message.timestamp
                previous = message.timestamp_sample
            print(f'PASS: {count} transformed fresh messages; output stopped with input')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
