"""Passive bounded topic-age profiler; never publishes or connects to devices."""
import json
import math
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, CameraInfo
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from dronehacks_msgs.msg import RoutePolyline, CommittedTrajectory
    from flight_executive_msgs.msg import TrajectorySetpoint
    rclpy.init()
    node = Node('planner_latency')
    duration = float(node.declare_parameter('duration_seconds', 30.0).value)
    if not math.isfinite(duration) or not 0 < duration <= 600:
        raise ValueError('duration_seconds must be in (0, 600]')
    destination = node.declare_parameter('trajectory_topic', '/autonomy/shadow/trajectory_in').value
    samples = {}
    received_counts = {}
    status_counts = {}
    subscriptions = []
    def callback(topic):
        def receive(msg):
            received_counts[topic] = received_counts.get(topic, 0) + 1
            if hasattr(msg, 'status'):
                counts = status_counts.setdefault(topic, {})
                counts[str(msg.status)] = counts.get(str(msg.status), 0) + 1
                if msg.status != msg.VALID:
                    return  # Invalid/REACHED notifications are not executable curves/routes.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            samples.setdefault(topic, []).append((time.monotonic(), age * 1000))
        return receive
    for topic, kind in (('/autonomy/imu/data', Imu),
                        ('/native_oak/left/camera_info', CameraInfo),
                        ('/native_oak/right/camera_info', CameraInfo),
                        ('/autonomy/poseimu', PoseWithCovarianceStamped),
                        ('/autonomy/odomimu', Odometry), ('/autonomy/odometry', Odometry),
                        ('/autonomy/route', RoutePolyline),
                        ('/autonomy/committed_trajectory', CommittedTrajectory),
                        (destination, TrajectorySetpoint)):
        samples[topic] = []
        subscriptions.append(node.create_subscription(kind, topic, callback(topic), qos_profile_sensor_data))
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        report = {}
        for topic, data in samples.items():
            ages = sorted(a for _, a in data)
            gaps = [(b[0]-a[0])*1000 for a, b in zip(data, data[1:])]
            def percentile(q):
                return ages[round((len(ages)-1)*q)] if ages else None
            report[topic] = {'count': len(data), 'received_count': received_counts.get(topic, 0),
                'status_counts': status_counts.get(topic, {}),
                'received_hz': received_counts.get(topic, 0)/elapsed,
                'valid_hz': len(data)/elapsed,
                'last_valid_receipt_age_ms': (time.monotonic()-data[-1][0])*1000 if data else None,
                'age_ms_p50': percentile(.5), 'age_ms_p95': percentile(.95),
                'age_ms_p99': percentile(.99), 'age_ms_max': max(ages) if ages else None,
                'max_receipt_gap_ms': max(gaps) if gaps else None}
        print(json.dumps({'measurement': 'ROS header age at passive DDS receipt; not causal end-to-end latency',
                          'duration_seconds': elapsed, 'topics': report}, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
