"""Bounded synthetic EGO test; no sensors, simulator, executive or PX4."""
import argparse
import os
import signal
import struct
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from scipy.interpolate import BSpline
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from dronehacks_msgs.msg import MapSnapshot, MapState, NavigationGoal
from larp_ego_planner.msg import ShadowTrajectory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '188', 'requires isolated test domain'
    rclpy.init()
    node = rclpy.create_node('ego_shadow_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(MapSnapshot, '/autonomy/map_snapshot', latched)
    states = node.create_publisher(MapState, '/autonomy/map_state', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/autonomy/odometry', qos_profile_sensor_data)
    health = node.create_publisher(Bool, '/autonomy/vio_healthy', 10)
    goals = node.create_publisher(NavigationGoal, '/autonomy/goal', latched)
    candidates = []
    subscription = node.create_subscription(ShadowTrajectory, '/autonomy/ego/shadow/trajectory', candidates.append, 10)
    healthy, publish_inputs, epoch, revision = True, True, 1, 1

    def inputs():
        if not publish_inputs:
            return
        stamp = node.get_clock().now().to_msg()
        body = Odometry(); body.header.frame_id = 'map'; body.header.stamp = stamp
        body.child_frame_id = 'base_link'; body.pose.pose.position.x = -1.2
        body.pose.pose.position.z = 1.5; body.pose.pose.orientation.w = 1.
        state = MapState(); state.header = body.header
        state.localization_epoch = epoch; state.revision = revision; state.body_pose = body.pose.pose
        odom.publish(body); states.publish(state); health.publish(Bool(data=healthy))

    timer = node.create_timer(.02, inputs)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'EGO process exited'
            rclpy.spin_once(node, timeout_sec=.02)

    def valid_plan():
        candidates.clear()
        deadline = time.monotonic() + 15
        while not any(m.valid for m in candidates) and time.monotonic() < deadline:
            spin(.1)
        valid = [m for m in candidates if m.valid]
        assert valid, 'EGO produced no candidate'
        result = valid[-1]
        assert result.order == 3 and len(result.control_points) >= 6
        assert len(result.knots) == len(result.control_points) + result.order + 1
        assert result.goal_id == goal.goal_id and result.map_revision == revision
        assert all(a <= b for a, b in zip(result.knots, result.knots[1:]))
        # Decode independently of EGO's evaluator and verify progress/geometry.
        spline = BSpline(result.knots, np.array([[p.x, p.y, p.z] for p in result.control_points]), result.order)
        points = spline(np.linspace(result.knots[result.order], result.knots[-result.order-1], 301))
        assert np.isfinite(points).all()
        assert np.linalg.norm(points[0] - [-1.2, 0., 1.5]) < .05
        # Upstream EGO uses a soft terminal cost, not an exact endpoint constraint.
        assert np.linalg.norm(points[-1] - [1.2, 0., 1.5]) < .5, points[-1]
        if revision in (2, 4):
            clearance = np.maximum(np.abs(points - [.05, .05, 1.55]) - .05, 0.)
            assert (np.linalg.norm(clearance, axis=1) >= .4).all(), 'candidate intersects inflated obstacle'

    def invalid_only():
        spin(.6)
        candidates.clear(); spin(.6)
        assert candidates and all(not m.valid and not m.control_points for m in candidates)

    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([args.node, '--ros-args', '--params-file', args.config,
                                    '-r', '__ns:=/autonomy/ego'], stdout=log, stderr=subprocess.STDOUT)
        try:
            snapshot = MapSnapshot(); snapshot.header.frame_id = 'map'
            snapshot.header.stamp = node.get_clock().now().to_msg()
            snapshot.localization_epoch = 1; snapshot.revision = 1; snapshot.resolution = .1
            snapshot.lower.x = -2.; snapshot.lower.y = -2.; snapshot.lower.z = 0.
            snapshot.upper.x = 2.; snapshot.upper.y = 2.; snapshot.upper.z = 3.
            cloud = PointCloud2(); cloud.header = snapshot.header; cloud.height = 1; cloud.point_step = 12
            cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate('xyz')]
            cloud.data = b''.join(struct.pack('<fff', (x+.5)*.1, (y+.5)*.1, (z+.5)*.1)
                                  for x in range(-20, 20) for y in range(-20, 20) for z in range(30))
            cloud.width = len(cloud.data)//12; cloud.row_step = len(cloud.data)
            snapshot.free = cloud
            maps.publish(snapshot)
            goal = NavigationGoal(); goal.header.frame_id = 'map'; goal.localization_epoch = 1; goal.goal_id = 1
            goal.pose.position.x = 1.2; goal.pose.position.z = 1.5; goal.pose.orientation.w = 1.
            goals.publish(goal)
            valid_plan()
            # Retained geometry can be old; measured pose leases must be fresh.
            spin(1.); valid_plan()
            healthy = False; invalid_only()
            healthy = True; valid_plan()
            publish_inputs = False; invalid_only()
            publish_inputs = True; valid_plan()
            goal.cancel = True; goals.publish(goal); invalid_only()
            goal.cancel = False; goals.publish(goal); invalid_only()  # cancelled ID cannot resurrect
            goal.goal_id = 2; goals.publish(goal); valid_plan()
            # Replace retained geometry with an obstacle on the direct path.
            obstacle = PointCloud2(); obstacle.header = cloud.header; obstacle.height = 1
            obstacle.fields = cloud.fields; obstacle.point_step = 12; obstacle.width = 1; obstacle.row_step = 12
            obstacle.data = struct.pack('<fff', .05, .05, 1.55)
            snapshot.occupied = obstacle; snapshot.revision = 2; revision = 2
            maps.publish(snapshot); valid_plan()
            # A map with no declared free cells cannot authorize a candidate.
            snapshot.free = PointCloud2(); snapshot.revision = 3; revision = 3
            maps.publish(snapshot); invalid_only()
            snapshot.free = cloud; snapshot.revision = 4; revision = 4
            maps.publish(snapshot); valid_plan()
            # A changed localization epoch latches output off until restart.
            epoch = 2; invalid_only()
            epoch = 1; invalid_only()
            pubs = node.get_publisher_names_and_types_by_node('ego_shadow', '/autonomy/ego')
            assert not any(topic.startswith(('/fmu/', '/larper/')) for topic, _ in pubs)
            assert not any('dronehacks_msgs/msg/CommittedTrajectory' in types for _, types in pubs)
            print('EGO plans around an obstacle; unknown space, stale inputs, unhealthy VIO, cancellation and epoch changes clear output; no command publishers')
        except BaseException:
            log.seek(0); print(log.read()[-3000:])
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
