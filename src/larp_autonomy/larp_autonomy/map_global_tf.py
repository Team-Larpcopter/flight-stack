"""Expose the fixed VINS-global registration using timestamp-matched poses."""
from collections import OrderedDict
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def registration(map_body, body_imu, global_imu):
    # T_map_global transforms points expressed in global into map coordinates.
    return map_body @ body_imu @ np.linalg.inv(global_imu)


def main():
    import yaml
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init()
    node = rclpy.create_node('map_global_tf')
    with open(node.declare_parameter('rig_file', '').value) as stream:
        body_imu = np.asarray(yaml.safe_load(stream)['T_body_imu'], dtype=float)
    if body_imu.shape != (4, 4) or not np.isfinite(body_imu).all():
        raise ValueError('Invalid T_body_imu')
    broadcaster = StaticTransformBroadcaster(node)
    pending = [OrderedDict(), OrderedDict()]
    frozen = False

    def receive(msg, side):
        nonlocal frozen
        if frozen:
            return
        expected = ('global', 'imu') if side == 0 else ('map', 'base_link')
        if (msg.header.frame_id, msg.child_frame_id) != expected:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        pending[side][stamp] = msg
        while len(pending[side]) > 512:
            pending[side].popitem(last=False)
        if stamp not in pending[1-side]:
            return
        raw, mapped = pending[0][stamp], pending[1][stamp]
        matrix = registration(pose_matrix(mapped.pose.pose), body_imu,
                              pose_matrix(raw.pose.pose))
        if not np.isfinite(matrix).all():
            return
        transform = TransformStamped()
        transform.header = mapped.header
        transform.child_frame_id = 'global'
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = matrix[:3, 3].tolist()
        q.x, q.y, q.z, q.w = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
        broadcaster.sendTransform(transform)
        frozen = True
        pending[0].clear(); pending[1].clear()
        node.get_logger().info('Published fixed map -> global TF from matched mission/VINS poses')

    node.create_subscription(Odometry, '/autonomy/odomimu', lambda m: receive(m, 0), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/autonomy/odometry', lambda m: receive(m, 1), qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
