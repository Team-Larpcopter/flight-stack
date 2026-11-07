import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from .core import Converter, LatestSample, Sample


class Bridge(Node):
    def __init__(self):
        super().__init__('vins_px4_bridge')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.declare_parameter('imu_to_body_frd_quaternion_wxyz', Parameter.Type.DOUBLE_ARRAY)
        try:
            mount = self.get_parameter('imu_to_body_frd_quaternion_wxyz').value
        except ParameterUninitializedException:
            mount = None
        if mount is None or len(mount) != 4:
            raise ValueError('Set imu_to_body_frd_quaternion_wxyz to the measured IMU-to-body FRD rotation')
        self.converter = Converter(mount)
        rate = param('publish_rate_hz', 50.0)
        age = param('max_sample_age_s', 0.2)
        if not math.isfinite(rate) or rate <= 0 or not math.isfinite(age) or age <= 0:
            raise ValueError('Rate and maximum age must be positive and finite')
        self.queue = LatestSample(int(age * 1e9))
        self.frame = param('world_frame', 'global')
        self.child = param('imu_frame', 'imu')
        self.reset = param('reset_counter', 0)
        if not 0 <= self.reset <= 255:
            raise ValueError('reset_counter must be in 0..255')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(VehicleOdometry, param('output_topic', '/fmu/in/vehicle_visual_odometry'), qos)
        self.subscription = self.create_subscription(Odometry, param('input_topic', '/autonomy/odomimu'), self.receive, qos)
        self.timer = self.create_timer(1.0/rate, self.publish)

    def receive(self, msg):
        if msg.header.frame_id != self.frame or msg.child_frame_id != self.child:
            self.get_logger().warning(
                f'Expected frames {self.frame}/{self.child}, got '
                f'{msg.header.frame_id}/{msg.child_frame_id}', throttle_duration_sec=5)
            return
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        sample = Sample(msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec,
                        [p.x,p.y,p.z], [q.w,q.x,q.y,q.z], [v.x,v.y,v.z], [w.x,w.y,w.z],
                        msg.pose.covariance, msg.twist.covariance)
        try:
            self.converter.convert(sample)
            now = self.get_clock().now().nanoseconds
            if not self.queue.offer(sample, now):
                self.get_logger().warning(
                    f'Rejected timestamp {sample.stamp_ns}; age={(now-sample.stamp_ns)/1e9:.6f}s, '
                    f'last accepted={self.queue.high_water}', throttle_duration_sec=5)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)

    def publish(self):
        now = self.get_clock().now().nanoseconds
        sample = self.queue.take(now)
        if sample is None:
            return
        out = VehicleOdometry()
        out.timestamp = now // 1000
        out.timestamp_sample = sample.stamp_ns // 1000
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out.reset_counter = self.reset
        out.quality = 0
        for key, value in self.converter.convert(sample).items():
            setattr(out, key, value)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
