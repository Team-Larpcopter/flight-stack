"""ROS 2 node that publishes a hard-coded course trajectory.

It feeds flight_executive_msgs/TrajectorySetpoint on the executive's input
topic. The flight executive owns arming, takeoff, mode, and safety checks.
"""

from math import nan, atan2, cos, sin
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from time import monotonic
from uuid import uuid4

import rclpy
from flight_executive_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from flight_trajectory.course_follower import CourseFollower, Phase, Pose
from flight_trajectory.dronehacks_course import course_trajectory
from flight_trajectory.triangle_course import TriangleCourse, TriangleFollower
from flight_trajectory.fixed_course import FixedCourse, FixedCourseFollower, load_waypoints

COURSES = {'dronehacks': course_trajectory, 'triangle': TriangleCourse, 'fixed_course': FixedCourse}


def topic_name(prefix, name, message_type):
    """Match PX4's DDS topic suffix for versioned messages."""
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/{name}{suffix}'


class CourseTrajectoryNode(Node):
    """Publish one course trajectory sample per timer tick."""

    def __init__(self):
        super().__init__('course_trajectory')
        simulation = self.declare_parameter('simulation_only', False).value
        hardware = self.declare_parameter('hardware_enabled', False).value
        course = self.declare_parameter('course', 'dronehacks').value
        if simulation == hardware or (hardware and course not in ('triangle', 'fixed_course')):
            raise ValueError('Select simulation, or explicitly enable triangle/fixed-course hardware')
        if course not in COURSES:
            raise ValueError(f'Unknown course {course!r}; choose from {sorted(COURSES)}')
        height = self.declare_parameter('cruise_height', 0.75).value
        max_speed = self.declare_parameter('max_speed', 1.0).value
        yaw_mode = self.declare_parameter('yaw_mode', 'tangent').value
        max_yaw_rate = self.declare_parameter('max_yaw_rate', 1.0).value
        max_error = self.declare_parameter('max_tracking_error', 0.4).value
        rate = self.declare_parameter('publish_rate_hz', 50.0).value
        prefix = self.declare_parameter('px4_topic_prefix', '/fmu').value.rstrip('/')
        if not prefix.startswith('/'):
            raise ValueError('px4_topic_prefix must be an absolute nonempty topic prefix')
        if not 20.0 <= rate <= 200.0:
            raise ValueError('publish_rate_hz must be within 20..200')
        if course == 'triangle':
            trajectory = TriangleCourse()
            max_speed, yaw_mode = .2, 'fixed'
        elif course == 'fixed_course':
            route_file = self.declare_parameter('fixed_course_file', '').value
            trajectory = FixedCourse(load_waypoints(route_file) if route_file else None)
            max_speed, yaw_mode = .2, 'stop-and-turn'
        else:
            trajectory = COURSES[course](max_speed=max_speed, height=height,
                                         yaw_mode=yaw_mode, max_yaw_rate=max_yaw_rate)
        follower = {'triangle': TriangleFollower, 'fixed_course': FixedCourseFollower}.get(course, CourseFollower)
        self.follower = follower(trajectory, max_error)
        self.follower.skybeta_compat = True
        self.trajectory_id = str(uuid4())
        self.landed = None

        telemetry_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.input_subscriptions = [
            self.create_subscription(
                VehicleLocalPosition,
                topic_name(prefix, 'out/vehicle_local_position', VehicleLocalPosition),
                self._position, telemetry_qos),
            self.create_subscription(
                VehicleLandDetected,
                topic_name(prefix, 'out/vehicle_land_detected', VehicleLandDetected),
                self._landed, telemetry_qos),
            self.create_subscription(
                String, self.declare_parameter(
                    'executive_state_topic', '/flight_executive/state').value,
                self._executive_state, latched),
        ]
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            self.declare_parameter('trajectory_topic', '/larper/trajectory_in').value, 10)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.last_report = None
        self.map_alignment = None
        if course == 'fixed_course':
            self.path_pub = self.create_publisher(Path, '/autonomy/committed_path', 1)
            self.goal_pub = self.create_publisher(PoseStamped, '/viz/autonomy/goal_pose', latched)
            self.input_subscriptions.append(self.create_subscription(
                PoseStamped, '/autonomy/map_to_px4', self._map_alignment, latched))
            self.view_timer = self.create_timer(1., self._publish_fixed_path)
        self.timer = self.create_timer(
            1.0 / rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Course {course}: {trajectory.length:.1f} m in {trajectory.duration:.1f} s, '
            f'up to {max_speed:.2f} m/s, yaw {yaw_mode}; waiting for executive HOLD')
        self._report()

    def _map_alignment(self, msg):
        if msg.header.frame_id == 'local_ned':
            self.map_alignment = msg.pose

    def _publish_fixed_path(self):
        # Only a visualization copy: no map lookup, collision checks or planner.
        if self.map_alignment is None or self.follower.anchor is None:
            return
        origin = self.map_alignment.position
        q = self.map_alignment.orientation
        heading = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s = cos(heading), sin(heading)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        course = self.follower.trajectory
        count = max(1, int(course.duration/.5))
        for i in range(count+1):
            sample = self.follower.local(course.sample(course.duration*i/count))
            dx, dy, dz = (sample.position[0]-origin.x,
                          sample.position[1]-origin.y, sample.position[2]-origin.z)
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = c*dx+s*dy
            pose.pose.position.y = s*dx-c*dy
            pose.pose.position.z = -dz
            yaw = heading-sample.yaw
            pose.pose.orientation.w, pose.pose.orientation.z = cos(yaw/2), sin(yaw/2)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.goal_pub.publish(path.poses[-1])

    def _position(self, msg):
        self.follower.update_pose(
            Pose(monotonic(), msg.x, msg.y, msg.z, msg.heading,
                 msg.xy_valid and msg.z_valid), self.landed)

    def _landed(self, msg):
        self.landed = msg.landed

    def _executive_state(self, msg):
        self.follower.update_executive_state(msg.data)

    def _report(self):
        f = self.follower
        report = (f.phase, f.reason)
        if report != self.last_report:
            self.last_report = report
            self.status_pub.publish(String(data=f.phase.value))
            if f.phase == Phase.ABORTED:
                self.get_logger().error(f'ABORTED: {f.reason}; executive will hold')
            else:
                self.get_logger().info(f.phase.value)

    def _tick(self):
        sample = self.follower.tick(monotonic())
        self._report()
        if sample is None:
            return
        msg = TrajectorySetpoint()
        msg.trajectory_id = self.trajectory_id
        msg.is_final = sample.is_final
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'local_ned'
        msg.position.x, msg.position.y, msg.position.z = map(float, sample.position)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, sample.velocity)
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = map(
            float, sample.acceleration)
        msg.yaw_valid = sample.yaw is not None
        msg.yaw = float(sample.yaw) if msg.yaw_valid else nan
        msg.yawspeed = float(sample.yawspeed) if msg.yaw_valid else nan
        self.trajectory_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CourseTrajectoryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
