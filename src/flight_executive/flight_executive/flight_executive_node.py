"""Simulation-only ROS 2 adapter for the flight state machine."""

from math import isfinite, nan, atan2, sin, cos, hypot, copysign
import struct

from time import monotonic

import rclpy
from flight_executive_msgs.msg import TrajectorySetpoint as LarperTrajectorySetpoint
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
)
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from flight_executive.model import Command, Landed, Position, Status, Trajectory, State, Phase
from flight_executive.state_machine import StateMachine, LANDED_TIMEOUT
from flight_executive.envelope import Envelope
from flight_executive.alignment import GroundAlignment
from flight_executive.extensions.tracking import stationary


def px4_float(value):
    """Encode float32, preserving NaN/Inf and representing overflow as Inf."""
    try:
        return struct.unpack("f", struct.pack("f", value))[0]
    except OverflowError:
        return copysign(float("inf"), value)


def topic_name(prefix, name, message_type):
    """Match PX4's DDS topic suffix for versioned messages."""
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/{name}{suffix}'


class FlightExecutiveNode(Node):
    """Translate PX4 messages and publish the state machine's outputs at 20 Hz."""

    def __init__(self):
        super().__init__('flight_executive')
        simulation = self.declare_parameter('simulation_only', False).value
        hardware = self.declare_parameter('hardware_enabled', False).value
        if bool(simulation) == bool(hardware):
            raise ValueError('Explicitly select exactly one of simulation_only or hardware_enabled')
        height = self.declare_parameter('takeoff_height', 1.0).value
        hold_timeout = self.declare_parameter('hold_timeout', 5.0).value
        landing_timeout = self.declare_parameter('landing_timeout', 45.0).value
        trigger = self.declare_parameter('takeoff_trigger', 'position').value
        self.skybeta_compat = self.declare_parameter('skybeta_compat', True).value
        self.state_machine = StateMachine(monotonic(), height, hold_timeout, landing_timeout, trigger, readiness_timeout=0.0 if hardware else 30.0,
                                          wait_for_planner=self.declare_parameter('wait_for_planner', False).value, skybeta_compat=self.skybeta_compat)
        self.prefix = self.declare_parameter('px4_topic_prefix', '/fmu').value.rstrip('/')
        if not self.prefix.startswith('/') or self.prefix == '':
            raise ValueError('px4_topic_prefix must be an absolute nonempty topic prefix')
        self.target_system = self.declare_parameter('target_system', 1).value
        self.target_component = self.declare_parameter('target_component', 1).value
        self.source_system = self.declare_parameter('source_system', 255).value
        self.source_component = self.declare_parameter('source_component', 191).value
        for value in (self.target_system, self.target_component,
                      self.source_system, self.source_component):
            if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 255:
                raise ValueError('Vehicle system and component IDs must be integers in 1..255')
        telemetry_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.mode_pub = self._publisher(OffboardControlMode, 'in/offboard_control_mode')
        self.setpoint_pub = self._publisher(TrajectorySetpoint, 'in/trajectory_setpoint')
        self.command_pub = self._publisher(VehicleCommand, 'in/vehicle_command')
        self.input_subscriptions = [
            self.create_subscription(message_type,
                                     topic_name(self.prefix, name, message_type),
                                     callback, telemetry_qos)
            for message_type, name, callback in (
                (VehicleStatus, 'out/vehicle_status', self._status),
                (VehicleLocalPosition, 'out/vehicle_local_position', self._position),
                (VehicleLandDetected, 'out/vehicle_land_detected', self._landed),
                (VehicleCommandAck, 'out/vehicle_command_ack', self._ack),
            )
        ]
        trajectory_topic = self.declare_parameter(
            'trajectory_topic', '/larper/trajectory_in').value
        self.input_subscriptions.append(self.create_subscription(
            LarperTrajectorySetpoint, trajectory_topic, self._trajectory, telemetry_qos))
        self.state_pub = self.create_publisher(String, '~/state', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        lower = self.declare_parameter('map_boundary_lower', [0.0, 0.0, 0.0]).value
        upper = self.declare_parameter('map_boundary_upper', [0.0, 0.0, 0.0]).value
        enforce = self.declare_parameter('enforce_boundary', False).value
        heading = self.declare_parameter('map_heading_ned', 0.0).value
        origin = self.declare_parameter('map_origin_ned', [0.0, 0.0, 0.0]).value
        padding = self.declare_parameter('geofence_padding_m', .5 if hardware else 0.).value
        self.envelope = Envelope(lower, upper, heading, origin, padding=padding) if enforce else None
        self.require_autonomy_ready = self.declare_parameter('require_autonomy_ready', False).value
        self.require_preplanned_route = self.declare_parameter('require_preplanned_route', False).value
        self.route_ready = False
        self.input_subscriptions.append(self.create_subscription(
            Bool, '/autonomy/route_ready', lambda msg: setattr(self, 'route_ready', msg.data), telemetry_qos))
        self.autonomy_ready = False
        self.autonomy_ready_at = -float('inf')
        self.input_subscriptions.append(self.create_subscription(
            Bool, '/autonomy/takeoff_ready', self._autonomy_ready, telemetry_qos))
        self.auto_alignment = self.declare_parameter('align_px4_to_map_on_ground', False).value
        self.ground_alignment = GroundAlignment()
        self.alignment_reset_counters = None
        self.map_pose = None
        self.map_pose_at = -float('inf')
        self.vio_ready = False
        self.vio_ready_at = -float('inf')
        self.alignment_pub = self.create_publisher(PoseStamped, '/autonomy/map_to_px4',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.auto_alignment:
            self.input_subscriptions.append(self.create_subscription(
                Odometry, '/autonomy/odometry', self._map_pose, telemetry_qos))
            self.input_subscriptions.append(self.create_subscription(
                Bool, '/autonomy/vio_healthy', self._vio_ready, telemetry_qos))
        self.abort_service = self.create_service(Trigger, '~/hold_and_land', self._hold_and_land)
        self.last_report = None
        self.timer = self.create_timer(
            0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            ('SITL: ' if simulation else 'HARDWARE ENABLED: ') +
            f'Pilot arming then {trigger} switch authorizes automatic takeoff. '
            'simulation_only is operator acknowledgement, not vehicle identification.')
        self._report()

    def _hold_and_land(self, request, response):
        response.success, response.message = self.state_machine.request_hold_land(monotonic())
        self.get_logger().warning(response.message)
        self._report()
        return response

    def _map_pose(self, msg):
        if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not self.skybeta_compat and not -.005 <= self.get_clock().now().nanoseconds*1e-9-stamp < .3:
            return
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        values = (p.x,p.y,p.z,q.w,q.x,q.y,q.z,v.x,v.y,v.z)
        if not all(isfinite(x) for x in values) or abs(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z-1.) > .002:
            return
        self.map_pose = ((p.x,p.y,p.z), atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                         hypot(v.x,v.y,v.z))
        self.map_pose_at = monotonic()

    def _vio_ready(self, msg):
        self.vio_ready, self.vio_ready_at = msg.data, monotonic()

    def _align_on_ground(self, now):
        c = self.state_machine
        if self.ground_alignment.value is not None:
            p, status, landed = c.position, c.status, c.landed
            # Only position/heading resets change the coordinate transform.
            # Keep the VIO/map origin fixed; refresh the map-to-PX4 relation.
            changed = (p is not None and self.alignment_reset_counters is not None
                       and p.reset_counters[:3] != self.alignment_reset_counters)
            grounded = (status is not None and landed is not None
                        and 0 <= now-status.received_at < 1.
                        and 0 <= now-landed.received_at < LANDED_TIMEOUT
                        and not status.armed and not status.offboard and landed.landed)
            if not (changed and grounded and c.state in (State.WAITING, State.TAKEOFF)):
                return True
            self.ground_alignment = GroundAlignment()
            c.state = State.WAITING
            c.takeoff = c.hold_target = None
            c.tracking.clear()
            c.active_trajectory_id = None
            c.started_at = now
            self.get_logger().info('PX4 ground frame reset: refreshing map alignment and takeoff origin')
        if c.state == State.ERROR:
            return False
        def fresh(msg):
            # Land detector may publish at 1 Hz; do not reset the one-second
            # alignment window between normal landed updates.
            limit = LANDED_TIMEOUT if isinstance(msg, Landed) else 1.0 if isinstance(msg, Status) else .3
            return msg is not None and 0 <= now-msg.received_at < limit
        if fresh(c.landed) and not c.landed.landed:
            c.fail('Refusing airborne ground-frame alignment')
            self._report()
            return False
        if fresh(c.status) and (c.status.armed or c.status.offboard):
            self.ground_alignment.reset_pending()
            self.get_logger().warning(
                'Waiting for disarm and exit from Offboard before ground-frame alignment; no commands sent',
                throttle_duration_sec=3.0)
            return False
        good = (fresh(c.status) and fresh(c.position) and fresh(c.landed) and
                c.status.preflight_ok and not c.status.failsafe and c.landed.landed and
                c.position.usable() and c.position.velocity_valid and
                hypot(c.position.vx,c.position.vy,c.position.vz) < .1 and
                self.map_pose is not None and now-self.map_pose_at < .3 and
                self.map_pose[2] < .1 and self.vio_ready and now-self.vio_ready_at < .3)
        if self.skybeta_compat:
            good = (c.status is not None and c.landed is not None and c.landed.landed
                    and c.position is not None and self.map_pose is not None
                    and all(isfinite(v) for v in (c.position.x,c.position.y,c.position.z,c.position.yaw)))
        if not good:
            self.ground_alignment.reset_pending()
            return False
        p = c.position
        alignment = self.ground_alignment.observe(now,self.map_pose[0],self.map_pose[1],
                                                  (p.x,p.y,p.z),p.yaw)
        if self.skybeta_compat:
            from .alignment import from_pair
            alignment = from_pair(self.map_pose[0],self.map_pose[1],(p.x,p.y,p.z),p.yaw)
            self.ground_alignment.value = alignment
        if alignment is None:
            return False
        if self.envelope is not None:
            self.envelope.origin, self.envelope.heading = alignment.origin, alignment.heading
            if not self.envelope.contains((p.x,p.y,p.z), ground_contact=True, ground_tolerance=.5):
                c.fail('Aligned start position violates body geofence')
                self._report()
                return False
        msg = PoseStamped()
        msg.header.frame_id = 'local_ned'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = alignment.origin
        msg.pose.orientation.w = cos(alignment.heading/2)
        msg.pose.orientation.z = sin(alignment.heading/2)
        self.alignment_reset_counters = p.reset_counters[:3]
        self.alignment_pub.publish(msg)
        self.get_logger().info(f'Map/PX4 ground alignment frozen: origin={alignment.origin}, heading={alignment.heading:.4f}')
        return True

    def _autonomy_ready(self, msg):
        self.autonomy_ready = msg.data
        self.autonomy_ready_at = monotonic()

    def _publisher(self, message_type, name):
        return self.create_publisher(
            message_type, topic_name(self.prefix, name, message_type), 10)

    def _status(self, msg):
        if (msg.system_id != self.target_system
                or msg.component_id != self.target_component):
            self.get_logger().warning('Ignoring telemetry for another vehicle', throttle_duration_sec=5.)
            return
        self.state_machine.status = Status(
            monotonic(), msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            msg.pre_flight_checks_pass, msg.failsafe,
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL)

    def _position(self, msg):
        self.state_machine.position = Position(
            monotonic(), msg.x, msg.y, msg.z, msg.heading, msg.vz,
            msg.xy_valid and msg.z_valid and msg.v_z_valid,
            (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter,
             msg.vxy_reset_counter, msg.vz_reset_counter),
            msg.vx, msg.vy, msg.v_xy_valid and msg.v_z_valid)

    def _landed(self, msg):
        self.state_machine.landed = Landed(monotonic(), msg.landed)

    def _ack(self, msg):
        if (msg.target_system == self.source_system
                and msg.target_component == self.source_component):
            self.state_machine.acknowledge(msg.command, msg.result)

    def _trajectory(self, msg):
        if msg.header.frame_id != 'local_ned':
            self.get_logger().warning(
                'Ignoring trajectory: frame_id must be local_ned', throttle_duration_sec=1.0)
            return
        vectors = tuple(tuple(getattr(v, axis) for axis in ('x', 'y', 'z'))
                        for v in (msg.position, msg.velocity, msg.acceleration))
        yaw = (msg.yaw, msg.yawspeed) if msg.yaw_valid else (None, None)
        received = monotonic()
        # Certificate freshness is handled by the planner bridge. The executive
        # uses its receipt timeout rather than rejecting the upstream lifetime.
        trajectory = Trajectory(received, *vectors, *yaw,
                                msg.trajectory_id, msg.is_final, None)
        self.state_machine.accept_trajectory(trajectory)

    def _report(self):
        c = self.state_machine
        report = (c.state, c.phase, c.reason)
        if report != self.last_report:
            self.last_report = report
            self.state_pub.publish(String(data=c.state.value))
            detail = f' phase={c.phase.value}' if c.phase else ''
            text = f'{c.state.value}{detail}'
            if c.reason:
                self.get_logger().error(f'{text}: {c.reason}')
            else:
                self.get_logger().info(text)

    def _tick(self):
        now = monotonic()
        c = self.state_machine
        if self.auto_alignment and not self._align_on_ground(now):
            return
        if self.auto_alignment and c.state == State.WAITING and c.status is not None and (
                c.status.armed or c.status.offboard):
            self.get_logger().warning(
                'Waiting for disarm before mission readiness; no commands sent',
                throttle_duration_sec=3.0)
            return
        if getattr(self, 'require_preplanned_route', False) and c.state == State.WAITING and not self.route_ready:
            c.started_at = now
            c.last_tick = now
            self.get_logger().info('Waiting for cached route before takeoff', throttle_duration_sec=3.0)
            return
        waiting = c.state == State.WAITING or (c.state == State.TAKEOFF and c.phase != Phase.ASCEND)
        if not self.skybeta_compat and self.require_autonomy_ready and waiting and not (
                self.autonomy_ready and 0 <= now-self.autonomy_ready_at < .3):
            # No proof-of-life until the map/VIO/launch column is ready. A lost
            # gate invalidates the prior warmup; the pilot must switch again.
            if c.status is not None and c.status.offboard:
                c.fail('Pilot selected Offboard without fresh autonomy readiness')
                self._report()
            else:
                c.started_at = now
                c.last_tick = now
                if c.takeoff is not None:
                    c.takeoff.heartbeat_started = now
            return
        output = c.tick(now)
        if self.envelope is not None and c.state not in (State.WAITING, State.ERROR):
            ground = c.state in (State.WAITING, State.TAKEOFF, State.LAND, State.DONE)
            # Operator-requested floor tolerance only while PX4 reports ground contact.
            tolerance = .5 if c._fresh(c.landed, now) and c.landed.landed else 0.
            p = c.position
            points = [] if p is None else [(p.x, p.y, p.z)]
            if output.setpoint is not None:
                p = output.setpoint
                points.append((p.x, p.y, p.z))
            if any(not self.envelope.contains(p, ground, ground_tolerance=tolerance) for p in points):
                c.fail('Measured position or command violates body geofence')
                self._report()
                return
        self._report()
        timestamp = self.get_clock().now().nanoseconds // 1000
        if output.setpoint is not None:
            mode = OffboardControlMode()
            mode.timestamp = timestamp
            mode.position = True
            self.mode_pub.publish(mode)
            target = output.setpoint
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = timestamp
            setpoint.position = [px4_float(v) for v in (target.x, target.y, target.z)]
            setpoint.velocity = [px4_float(v) for v in target.velocity]
            setpoint.acceleration = [px4_float(v) for v in target.acceleration]
            setpoint.jerk = [nan, nan, nan]
            setpoint.yaw = px4_float(target.yaw)
            setpoint.yawspeed = px4_float(target.yawspeed)
            self.setpoint_pub.publish(setpoint)
        if output.command is not None:
            command = VehicleCommand()
            command.timestamp = timestamp
            command.command = output.command.value
            if output.command == Command.LAND:
                command.param1 = command.param2 = 0.0
                command.param4 = command.param5 = command.param6 = command.param7 = nan
            else:
                command.param1 = 1.0
                command.param2 = 6.0
            command.target_system = self.target_system
            command.target_component = self.target_component
            command.source_system = self.source_system
            command.source_component = self.source_component
            command.from_external = True
            self.command_pub.publish(command)
            self.get_logger().info(f'Requested {output.command.name}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightExecutiveNode()
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
