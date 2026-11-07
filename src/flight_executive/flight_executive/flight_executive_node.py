"""Simulation-only ROS 2 adapter for the flight state machine."""

from math import isfinite, nan
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
from std_msgs.msg import String

from flight_executive.model import Command, Landed, Position, Status, Trajectory
from flight_executive.state_machine import StateMachine


def topic_name(prefix, name, message_type):
    """Match PX4's DDS topic suffix for versioned messages."""
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/{name}{suffix}'


class FlightExecutiveNode(Node):
    """Translate PX4 messages and publish the state machine's outputs at 20 Hz."""

    def __init__(self):
        super().__init__('flight_executive')
        if not self.declare_parameter('simulation_only', False).value:
            raise ValueError('Use the simulation launch file; hardware is not supported')
        height = self.declare_parameter('takeoff_height', 1.0).value
        self.state_machine = StateMachine(monotonic(), height)
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
        self.last_report = None
        self.timer = self.create_timer(
            0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            'SITL ONLY: automatic arming and takeoff enabled. '
            'simulation_only is operator acknowledgement, not vehicle identification.')
        self._report()

    def _publisher(self, message_type, name):
        return self.create_publisher(
            message_type, topic_name(self.prefix, name, message_type), 10)

    def _status(self, msg):
        if (msg.system_id != self.target_system
                or msg.component_id != self.target_component):
            self.state_machine.fail('PX4 telemetry vehicle identity mismatch')
            return
        self.state_machine.status = Status(
            monotonic(), msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            msg.pre_flight_checks_pass, msg.failsafe)

    def _position(self, msg):
        self.state_machine.position = Position(
            monotonic(), msg.x, msg.y, msg.z, msg.heading, msg.vz,
            msg.xy_valid and msg.z_valid and msg.v_z_valid,
            (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter,
             msg.vxy_reset_counter, msg.vz_reset_counter))

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
        if not all(isfinite(v) and abs(v) <= 3.4028234663852886e38
                   for vector in vectors for v in vector):
            self.get_logger().warning(
                'Ignoring trajectory: invalid PX4 float32 component', throttle_duration_sec=1.0)
            return
        self.state_machine.accept_trajectory(Trajectory(monotonic(), *vectors))

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
        output = self.state_machine.tick(monotonic())
        self._report()
        if output.setpoint is None:
            return
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.mode_pub.publish(mode)
        target = output.setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [target.x, target.y, target.z]
        setpoint.velocity = list(target.velocity)
        setpoint.acceleration = list(target.acceleration)
        setpoint.jerk = [nan, nan, nan]
        setpoint.yaw = target.yaw
        setpoint.yawspeed = nan
        self.setpoint_pub.publish(setpoint)
        if output.command is not None:
            command = VehicleCommand()
            command.timestamp = timestamp
            command.command = output.command.value
            command.param1 = 1.0
            command.param2 = 6.0 if output.command == Command.OFFBOARD else 0.0
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
