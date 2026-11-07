"""Check adapter behavior against installed ROS and pinned PX4 message types."""

from math import isnan
from time import monotonic

import pytest
import rclpy
from px4_msgs.msg import VehicleCommandAck, VehicleLocalPosition, VehicleStatus

from flight_executive.model import (
    Command, Landed, Phase, Position, Setpoint, State, Status,
)
from flight_executive.extensions.takeoff import Takeoff
from flight_executive_msgs.msg import TrajectorySetpoint as LarperTrajectorySetpoint
from flight_executive.flight_executive_node import FlightExecutiveNode, topic_name


@pytest.fixture
def node():
    rclpy.init(args=[
        '--ros-args', '-p', 'simulation_only:=true',
        '-p', 'px4_topic_prefix:=/executive_unit_test/fmu',
        '-p', 'trajectory_topic:=/executive_unit_test/larper/trajectory_in',
        '-r', '__ns:=/executive_unit_test',
    ])
    instance = FlightExecutiveNode()
    try:
        yield instance
    finally:
        try:
            instance.destroy_node()
        finally:
            rclpy.shutdown()


def test_versioned_topic_names():
    assert topic_name('/fmu', 'out/vehicle_status', VehicleStatus).endswith('_v1')
    assert topic_name('/fmu', 'out/vehicle_command_ack', VehicleCommandAck).endswith(
        '/vehicle_command_ack')


def test_px4_ack_without_external_flag_is_processed(node):
    c = node.state_machine
    c.state = State.TAKEOFF
    c.takeoff = Takeoff(Setpoint(0, 0, 0, 0), 1, monotonic())
    c.takeoff.phase = Phase.ARM
    c.takeoff.attempts = 1
    ack = VehicleCommandAck()
    ack.command = Command.ARM.value
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
    ack.target_system = node.source_system
    ack.target_component = node.source_component
    assert not ack.from_external
    node._ack(ack)
    assert c.state == State.ERROR


def test_ack_for_another_controller_is_ignored(node):
    c = node.state_machine
    c.state = State.TAKEOFF
    c.takeoff = Takeoff(Setpoint(0, 0, 0, 0), 1, monotonic())
    c.takeoff.phase = Phase.ARM
    c.takeoff.attempts = 1
    ack = VehicleCommandAck()
    ack.command = Command.ARM.value
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
    ack.target_system = 42
    ack.target_component = node.source_component
    node._ack(ack)
    assert c.state == State.TAKEOFF


class Publications:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_actual_messages_select_position_and_disable_unused_feedforward(node):
    position = VehicleLocalPosition()
    position.x, position.y, position.z = 3.0, -2.0, 7.0
    position.heading = 0.8
    position.xy_valid = True
    position.z_valid = True
    position.v_z_valid = True
    # PX4's magnetometer final alignment can complete only after takeoff.
    position.heading_good_for_control = False
    node._position(position)
    node.state_machine.status = Status(monotonic(), False, False, True)
    node.state_machine.landed = Landed(monotonic(), True)
    node.mode_pub = Publications()
    node.setpoint_pub = Publications()
    node.command_pub = Publications()
    node._tick()
    mode = node.mode_pub.messages[0]
    assert mode.position
    assert not mode.velocity and not mode.acceleration
    target = node.setpoint_pub.messages[0]
    assert list(target.position) == [3.0, -2.0, 7.0]
    assert all(isnan(v) for v in target.velocity)
    assert all(isnan(v) for v in target.acceleration)
    assert isnan(target.yawspeed)
    assert not node.command_pub.messages
    assert len(node.state_machine.position.reset_counters) == 5


def test_vehicle_identity_mismatch_fails_closed(node):
    status = VehicleStatus()
    status.system_id = 42
    status.component_id = node.target_component
    node._status(status)
    assert node.state_machine.state == State.ERROR


def input_trajectory():
    msg = LarperTrajectorySetpoint()
    msg.header.frame_id = 'local_ned'
    msg.trajectory_id = 'test-trajectory'
    msg.position.x, msg.position.y, msg.position.z = 3.0, -2.0, -1.0
    msg.velocity.x, msg.velocity.y, msg.velocity.z = .2, -.3, .4
    msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = .1, .2, -.1
    return msg


def test_tracking_input_preserves_px4_ned_axes(node):
    machine = node.state_machine
    machine.state = State.HOLD
    node._trajectory(input_trajectory())
    sample = machine.tracking.latest
    assert sample.position == (3, -2, -1)
    assert sample.velocity == (.2, -.3, .4)
    assert sample.acceleration == (.1, .2, -.1)


@pytest.mark.parametrize('invalid', ['frame', 'nan', 'inf', 'overflow'])
def test_invalid_trajectory_does_not_refresh_sample(node, invalid):
    machine = node.state_machine
    machine.state = State.HOLD
    node._trajectory(input_trajectory())
    previous = machine.tracking.latest
    msg = input_trajectory()
    if invalid == 'frame':
        msg.header.frame_id = 'map'
    else:
        msg.acceleration.z = {'nan': float('nan'), 'inf': float('inf'),
                              'overflow': 1e100}[invalid]
    node._trajectory(msg)
    assert machine.tracking.latest is previous


def test_tracking_input_during_takeoff_is_discarded(node):
    node.state_machine.state = State.TAKEOFF
    node._trajectory(input_trajectory())
    assert node.state_machine.tracking.latest is None


def test_tracking_publishes_full_nonzero_ned_vectors(node):
    now = monotonic()
    machine = node.state_machine
    machine.state = State.HOLD
    machine.status = Status(now, True, True, True)
    machine.position = Position(now, 1, 2, -1, .8, 0)
    machine.landed = Landed(now, False)
    machine.resets = (0, 0, 0)
    machine.hold_target = Setpoint(1, 2, -1, .8, (0, 0, 0), (0, 0, 0))
    node.mode_pub = Publications()
    node.setpoint_pub = Publications()
    node.command_pub = Publications()
    node._trajectory(input_trajectory())
    node._tick()
    assert machine.state == State.TRACKING
    target = node.setpoint_pub.messages[0]
    assert list(target.position) == [3, -2, -1]
    assert list(target.velocity) == pytest.approx([.2, -.3, .4])
    assert list(target.acceleration) == pytest.approx([.1, .2, -.1])
    assert target.yaw == pytest.approx(.8)
    assert node.mode_pub.messages[0].position
    assert not node.mode_pub.messages[0].velocity
    assert not node.command_pub.messages


def test_tracking_input_yaw_is_used_only_when_flagged(node):
    machine = node.state_machine
    machine.state = State.HOLD
    msg = input_trajectory()
    msg.yaw, msg.yawspeed = 1.2, -0.4
    node._trajectory(msg)
    assert machine.tracking.latest.yaw is None
    msg.yaw_valid = True
    node._trajectory(msg)
    assert (machine.tracking.latest.yaw, machine.tracking.latest.yawspeed) == pytest.approx(
        (1.2, -0.4))
    previous = machine.tracking.latest
    msg.yawspeed = float('nan')
    node._trajectory(msg)
    assert machine.tracking.latest is previous


def test_land_command_can_publish_without_an_offboard_setpoint(node):
    from flight_executive.model import Output

    node.state_machine.tick = lambda now: Output(command=Command.LAND)
    node.mode_pub = Publications()
    node.setpoint_pub = Publications()
    node.command_pub = Publications()
    node._tick()
    assert not node.mode_pub.messages and not node.setpoint_pub.messages
    command = node.command_pub.messages[0]
    assert command.command == 21
    assert command.param1 == 0 and command.param2 == 0
    assert all(isnan(v) for v in (command.param4, command.param5,
                                  command.param6, command.param7))
    assert command.from_external


@pytest.mark.parametrize('invalid', ['empty_id', 'moving_final', 'other_id'])
def test_invalid_completion_does_not_refresh_accepted_sample(node, invalid):
    node.state_machine.state = State.HOLD
    node._trajectory(input_trajectory())
    old = node.state_machine.tracking.latest
    msg = input_trajectory()
    if invalid == 'empty_id':
        msg.trajectory_id = ''
    elif invalid == 'other_id':
        msg.trajectory_id = 'previous-flight'
    else:
        msg.is_final = True
    node._trajectory(msg)
    assert node.state_machine.tracking.latest is old
