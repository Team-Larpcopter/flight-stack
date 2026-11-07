"""Check adapter behavior against installed ROS and pinned PX4 message types."""

from math import isnan
from time import monotonic

import pytest
import rclpy
from px4_msgs.msg import VehicleCommandAck, VehicleLocalPosition, VehicleStatus

from flight_executive.controller import Command, Landed, Phase, State, Status
from flight_executive.flight_executive_node import FlightExecutiveNode, topic_name


@pytest.fixture
def node():
    rclpy.init(args=[
        '--ros-args', '-p', 'simulation_only:=true',
        '-p', 'px4_topic_prefix:=/executive_unit_test/fmu',
        '-r', '__ns:=/executive_unit_test',
    ])
    instance = FlightExecutiveNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_versioned_topic_names():
    assert topic_name('/fmu', 'out/vehicle_status', VehicleStatus).endswith('_v1')
    assert topic_name('/fmu', 'out/vehicle_command_ack', VehicleCommandAck).endswith(
        '/vehicle_command_ack')


def test_px4_ack_without_external_flag_is_processed(node):
    c = node.controller
    c.state = State.TAKEOFF
    c.phase = Phase.ARM
    c.attempts = 1
    ack = VehicleCommandAck()
    ack.command = Command.ARM.value
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
    ack.target_system = node.source_system
    ack.target_component = node.source_component
    assert not ack.from_external
    node._ack(ack)
    assert c.state == State.ERROR


def test_ack_for_another_controller_is_ignored(node):
    c = node.controller
    c.state = State.TAKEOFF
    c.phase = Phase.ARM
    c.attempts = 1
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
    node.controller.status = Status(monotonic(), False, False, True)
    node.controller.landed = Landed(monotonic(), True)
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
    assert len(node.controller.position.reset_counters) == 5


def test_vehicle_identity_mismatch_fails_closed(node):
    status = VehicleStatus()
    status.system_id = 42
    status.component_id = node.target_component
    node._status(status)
    assert node.controller.state == State.ERROR
