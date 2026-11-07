"""Exercise observable publications and state transitions without ROS."""

from dataclasses import replace
from math import nan

import pytest

from flight_executive.controller import (
    Command, Controller, Landed, Phase, Position, State, Status,
)


class Flight:
    def __init__(self):
        self.now = 0.0
        self.c = Controller(self.now)
        self.status = Status(0.0, False, False, True)
        self.position = Position(0.0, 3.0, -2.0, 7.0, 0.8, 0.0)
        self.landed = Landed(0.0, True)

    def tick(self, dt=0.05):
        self.now = round(self.now + dt, 6)
        self.c.status = replace(self.status, received_at=self.now)
        self.c.position = replace(self.position, received_at=self.now)
        self.c.landed = replace(self.landed, received_at=self.now)
        return self.c.tick(self.now)

    def mode_request(self):
        self.tick()
        for _ in range(40):
            output = self.tick()
        assert output.command == Command.OFFBOARD
        return output

    def arm_request(self):
        self.mode_request()
        self.status = replace(self.status, offboard=True)
        output = self.tick()
        assert output.command == Command.ARM
        return output

    def ascend(self):
        self.arm_request()
        self.status = replace(self.status, armed=True)
        output = self.tick()
        assert output.setpoint.z == 6.0
        return output

    def hold(self):
        self.ascend()
        self.position = replace(self.position, z=6.0)
        for _ in range(21):
            self.tick()
        assert self.c.state == State.HOLD


def test_nonzero_origin_sequence_and_indefinite_hold():
    f = Flight()
    output = f.tick()
    assert f.c.state == State.TAKEOFF
    assert output.setpoint.x == 3.0
    assert output.setpoint.y == -2.0
    assert output.setpoint.z == 7.0
    assert output.setpoint.yaw == 0.8
    for _ in range(40):
        output = f.tick()
    assert output.command == Command.OFFBOARD
    f.c.acknowledge(Command.OFFBOARD.value, 0)
    assert f.tick().command is None
    assert f.c.phase == Phase.MODE
    f.status = replace(f.status, offboard=True)
    assert f.tick().command == Command.ARM
    f.c.acknowledge(Command.ARM.value, 0)
    assert f.tick().setpoint.z == 7.0
    f.status = replace(f.status, armed=True)
    target = f.tick().setpoint
    assert (target.x, target.y, target.z, target.yaw) == (3.0, -2.0, 6.0, 0.8)
    f.position = replace(f.position, z=6.0)
    for _ in range(21):
        f.tick()
    assert f.c.state == State.HOLD
    for _ in range(240):
        output = f.tick()
        assert output.setpoint == target
        assert output.command is None
    assert f.c.state == State.HOLD


@pytest.mark.parametrize('missing', ['status', 'position', 'landed'])
def test_missing_telemetry_times_out_without_commands(missing):
    f = Flight()
    f.tick()
    c = Controller(0.0)
    c.status = f.c.status
    c.position = f.c.position
    c.landed = f.c.landed
    setattr(c, missing, None)
    assert c.tick(0.1).setpoint is None
    assert c.state == State.WAITING
    assert c.tick(30.0).setpoint is None
    assert c.state == State.ERROR


@pytest.mark.parametrize('field,value', [
    ('valid', False), ('x', nan), ('yaw', nan), ('vz', nan),
])
def test_invalid_initial_position_waits(field, value):
    f = Flight()
    f.position = replace(f.position, **{field: value})
    assert f.tick().setpoint is None
    assert f.c.state == State.WAITING


def test_preflight_gate_and_startup_refusal():
    f = Flight()
    f.status = replace(f.status, preflight_ok=False)
    assert f.tick().setpoint is None
    f.status = replace(f.status, armed=True)
    assert f.tick().setpoint is None
    assert f.c.state == State.ERROR
    f = Flight()
    f.landed = replace(f.landed, landed=False)
    assert f.tick().setpoint is None
    assert f.c.state == State.ERROR


@pytest.mark.parametrize('height', [0, -1, nan, float('inf')])
def test_bad_height_is_rejected(height):
    with pytest.raises(ValueError):
        Controller(0, height)


def test_command_retries_are_bounded_and_accepted_ack_is_not_confirmation():
    f = Flight()
    f.mode_request()
    sent = 1
    for _ in range(60):
        f.c.acknowledge(Command.OFFBOARD.value, 0)
        if f.tick().command is not None:
            sent += 1
    assert sent == 3
    assert f.c.state == State.ERROR
    assert '3 attempts' in f.c.reason
    for _ in range(20):
        assert f.tick().setpoint is None


@pytest.mark.parametrize('result', [1, 2, 3, 4, 6])
def test_matching_ack_rejection_is_terminal(result):
    f = Flight()
    f.arm_request()
    f.c.acknowledge(Command.OFFBOARD.value, result)
    assert f.c.state == State.TAKEOFF
    f.c.acknowledge(Command.ARM.value, result)
    assert f.tick().setpoint is None
    assert f.c.state == State.ERROR


def test_unsolicited_ack_does_not_fail_waiting_or_hold():
    f = Flight()
    f.c.acknowledge(Command.ARM.value, 2)
    assert f.c.state == State.WAITING
    f.hold()
    f.c.acknowledge(Command.ARM.value, 2)
    assert f.tick().setpoint is not None
    assert f.c.state == State.HOLD


@pytest.mark.parametrize('failure', [
    'failsafe', 'disarm', 'mode_exit', 'invalid', 'reset_xy', 'reset_z', 'reset_yaw',
])
@pytest.mark.parametrize('holding', [False, True])
def test_active_failures_stop_all_publications_and_never_recover(failure, holding):
    f = Flight()
    if holding:
        f.hold()
    else:
        f.ascend()
    if failure == 'failsafe':
        f.status = replace(f.status, failsafe=True)
    elif failure == 'disarm':
        f.status = replace(f.status, armed=False)
    elif failure == 'mode_exit':
        f.status = replace(f.status, offboard=False)
    elif failure == 'invalid':
        f.position = replace(f.position, valid=False)
    else:
        counters = {'reset_xy': (1, 0, 0), 'reset_z': (0, 1, 0),
                    'reset_yaw': (0, 0, 1)}
        f.position = replace(f.position, reset_counters=counters[failure])
    output = f.tick()
    assert output.setpoint is None
    assert output.command is None
    assert f.c.state == State.ERROR
    f.status = Status(f.now, False, False, True)
    f.position = Position(f.now, 3, -2, 7, .8, 0)
    for _ in range(50):
        assert f.tick().setpoint is None
    assert f.c.state == State.ERROR


@pytest.mark.parametrize('snapshot', ['status', 'position', 'landed'])
def test_stale_active_telemetry_stops_publications(snapshot):
    f = Flight()
    f.ascend()
    setattr(f.c, snapshot, replace(getattr(f.c, snapshot), received_at=f.now - 1.01))
    assert f.c.tick(f.now).setpoint is None
    assert f.c.state == State.ERROR


def test_timer_gap_restarts_prepare_but_aborts_active_flight():
    f = Flight()
    f.tick()
    f.tick(0.6)
    for _ in range(39):
        assert f.tick().command is None
    assert f.tick().command == Command.OFFBOARD
    f = Flight()
    f.ascend()
    assert f.tick(0.6).setpoint is None
    assert f.c.state == State.ERROR


def test_settling_requires_position_speed_and_continuous_dwell():
    f = Flight()
    f.ascend()
    for position in (
            replace(f.position, z=6.0, vz=.3),
            replace(f.position, z=6.0, x=4.0),
            replace(f.position, z=6.3)):
        f.position = position
        for _ in range(30):
            f.tick()
        assert f.c.state == State.TAKEOFF
    f.position = Position(f.now, 3, -2, 6, .8, 0)
    for _ in range(15):
        f.tick()
    f.position = replace(f.position, vz=.3)
    f.tick()
    f.position = replace(f.position, vz=0)
    for _ in range(20):
        f.tick()
    assert f.c.state == State.TAKEOFF
    f.tick()
    assert f.c.state == State.HOLD


def test_ascent_timeout_stops_heartbeat():
    f = Flight()
    f.ascend()
    for _ in range(400):
        output = f.tick()
    assert f.c.state == State.ERROR
    assert output.setpoint is None
    assert 'ascent timed out' in f.c.reason


@pytest.mark.parametrize('phase', ['prepare', 'mode'])
@pytest.mark.parametrize('failure', ['preflight', 'landed'])
def test_lost_readiness_before_arm_is_terminal(phase, failure):
    f = Flight()
    if phase == 'mode':
        f.mode_request()
    else:
        f.tick()
    if failure == 'preflight':
        f.status = replace(f.status, preflight_ok=False)
    else:
        f.landed = replace(f.landed, landed=False)
    assert f.tick().setpoint is None
    assert f.c.state == State.ERROR


@pytest.mark.parametrize('counter', [3, 4])
def test_velocity_estimator_resets_are_terminal(counter):
    f = Flight()
    f.position = replace(f.position, reset_counters=(0, 0, 0, 0, 0))
    f.ascend()
    counters = [0, 0, 0, 0, 0]
    counters[counter] = 1
    f.position = replace(f.position, reset_counters=tuple(counters))
    assert f.tick().setpoint is None
    assert f.c.state == State.ERROR
