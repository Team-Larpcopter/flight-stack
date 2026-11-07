"""Exercise observable publications and state transitions without ROS."""

from dataclasses import replace
from math import nan

import pytest

from flight_executive.controller import (
    Command, Controller, Landed, Output, Phase, Position, State, Status,
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
    age = 2.51 if snapshot == 'landed' else 1.01
    setattr(f.c, snapshot, replace(getattr(f.c, snapshot), received_at=f.now - age))
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


def tracking_flight(**kwargs):
    from flight_executive.trajectory import Trajectory, Waypoint
    trajectory = Trajectory(
        [Waypoint(0, 0, 1.0), Waypoint(1.0, 0, 1.0), Waypoint(1.0, 1.0, 1.0)],
        0.3, 1.0, 1.0)
    f = Flight()
    f.c = Controller(0.0, 1.0, trajectory, **kwargs)
    f.ascend()
    f.position = replace(f.position, z=6.0)
    for _ in range(21):
        output = f.tick()
    assert f.c.state == State.TRACKING
    return f, trajectory, output


def follow(f, output):
    """Make telemetry track the published setpoint exactly."""
    sp = output.setpoint
    f.position = replace(f.position, x=sp.x, y=sp.y, z=sp.z, vz=sp.velocity[2])


def test_tracking_maps_east_north_up_into_ned_and_lands():
    f, trajectory, output = tracking_flight()
    assert (output.setpoint.x, output.setpoint.y, output.setpoint.z) == pytest.approx(
        (3.0, -2.0, 6.0))
    middle = None
    while f.c.state == State.TRACKING:
        follow(f, output)
        output = f.tick()
        if middle is None and f.now - f.c.tracking_started > 1.0:
            middle = output.setpoint  # on the first, eastbound leg
    # East (+x course) is +y NED; north is +x NED; up is -z NED.
    assert middle.velocity[1] > 0.3
    assert middle.velocity[0] == pytest.approx(0.0, abs=1e-6)
    assert middle.velocity[2] == pytest.approx(0.0, abs=1e-6)
    assert middle.y > -2.0 and middle.x == pytest.approx(3.0, abs=1e-6)
    assert middle.yaw == 0.8
    assert f.c.state == State.LAND
    assert output.command == Command.LAND
    assert (output.setpoint.x, output.setpoint.y, output.setpoint.z) == pytest.approx(
        (4.0, -1.0, 6.0), abs=1e-6)
    f.status = replace(f.status, offboard=False, landing=True)
    assert f.tick().setpoint is None
    assert f.c.land_confirmed
    f.status = replace(f.status, armed=False)
    f.landed = replace(f.landed, landed=True)
    assert f.tick().setpoint is None
    assert f.c.state == State.DONE
    assert f.tick() == Output()


def test_tracking_error_aborts_without_commands():
    f, _, output = tracking_flight(max_tracking_error=0.3)
    f.position = replace(f.position, x=output.setpoint.x + 0.31)
    assert f.tick() == Output()
    assert f.c.state == State.ERROR
    assert 'tracking error' in f.c.reason


def test_leaving_offboard_while_tracking_aborts():
    f, _, output = tracking_flight()
    follow(f, output)
    f.status = replace(f.status, offboard=False)
    assert f.tick() == Output()
    assert f.c.state == State.ERROR


def test_land_rejection_aborts():
    f, _, output = tracking_flight()
    while f.c.state == State.TRACKING:
        follow(f, output)
        output = f.tick()
    f.c.acknowledge(Command.LAND.value, 4)
    assert f.c.state == State.ERROR


def test_trajectory_must_start_at_hold_point():
    from flight_executive.trajectory import Trajectory, Waypoint
    trajectory = Trajectory([Waypoint(0, 0, 0.5), Waypoint(1, 0, 0.5)], 0.3, 1, 1)
    with pytest.raises(ValueError):
        Controller(0.0, 1.0, trajectory)


def test_landed_telemetry_tolerates_px4_one_hertz_rate():
    f = Flight()
    f.hold()
    stamp = f.now
    for _ in range(30):  # 1.5 s without a land-detector update
        f.now = round(f.now + 0.05, 6)
        f.c.status = replace(f.status, received_at=f.now)
        f.c.position = replace(f.position, received_at=f.now)
        f.c.landed = replace(f.landed, received_at=stamp)
        f.c.tick(f.now)
    assert f.c.state == State.HOLD
    f.now = stamp + 2.6
    f.c.status = replace(f.status, received_at=f.now)
    f.c.position = replace(f.position, received_at=f.now)
    f.c.tick(f.now)
    assert f.c.state == State.ERROR


def tangent_trajectory():
    from flight_executive.trajectory import Trajectory, Waypoint
    return Trajectory([Waypoint(0, 0, 1.0), Waypoint(1.0, 0, 1.0), Waypoint(1.0, 1.0, 1.0)],
                      0.3, 1.0, 1.0, yaw_mode='tangent')


def test_planned_yaw_is_published_as_ned_heading_and_yawspeed():
    from math import pi
    f = Flight()
    f.position = replace(f.position, yaw=pi / 2)  # facing east, the first leg
    f.c = Controller(0.0, 1.0, tangent_trajectory())
    f.ascend()
    f.position = replace(f.position, z=6.0)
    for _ in range(21):
        output = f.tick()
    assert f.c.state == State.TRACKING
    assert output.setpoint.yaw == pytest.approx(pi / 2)
    rates = []
    while f.c.state == State.TRACKING:
        follow(f, output)
        output = f.tick()
        if output.setpoint is not None:
            rates.append(output.setpoint.yawspeed)
    # A left turn in ENU (east to north) is a negative NED yawspeed.
    assert min(rates) < -0.3 and max(rates) <= 1e-9
    assert f.c.target.yaw == pytest.approx(0.0, abs=1e-6)


def test_planned_yaw_refuses_misaligned_vehicle_before_arming():
    f = Flight()  # heading 0.8 rad; the trajectory starts facing east (pi / 2)
    f.c = Controller(0.0, 1.0, tangent_trajectory())
    assert f.tick() == Output()
    assert f.c.state == State.ERROR
    assert 'heading' in f.c.reason
