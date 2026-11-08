"""Check trajectory completion, HOLD expiry, and autonomous PX4 landing handoff."""

from dataclasses import replace

import pytest

from flight_executive.model import Command, Landed, Position, Setpoint, State, Status, Trajectory
from flight_executive.state_machine import StateMachine


class Rig:
    def __init__(self, **kwargs):
        self.now = 0.0
        self.machine = StateMachine(0.0, **kwargs)
        self.machine.state = State.HOLD
        self.machine.hold_started = 0.0
        self.machine.hold_target = Setpoint(3, -2, -1, .8, (0, 0, 0), (0, 0, 0))
        self.status = Status(0, True, True, True)
        self.position = Position(0, 3, -2, -1, .8, 0)
        self.landed = Landed(0, False)
        self.machine.resets = self.position.reset_counters

    def tick(self):
        self.now = round(self.now + .05, 6)
        self.machine.status = replace(self.status, received_at=self.now)
        self.machine.position = replace(self.position, received_at=self.now)
        self.machine.landed = replace(self.landed, received_at=self.now)
        return self.machine.tick(self.now)

    def sample(self, final=False, trajectory_id='run-a', position=(3, -2, -1), **kwargs):
        sample = Trajectory(self.now, position, (0, 0, 0), (0, 0, 0),
                            trajectory_id=trajectory_id, is_final=final, **kwargs)
        return self.machine.accept_trajectory(sample)

    def request_land(self):
        for _ in range(101):
            output = self.tick()
        assert self.machine.state == State.LAND
        assert output.command == Command.LAND
        return output


def test_default_hold_waits_five_seconds_then_requests_land():
    r = Rig()
    for _ in range(100):
        output = r.tick()
        assert r.machine.state == State.HOLD
        assert output.command is None
    output = r.tick()
    assert r.machine.state == State.LAND
    assert output.command == Command.LAND
    assert output.setpoint.velocity == (0, 0, 0)


def test_input_before_hold_deadline_starts_tracking():
    r = Rig()
    for _ in range(99):
        r.tick()
    r.sample()
    assert r.tick().command is None
    assert r.machine.state == State.TRACKING


def test_stale_tracking_has_one_second_repeat_then_five_second_hold():
    r = Rig()
    r.sample()
    for _ in range(20):
        r.tick()
        assert r.machine.state == State.TRACKING
    r.tick()
    assert r.machine.state == State.HOLD
    for _ in range(100):
        r.tick()
        assert r.machine.state == State.HOLD
    assert r.tick().command == Command.LAND


@pytest.mark.parametrize("wait_for_planner", [False, True])
def test_stationary_final_sample_requires_continuous_measured_settling(wait_for_planner):
    r = Rig(wait_for_planner=wait_for_planner)
    r.position = replace(r.position, vx=.3)
    for _ in range(25):
        r.sample(final=True)
        r.tick()
        assert r.machine.state == State.TRACKING
    r.position = replace(r.position, vx=0)
    for _ in range(20):
        r.sample(final=True)
        r.tick()
        assert r.machine.state == State.TRACKING
    r.sample(final=True)
    assert r.tick().command == Command.LAND


def test_nonfinal_and_changed_endpoint_reset_terminal_dwell():
    r = Rig()
    for _ in range(15):
        r.sample(final=True)
        r.tick()
    r.sample(final=False)
    r.tick()
    for _ in range(15):
        r.sample(final=True)
        r.tick()
    r.position = replace(r.position, x=3.1)
    for _ in range(20):
        r.sample(final=True, position=(3.1, -2, -1))
        r.tick()
        assert r.machine.state == State.TRACKING
    r.sample(final=True, position=(3.1, -2, -1))
    assert r.tick().command == Command.LAND


def test_other_trajectory_id_and_moving_final_do_not_refresh_input():
    r = Rig()
    r.sample()
    previous = r.machine.tracking.latest
    assert not r.sample(final=True, trajectory_id='old-run')
    moving = replace(previous, is_final=True, velocity=(1, 0, 0))
    assert not r.machine.accept_trajectory(moving)
    assert r.machine.tracking.latest is previous


def test_expired_final_sample_does_not_count_as_completion():
    r = Rig()
    r.position = replace(r.position, x=4)
    r.sample(final=True)
    for _ in range(21):
        r.tick()
    assert r.machine.state == State.HOLD
    assert r.machine.landing is None


def test_land_handoff_stops_offboard_until_landed_and_disarmed():
    r = Rig()
    r.request_land()
    r.status = replace(r.status, offboard=False, auto_land=True)
    assert r.tick().setpoint is None
    for _ in range(20):
        output = r.tick()
        assert output.setpoint is None and output.command is None
    r.position = replace(r.position, valid=False, reset_counters=(5, 6, 7))
    assert r.tick().setpoint is None
    assert r.machine.state == State.LAND
    r.status = replace(r.status, armed=False, auto_land=False)
    r.landed = replace(r.landed, landed=True)
    assert r.tick().setpoint is None
    assert r.machine.state == State.DONE
    r.sample()
    for _ in range(10):
        output = r.tick()
        assert output.setpoint is None and output.command is None
        assert r.machine.state == State.DONE


def test_touchdown_disarm_completes_without_observing_brief_land_mode():
    r = Rig()
    r.request_land()
    r.status = replace(r.status, armed=False, offboard=False)
    r.landed = replace(r.landed, landed=True)
    r.tick()
    assert r.machine.state == State.DONE


def test_land_mode_takeover_never_reacquires_control():
    r = Rig()
    r.request_land()
    r.status = replace(r.status, offboard=False, auto_land=True)
    r.tick()
    r.status = replace(r.status, auto_land=False)
    output = r.tick()
    assert r.machine.state == State.ERROR
    assert output.command is None and output.setpoint is None


def test_land_retries_are_bounded_and_accepted_ack_is_not_confirmation():
    r = Rig()
    r.request_land()
    commands = 1
    for _ in range(60):
        r.machine.acknowledge(Command.LAND.value, 0)
        commands += r.tick().command == Command.LAND
    assert commands == 3
    assert r.machine.state == State.ERROR


def test_land_rejection_and_deadline_are_terminal():
    r = Rig()
    r.request_land()
    r.machine.acknowledge(400, 2)
    assert r.machine.state == State.LAND
    r.machine.acknowledge(Command.LAND.value, 2)
    assert r.tick().setpoint is None
    assert r.machine.state == State.ERROR
    r = Rig(landing_timeout=.5)
    r.request_land()
    r.status = replace(r.status, offboard=False, auto_land=True)
    for _ in range(10):
        r.tick()
    assert r.machine.state == State.ERROR


@pytest.mark.parametrize('parameter', ['hold_timeout', 'landing_timeout'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_landing_deadlines_are_rejected(parameter, value):
    with pytest.raises(ValueError):
        StateMachine(0, **{parameter: value})


@pytest.mark.parametrize('confirmed', [False, True])
def test_failsafe_during_land_handoff_or_descent_stops_all_outputs(confirmed):
    r = Rig()
    r.request_land()
    if confirmed:
        r.status = replace(r.status, offboard=False, auto_land=True)
        r.tick()
    r.status = replace(r.status, failsafe=True)
    output = r.tick()
    assert r.machine.state == State.ERROR
    assert output.command is None and output.setpoint is None
    r.status = replace(r.status, failsafe=False)
    assert r.tick().command is None
    assert r.machine.state == State.ERROR


@pytest.mark.parametrize('snapshot,age', [('status', 1.1), ('landed', 2.6)])
def test_stale_landing_telemetry_stops_all_outputs(snapshot, age):
    r = Rig()
    r.request_land()
    setattr(r.machine, snapshot,
            replace(getattr(r.machine, snapshot), received_at=r.now - age))
    output = r.machine.tick(r.now)
    assert r.machine.state == State.ERROR
    assert output.command is None and output.setpoint is None
