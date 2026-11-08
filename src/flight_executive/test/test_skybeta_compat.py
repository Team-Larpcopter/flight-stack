"""Runtime compatibility keeps heartbeat output through diagnostic failures."""
from dataclasses import replace
import pytest
from test_state_machine import Flight, trajectory
from flight_executive.model import Command, State
from flight_executive.state_machine import StateMachine


def flight():
    f = Flight()
    f.c = StateMachine(0., skybeta_compat=True)
    return f


@pytest.mark.parametrize('fault', ['invalid', 'failsafe', 'stale', 'timer_gap'])
def test_diagnostics_do_not_stop_active_publications(fault):
    f = flight(); f.ascend()
    if fault == 'invalid': f.position = replace(f.position, valid=False)
    if fault == 'failsafe': f.status = replace(f.status, failsafe=True)
    if fault == 'stale': out = f.c.tick(f.now+5.)
    else: out = f.tick(1. if fault == 'timer_gap' else .05)
    assert out.setpoint is not None
    assert f.c.state != State.ERROR


def test_preflight_loss_and_short_warmup_do_not_abort_pilot_trigger():
    f=flight(); f.tick()
    f.status=replace(f.status, preflight_ok=False)
    f.tick()
    f.status=replace(f.status, armed=True)
    f.tick()
    f.status=replace(f.status, in_position=True)
    assert f.tick().command == Command.OFFBOARD


def test_takeoff_has_no_deadline_and_position_samples_do_not_expire():
    f=flight();f.ascend()
    assert f.tick(25.).setpoint is not None
    f.position=replace(f.position,z=6.)
    f.tick(); assert f.c.state==State.HOLD
    f.c.accept_trajectory(trajectory(f.now))
    target=f.tick().setpoint
    out=f.tick(5.)
    assert out.setpoint == target
    assert f.c.state==State.TRACKING


def test_pilot_mode_exit_does_not_request_offboard_again():
    f=flight();f.ascend()
    f.status=replace(f.status,offboard=False)
    out=f.tick()
    assert out.setpoint is not None and out.command is None
    assert f.c.state!=State.ERROR
