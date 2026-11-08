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


def test_takeoff_and_command_stream_have_no_local_deadline():
    f=flight();f.ascend()
    assert f.tick(25.).setpoint is not None
    f.position=replace(f.position,z=6.)
    f.tick(); assert f.c.state==State.HOLD
    f.c.accept_trajectory(trajectory(f.now))
    target=f.tick().setpoint
    out=f.tick(5.)
    assert out.setpoint == target
    assert f.c.state==State.TRACKING
    f.c.accept_trajectory(trajectory(f.now))
    assert f.tick().setpoint.velocity == target.velocity
    assert f.c.state==State.TRACKING


def test_pilot_mode_exit_does_not_request_offboard_again():
    f=flight();f.ascend()
    f.status=replace(f.status,offboard=False)
    out=f.tick()
    assert out.setpoint is not None and out.command is None
    assert f.c.state!=State.ERROR


def test_stale_trajectory_with_unusable_position_holds_last_command():
    f=flight(); f.hold()
    f.c.accept_trajectory(trajectory(f.now))
    target=f.tick().setpoint
    f.position=replace(f.position,x=float('nan'),valid=False)
    out=f.tick(1.1)
    assert (out.setpoint.x,out.setpoint.y,out.setpoint.z)==(target.x,target.y,target.z)
    assert out.setpoint == target
    assert f.c.state==State.TRACKING


def test_compat_receipt_timeout_does_not_restore_certificate_expiry():
    f=flight(); f.hold()
    f.c.accept_trajectory(replace(trajectory(f.now),valid_until=f.now-10.))
    assert f.tick().setpoint.velocity==(.2,-.3,.4)
    assert f.c.state==State.TRACKING


def test_offboard_rejection_or_delayed_confirmation_does_not_latch_stream():
    from flight_executive.extensions.takeoff import Takeoff
    from flight_executive.model import Setpoint, Phase, Status, Position, Landed
    takeoff = Takeoff(Setpoint(0,0,0,0),1.,0.)
    takeoff.skybeta_compat=True
    takeoff.phase=Phase.MODE
    takeoff.attempts=1
    takeoff.acknowledge(Command.OFFBOARD.value,1)
    result=takeoff.tick(10.,Status(10.,True,False,True,in_position=True),
                        Position(10.,0,0,0,0,0),Landed(10.,True))
    assert not result.reason
    assert result.output.setpoint is not None
    assert result.output.command == Command.OFFBOARD
