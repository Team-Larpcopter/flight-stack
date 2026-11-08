from dataclasses import replace
from flight_executive.state_machine import StateMachine
from flight_executive.model import Status, Position, Landed, State, Phase
from flight_executive.envelope import Envelope
import pytest


def feed(c,t,armed=False,offboard=False,position=True,landed=True):
    c.status=Status(t,armed,offboard,True,in_position=position)
    c.position=Position(t,0.,0.,0.,0.,0.)
    c.landed=Landed(t,landed)
    return c.tick(t)


def warmed():
    c=StateMachine(0.,takeoff_trigger='offboard')
    for i in range(45):
        out=feed(c,i*.05,armed=True)
        assert out.command is None
        assert out.setpoint.z==0.
    return c


def test_position_never_starts_and_offboard_switch_ascends_without_command():
    c=warmed()
    assert c.phase==Phase.WAIT_OFFBOARD
    out=feed(c,2.25,armed=True,offboard=True,position=False)
    assert c.phase==Phase.ASCEND
    assert out.setpoint.z==-1. and out.command is None
    feed(c,2.3,armed=True,offboard=False)
    assert c.state==State.ERROR  # pilot takeover never triggers automatic reentry


def test_existing_offboard_is_not_adopted():
    c=StateMachine(0.,takeoff_trigger='offboard')
    feed(c,0.,armed=True,offboard=True)
    feed(c,.05,armed=True,offboard=True)
    assert c.state==State.ERROR


def test_disarmed_offboard_and_short_warmup_are_rejected():
    c=warmed();feed(c,2.25,offboard=True)
    assert c.state==State.ERROR
    c=StateMachine(0.,takeoff_trigger='offboard')
    feed(c,0.);feed(c,.05,armed=True);feed(c,.1,armed=True,offboard=True)
    assert c.state==State.ERROR


def test_airborne_start_is_rejected():
    c=StateMachine(0.,takeoff_trigger='offboard')
    feed(c,0.,armed=True,landed=False)
    assert c.state==State.ERROR


def test_whole_body_ceiling_and_ground_contact():
    e=Envelope([-2.,-2.,0.],[2.,2.,1.8])
    assert e.contains((0.,0.,-1.4))
    assert not e.contains((0.,0.,-1.41))
    assert e.contains((0.,0.,0.),ground_contact=True)
    assert not e.contains((0.,0.,0.))
    assert not e.contains((1.7,0.,-.8),ground_contact=True)
    shifted=Envelope([-2.,-2.,0.],[2.,2.,1.8],origin=(1.,2.,3.))
    assert shifted.contains((1.,2.,2.))
    assert not shifted.contains((1.,2.,1.5))


def test_pilot_trigger_waits_without_startup_deadline_or_commands():
    c=StateMachine(0.,takeoff_trigger='offboard')
    out=c.tick(120.)
    assert c.state==State.WAITING
    assert out.command is None and out.setpoint is None
