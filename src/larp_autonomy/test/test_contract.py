import math
import pytest
from larp_autonomy.trajectory_bridge import local_ned


def test_map_heading_and_translation_are_not_confused_with_enu():
    assert local_ned((1,2,3),0,(10,20,30))==(11,18,27)
    assert local_ned((1,2,3),math.pi/2)==pytest.approx((2,1,-3))

from types import SimpleNamespace
from larp_autonomy.trajectory_bridge import MissionGate


def test_active_mission_waits_for_hold_and_stops_after_tracking():
    gate = MissionGate(True, 'flight-1')
    for phase in (None, 'WAITING', 'TAKEOFF', 'LAND', 'DONE', 'ERROR'):
        gate.phase = phase
        assert not gate.ready
    for phase in ('HOLD', 'TRACKING'):
        gate.phase = phase
        assert gate.ready


def test_replanning_and_goal_changes_preserve_executive_id():
    gate = MissionGate(True, 'flight-1')
    assert gate.accepts_epoch(1)
    assert gate.trajectory_id(SimpleNamespace(localization_epoch=1, goal_id=10)) == 'flight-1'
    assert gate.trajectory_id(SimpleNamespace(localization_epoch=1, goal_id=11)) == 'flight-1'
    assert not gate.accepts_epoch(2)
    assert not gate.accepts_epoch(1)  # Delayed old goal cannot clear a frame reset.
    gate.phase = 'TRACKING'
    assert not gate.ready


def test_shadow_can_profile_without_flight_executive():
    gate = MissionGate(False, '')
    assert gate.ready
    assert gate.accepts_epoch(1)
    assert gate.accepts_epoch(2)
    with pytest.raises(ValueError):
        MissionGate(True, '')


def test_extended_certificate_budget_preserves_capture_time_expiry():
    from larp_autonomy.polynomial import Trajectory
    M = SimpleNamespace
    stamp = lambda t: M(sec=int(t), nanosec=round((t-int(t))*1e9))
    message = M(VALID=1, status=1, header=M(frame_id='map', stamp=stamp(10.)),
                start_time=stamp(10.), localization_epoch=1, goal_id=1, sequence=1,
                position=[M(duration=1., degree=1, coefficients=[0.,1.,0.,2.,0.,3.])],
                yaw=[M(duration=1., degree=1, coefficients=[0.,0.])])
    with pytest.raises(ValueError):
        Trajectory.decode(message,10.4,1)
    assert Trajectory.decode(message,10.4,1,max_age=.6).certificate == 10.
    with pytest.raises(ValueError):
        Trajectory.decode(message,10.601,1,max_age=.6)


def test_completed_curve_holds_endpoint_with_zero_feedforward():
    from larp_autonomy.polynomial import Trajectory, Piece
    # Small accepted terminal derivative residuals must not move an endpoint hold.
    p=Piece(1., ((.01, 1.), (0., 2.), (0., 3.)))
    y=Piece(1., ((.01, 0.),))
    curve=Trajectory(10., 11., 1, 1, (p,), (y,))
    assert curve.sample(10.5)[1][0] == pytest.approx(.01)
    for stamp in (11., 1000.):
        position,velocity,acceleration,yaw,rate=curve.sample(stamp)
        assert position == pytest.approx((1.01,2.,3.))
        assert velocity == acceleration == (0.,0.,0.)
        assert yaw == pytest.approx(.01)
        assert rate == 0.
