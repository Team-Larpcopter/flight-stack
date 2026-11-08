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
