"""Check anchoring, start gating, frame mapping, and aborts without ROS."""

from dataclasses import replace
from math import pi

import pytest

from flight_trajectory.course_follower import CourseFollower, Phase, Pose
from flight_trajectory.trajectory import Trajectory, Waypoint

# Landed local NED pose with a nonzero estimator origin, facing east.
GROUND = Pose(0.0, 3.0, -2.0, 7.0, pi / 2)


def path(yaw_mode='tangent'):
    # East 1 m, then north 1 m, at 1 m above takeoff (ENU, launch-relative).
    return Trajectory([Waypoint(0, 0, 1.0), Waypoint(1.0, 0, 1.0), Waypoint(1.0, 1.0, 1.0)],
                      0.3, 1.0, 1.0, yaw_mode=yaw_mode)


class Run:
    def __init__(self, yaw_mode='tangent', **kwargs):
        self.f = CourseFollower(path(yaw_mode), **kwargs)
        self.now = 0.0
        self.pose = GROUND
        self.landed = True

    def tick(self, state=None, dt=0.02):
        self.now = round(self.now + dt, 6)
        if state is not None:
            self.f.update_executive_state(state)
        self.f.update_pose(replace(self.pose, received_at=self.now), self.landed)
        return self.f.tick(self.now)

    def take_off(self):
        assert self.tick('TAKEOFF') is None
        self.landed = False
        self.pose = replace(GROUND, z=6.0)  # 1 m up in NED
        assert self.tick('TAKEOFF') is None

    def follow(self, sample):
        x, y, z = sample.position
        self.pose = replace(self.pose, x=x, y=y, z=z)


def test_streams_course_from_landed_anchor_in_px4_ned():
    r = Run()
    r.take_off()
    start = r.tick('HOLD')
    assert r.f.phase == Phase.TRACKING
    assert start.position == pytest.approx((3.0, -2.0, 6.0))
    assert start.yaw == pytest.approx(pi / 2)  # east is a PX4 heading of +pi/2
    east_leg = None
    sample = start
    while r.f.phase == Phase.TRACKING:
        r.follow(sample)
        sample = r.tick('TRACKING')
        if east_leg is None and r.now - r.f.started_at > 1.0:
            east_leg = sample
    # Course east is NED +y; course north is NED +x; up is NED -z.
    assert east_leg.velocity[1] > 0.3
    assert east_leg.velocity[0] == pytest.approx(0.0, abs=1e-6)
    assert east_leg.velocity[2] == pytest.approx(0.0, abs=1e-6)
    assert sample.position == pytest.approx((4.0, -1.0, 6.0), abs=1e-6)
    assert sample.yaw == pytest.approx(0.0, abs=1e-6)  # finishes facing north
    # COMPLETE keeps streaming the stationary end point.
    final = r.tick('TRACKING')
    assert r.f.phase == Phase.COMPLETE
    assert final.velocity == pytest.approx((0, 0, 0), abs=1e-9)


def test_left_turn_is_negative_px4_yawspeed():
    r = Run()
    r.take_off()
    sample = r.tick('HOLD')
    rates = []
    while r.f.phase == Phase.TRACKING:
        r.follow(sample)
        sample = r.tick('TRACKING')
        rates.append(sample.yawspeed)
    assert min(rates) < -0.3 and max(rates) <= 1e-9


def test_fixed_yaw_leaves_heading_to_executive():
    r = Run('fixed')
    r.take_off()
    sample = r.tick('HOLD')
    assert sample.yaw is None and sample.yawspeed is None


def test_waits_for_hold_and_never_streams_before_it():
    r = Run()
    for state in ('WAITING', 'TAKEOFF'):
        assert r.tick(state) is None
    assert r.f.phase == Phase.WAITING


@pytest.mark.parametrize('fault,reason', [
    ('position', 'not at the trajectory start'),
    ('height', 'not at the trajectory start'),
    ('heading', 'heading'),
    ('stale', 'stale'),
])
def test_bad_start_aborts_without_streaming(fault, reason):
    r = Run()
    r.take_off()
    if fault == 'position':
        r.pose = replace(r.pose, x=r.pose.x + 0.31)
    elif fault == 'height':
        r.pose = replace(r.pose, z=r.pose.z - 0.31)
    elif fault == 'heading':
        r.pose = replace(r.pose, heading=0.0)
    if fault == 'stale':
        r.f.update_executive_state('HOLD')
        assert r.f.tick(r.now + 1.0) is None
    else:
        assert r.tick('HOLD') is None
    assert r.f.phase == Phase.ABORTED
    assert reason in r.f.reason
    assert r.tick('HOLD') is None


def test_missing_landed_anchor_aborts():
    r = Run()
    r.landed = False
    r.pose = replace(GROUND, z=6.0)
    assert r.tick('HOLD') is None
    assert 'anchor' in r.f.reason


def test_tracking_error_or_executive_error_stops_streaming():
    r = Run(max_tracking_error=0.3)
    r.take_off()
    sample = r.tick('HOLD')
    r.follow(sample)
    r.pose = replace(r.pose, y=r.pose.y + 0.31)
    assert r.tick('TRACKING') is None
    assert 'Tracking error' in r.f.reason
    r = Run()
    r.take_off()
    r.follow(r.tick('HOLD'))
    assert r.tick('ERROR') is None
    assert r.f.phase == Phase.ABORTED


def test_complete_marks_stationary_final_sample_until_land_handoff():
    r = Run('fixed')
    r.take_off()
    sample = r.tick('HOLD')
    assert not sample.is_final
    while r.f.phase == Phase.TRACKING:
        r.follow(sample)
        sample = r.tick('TRACKING')
    assert sample.is_final
    assert sample.velocity == pytest.approx((0, 0, 0), abs=1e-9)
    for _ in range(10):
        assert r.tick('TRACKING').is_final
    assert r.tick('LAND') is None
    assert r.f.phase == Phase.FINISHED
    assert r.tick('DONE') is None
    assert r.tick('HOLD') is None


def test_abort_does_not_produce_a_final_sample():
    r = Run('fixed')
    r.take_off()
    assert not r.tick('HOLD').is_final
    assert r.tick('ERROR') is None
    assert r.f.phase == Phase.ABORTED
