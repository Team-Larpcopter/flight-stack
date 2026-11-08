"""Verify exact legs, launch-heading mapping, and the executive landing handoff."""

from dataclasses import replace
from math import pi

import pytest

from flight_trajectory.course_follower import Phase, Pose
from flight_trajectory.triangle_course import TriangleCourse, TriangleFollower


def test_triangle_stops_at_each_vertex_and_returns_home():
    course = TriangleCourse()
    elapsed = 0.
    for leg, target in zip(course.legs, [( .75, 0., .5), (.75, .75, .5), (0., 0., .5)]):
        elapsed += leg.duration
        sample = course.sample(elapsed)
        assert sample.position == pytest.approx(target)
        assert sample.velocity == pytest.approx((0., 0., 0.), abs=1e-9)
        assert sample.acceleration == pytest.approx((0., 0., 0.), abs=1e-9)
    assert elapsed == course.duration


@pytest.mark.parametrize('heading,first,second', [
    (0., (3.75, -2., 6.5), (3.75, -2.75, 6.5)),
    (pi / 2, (3., -1.25, 6.5), (3.75, -1.25, 6.5)),
])
def test_forward_left_are_relative_to_initial_heading(heading, first, second):
    course = TriangleCourse()
    follower = TriangleFollower(course)
    follower.update_pose(Pose(0., 3., -2., 7., heading), True)
    for elapsed, target in [(course.legs[0].duration, first),
                            (sum(x.duration for x in course.legs[:2]), second),
                            (course.duration, (3., -2., 6.5))]:
        sample = follower.local(course.sample(elapsed))
        assert sample.position == pytest.approx(target)
        assert sample.yaw == heading


def test_streams_until_executive_lands_and_stops_on_stale_pose():
    follower = TriangleFollower(TriangleCourse())
    ground = Pose(0., 3., -2., 7., .8)
    follower.update_pose(ground, True)
    assert follower.tick(0.) is None
    follower.update_pose(replace(ground, z=6.5), False)
    follower.update_executive_state('HOLD')
    sample = follower.tick(0.)
    assert sample is not None
    t = 0.
    while t < follower.trajectory.duration + 1.:
        t += .02
        follower.update_pose(Pose(t, *sample.position, .8), False)
        follower.update_executive_state('TRACKING')
        sample = follower.tick(t)
        assert sample is not None
    assert sample.is_final
    assert sample.position == pytest.approx((3., -2., 6.5))
    follower.update_executive_state('LAND')
    assert follower.tick(t) is None
    assert follower.phase == Phase.FINISHED

    stale = TriangleFollower(TriangleCourse())
    stale.update_pose(ground, True)
    stale.update_executive_state('HOLD')
    assert stale.tick(1.) is None
    assert stale.phase == Phase.ABORTED
