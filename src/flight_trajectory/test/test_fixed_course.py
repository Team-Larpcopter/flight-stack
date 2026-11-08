from dataclasses import replace
from math import cos, sin, pi
import pytest
from flight_trajectory.fixed_course import FixedCourse, FixedCourseFollower, ROUTE_XY
from flight_trajectory.course_follower import Pose


def test_route_height_window_alignment_and_terminal_stop():
    course = FixedCourse()
    assert course.waypoints[0] == (0., 0., 1.0)
    assert course.waypoints[-1] == (-2.82128, -.77724, 1.0)
    assert ROUTE_XY[3][0] == ROUTE_XY[4][0] == pytest.approx(-3.3863)
    final = course.sample(course.duration+10.)
    assert final.position == pytest.approx(course.waypoints[-1])
    assert final.velocity == pytest.approx((0., 0., 0.))
    assert final.acceleration == pytest.approx((0., 0., 0.))
    assert final.yaw_rate == 0.
    elapsed = 0.
    for kind, duration, value, heading, delta in course.segments:
        if kind == 'turn':
            for i in range(101):
                r = course.sample(elapsed+duration*i/100)
                assert r.position == pytest.approx(value)
                assert r.velocity == pytest.approx((0., 0., 0.))
                assert abs(r.yaw_rate) <= .30001
        elapsed += duration


def test_full_route_streams_in_rotated_start_frame_then_finishes():
    follower = FixedCourseFollower(FixedCourse())
    ground = Pose(0., 3., -2., 7., .8)
    follower.update_pose(ground, True)
    follower.update_pose(replace(ground, z=6.0), False)
    follower.update_executive_state('HOLD')
    sample = follower.tick(0.)
    assert sample.position == pytest.approx((3., -2., 6.0))
    t = 0.
    while t < follower.trajectory.duration+1.:
        t += .02
        follower.update_pose(Pose(t, *sample.position, sample.yaw), False)
        sample = follower.tick(t)
        assert sample is not None
    assert sample.is_final
    x,y = ROUTE_XY[-1]
    assert sample.position == pytest.approx((3.+cos(.8)*x+sin(.8)*y,
                                             -2.+sin(.8)*x-cos(.8)*y,6.0))
    follower.update_executive_state('LAND')
    assert follower.tick(t+.02) is None


def test_runtime_compat_ignores_start_heading_age_and_tracking_error():
    follower = FixedCourseFollower(FixedCourse())
    follower.skybeta_compat = True
    follower.update_pose(Pose(0.,0.,0.,0.,0.), True)
    follower.update_pose(Pose(0.,3.,4.,-1.,1.,False), False)
    follower.update_executive_state('HOLD')
    assert follower.tick(10.) is not None
    assert follower.tick(11.) is not None


def test_yaml_xyz_changes_are_loaded_on_each_start(tmp_path):
    from flight_trajectory.fixed_course import load_waypoints
    p = tmp_path / 'fixed_course.yaml'
    p.write_text('waypoints: [[0, 0, 0.8], [1, -2, 1.2], [1, -2, 0.9]]')
    course = FixedCourse(load_waypoints(p))
    assert course.sample(course.duration + 1).position == pytest.approx((1, -2, .9))
    p.write_text('waypoints: [[0.2, 0.1, 1.1], [2, -1, 1.3]]')
    changed = FixedCourse(load_waypoints(p))
    assert changed.waypoints[0] == (0., 0., 1.1)
    assert changed.sample(changed.duration + 1).position == pytest.approx((2, -1, 1.3))


@pytest.mark.parametrize('points', [[], [[0,0,1]], [[0,0,1],[1,2]],
    [[0,0,1],[1,2,float('nan')]], [[0,0,0],[1,2,1]], [[0,0,1],[0,0,1]]])
def test_bad_waypoints_fail_before_flight(points):
    with pytest.raises(ValueError):
        FixedCourse(points)


def test_new_start_first_leg_keeps_launch_heading_then_resumes_tangent():
    course = FixedCourse([(0.,0.,1.), (0.,1.2,1.), (4.8261,1.2,1.),
                          (4.8261,-3.3863,1.)], [0., None, None])
    assert course.segments[0][0] == 'line'
    sample = course.sample(course.segments[0][1]/2)
    assert sample.yaw == pytest.approx(0.)
    assert sample.position[0] == pytest.approx(0.)
    assert 0 < sample.position[1] < 1.2
    assert course.segments[1][0] == 'line'
    assert course.segments[1][3] == pytest.approx(0.)
    assert course.segments[2][0] == 'turn'
    assert course.sample(course.duration).yaw == pytest.approx(-pi/2)
