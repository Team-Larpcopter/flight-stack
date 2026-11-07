"""Check the reference is feasible and the DroneHacks route clears the course."""

from math import atan2, dist, hypot, pi

import pytest

from flight_executive import dronehacks_course as course
from flight_executive.trajectory import Trajectory, Waypoint

# x500 rotor centres are 0.246 m from the body axis and props have 0.14 m radius.
X500_RADIUS = 0.386


def samples(trajectory, step=0.005):
    t = 0.0
    while t <= trajectory.duration:
        yield t, trajectory.sample(t)
        t += step


def square():
    points = [(0, 0), (2, 0), (2, 2), (0, 2)]
    return Trajectory([Waypoint(x, y, 1.0) for x, y in points], 0.4, 1.0, 1.0)


def test_rest_to_rest_with_exact_end_points():
    trajectory = square()
    start, end = trajectory.sample(0.0), trajectory.sample(trajectory.duration)
    assert start.position == pytest.approx((0, 0, 1))
    assert end.position == pytest.approx((0, 2, 1))
    for reference in (start, end, trajectory.sample(-1), trajectory.sample(1e6)):
        assert reference.velocity == pytest.approx((0, 0, 0), abs=1e-9)
        assert reference.acceleration == pytest.approx((0, 0, 0), abs=1e-9)


@pytest.mark.parametrize('trajectory', [square(), course.course_trajectory()])
def test_limits_and_derivative_consistency(trajectory):
    step = 0.005
    previous = None
    for _, ref in samples(trajectory, step):
        assert hypot(*ref.velocity) <= 1.0 + 1e-6
        assert hypot(*ref.acceleration) <= 1.0 + 1e-6
        if previous is not None:
            slope = [(a - b) / step for a, b in zip(ref.position, previous.position)]
            mean = [(a + b) / 2 for a, b in zip(ref.velocity, previous.velocity)]
            assert dist(slope, mean) < 0.01
            accel_slope = [(a - b) / step for a, b in zip(ref.velocity, previous.velocity)]
            mean = [(a + b) / 2 for a, b in zip(ref.acceleration, previous.acceleration)]
            assert dist(accel_slope, mean) < 0.05
            # Continuous acceleration: bounded jerk, no steps.
            assert dist(ref.acceleration, previous.acceleration) / step < 6.0
        previous = ref


def test_invalid_inputs():
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0, 0, 1)], 0.4, 1, 1)
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0, 0, 1), Waypoint(0, 0, 1)], 0.4, 1, 1)
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0, 0, 1), Waypoint(1, 0, 1), Waypoint(0, 0, 1)], 0.4, 1, 1)
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0, 0, 1), Waypoint(1, 0, 1)], 0.4, 0.0, 1)


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def course_points():
    trajectory = course.course_trajectory()
    for t, ref in samples(trajectory, 0.01):
        x, y, z = ref.position
        yield t, (x + course.LAUNCH[0], y + course.LAUNCH[1], z)


def test_route_starts_on_launch_mark_and_ends_on_pad():
    trajectory = course.course_trajectory()
    assert trajectory.start == (0.0, 0.0, course.CRUISE_HEIGHT)
    x, y, _ = trajectory.sample(trajectory.duration).position
    end = (x + course.LAUNCH[0], y + course.LAUNCH[1])
    assert end == pytest.approx(course.PAD, abs=1e-6)


def test_route_clears_every_wall_with_the_x500_footprint():
    for t, (x, y, _) in course_points():
        for wall in course.WALLS:
            clearance = segment_distance((x, y), wall.a, wall.b) - wall.half_thickness
            assert clearance > X500_RADIUS + 0.02, (wall.name, t, x, y)


def test_route_flies_the_tunnel_bore_and_passes_the_scored_gates():
    inside = [(x, z) for _, (x, y, z) in course_points()
              if course.TUNNEL_Y[0] <= y <= course.TUNNEL_Y[1] and x > 4.623]
    assert inside
    assert all(abs(x - course.TUNNEL_X) < 0.02 for x, _ in inside)
    gates = {'gate_a': (3.353, (5.169, course.FLOOR)), 'gate_b': (3.205, (2.518, 3.454))}
    path = [p for _, p in course_points()]
    for name, (gate_x, (low, high)) in gates.items():
        crossings = [(a, b) for a, b in zip(path, path[1:])
                     if (a[0] - gate_x) * (b[0] - gate_x) <= 0 and low <= a[1] <= high]
        assert crossings, name
        for a, _ in crossings:
            assert min(a[1] - low, high - a[1]) > 0.43, name


def test_rocks_are_overflown_with_margin():
    # Legs reach about 0.23 m below the base link; the base link starts about
    # 0.2 m above the floor, so the lowest point is near the cruise height.
    lowest = course.CRUISE_HEIGHT - 0.05
    assert lowest > course.ROCK_HEIGHT + 0.15


def yaw_square(**kwargs):
    points = [(0, 0), (2, 0), (2, 2), (0, 2)]
    return Trajectory([Waypoint(x, y, 1.0) for x, y in points], 0.4, 1.0, 1.0,
                      yaw_mode='tangent', **kwargs)


def test_fixed_yaw_mode_plans_no_heading():
    reference = square().sample(1.0)
    assert reference.yaw is None and reference.yaw_rate is None


def test_tangent_yaw_follows_each_leg_and_turns_left():
    trajectory = yaw_square()
    assert trajectory.sample(0.0).yaw == pytest.approx(0.0)
    assert trajectory.sample(trajectory.duration).yaw == pytest.approx(pi)
    for _, ref in samples(trajectory, 0.01):
        vx, vy, _ = ref.velocity
        if hypot(vx, vy) > 0.3 and abs(ref.yaw_rate) < 1e-6:
            assert ref.yaw == pytest.approx(atan2(vy, vx), abs=1e-3)
        assert ref.yaw_rate >= -1e-9


@pytest.mark.parametrize('trajectory,limit', [(yaw_square(max_yaw_rate=0.8), 0.8),
                                              (course.course_trajectory(), 1.0)])
def test_yaw_rate_is_capped_continuous_and_consistent(trajectory, limit):
    step = 0.005
    previous = None
    for _, ref in samples(trajectory, step):
        assert abs(ref.yaw_rate) <= limit + 1e-3
        if previous is not None:
            slope = (ref.yaw - previous.yaw) / step
            assert slope == pytest.approx((ref.yaw_rate + previous.yaw_rate) / 2, abs=0.03)
            assert abs(ref.yaw_rate - previous.yaw_rate) / step < 3.0
        previous = ref


def test_course_yaw_starts_north_and_rejects_bad_mode():
    assert course.course_trajectory().sample(0.0).yaw == pytest.approx(pi / 2)
    with pytest.raises(ValueError):
        course.course_trajectory(yaw_mode='spin')
