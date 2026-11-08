"""Operator-defined launch-relative route; no map queries or collision checking."""
from dataclasses import replace
from math import atan2, sqrt, isfinite
from pathlib import Path
import yaml

from flight_trajectory.course_follower import wrap
from flight_trajectory.triangle_course import TriangleFollower
from flight_trajectory.trajectory import Reference, Trajectory, Waypoint

# Forward, left coordinates relative to the starting pose, in metres.
# Tunnel centre comes from the reference CAD; actual course placement is operator-confirmed.
ROUTE_XY = ((0., 0.), (1.2, 0.), (1.2, -4.8261),
            (-3.3863, -4.8261), (-3.3863, -.77724),
            (-2.82128, -.77724))
HEIGHT = 1.0


def load_waypoints(path):
    with Path(path).open() as stream:
        data = yaml.safe_load(stream)
    return validate_waypoints(data.get('waypoints') if isinstance(data, dict) else None)


def validate_waypoints(points):
    if not isinstance(points, (list, tuple)) or len(points) < 2:
        raise ValueError('fixed course needs at least two [x, y, z] waypoints')
    result = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 3:
            raise ValueError('each fixed-course waypoint must be [x, y, z]')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v) for v in point):
            raise ValueError('waypoint coordinates must be finite numbers')
        result.append(tuple(float(v) for v in point))
    if result[0][2] <= 0:
        raise ValueError('first waypoint z must be positive: it sets takeoff height')
    if any(a == b for a, b in zip(result, result[1:])):
        raise ValueError('consecutive waypoints must differ')
    return tuple(result)


class FixedCourse:
    def __init__(self, waypoints=None):
        self.waypoints = validate_waypoints(waypoints if waypoints is not None else
                                            tuple((x, y, HEIGHT) for x, y in ROUTE_XY))
        # Take off vertically first, then visit the first configured XY if needed.
        if self.waypoints[0][:2] != (0., 0.):
            self.waypoints = ((0., 0., self.waypoints[0][2]),) + self.waypoints
        self.segments = []
        self.length = 0.
        heading = 0.
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            target = (heading + wrap(atan2(b[1]-a[1], b[0]-a[0])-heading)
                      if a[:2] != b[:2] else heading)
            delta = target-heading
            if abs(delta) > 1e-9:
                # Quintic yaw transition: <=0.3 rad/s and <=0.4 rad/s^2.
                duration = max(1.875*abs(delta)/.3, sqrt(5.774*abs(delta)/.4))
                self.segments.append(('turn', duration, a, heading, delta))
            leg = Trajectory([Waypoint(*a), Waypoint(*b)], corner_radius=.1,
                             max_speed=.2, max_acceleration=.2)
            self.segments.append(('line', leg.duration, leg, target, 0.))
            self.length += leg.length
            heading = target
        self.duration = sum(s[1] for s in self.segments)

    def sample(self, elapsed):
        elapsed = max(0., elapsed)
        for kind, duration, value, heading, delta in self.segments:
            if elapsed <= duration:
                if kind == 'line':
                    return replace(value.sample(elapsed), yaw=heading, yaw_rate=0.)
                u = elapsed/duration
                smooth = 10*u**3-15*u**4+6*u**5
                rate = (30*u**2-60*u**3+30*u**4)/duration
                return Reference(value, (0.,0.,0.), (0.,0.,0.),
                                 heading+delta*smooth, delta*rate)
            elapsed -= duration
        kind, duration, leg, heading, _ = self.segments[-1]
        return replace(leg.sample(duration), yaw=heading, yaw_rate=0.)


class FixedCourseFollower(TriangleFollower):
    def local(self, reference):
        out = super().local(reference)
        return replace(out, yaw=wrap(self.anchor.heading-reference.yaw),
                       yawspeed=-reference.yaw_rate)
