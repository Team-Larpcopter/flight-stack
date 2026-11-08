"""Operator-defined launch-relative route; no map queries or collision checking."""
from dataclasses import replace
from math import atan2, sqrt

from flight_trajectory.course_follower import wrap
from flight_trajectory.triangle_course import TriangleFollower
from flight_trajectory.trajectory import Reference, Trajectory, Waypoint

# Forward, left coordinates relative to the starting pose, in metres.
# Tunnel centre comes from the reference CAD; actual course placement is operator-confirmed.
ROUTE_XY = ((0., 0.), (1.2, 0.), (1.2, -4.8261),
            (-3.3863, -4.8261), (-3.3863, -.77724),
            (-2.82128, -.77724))
HEIGHT = 1.0


class FixedCourse:
    def __init__(self):
        self.waypoints = tuple((x, y, HEIGHT) for x, y in ROUTE_XY)
        self.segments = []
        self.length = 0.
        heading = 0.
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            target = heading + wrap(atan2(b[1]-a[1], b[0]-a[0])-heading)
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
