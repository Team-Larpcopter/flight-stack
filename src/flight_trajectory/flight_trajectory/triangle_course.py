"""Three rest-to-rest legs in the takeoff frame, forward/left/up in metres."""

from math import cos, sin

from flight_trajectory.course_follower import CourseFollower, Sample
from flight_trajectory.trajectory import Trajectory, Waypoint


class TriangleCourse:
    def __init__(self):
        points = [(0., 0., .75), (.75, 0., .75), (.75, .75, .75), (0., 0., .75)]
        self.legs = [Trajectory([Waypoint(*a), Waypoint(*b)], corner_radius=.1,
                                max_speed=.2, max_acceleration=.2)
                     for a, b in zip(points, points[1:])]
        self.duration = sum(leg.duration for leg in self.legs)
        self.length = sum(leg.length for leg in self.legs)

    def sample(self, elapsed):
        for leg in self.legs:
            if elapsed <= leg.duration:
                return leg.sample(elapsed)
            elapsed -= leg.duration
        return self.legs[-1].sample(self.legs[-1].duration)


class TriangleFollower(CourseFollower):
    def local(self, reference):
        c, s = cos(self.anchor.heading), sin(self.anchor.heading)

        def ned(vector):
            forward, left, up = vector
            return (c * forward + s * left, s * forward - c * left, -up)

        offset = ned(reference.position)
        anchor = (self.anchor.x, self.anchor.y, self.anchor.z)
        return Sample(tuple(a + b for a, b in zip(anchor, offset)),
                      ned(reference.velocity), ned(reference.acceleration),
                      self.anchor.heading, 0.)
