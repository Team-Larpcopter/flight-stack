"""One-shot map/PX4 registration from a stationary, grounded body pose."""
from dataclasses import dataclass
from math import atan2, cos, sin, isfinite, dist


def wrap(angle):
    return atan2(sin(angle), cos(angle))


@dataclass(frozen=True)
class Alignment:
    origin: tuple
    heading: float

    def to_ned(self, point):
        x, y, z = point
        c, s = cos(self.heading), sin(self.heading)
        return (self.origin[0]+c*x+s*y,
                self.origin[1]+s*x-c*y, self.origin[2]-z)


def from_pair(map_position, map_yaw, ned_position, ned_yaw):
    if not all(isfinite(v) for v in (*map_position, map_yaw, *ned_position, ned_yaw)):
        raise ValueError('Nonfinite frame alignment')
    heading = wrap(ned_yaw+map_yaw)
    rotated = Alignment((0., 0., 0.), heading).to_ned(map_position)
    return Alignment(tuple(n-r for n, r in zip(ned_position, rotated)), heading)


class GroundAlignment:
    """Only the adapter's disarmed/landed/healthy branch may offer samples.

    A stable one-second window rejects estimator startup changes. Once accepted,
    the transform is immutable until the adapter explicitly restarts ground
    registration after a PX4 position/heading reset.
    """
    def __init__(self):
        self.value = None
        self.candidate = None
        self.since = None
        self.last = None
        self.map_start = None

    def reset_pending(self):
        self.candidate = self.since = self.last = self.map_start = None

    def observe(self, now, map_position, map_yaw, ned_position, ned_yaw):
        if self.value is not None:
            return self.value
        value = from_pair(map_position, map_yaw, ned_position, ned_yaw)
        if (self.candidate is None or now < self.last or now-self.last > .3 or
                dist(value.origin, self.candidate.origin) > .05 or
                abs(wrap(value.heading-self.candidate.heading)) > .05 or
                dist(map_position, self.map_start) > .05):
            self.candidate, self.since, self.map_start = value, now, tuple(map_position)
        self.last = now
        if now-self.since >= 1.:
            self.value = value
        return self.value
