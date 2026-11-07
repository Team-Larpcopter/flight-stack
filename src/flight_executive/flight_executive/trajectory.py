"""Kinematically feasible, time-parameterized reference built from waypoints.

Straight segments are joined by circular arcs, so the path stays inside a known
band around the waypoint polyline. A speed profile limits speed, tangential
acceleration, and lateral acceleration on arcs. That reference is continuous in
position and velocity, but its acceleration steps at arc ends and speed-profile
breakpoints, and a multicopter cannot change attitude instantly.

The reference is therefore convolved in time with a raised-cosine window. The
result has continuous acceleration and bounded, continuous jerk. Averaging
cannot raise peak speed or acceleration, and it moves the path by about
a * sigma^2 / 2 (a couple of centimetres at the defaults). Sampling returns
position, velocity, and acceleration so PX4 can use velocity and acceleration
as feedforward around its position loop.

Yaw is planned the same way. In ``tangent`` mode the raw heading is the path
direction, whose rate is speed times heading change per metre. The speed
profile also caps that rate at ``max_yaw_rate``, and the same window smooths
heading and rate together, so yaw rate is continuous and yaw acceleration is
bounded. In ``fixed`` mode no yaw is planned and the caller holds a heading.
Yaw here is ENU: radians counter-clockwise from +x (east).
"""

from bisect import bisect_right
from dataclasses import dataclass
from math import acos, atan2, ceil, cos, hypot, pi, sin, sqrt, tan
from typing import Optional, Sequence, Tuple

Vector = Tuple[float, float, float]


def _add(a, b, k=1.0):
    return tuple(x + k * y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(a):
    return sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return tuple(x / n for x in a)


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    corner_radius: Optional[float] = None

    @property
    def point(self) -> Vector:
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class Reference:
    position: Vector
    velocity: Vector
    acceleration: Vector
    yaw: Optional[float] = None
    yaw_rate: Optional[float] = None


YAW_MODES = ('fixed', 'tangent')


@dataclass(frozen=True)
class _Line:
    start: Vector
    direction: Vector
    length: float

    def evaluate(self, s):
        return _add(self.start, self.direction, s), self.direction, (0.0, 0.0, 0.0)

    def curvature(self):
        return 0.0


@dataclass(frozen=True)
class _Arc:
    center: Vector
    u_in: Vector
    normal: Vector
    radius: float
    length: float

    def evaluate(self, s):
        phi = s / self.radius
        to_center = _add(_add((0.0, 0.0, 0.0), self.normal, cos(phi)), self.u_in, -sin(phi))
        tangent = _add(_add((0.0, 0.0, 0.0), self.normal, sin(phi)), self.u_in, cos(phi))
        position = _add(self.center, to_center, -self.radius)
        return position, tangent, tuple(c / self.radius for c in to_center)

    def curvature(self):
        return 1.0 / self.radius


class Trajectory:
    """Rest-to-rest reference along a rounded waypoint polyline."""

    def __init__(self, waypoints: Sequence[Waypoint], corner_radius: float,
                 max_speed: float, max_acceleration: float,
                 max_lateral_acceleration: Optional[float] = None,
                 step: float = 0.01, smoothing: float = 0.4,
                 sample_period: float = 0.01, yaw_mode: str = 'fixed',
                 max_yaw_rate: float = 1.0):
        if len(waypoints) < 2:
            raise ValueError('A trajectory needs at least two waypoints')
        for name, value in (('corner_radius', corner_radius), ('max_speed', max_speed),
                            ('max_acceleration', max_acceleration), ('step', step),
                            ('smoothing', smoothing), ('sample_period', sample_period),
                            ('max_yaw_rate', max_yaw_rate)):
            if not value > 0:
                raise ValueError(f'{name} must be positive')
        if yaw_mode not in YAW_MODES:
            raise ValueError(f'yaw_mode must be one of {YAW_MODES}')
        lateral = max_lateral_acceleration or max_acceleration
        self.yaw_mode = yaw_mode
        self.waypoints = tuple(waypoints)
        self.pieces = self._build(corner_radius)
        self.length = sum(p.length for p in self.pieces)
        self._offsets = []
        total = 0.0
        for piece in self.pieces:
            self._offsets.append(total)
            total += piece.length
        self._profile(max_speed, max_acceleration, lateral, max_yaw_rate, step)
        self._smooth(smoothing, sample_period)

    def _build(self, default_radius):
        points = [w.point for w in self.waypoints]
        lengths = [_norm(_add(b, a, -1.0)) for a, b in zip(points, points[1:])]
        if min(lengths) <= 1e-9:
            raise ValueError('Consecutive waypoints must be distinct')
        dirs = [_unit(_add(b, a, -1.0)) for a, b in zip(points, points[1:])]
        corners = [None]
        for i in range(1, len(points) - 1):
            u_in, u_out = dirs[i - 1], dirs[i]
            cos_turn = max(-1.0, min(1.0, _dot(u_in, u_out)))
            turn = acos(cos_turn)
            if turn < 1e-6:
                corners.append(None)
                continue
            if turn > 3.1:
                raise ValueError(f'Waypoint {i} reverses direction')
            radius = self.waypoints[i].corner_radius or default_radius
            # Each corner may use at most half of its neighbouring segments,
            # except the first and last segment, which have no other corner.
            limit_in = lengths[i - 1] * (1.0 if i == 1 else 0.5)
            limit_out = lengths[i] * (1.0 if i == len(points) - 2 else 0.5)
            tangent = min(radius * tan(turn / 2), limit_in, limit_out)
            radius = tangent / tan(turn / 2)
            perp = _add(u_out, u_in, -cos_turn)
            corners.append((tangent, radius, turn, _unit(perp)))
        corners.append(None)

        pieces = []
        for i, (a, direction, length) in enumerate(zip(points, dirs, lengths)):
            trim_start = corners[i][0] if corners[i] else 0.0
            trim_end = corners[i + 1][0] if corners[i + 1] else 0.0
            line_length = length - trim_start - trim_end
            if line_length > 1e-9:
                pieces.append(_Line(_add(a, direction, trim_start), direction, line_length))
            corner = corners[i + 1]
            if corner:
                tangent, radius, turn, normal = corner
                arc_start = _add(points[i + 1], direction, -tangent)
                pieces.append(_Arc(_add(arc_start, normal, radius), direction,
                                   normal, radius, radius * turn))
        return pieces

    def _locate(self, s):
        s = max(0.0, min(self.length, s))
        i = max(0, bisect_right(self._offsets, s) - 1)
        piece = self.pieces[i]
        return piece, min(piece.length, s - self._offsets[i])

    def _headings(self):
        """Return the unwrapped path heading at each arc-length grid point."""
        headings = []
        for s in self._s:
            piece, local = self._locate(s)
            tx, ty, _ = piece.evaluate(local)[1]
            if hypot(tx, ty) < 0.1:
                raise ValueError('Tangent yaw needs a mostly horizontal path')
            h = atan2(ty, tx)
            if headings:
                h += 2 * pi * round((headings[-1] - h) / (2 * pi))
            headings.append(h)
        return headings

    def _profile(self, max_speed, max_acceleration, lateral, max_yaw_rate, step):
        count = max(2, int(round(self.length / step)) + 1)
        self._s = [self.length * k / (count - 1) for k in range(count)]
        ds = self._s[1]
        self._heading = self._headings() if self.yaw_mode == 'tangent' else None
        limit = []
        for k, s in enumerate(self._s):
            piece, local = self._locate(s)
            curvature = piece.curvature()
            v = max_speed if curvature == 0 else min(max_speed, sqrt(lateral / curvature))
            if self._heading is not None:
                h = self._heading
                turn_rate = abs(h[min(k + 1, count - 1)] - h[max(k - 1, 0)]) / (
                    self._s[min(k + 1, count - 1)] - self._s[max(k - 1, 0)])
                if turn_rate > 1e-9:
                    v = min(v, max_yaw_rate / turn_rate)
            limit.append(v)
        v = list(limit)
        v[0] = v[-1] = 0.0
        for k in range(1, count):
            v[k] = min(v[k], sqrt(v[k - 1] ** 2 + 2 * max_acceleration * ds))
        for k in range(count - 2, -1, -1):
            v[k] = min(v[k], sqrt(v[k + 1] ** 2 + 2 * max_acceleration * ds))
        self._v = v
        self._t = [0.0]
        for k in range(1, count):
            self._t.append(self._t[-1] + 2 * ds / (v[k - 1] + v[k]))
        self.path_duration = self._t[-1]

    def _smooth(self, half_width, period):
        """Tabulate the raw reference convolved with a raised-cosine window."""
        taps = max(1, int(round(half_width / period)))
        weights = [cos(pi * i / (2 * taps)) ** 2 for i in range(-taps, taps + 1)]
        total = sum(weights)
        weights = [w / total for w in weights]
        # Output time 0 is raw time -half_width, so the window starts at rest.
        count = int(ceil(self.path_duration / period)) + 2 * taps + 1
        raw = [self._raw_sample((k - 2 * taps) * period) for k in range(count + 2 * taps)]
        self._period = period
        self._table = [
            tuple(sum(w * r[i] for w, r in zip(weights, raw[j:j + 2 * taps + 1]))
                  for i in range(len(raw[0])))
            for j in range(count)]
        self.duration = (count - 1) * period

    def _reference(self, flat):
        yaw = (flat[9], flat[10]) if self._heading is not None else (None, None)
        return Reference(tuple(flat[0:3]), tuple(flat[3:6]), tuple(flat[6:9]), *yaw)

    @property
    def start(self) -> Vector:
        return self.waypoints[0].point

    @property
    def end(self) -> Vector:
        return self.waypoints[-1].point

    def point_at(self, s: float) -> Vector:
        """Return the path position at arc length s."""
        piece, local = self._locate(s)
        return piece.evaluate(local)[0]

    def sample(self, t: float) -> Reference:
        """Return the smoothed reference at time t seconds after the start."""
        if t <= 0:
            return self._reference(self._table[0])
        if t >= self.duration:
            return self._reference(self._table[-1])
        k = min(int(t / self._period), len(self._table) - 2)
        f = t / self._period - k
        a, b = self._table[k], self._table[k + 1]
        return self._reference([x + f * (y - x) for x, y in zip(a, b)])

    def _raw_sample(self, t: float) -> tuple:
        """Return the unsmoothed reference as position, velocity, acceleration, yaw, rate."""
        zero = (0.0, 0.0, 0.0)
        heading = self._heading or (0.0, 0.0)
        if t <= 0:
            return (*self.start, *zero, *zero, heading[0], 0.0)
        if t >= self.path_duration:
            return (*self.end, *zero, *zero, heading[-1], 0.0)
        k = bisect_right(self._t, t) - 1
        ds = self._s[k + 1] - self._s[k]
        accel = (self._v[k + 1] ** 2 - self._v[k] ** 2) / (2 * ds)
        tau = t - self._t[k]
        speed = self._v[k] + accel * tau
        s = self._s[k] + self._v[k] * tau + 0.5 * accel * tau * tau
        piece, local = self._locate(s)
        position, tangent, curvature = piece.evaluate(local)
        velocity = tuple(speed * c for c in tangent)
        acceleration = tuple(accel * a + speed * speed * c
                             for a, c in zip(tangent, curvature))
        yaw = yaw_rate = 0.0
        if self._heading is not None:
            dh = (self._heading[k + 1] - self._heading[k]) / ds
            yaw = self._heading[k] + (s - self._s[k]) * dh
            yaw_rate = speed * dh
        return (*position, *velocity, *acceleration, yaw, yaw_rate)
