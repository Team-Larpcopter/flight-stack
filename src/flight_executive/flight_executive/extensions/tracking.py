"""Fresh trajectory sampling and measured arrival at a terminal target."""

from math import hypot, isfinite

from flight_executive.model import Setpoint, Trajectory


def stationary(trajectory):
    """Return whether the reference describes a stationary endpoint."""
    return (all(abs(v) <= 1e-6 for v in trajectory.velocity + trajectory.acceleration)
            and (trajectory.yawspeed is None or abs(trajectory.yawspeed) <= 1e-6))


def fresh(t, now):
    return (t is not None and t.received_at <= now <= t.received_at + 1.0
            and (t.valid_until is None or (isfinite(t.valid_until) and now < t.valid_until)))


class Tracking:
    """Retain one accepted sample and verify a continuous terminal dwell."""

    def __init__(self, persistent=False):
        self.persistent = persistent
        self.latest = None
        self.settled_since = None

    def accept(self, trajectory: Trajectory) -> None:
        old = self.latest
        if (not trajectory.is_final or old is None or not old.is_final
                or (old.trajectory_id, old.position, old.yaw)
                != (trajectory.trajectory_id, trajectory.position, trajectory.yaw)):
            self.settled_since = None
        self.latest = trajectory

    def clear(self) -> None:
        self.latest = None
        self.settled_since = None

    def sample(self, now: float, yaw: float):
        if self.latest is None:
            return None
        if not self.persistent and not fresh(self.latest, now):
            self.settled_since = None
            return None
        t = self.latest
        if t.yaw is None:
            return Setpoint(*t.position, yaw, t.velocity, t.acceleration)
        return Setpoint(*t.position, t.yaw, t.velocity, t.acceleration, t.yawspeed)

    def complete(self, now, position):
        t = self.latest
        if (t is None or not t.is_final or not stationary(t)
                or (not self.persistent and not fresh(t, now))):
            self.settled_since = None
            return False
        speed_valid = position.velocity_valid and all(isfinite(v) for v in (
            position.vx, position.vy, position.vz))
        settled = (speed_valid
                   and hypot(position.x - t.position[0], position.y - t.position[1]) <= .25
                   and abs(position.z - t.position[2]) <= .15
                   and hypot(position.vx, position.vy, position.vz) < .2)
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        return self.settled_since is not None and now >= self.settled_since + 1.0
