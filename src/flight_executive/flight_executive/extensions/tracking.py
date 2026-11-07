"""Fresh trajectory sampling for the tracking state."""

from flight_executive.model import Setpoint, Trajectory


class Tracking:
    """Retain one accepted NED sample with a monotonic receipt deadline."""

    def __init__(self):
        self.latest = None

    def accept(self, trajectory: Trajectory) -> None:
        self.latest = trajectory

    def clear(self) -> None:
        self.latest = None

    def sample(self, now: float, yaw: float):
        if self.latest is None:
            return None
        if not self.latest.received_at <= now <= self.latest.received_at + 1.0:
            return None
        t = self.latest
        if t.yaw is None:
            return Setpoint(*t.position, yaw, t.velocity, t.acceleration)
        return Setpoint(*t.position, t.yaw, t.velocity, t.acceleration, t.yawspeed)
