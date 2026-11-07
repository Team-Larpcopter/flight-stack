"""Mission transitions and shared health checks for flight extensions."""

from math import isfinite

from flight_executive.extensions.takeoff import Takeoff
from flight_executive.extensions.tracking import Tracking
from flight_executive.model import Landed, Output, Phase, Setpoint, State, Trajectory

LANDED_TIMEOUT = 2.5


class StateMachine:
    """Own mission state while extensions own state-specific behavior."""

    def __init__(self, started_at: float, takeoff_height: float = 1.0):
        if not isfinite(takeoff_height) or takeoff_height <= 0:
            raise ValueError('takeoff_height must be finite and positive')
        self.height = takeoff_height
        self.started_at = started_at
        self.state = State.WAITING
        self.reason = ''
        self.status = None
        self.position = None
        self.landed = None
        self.takeoff = None
        self.tracking = Tracking()
        self.hold_target = None
        self.resets = None
        self.last_tick = None

    @property
    def phase(self):
        return self.takeoff.phase if self.state == State.TAKEOFF else None

    def fail(self, reason: str) -> None:
        if self.state != State.ERROR:
            self.state = State.ERROR
            self.reason = reason
            self.tracking.clear()

    def acknowledge(self, command: int, result: int) -> None:
        if self.state == State.TAKEOFF:
            self.takeoff.acknowledge(command, result)
            if self.takeoff.reason:
                self.fail(self.takeoff.reason)

    def accept_trajectory(self, trajectory: Trajectory) -> None:
        if self.state in (State.HOLD, State.TRACKING):
            self.tracking.accept(trajectory)

    @staticmethod
    def _fresh(snapshot, now: float) -> bool:
        # PX4 publishes vehicle_land_detected only on change or at 1 Hz, so a
        # 1 s limit trips on ordinary jitter; allow that topic 2.5 s.
        limit = LANDED_TIMEOUT if isinstance(snapshot, Landed) else 1.0
        return snapshot is not None and 0 <= now - snapshot.received_at <= limit

    def _hold(self, target: Setpoint) -> Output:
        self.hold_target = Setpoint(target.x, target.y, target.z, target.yaw,
                                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.tracking.clear()
        self.state = State.HOLD
        return Output(self.hold_target)

    def tick(self, now: float) -> Output:
        """Advance the mission and return this tick's PX4 publications."""
        if self.state == State.ERROR:
            return Output()
        gap = self.last_tick is not None and now - self.last_tick > 0.5
        self.last_tick = now
        if self._fresh(self.status, now) and self.status.failsafe:
            self.fail('PX4 reports failsafe')
            return Output()
        if self.state == State.WAITING:
            if self._fresh(self.status, now) and self.status.armed:
                self.fail('Refusing to adopt an armed vehicle')
            elif self._fresh(self.landed, now) and not self.landed.landed:
                self.fail('Refusing to adopt an airborne vehicle')
            elif now >= self.started_at + 30.0:
                self.fail('PX4 readiness timed out')
            elif (self._fresh(self.status, now)
                  and self._fresh(self.position, now)
                  and self._fresh(self.landed, now)
                  and self.status.preflight_ok and self.position.usable()):
                p = self.position
                origin = Setpoint(p.x, p.y, p.z, p.yaw)
                self.resets = p.reset_counters
                self.takeoff = Takeoff(origin, self.height, now)
                self.state = State.TAKEOFF
                return Output(origin)
            return Output()
        if (not self._fresh(self.status, now) or not self._fresh(self.position, now)
                or not self._fresh(self.landed, now)):
            self.fail('PX4 status, local position, or landed telemetry is stale')
        elif not self.position.usable():
            self.fail('Local position, vertical velocity, or heading is invalid')
        elif self.position.reset_counters != self.resets:
            self.fail('Local position or heading estimator reset')
        elif gap and self.phase != Phase.PREPARE:
            self.fail('Control timer gap exceeded 0.5 seconds')
        elif self.state in (State.HOLD, State.TRACKING):
            if not self.status.offboard:
                self.fail('PX4 departed Offboard mode')
            elif not self.status.armed:
                self.fail('PX4 unexpectedly disarmed')
        if self.state == State.ERROR:
            return Output()
        if self.state == State.TAKEOFF:
            result = self.takeoff.tick(now, self.status, self.position, self.landed, gap)
            if result.reason:
                self.fail(result.reason)
                return Output()
            if result.complete:
                return self._hold(self.takeoff.target)
            return result.output
        target = self.tracking.sample(now, self.hold_target.yaw)
        if target is not None:
            self.state = State.TRACKING
            return Output(target)
        if self.state == State.TRACKING:
            p = self.position
            return self._hold(Setpoint(p.x, p.y, p.z, p.yaw))
        return Output(self.hold_target)
