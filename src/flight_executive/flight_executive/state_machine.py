"""Mission transitions and shared health checks for flight extensions."""

from math import isfinite

from flight_executive.extensions.land import Land, LandPhase
from flight_executive.extensions.takeoff import Takeoff
from flight_executive.extensions.tracking import Tracking, stationary
from flight_executive.model import Landed, Output, Phase, Setpoint, State, Trajectory

LANDED_TIMEOUT = 2.5


class StateMachine:
    """Own mission state while extensions own state-specific behavior."""

    def __init__(self, started_at: float, takeoff_height: float = 1.0,
                 hold_timeout: float = 5.0, landing_timeout: float = 45.0,
                 takeoff_trigger: str = "position", readiness_timeout: float = 30.0, wait_for_planner: bool = False):
        if takeoff_trigger not in ("position", "offboard"):
            raise ValueError("takeoff_trigger must be position or offboard")
        self.takeoff_trigger = takeoff_trigger
        if not isfinite(readiness_timeout) or readiness_timeout < 0:
            raise ValueError("readiness_timeout must be finite and nonnegative")
        self.readiness_timeout = readiness_timeout
        if not isfinite(takeoff_height) or takeoff_height <= 0:
            raise ValueError('takeoff_height must be finite and positive')
        for name, value in (('hold_timeout', hold_timeout),
                            ('landing_timeout', landing_timeout)):
            if not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.wait_for_planner = wait_for_planner
        self.hold_timeout = hold_timeout
        self.landing_timeout = landing_timeout
        self.hold_started = None
        self.active_trajectory_id = None
        self.landing = None
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
        if self.state == State.LAND:
            return self.landing.phase
        return self.takeoff.phase if self.state == State.TAKEOFF else None

    def fail(self, reason: str) -> None:
        if self.state != State.ERROR:
            self.state = State.ERROR
            self.reason = reason
            self.tracking.clear()

    def acknowledge(self, command: int, result: int) -> None:
        if self.state == State.LAND:
            self.landing.acknowledge(command, result)
        if self.state == State.TAKEOFF:
            self.takeoff.acknowledge(command, result)
            if self.takeoff.reason:
                self.fail(self.takeoff.reason)

    def accept_trajectory(self, trajectory: Trajectory) -> bool:
        if self.state not in (State.HOLD, State.TRACKING):
            return False
        if self.active_trajectory_id not in (None, trajectory.trajectory_id):
            return False
        if trajectory.is_final and not stationary(trajectory):
            return False
        self.active_trajectory_id = trajectory.trajectory_id
        self.tracking.accept(trajectory)
        return True

    @staticmethod
    def _fresh(snapshot, now: float) -> bool:
        # PX4 publishes vehicle_land_detected only on change or at 1 Hz, so a
        # 1 s limit trips on ordinary jitter; allow that topic 2.5 s.
        limit = LANDED_TIMEOUT if isinstance(snapshot, Landed) else 1.0
        return snapshot is not None and 0 <= now - snapshot.received_at <= limit

    def _hold(self, target: Setpoint, now: float) -> Output:
        self.hold_target = Setpoint(target.x, target.y, target.z, target.yaw,
                                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.tracking.clear()
        self.state = State.HOLD
        self.hold_started = now
        return Output(self.hold_target)

    def _start_land(self, now):
        p = self.position
        target = Setpoint(p.x, p.y, p.z, p.yaw, (0., 0., 0.), (0., 0., 0.))
        self.tracking.clear()
        self.landing = Land(target, now, self.landing_timeout)
        self.state = State.LAND
        return self._land(now)

    def _land(self, now):
        if not self._fresh(self.status, now) or not self._fresh(self.landed, now):
            self.fail('PX4 status or landed telemetry is stale during landing')
            return Output()
        if self.landed.landed and not self.status.armed:
            self.state = State.DONE
            return Output()
        if self.status.failsafe:
            self.fail('PX4 reports failsafe during landing')
            return Output()
        requesting = self.landing.phase == LandPhase.REQUEST and not self.status.auto_land
        gap = self.last_tick is not None and now > self.last_tick + .5
        self.last_tick = now
        if requesting and gap:
            self.fail('Control timer gap before Land confirmation')
            return Output()
        if requesting and (not self._fresh(self.position, now)
                           or not self.position.usable()
                           or self.position.reset_counters != self.resets):
            self.fail('Local position invalid or reset before Land confirmation')
            return Output()
        result = self.landing.tick(now, self.status, self.landed)
        if result.reason:
            self.fail(result.reason)
            return Output()
        if result.complete:
            self.state = State.DONE
            return Output()
        return result.output

    def tick(self, now: float) -> Output:
        """Advance the mission and return this tick's PX4 publications."""
        if self.state in (State.ERROR, State.DONE):
            return Output()
        if self.state == State.LAND:
            return self._land(now)
        gap = self.last_tick is not None and now - self.last_tick > 0.5
        self.last_tick = now
        if self._fresh(self.status, now) and self.status.failsafe:
            self.fail('PX4 reports failsafe')
            return Output()
        if self.state == State.WAITING:
            if self._fresh(self.status, now) and self.status.armed and self.takeoff_trigger != "offboard":
                self.fail('Refusing to adopt an armed vehicle')
            elif self._fresh(self.landed, now) and not self.landed.landed:
                self.fail('Refusing to adopt an airborne vehicle')
            elif (self.readiness_timeout > 0 and now >= self.started_at + self.readiness_timeout
                  and self.takeoff_trigger != "offboard"):
                self.fail('PX4 readiness timed out')
            elif (self._fresh(self.status, now)
                  and self._fresh(self.position, now)
                  and self._fresh(self.landed, now)
                  and self.status.preflight_ok and self.position.usable()):
                p = self.position
                origin = Setpoint(p.x, p.y, p.z, p.yaw)
                self.resets = p.reset_counters
                self.takeoff = Takeoff(origin, self.height, now, self.takeoff_trigger)
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
        elif gap and self.phase not in (Phase.PREPARE, Phase.WAIT_ARM,
                                        Phase.ARMED_IN_POSITION, Phase.WAIT_POSITION, Phase.WAIT_OFFBOARD):
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
                return self._hold(self.takeoff.target, now)
            return result.output
        target = self.tracking.sample(now, self.hold_target.yaw)
        if target is not None:
            self.state = State.TRACKING
            if self.tracking.complete(now, self.position):
                return self._start_land(now)
            return Output(target)
        if self.state == State.TRACKING:
            p = self.position
            return self._hold(Setpoint(p.x, p.y, p.z, p.yaw), now)
        if self.hold_started is None:
            self.hold_started = now
        if not self.wait_for_planner and now > self.hold_started + self.hold_timeout:
            return self._start_land(now)
        return Output(self.hold_target)
