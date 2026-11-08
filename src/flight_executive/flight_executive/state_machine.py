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
                 takeoff_trigger: str = "position", readiness_timeout: float = 30.0, wait_for_planner: bool = False, skybeta_compat: bool = False):
        if takeoff_trigger not in ("position", "offboard"):
            raise ValueError("takeoff_trigger must be position or offboard")
        self.skybeta_compat = skybeta_compat
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
        self.tracking = Tracking(persistent=skybeta_compat)
        self.hold_target = None
        self.last_tracking_target = None
        self.last_tick = None
        self.manual_land_at = None

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

    def request_hold_land(self, now: float):
        """Latch operator cancellation; never let later route samples resume it."""
        if self.state in (State.ERROR, State.DONE):
            return False, 'Mission is stopped; no automatic landing in progress'
        if self.manual_land_at is not None or self.state == State.LAND:
            return True, 'Abort/landing already in progress'
        if self.state not in (State.TAKEOFF, State.HOLD, State.TRACKING):
            return False, 'No active mission to abort'
        if (not self._fresh(self.status, now) or not self.status.armed
                or not self.status.offboard):
            return False, 'Hold then land requires an armed vehicle in Offboard'
        p = self.position
        if not self._fresh(p, now) or not p.usable():
            return False, 'Cannot capture hold position: local position unavailable'
        self.manual_land_at = now + 2.0
        self._hold(Setpoint(p.x, p.y, p.z, p.yaw), now)
        return True, 'Mission aborted: holding current position for 2 seconds, then landing'

    def accept_trajectory(self, trajectory: Trajectory) -> bool:
        if self.manual_land_at is not None:
            return False
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
                                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
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
        if not self.skybeta_compat and (not self._fresh(self.status, now) or not self._fresh(self.landed, now)):
            self.fail('PX4 status or landed telemetry is stale during landing')
            return Output()
        if self.landed.landed and not self.status.armed:
            self.state = State.DONE
            return Output()
        if not self.skybeta_compat and self.status.failsafe:
            self.fail('PX4 reports failsafe during landing')
            return Output()
        requesting = self.landing.phase == LandPhase.REQUEST and not self.status.auto_land
        gap = self.last_tick is not None and now > self.last_tick + .5
        self.last_tick = now
        if not self.skybeta_compat and requesting and gap:
            self.fail('Control timer gap before Land confirmation')
            return Output()
        if not self.skybeta_compat and requesting and (not self._fresh(self.position, now)
                           or not self.position.usable()):
            self.fail('Local position invalid before Land confirmation')
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
        if self.manual_land_at is not None:
            # Pilot mode changes retain priority over the pending land request.
            if not self._fresh(self.status, now) or not self.status.armed or not self.status.offboard:
                self.fail('Manual landing cancelled: PX4 left armed Offboard or telemetry was lost')
                return Output()
            self.last_tick = now
            if now >= self.manual_land_at:
                return self._start_land(now)
            return Output(self.hold_target)
        gap = self.last_tick is not None and now - self.last_tick > 0.5
        self.last_tick = now
        if not self.skybeta_compat and self._fresh(self.status, now) and self.status.failsafe:
            self.fail('PX4 reports failsafe')
            return Output()
        if self.state == State.WAITING:
            if self._fresh(self.status, now) and self.status.armed and self.takeoff_trigger != "offboard":
                self.fail('Refusing to adopt an armed vehicle')
            elif self._fresh(self.landed, now) and not self.landed.landed:
                self.fail('Refusing to adopt an airborne vehicle')
            elif (not self.skybeta_compat and self.readiness_timeout > 0 and now >= self.started_at + self.readiness_timeout
                  and self.takeoff_trigger != "offboard"):
                self.fail('PX4 readiness timed out')
            elif (self._fresh(self.status, now)
                  and self._fresh(self.position, now)
                  and self._fresh(self.landed, now)
                  and (self.skybeta_compat or self.status.preflight_ok) and self.position.usable()):
                p = self.position
                origin = Setpoint(p.x, p.y, p.z, p.yaw)
                self.takeoff = Takeoff(origin, self.height, now, self.takeoff_trigger)
                self.takeoff.skybeta_compat = self.skybeta_compat
                self.state = State.TAKEOFF
                return Output(origin)
            return Output()
        if not self.skybeta_compat and (not self._fresh(self.status, now) or not self._fresh(self.position, now)
                or not self._fresh(self.landed, now)):
            self.fail('PX4 status, local position, or landed telemetry is stale')
        elif not self.skybeta_compat and not self.position.usable():
            self.fail('Local position, vertical velocity, or heading is invalid')
        elif not self.skybeta_compat and gap and self.phase not in (Phase.PREPARE, Phase.WAIT_ARM,
                                        Phase.ARMED_IN_POSITION, Phase.WAIT_POSITION, Phase.WAIT_OFFBOARD):
            self.fail('Control timer gap exceeded 0.5 seconds')
        elif not self.skybeta_compat and self.state in (State.HOLD, State.TRACKING):
            if not self.status.offboard:
                self.fail('PX4 departed Offboard mode')
            elif not self.status.armed:
                self.fail('PX4 unexpectedly disarmed')
        if self.state == State.ERROR:
            return Output()
        if (not self.skybeta_compat and self.state == State.TAKEOFF
                and self.phase in (Phase.PREPARE, Phase.WAIT_ARM)
                and not self.status.armed and not self.status.offboard
                and self.landed.landed and not self.status.preflight_ok):
            # A disarmed preflight fluctuation revokes readiness, not the mission.
            # Discard warmup/authorization; recovery needs a new arm/Position sequence.
            self.state = State.WAITING
            self.takeoff = None
            self.started_at = now
            return Output()
        if (self.skybeta_compat and self.state in (State.HOLD, State.TRACKING)
                and (not self.status.offboard or not self.status.armed)):
            return Output(self.hold_target)
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
            self.last_tracking_target = Setpoint(target.x, target.y, target.z, target.yaw,
                                        (0., 0., 0.), (0., 0., 0.), 0.)
            return Output(target)
        if self.state == State.TRACKING:
            p = self.position
            target = (Setpoint(p.x, p.y, p.z, p.yaw)
                      if self._fresh(p, now) and p.usable()
                      else self.last_tracking_target or self.hold_target)
            return self._hold(target, now)
        if self.hold_started is None:
            self.hold_started = now
        if not self.skybeta_compat and not self.wait_for_planner and now > self.hold_started + self.hold_timeout:
            return self._start_land(now)
        return Output(self.hold_target)
