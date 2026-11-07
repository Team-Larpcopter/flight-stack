"""ROS-independent, one-shot takeoff, trajectory tracking, and landing controller."""

from dataclasses import dataclass
from enum import Enum
from math import atan2, cos, hypot, isfinite, nan, pi, sin
from typing import Optional


LANDED_TIMEOUT = 2.5


class State(Enum):
    WAITING = 'WAITING'
    TAKEOFF = 'TAKEOFF'
    HOLD = 'HOLD'
    TRACKING = 'TRACKING'
    LAND = 'LAND'
    DONE = 'DONE'
    ERROR = 'ERROR'


class Phase(Enum):
    PREPARE = 'PREPARE'
    MODE = 'MODE'
    ARM = 'ARM'
    ASCEND = 'ASCEND'


class Command(Enum):
    OFFBOARD = 176
    ARM = 400
    LAND = 21


@dataclass(frozen=True)
class Status:
    received_at: float
    armed: bool
    offboard: bool
    preflight_ok: bool
    failsafe: bool = False
    landing: bool = False


@dataclass(frozen=True)
class Position:
    received_at: float
    x: float
    y: float
    z: float
    yaw: float
    vz: float
    valid: bool = True
    reset_counters: tuple = (0, 0, 0)

    def usable(self) -> bool:
        return self.valid and all(isfinite(v) for v in (
            self.x, self.y, self.z, self.yaw, self.vz))


@dataclass(frozen=True)
class Landed:
    received_at: float
    landed: bool


@dataclass(frozen=True)
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float
    velocity: tuple = (nan, nan, nan)
    acceleration: tuple = (nan, nan, nan)
    yawspeed: float = nan


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return atan2(sin(angle), cos(angle))


def enu_to_ned_yaw(yaw: float) -> float:
    """Convert ENU yaw (counter-clockwise from east) to a PX4 heading."""
    return wrap(pi / 2 - yaw)


START_HEADING_TOLERANCE = 0.35


@dataclass(frozen=True)
class Output:
    setpoint: Optional[Setpoint] = None
    command: Optional[Command] = None


class Controller:
    """Own mission state; callers provide monotonic time and telemetry."""

    def __init__(self, started_at: float, takeoff_height: float = 1.0,
                 trajectory=None, max_tracking_error: float = 0.4):
        if not isfinite(takeoff_height) or takeoff_height <= 0:
            raise ValueError('takeoff_height must be finite and positive')
        if trajectory is not None:
            x, y, z = trajectory.start
            if abs(x) > 1e-6 or abs(y) > 1e-6 or abs(z - takeoff_height) > 1e-6:
                raise ValueError('Trajectory must start at (0, 0, takeoff_height)')
        if not isfinite(max_tracking_error) or max_tracking_error <= 0:
            raise ValueError('max_tracking_error must be finite and positive')
        self.height = takeoff_height
        self.trajectory = trajectory
        self.max_error = max_tracking_error
        self.tracking_started = None
        self.land_confirmed = False
        self.started_at = started_at
        self.state = State.WAITING
        self.phase = None
        self.reason = ''
        self.status = None
        self.position = None
        self.landed = None
        self.origin = None
        self.target = None
        self.resets = None
        self.phase_started = started_at
        self.last_tick = None
        self.last_command = None
        self.attempts = 0
        self.settled_since = None

    def fail(self, reason: str) -> None:
        if self.state != State.ERROR:
            self.state = State.ERROR
            self.reason = reason

    def acknowledge(self, command: int, result: int) -> None:
        pending = {Phase.MODE: Command.OFFBOARD, Phase.ARM: Command.ARM}
        expected = pending.get(self.phase) if self.state == State.TAKEOFF else None
        if self.state == State.LAND and not self.land_confirmed:
            expected = Command.LAND
        if expected is not None and self.attempts and command == expected.value:
            if result not in (0, 5):
                self.fail(f'PX4 rejected {expected.name}, result={result}')

    def _enter_phase(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self.phase_started = now
        self.last_command = None
        self.attempts = 0

    @staticmethod
    def _fresh(snapshot, now: float) -> bool:
        # PX4 publishes vehicle_land_detected only on change or at 1 Hz, so a
        # 1 s limit would trip on ordinary jitter; allow it 2.5 s.
        limit = LANDED_TIMEOUT if isinstance(snapshot, Landed) else 1.0
        return snapshot is not None and 0 <= now - snapshot.received_at <= limit

    def _request(self, command: Command, now: float, setpoint=None) -> Output:
        setpoint = setpoint or self.origin
        if self.last_command is None or now >= self.last_command + 1.0:
            if self.attempts == 3:
                self.fail(f'{command.name} confirmation timed out after 3 attempts')
                return Output()
            self.last_command = now
            self.attempts += 1
            return Output(setpoint, command)
        return Output(setpoint)

    def _local(self, reference) -> Setpoint:
        """Map a launch-relative east/north/up reference into PX4 local NED."""
        (east, north, up) = reference.position
        (ve, vn, vu) = reference.velocity
        (ae, an, au) = reference.acceleration
        o = self.origin
        yaw, yawspeed = o.yaw, nan
        if reference.yaw is not None:
            yaw, yawspeed = enu_to_ned_yaw(reference.yaw), -reference.yaw_rate
        return Setpoint(o.x + north, o.y + east, o.z - up, yaw,
                        (vn, ve, -vu), (an, ae, -au), yawspeed)

    def _heading_mismatch(self, heading: float) -> bool:
        """Refuse a planned-yaw trajectory unless the vehicle already faces its start."""
        start = self.trajectory.sample(0.0).yaw if self.trajectory else None
        return start is not None and abs(
            wrap(heading - enu_to_ned_yaw(start))) > START_HEADING_TOLERANCE

    def _settled(self, target: Setpoint, now: float) -> bool:
        p = self.position
        settled = (abs(p.z - target.z) <= 0.15
                   and hypot(p.x - target.x, p.y - target.y) <= 0.25
                   and abs(p.vz) < 0.2)
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        return settled and now >= self.settled_since + 1.0

    def _track(self, now: float) -> Output:
        elapsed = now - self.tracking_started
        target = self._local(self.trajectory.sample(elapsed))
        p = self.position
        if (hypot(p.x - target.x, p.y - target.y) > self.max_error
                or abs(p.z - target.z) > self.max_error):
            self.fail('Trajectory tracking error exceeded '
                      f'{self.max_error:.2f} m at t={elapsed:.1f} s')
            return Output()
        if elapsed >= self.trajectory.duration:
            if self._settled(target, now):
                self.target = target
                self.state = State.LAND
                self.phase_started = now
                self.last_command = None
                self.attempts = 0
                return self._request(Command.LAND, now, self.target)
            if elapsed >= self.trajectory.duration + 10.0:
                self.fail('Trajectory end point not reached within 10 seconds')
                return Output()
        return Output(target)

    def _land(self, now: float) -> Output:
        if self.land_confirmed:
            if self.landed.landed and not self.status.armed:
                self.state = State.DONE
            return Output()
        if self.status.landing and self.status.received_at >= self.phase_started:
            # PX4 Land owns the descent; stop the Offboard stream.
            self.land_confirmed = True
            return Output()
        return self._request(Command.LAND, now, self.target)

    def tick(self, now: float) -> Output:
        """Advance once and return this tick's publications, if any."""
        if self.state in (State.ERROR, State.DONE):
            return Output()
        gap = self.last_tick is not None and now - self.last_tick > 0.5
        self.last_tick = now
        if self.status is not None and self._fresh(self.status, now):
            if self.status.failsafe:
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
                if self._heading_mismatch(p.yaw):
                    self.fail('Vehicle heading is more than 20 degrees from the '
                              'trajectory start heading')
                    return Output()
                self.origin = Setpoint(p.x, p.y, p.z, p.yaw)
                self.target = Setpoint(p.x, p.y, p.z - self.height, p.yaw)
                self.resets = p.reset_counters
                self.state = State.TAKEOFF
                self._enter_phase(Phase.PREPARE, now)
            return Output(self.origin) if self.state == State.TAKEOFF else Output()

        if (not self._fresh(self.status, now) or not self._fresh(self.position, now)
                or not self._fresh(self.landed, now)):
            self.fail('PX4 status, local position, or landed telemetry is stale')
        elif not self.position.usable():
            self.fail('Local position, vertical velocity, or heading is invalid')
        elif self.position.reset_counters != self.resets:
            self.fail('Local position or heading estimator reset')
        elif gap and self.phase != Phase.PREPARE:
            self.fail('Control timer gap exceeded 0.5 seconds')
        elif (self.phase in (Phase.ARM, Phase.ASCEND) and not self.status.offboard
              and not (self.state == State.LAND and self.status.landing)):
            self.fail('PX4 departed Offboard mode')
        elif (self.phase == Phase.ASCEND and not self.status.armed
              and not (self.land_confirmed and self.landed.landed)):
            self.fail('PX4 unexpectedly disarmed')
        elif self.phase in (Phase.PREPARE, Phase.MODE) and not self.landed.landed:
            self.fail('Vehicle left ground before executive requested arming')
        elif not self.status.armed and not self.status.preflight_ok:
            self.fail('PX4 preflight readiness lost before arming')
        elif self.phase in (Phase.PREPARE, Phase.MODE) and self.status.armed:
            self.fail('Vehicle armed before executive requested arming')
        if self.state == State.ERROR:
            return Output()

        if self.state == State.HOLD:
            return Output(self.target)
        if self.state == State.TRACKING:
            return self._track(now)
        if self.state == State.LAND:
            return self._land(now)
        if self.phase == Phase.PREPARE:
            if gap:
                self.phase_started = now
            if now >= self.phase_started + 2.0:
                self._enter_phase(Phase.MODE, now)
                return self._request(Command.OFFBOARD, now)
            return Output(self.origin)
        if self.phase == Phase.MODE:
            if self.status.offboard and self.status.received_at >= self.phase_started:
                self._enter_phase(Phase.ARM, now)
                return self._request(Command.ARM, now)
            return self._request(Command.OFFBOARD, now)
        if self.phase == Phase.ARM:
            if self.status.armed and self.status.received_at >= self.phase_started:
                self._enter_phase(Phase.ASCEND, now)
            else:
                return self._request(Command.ARM, now)
        if now >= self.phase_started + 20.0:
            self.fail('Takeoff ascent timed out')
            return Output()
        if self._settled(self.target, now):
            self.settled_since = None
            if self.trajectory is None:
                self.state = State.HOLD
            else:
                self.state = State.TRACKING
                self.tracking_started = now
                return self._track(now)
        return Output(self.target)
