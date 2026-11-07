"""ROS-independent, one-shot takeoff and hold controller."""

from dataclasses import dataclass
from enum import Enum
from math import hypot, isfinite
from typing import Optional


class State(Enum):
    WAITING = 'WAITING'
    TAKEOFF = 'TAKEOFF'
    HOLD = 'HOLD'
    TRACKING = 'TRACKING'
    LAND = 'LAND'
    ERROR = 'ERROR'


class Phase(Enum):
    PREPARE = 'PREPARE'
    MODE = 'MODE'
    ARM = 'ARM'
    ASCEND = 'ASCEND'


class Command(Enum):
    OFFBOARD = 176
    ARM = 400


@dataclass(frozen=True)
class Status:
    received_at: float
    armed: bool
    offboard: bool
    preflight_ok: bool
    failsafe: bool = False


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


@dataclass(frozen=True)
class Output:
    setpoint: Optional[Setpoint] = None
    command: Optional[Command] = None


class Controller:
    """Own mission state; callers provide monotonic time and telemetry."""

    def __init__(self, started_at: float, takeoff_height: float = 1.0):
        if not isfinite(takeoff_height) or takeoff_height <= 0:
            raise ValueError('takeoff_height must be finite and positive')
        self.height = takeoff_height
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
        return snapshot is not None and 0 <= now - snapshot.received_at <= 1.0

    def _request(self, command: Command, now: float) -> Output:
        if self.last_command is None or now >= self.last_command + 1.0:
            if self.attempts == 3:
                self.fail(f'{command.name} confirmation timed out after 3 attempts')
                return Output()
            self.last_command = now
            self.attempts += 1
            return Output(self.origin, command)
        return Output(self.origin)

    def tick(self, now: float) -> Output:
        """Advance once and return this tick's publications, if any."""
        if self.state == State.ERROR:
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
        elif self.phase in (Phase.ARM, Phase.ASCEND) and not self.status.offboard:
            self.fail('PX4 departed Offboard mode')
        elif self.phase == Phase.ASCEND and not self.status.armed:
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
        p = self.position
        settled = (abs(p.z - self.target.z) <= 0.15
                   and hypot(p.x - self.target.x, p.y - self.target.y) <= 0.25
                   and abs(p.vz) < 0.2)
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        elif now >= self.settled_since + 1.0:
            self.state = State.HOLD
        return Output(self.target)
