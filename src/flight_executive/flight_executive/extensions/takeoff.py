"""Offboard preparation, arming, and ascent behavior."""

from dataclasses import dataclass
from math import hypot

from flight_executive.model import Command, Output, Phase, Setpoint


@dataclass(frozen=True)
class TakeoffResult:
    output: Output = Output()
    complete: bool = False
    reason: str = ''


class Takeoff:
    """Own takeoff phases without changing mission state."""

    def __init__(self, origin: Setpoint, height: float, now: float):
        self.origin = origin
        self.target = Setpoint(origin.x, origin.y, origin.z - height, origin.yaw)
        self.phase = Phase.PREPARE
        self.phase_started = now
        self.last_command = None
        self.attempts = 0
        self.settled_since = None
        self.reason = ''

    def acknowledge(self, command: int, result: int) -> None:
        pending = {Phase.MODE: Command.OFFBOARD, Phase.ARM: Command.ARM}
        expected = pending.get(self.phase)
        if expected is not None and self.attempts and command == expected.value:
            if result not in (0, 5):
                self.reason = f'PX4 rejected {expected.name}, result={result}'

    def _enter_phase(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self.phase_started = now
        self.last_command = None
        self.attempts = 0

    def _request(self, command: Command, now: float) -> TakeoffResult:
        if self.last_command is None or now >= self.last_command + 1.0:
            if self.attempts == 3:
                return TakeoffResult(reason=(
                    f'{command.name} confirmation timed out after 3 attempts'))
            self.last_command = now
            self.attempts += 1
            return TakeoffResult(Output(self.origin, command))
        return TakeoffResult(Output(self.origin))

    def tick(self, now, status, position, landed, gap=False) -> TakeoffResult:
        """Return takeoff output or a completion/failure result."""
        if self.reason:
            return TakeoffResult(reason=self.reason)
        if self.phase in (Phase.ARM, Phase.ASCEND) and not status.offboard:
            return TakeoffResult(reason='PX4 departed Offboard mode')
        if self.phase == Phase.ASCEND and not status.armed:
            return TakeoffResult(reason='PX4 unexpectedly disarmed')
        if self.phase in (Phase.PREPARE, Phase.MODE):
            if not landed.landed:
                return TakeoffResult(reason='Vehicle left ground before arming request')
            if status.armed:
                return TakeoffResult(reason='Vehicle armed before arming request')
        if not status.armed and not status.preflight_ok:
            return TakeoffResult(reason='PX4 preflight readiness lost before arming')
        if self.phase == Phase.PREPARE:
            if gap:
                self.phase_started = now
            if now >= self.phase_started + 2.0:
                self._enter_phase(Phase.MODE, now)
                return self._request(Command.OFFBOARD, now)
            return TakeoffResult(Output(self.origin))
        if self.phase == Phase.MODE:
            if status.offboard and status.received_at >= self.phase_started:
                self._enter_phase(Phase.ARM, now)
                return self._request(Command.ARM, now)
            return self._request(Command.OFFBOARD, now)
        if self.phase == Phase.ARM:
            if status.armed and status.received_at >= self.phase_started:
                self._enter_phase(Phase.ASCEND, now)
            else:
                return self._request(Command.ARM, now)
        if now >= self.phase_started + 20.0:
            return TakeoffResult(reason='Takeoff ascent timed out')
        p = position
        settled = (abs(p.z - self.target.z) <= 0.15
                   and hypot(p.x - self.target.x, p.y - self.target.y) <= 0.25
                   and abs(p.vz) < 0.2)
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        elif now >= self.settled_since + 1.0:
            return TakeoffResult(Output(self.target), complete=True)
        return TakeoffResult(Output(self.target))
