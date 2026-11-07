"""Pilot-authorized Offboard preparation and ascent behavior."""

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
        self.height = height
        self.heartbeat_started = now
        self.origin = origin
        self.target = Setpoint(origin.x, origin.y, origin.z - height, origin.yaw)
        self.phase = Phase.PREPARE
        self.phase_started = now
        self.last_command = None
        self.attempts = 0
        self.settled_since = None
        self.reason = ''

    def acknowledge(self, command: int, result: int) -> None:
        if (self.phase == Phase.MODE and self.attempts
                and command == Command.OFFBOARD.value and result not in (0, 5)):
            self.reason = f'PX4 rejected OFFBOARD, result={result}'

    def _enter_phase(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self.phase_started = now
        self.last_command = None
        self.attempts = 0

    def _request(self, now: float) -> TakeoffResult:
        if now >= self.phase_started + 2.0:
            return TakeoffResult(reason='OFFBOARD confirmation timed out after 2 seconds')
        if self.last_command is None or now >= self.last_command + 0.5:
            self.last_command = now
            self.attempts += 1
            return TakeoffResult(Output(self.origin, Command.OFFBOARD))
        return TakeoffResult(Output(self.origin))

    def tick(self, now, status, position, landed, gap=False) -> TakeoffResult:
        """Wait for the pilot's arm then Position switch before requesting control."""
        if self.reason:
            return TakeoffResult(reason=self.reason)
        if self.phase == Phase.ASCEND:
            if not status.offboard:
                return TakeoffResult(reason='PX4 departed Offboard mode')
            if not status.armed:
                return TakeoffResult(reason='PX4 unexpectedly disarmed')
        else:
            if not landed.landed:
                return TakeoffResult(reason='Vehicle left ground before Offboard confirmation')
            if not status.preflight_ok:
                return TakeoffResult(reason='PX4 preflight readiness lost before Offboard')
            self.origin = Setpoint(position.x, position.y, position.z, position.yaw)
            if gap:
                self.heartbeat_started = now
            if self.phase == Phase.PREPARE:
                self._enter_phase(Phase.WAIT_ARM, now)
            if self.phase == Phase.WAIT_ARM:
                if status.offboard:
                    return TakeoffResult(reason='Unexpected Offboard before pilot authorization')
                if status.armed:
                    self._enter_phase(Phase.ARMED_IN_POSITION if status.in_position
                                      else Phase.WAIT_POSITION, now)
                return TakeoffResult(Output(self.origin))
            if not status.armed:
                return TakeoffResult(reason='PX4 disarmed before Offboard confirmation')
            if self.phase == Phase.ARMED_IN_POSITION:
                if status.offboard:
                    return TakeoffResult(reason='Unexpected Offboard before pilot authorization')
                if not status.in_position:
                    self._enter_phase(Phase.WAIT_POSITION, now)
                return TakeoffResult(Output(self.origin))
            if self.phase == Phase.WAIT_POSITION:
                if status.offboard:
                    return TakeoffResult(reason='Unexpected Offboard before pilot authorization')
                if not status.in_position:
                    return TakeoffResult(Output(self.origin))
                if now < self.heartbeat_started + 2.0:
                    return TakeoffResult(reason=
                        'Position switch before continuous heartbeat warmup')
                self._enter_phase(Phase.MODE, now)
                return self._request(now)
            if status.offboard and status.received_at >= self.phase_started:
                self.target = Setpoint(self.origin.x, self.origin.y,
                                       self.origin.z - self.height, self.origin.yaw)
                self._enter_phase(Phase.ASCEND, now)
            elif not status.in_position:
                return TakeoffResult(reason='Pilot changed mode before Offboard confirmation')
            else:
                return self._request(now)
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
