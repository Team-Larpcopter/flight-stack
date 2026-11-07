"""Bounded handoff from Offboard control to PX4's current-position Land mode."""

from dataclasses import dataclass
from enum import Enum

from flight_executive.model import Command, Output


class LandPhase(Enum):
    REQUEST = 'REQUEST_LAND'
    DESCEND = 'DESCEND'


@dataclass(frozen=True)
class LandResult:
    output: Output = Output()
    complete: bool = False
    reason: str = ''


class Land:
    """Request Land once confirmed, then observe touchdown and disarm."""

    def __init__(self, target, now, timeout=45.0):
        self.target = target
        self.started_at = now
        self.timeout = timeout
        self.phase = LandPhase.REQUEST
        self.last_command = None
        self.attempts = 0
        self.reason = ''

    def acknowledge(self, command, result):
        if (self.phase == LandPhase.REQUEST and self.attempts
                and command == Command.LAND.value and result not in (0, 5)):
            self.reason = f'PX4 rejected LAND, result={result}'

    def tick(self, now, status, landed):
        if landed.landed and not status.armed:
            return LandResult(complete=True)
        if self.reason:
            return LandResult(reason=self.reason)
        if now >= self.started_at + self.timeout:
            return LandResult(reason='Landing timed out')
        if not status.armed:
            return LandResult(reason='PX4 disarmed before touchdown')
        if status.auto_land:
            self.phase = LandPhase.DESCEND
            return LandResult()
        if self.phase == LandPhase.DESCEND:
            return LandResult(reason='PX4 departed Land mode')
        if not status.offboard:
            return LandResult(reason='PX4 mode changed before Land confirmation')
        if self.last_command is None or now >= self.last_command + 1.0:
            if self.attempts == 3:
                return LandResult(reason='LAND confirmation timed out after 3 attempts')
            self.last_command = now
            self.attempts += 1
            return LandResult(Output(self.target, Command.LAND))
        return LandResult(Output(self.target))
