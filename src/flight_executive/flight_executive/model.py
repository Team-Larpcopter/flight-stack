"""ROS-independent telemetry, mission states, and setpoint values."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
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
    velocity: tuple = (float("nan"),) * 3
    acceleration: tuple = (float("nan"),) * 3


@dataclass(frozen=True)
class Trajectory:
    received_at: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class Output:
    setpoint: Optional[Setpoint] = None
    command: Optional[Command] = None
