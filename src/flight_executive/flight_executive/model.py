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
    DONE = 'DONE'
    ERROR = 'ERROR'


class Phase(Enum):
    PREPARE = 'PREPARE'
    MODE = 'MODE'
    WAIT_ARM = 'WAIT_ARM'
    ARMED_IN_POSITION = 'ARMED_IN_POSITION'
    WAIT_POSITION = 'WAIT_POSITION'
    ASCEND = 'ASCEND'


class Command(Enum):
    OFFBOARD = 176
    LAND = 21


@dataclass(frozen=True)
class Status:
    received_at: float
    armed: bool
    offboard: bool
    preflight_ok: bool
    failsafe: bool = False
    auto_land: bool = False
    in_position: bool = False


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
    vx: float = 0.0
    vy: float = 0.0
    velocity_valid: bool = True

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
    yawspeed: float = float("nan")


@dataclass(frozen=True)
class Trajectory:
    received_at: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: Optional[float] = None
    yawspeed: Optional[float] = None
    trajectory_id: str = ''
    is_final: bool = False


@dataclass(frozen=True)
class Output:
    setpoint: Optional[Setpoint] = None
    command: Optional[Command] = None
