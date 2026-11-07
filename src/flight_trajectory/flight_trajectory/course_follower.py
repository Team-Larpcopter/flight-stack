"""ROS-independent logic that streams a course trajectory into the executive.

The follower anchors the launch-relative course at the vehicle's last landed
local position, starts the clock when the executive reports HOLD, and returns
one PX4 local NED sample per tick. It never commands PX4. If it stops
returning samples, the executive returns to HOLD at the measured pose after
its 1 second input deadline.
"""

from dataclasses import dataclass
from enum import Enum
from math import atan2, cos, hypot, pi, sin
from typing import Optional


class Phase(Enum):
    WAITING = 'WAITING'      # no landed anchor yet, or executive not in HOLD
    TRACKING = 'TRACKING'    # streaming the course
    COMPLETE = 'COMPLETE'    # streaming the final, stationary end point
    ABORTED = 'ABORTED'      # stopped streaming; the executive will hold


@dataclass(frozen=True)
class Pose:
    received_at: float
    x: float
    y: float
    z: float
    heading: float
    valid: bool = True


@dataclass(frozen=True)
class Sample:
    """One local NED setpoint for flight_executive_msgs/TrajectorySetpoint."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: Optional[float] = None
    yawspeed: Optional[float] = None


def wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def enu_to_ned_yaw(yaw: float) -> float:
    """Convert ENU yaw (counter-clockwise from east) to a PX4 heading."""
    return wrap(pi / 2 - yaw)


class CourseFollower:
    """Stream one rest-to-rest trajectory once per process."""

    def __init__(self, trajectory, max_tracking_error: float = 0.4,
                 start_tolerance: float = 0.3, heading_tolerance: float = 0.35,
                 pose_timeout: float = 0.5):
        for name, value in (('max_tracking_error', max_tracking_error),
                            ('start_tolerance', start_tolerance),
                            ('heading_tolerance', heading_tolerance),
                            ('pose_timeout', pose_timeout)):
            if not value > 0:
                raise ValueError(f'{name} must be positive')
        self.trajectory = trajectory
        self.max_error = max_tracking_error
        self.start_tolerance = start_tolerance
        self.heading_tolerance = heading_tolerance
        self.pose_timeout = pose_timeout
        self.phase = Phase.WAITING
        self.reason = ''
        self.pose = None
        self.anchor = None
        self.executive_state = None
        self.started_at = None

    def abort(self, reason: str) -> None:
        if self.phase != Phase.ABORTED:
            self.phase = Phase.ABORTED
            self.reason = reason

    def update_pose(self, pose: Pose, landed: Optional[bool]) -> None:
        self.pose = pose
        # Keep re-anchoring while on the ground; freeze it once airborne.
        if landed and pose.valid and self.phase == Phase.WAITING:
            self.anchor = pose

    def update_executive_state(self, state: str) -> None:
        self.executive_state = state
        if state == 'ERROR' and self.phase in (Phase.TRACKING, Phase.COMPLETE):
            self.abort('Flight executive entered ERROR')

    def local(self, reference) -> Sample:
        """Map a launch-relative east/north/up reference into PX4 local NED."""
        east, north, up = reference.position
        ve, vn, vu = reference.velocity
        ae, an, au = reference.acceleration
        a = self.anchor
        yaw = yawspeed = None
        if reference.yaw is not None:
            yaw, yawspeed = enu_to_ned_yaw(reference.yaw), -reference.yaw_rate
        return Sample((a.x + north, a.y + east, a.z - up), (vn, ve, -vu),
                      (an, ae, -au), yaw, yawspeed)

    def _pose_fresh(self, now: float) -> bool:
        p = self.pose
        return p is not None and p.valid and 0 <= now - p.received_at <= self.pose_timeout

    def _start(self, now: float) -> Optional[Sample]:
        if self.anchor is None:
            self.abort('Executive reached HOLD before a landed anchor was recorded')
            return None
        if not self._pose_fresh(now):
            self.abort('Local position is stale or invalid at HOLD')
            return None
        start = self.local(self.trajectory.sample(0.0))
        p = self.pose
        if (hypot(p.x - start.position[0], p.y - start.position[1]) > self.start_tolerance
                or abs(p.z - start.position[2]) > self.start_tolerance):
            self.abort('HOLD position is not at the trajectory start; '
                       'check takeoff_height matches the course height')
            return None
        if start.yaw is not None and abs(wrap(p.heading - start.yaw)) > self.heading_tolerance:
            self.abort('Vehicle heading is more than 20 degrees from the course start heading')
            return None
        self.phase = Phase.TRACKING
        self.started_at = now
        return start

    def tick(self, now: float) -> Optional[Sample]:
        """Return this tick's sample, or None to publish nothing."""
        if self.phase == Phase.ABORTED:
            return None
        if self.phase == Phase.WAITING:
            if self.executive_state != 'HOLD':
                return None
            return self._start(now)
        if not self._pose_fresh(now):
            self.abort('Local position is stale or invalid')
            return None
        elapsed = now - self.started_at
        sample = self.local(self.trajectory.sample(elapsed))
        p = self.pose
        if (hypot(p.x - sample.position[0], p.y - sample.position[1]) > self.max_error
                or abs(p.z - sample.position[2]) > self.max_error):
            self.abort(f'Tracking error exceeded {self.max_error:.2f} m at t={elapsed:.1f} s')
            return None
        if elapsed >= self.trajectory.duration:
            self.phase = Phase.COMPLETE
        return sample
