"""Run the triangle samples through the existing executive landing contract."""

from flight_executive.model import Command, Landed, Position, Setpoint, State, Status, Trajectory
from flight_executive.state_machine import StateMachine
from flight_trajectory.course_follower import Pose
from flight_trajectory.triangle_course import TriangleCourse, TriangleFollower


def test_triangle_requests_land_only_after_return_and_settle():
    course = TriangleCourse()
    follower = TriangleFollower(course)
    follower.update_pose(Pose(0., 3., -2., 7., .8), True)
    follower.update_pose(Pose(0., 3., -2., 6.5, .8), False)
    follower.update_executive_state('HOLD')
    sample = follower.tick(0.)
    machine = StateMachine(0., takeoff_height=.5, hold_timeout=2.)
    machine.state = State.HOLD
    machine.hold_started = 0.
    machine.hold_target = Setpoint(*sample.position, .8)
    now = 0.
    while now < course.duration + 2.:
        now += .02
        follower.update_pose(Pose(now, *sample.position, .8), False)
        follower.update_executive_state(machine.state.value)
        sample = follower.tick(now)
        assert sample is not None
        machine.status = Status(now, True, True, True)
        machine.position = Position(now, *sample.position, .8, sample.velocity[2],
                                    vx=sample.velocity[0], vy=sample.velocity[1])
        machine.resets = machine.position.reset_counters
        machine.landed = Landed(now, False)
        machine.accept_trajectory(Trajectory(
            now, sample.position, sample.velocity, sample.acceleration,
            sample.yaw, sample.yawspeed, 'triangle-test', sample.is_final))
        output = machine.tick(now)
        if output.command == Command.LAND:
            assert now >= course.duration + 1.
            assert machine.state == State.LAND
            return
        assert machine.state == State.TRACKING
    assert False, 'Executive did not request Land after the triangle settled at home'
