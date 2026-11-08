"""Exercise the triangle's hardware gate without connecting to a vehicle."""

import pytest
import rclpy

from flight_trajectory.course_trajectory_node import CourseTrajectoryNode
from flight_trajectory.triangle_course import TriangleFollower


@pytest.mark.parametrize('course,simulation,hardware,allowed', [
    ('triangle', False, True, True),
    ('triangle', True, False, True),
    ('triangle', False, False, False),
    ('triangle', True, True, False),
    ('dronehacks', False, True, False),
])
def test_hardware_requires_explicit_triangle_selection(course, simulation, hardware, allowed):
    rclpy.init(args=['--ros-args', '-p', f'course:={course}',
                     '-p', f'simulation_only:={str(simulation).lower()}',
                     '-p', f'hardware_enabled:={str(hardware).lower()}'])
    node = None
    try:
        if allowed:
            node = CourseTrajectoryNode()
            assert isinstance(node.follower, TriangleFollower)
        else:
            with pytest.raises(ValueError, match='Select simulation'):
                CourseTrajectoryNode()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
