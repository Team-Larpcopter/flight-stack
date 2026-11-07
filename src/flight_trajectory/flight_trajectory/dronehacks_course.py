"""Hard-coded DroneHacks 2026 course route and obstacle map.

Coordinates are course metres, +X east and +Y north, with (0, 0) at the
south-west corner of the 20 x 20 ft floor. They match the parent repository's
``sim/worlds/dronehacks_course.sdf``, where Gazebo world (0, 0) is that corner.
Wall positions come from ``dronehacks-course-v3.obj`` (Onshape export, metres).

Route: launch -> Point 1 (corridor exit) -> maze -> Point 2 (Gate A) ->
tunnel north to south -> Point 3 (tunnel exit) -> chicane around the diagonal
barrier tip -> Point 4 (Gate B) -> Point 5 (landing pad). The window
alternative and the over-the-barrier return bonus are not flown.
"""

from dataclasses import dataclass

from flight_trajectory.trajectory import Trajectory, Waypoint

FLOOR = 6.096
LAUNCH = (0.5334, 3.9624)
PAD = (1.006, 2.2555)

# Height above the takeoff point. The landed x500 base link sits about 0.2 m
# above the floor, so this flies about 0.95 m above the floor: on the tunnel
# axis (0.94 m) and well above the 0.53 m rocks, under the 1.73 m walls.
CRUISE_HEIGHT = 0.75

TUNNEL_X = 5.3595
TUNNEL_Y = (2.743, 4.267)
TUNNEL_AXIS_HEIGHT = 0.9395
TUNNEL_INNER_RADIUS = 0.73

# Route vertices in course metres. Maze legs follow the red floor guideline,
# which is the corridor centreline. Optional per-corner radii override the
# default where geometry needs a tighter or wider arc.
ROUTE = (
    ('launch', LAUNCH[0], LAUNCH[1], None),
    ('point1_corridor_exit', 0.5334, 5.6325, None),
    ('maze_a', 1.676, 5.6325, None),
    ('maze_b', 1.676, 4.0195, None),
    ('maze_c', 2.794, 4.0195, None),
    ('maze_d', 2.794, 5.6325, None),
    ('tunnel_turn', TUNNEL_X, 5.6325, 0.5),
    ('point3_tunnel_exit', TUNNEL_X, 1.2, 0.4),
    ('chicane_hairpin', 3.245, 1.2, 0.35),
    ('point4_gate_b_approach', 3.894, 2.986, 0.25),
    ('gate_b_exit', 2.75, 2.986, 0.3),
    ('landing_dogleg', 2.4, 1.8, 0.3),
    ('point5_landing_pad', PAD[0], PAD[1], None),
)


@dataclass(frozen=True)
class Wall:
    """Vertical wall: centreline from a to b, half thickness, and height."""

    name: str
    a: tuple
    b: tuple
    half_thickness: float
    height: float


WALL_HEIGHT = 1.727
WALLS = (
    Wall('outer_south', (0.0, 0.0), (FLOOR, 0.0), 0.0, 3.048),
    Wall('outer_east', (FLOOR, 0.0), (FLOOR, FLOOR), 0.0, 3.048),
    Wall('outer_north', (0.0, FLOOR), (FLOOR, FLOOR), 0.0, 3.048),
    Wall('outer_west', (0.0, 0.0), (0.0, FLOOR), 0.0, 3.048),
    Wall('barrier', (0.0, 3.505), (4.623, 3.505), 0.051, WALL_HEIGHT),
    Wall('corridor_maze', (1.1175, 3.556), (1.1175, 5.169), 0.0505, WALL_HEIGHT),
    Wall('maze_centre', (2.235, 4.483), (2.235, FLOOR), 0.051, WALL_HEIGHT),
    Wall('maze_gate_a', (3.353, 3.556), (3.353, 5.169), 0.051, WALL_HEIGHT),
    Wall('landing_divider', (2.0725, 2.426), (2.0725, 3.454), 0.0505, WALL_HEIGHT),
    Wall('chicane_barrier', (4.0215, 1.9195), (4.573, 3.435), 0.05, WALL_HEIGHT),
    Wall('window_wall', (2.3015, 0.0185), (3.205, 2.5005), 0.05, WALL_HEIGHT),
    # Tunnel tube shell outside the bore; the bore is checked separately.
    Wall('tunnel_west', (4.623, TUNNEL_Y[0]), (4.623, TUNNEL_Y[1]), 0.0, 1.854),
    Wall('tunnel_east', (FLOOR, TUNNEL_Y[0]), (FLOOR, TUNNEL_Y[1]), 0.0, 1.854),
)

ROCK_RADIUS = 0.2285
ROCK_HEIGHT = 0.533
ROCKS = ((5.060, 0.580), (3.841, 0.701), (0.487, 0.548), (1.707, 0.670),
         (5.578, 1.494))


def course_waypoints(height: float = CRUISE_HEIGHT):
    """Return the route in course metres at a fixed height above takeoff."""
    return [Waypoint(x, y, height, radius) for _, x, y, radius in ROUTE]


def course_trajectory(max_speed: float = 1.0, max_acceleration: float = 1.0,
                      corner_radius: float = 0.45,
                      height: float = CRUISE_HEIGHT, yaw_mode: str = 'tangent',
                      max_yaw_rate: float = 1.0) -> Trajectory:
    """Return the reference relative to the launch mark (east, north, up).

    The launch mark is the origin, so a constant GPS bias cancels when PX4's
    local position at takeoff is used as the anchor. Tangent yaw faces along
    the path and starts facing north, the spawn heading.
    """
    relative = [Waypoint(w.x - LAUNCH[0], w.y - LAUNCH[1], w.z, w.corner_radius)
                for w in course_waypoints(height)]
    return Trajectory(relative, corner_radius, max_speed, max_acceleration,
                      yaw_mode=yaw_mode, max_yaw_rate=max_yaw_rate)
