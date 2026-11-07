# Fly the hard-coded DroneHacks course

`flight_trajectory` streams a hard-coded, kinematically feasible course
trajectory into the flight executive's `/larper/trajectory_in` input. It is a
SITL prototype for the parent repository's `comp` world with GPS on. The
executive still owns Offboard entry, arming, takeoff, HOLD, and every safety
check; this package never commands PX4.

## Run in the Dev Container

Start the GPS-on course world from the parent repository, then set PX4's
Offboard-loss landing for that simulator instance (see the executive README).

```bash
cd /workspaces/flight-systems
./scripts/dev status
./scripts/dev run comp
B=src/PX4-Autopilot/build/px4_sitl_default/bin
$B/px4-param set COM_OBL_RC_ACT 4
$B/px4-param set COM_OF_LOSS_T 1
```

Build this workspace's packages into `ros2_ws` and launch both nodes:

```bash
cd /workspaces/flight-systems/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base log/flight-stack build --base-paths ../flight_stack_ws/src --build-base build/flight-stack --install-base install/flight-stack --packages-up-to flight_executive --symlink-install
source install/flight-stack/local_setup.bash
ros2 launch flight_executive dronehacks.launch.py max_speed:=0.5
```

Use the path of your flight-stack checkout in `--base-paths`. The launch file
lives in `flight_executive` and starts `flight_executive_node` with `takeoff_height` equal to the course
cruise height (0.75 m) and `course_trajectory_node`. Arguments: `max_speed`
(1.0 m/s), `yaw_mode` (`tangent` or `fixed`), `max_yaw_rate` (1.0 rad/s),
`max_tracking_error` (0.4 m), `cruise_height`, `trajectory_topic`, and
`px4_topic_prefix`. Watch progress with:

```bash
ros2 topic echo /flight_executive/state
ros2 topic echo /course_trajectory/status
```

## Behaviour

The node records the last valid local position while PX4 reports landed. That
pose is the launch mark: the course is defined relative to it, so a constant
GPS bias cancels. When the executive reports HOLD, the node checks the vehicle
is within 0.3 m of the trajectory start and, with tangent yaw, within 20
degrees of its start heading (north). It then publishes one
`flight_executive_msgs/TrajectorySetpoint` at 50 Hz in PX4 local NED, with
position, velocity, acceleration, and optional yaw and yawspeed. The first
sample switches the executive to TRACKING. At the end it keeps publishing the
stationary end point above the landing pad. Landing is not automated; use
PX4 Land from QGroundControl.

The node stops publishing (ABORTED) if the start checks fail, local position
is stale, the vehicle is more than `max_tracking_error` from the reference, or
the executive enters ERROR. The executive then returns to HOLD at the measured
pose after its 1 second input deadline, or follows its own ERROR behaviour.

## Trajectory

`dronehacks_course.py` holds the route in course metres (+X east, +Y north,
origin at the floor's south-west corner, matching `sim/worlds/dronehacks_course.sdf`)
and the wall map taken from the course mesh: launch, first corridor, maze,
Gate A, the tunnel bore, the chicane, Gate B, and the landing pad.

`trajectory.py` joins the waypoints with straight lines and circular arcs,
limits speed, tangential and lateral acceleration, and yaw rate along the
path, then convolves the reference in time with a 0.8 s raised-cosine window.
Acceleration and yaw rate are continuous and jerk is bounded (about 5 m/s^3 at
the defaults), so the reference is feasible for a multicopter. Tangent yaw
faces along the path. Tests check limits, smoothness, and that the reference
keeps more than 0.40 m from every wall (the x500 reaches 0.39 m).

Route clearance at Gate A, Gate B, and maze wall ends is only about 0.07 to
0.15 m from the prop tips. One SITL run at 1 m/s clipped a maze wall end, so
start at `max_speed:=0.5`. This needs centimetre-level localization and is not
a plan for a real GPS receiver or vehicle.
