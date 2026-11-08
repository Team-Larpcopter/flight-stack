# Hard-coded flight courses

`flight_trajectory` streams a hard-coded, kinematically feasible course
trajectory into the flight executive's `/larper/trajectory_in` input. The DroneHacks course is a
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
stationary endpoint with `is_final=true` and its per-process trajectory UUID.
The executive verifies measured arrival and settling, then requests PX4 Land.
The publisher stops when the executive reports LAND or DONE.

The node stops publishing (ABORTED) if the start checks fail, local position
is stale, the vehicle is more than `max_tracking_error` from the reference, or
the executive enters ERROR. The executive then returns to HOLD at the measured
pose after its 1 second input deadline. After 5 more seconds without accepted
input, it requests Land, or follows its own ERROR behaviour for telemetry faults.

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

The first planner sample must arrive during the executive's 5-second HOLD grace
period. The DroneHacks launch exposes `hold_timeout` and `landing_timeout` for
this behavior. A different planner must publish the same custom message with a
nonempty trajectory ID and a stationary final sample; completion does not depend
on `/course_trajectory/status`. Rebuild both packages after message changes.

## Jetson triangle

`bash jetson/offboard/launch_triangle_tmux.sh --check` checks the deployed
runtime. Omit `--check` to start the same sensor, VIO, DDS, Foxglove and manual
recording panes as the takeoff/hold launcher, plus
`ros2 launch larp_autonomy triangle.launch.py mission_file:=/config/mission.yaml`.
Build `flight_trajectory` and `larp_autonomy` into the deployed runtime first.

Start with the vehicle disarmed. The pilot arms and selects Position to trigger
the existing executive. It climbs 0.75 m, then follows the hardcoded route
`(0, 0) -> (0.75, 0) -> (0.75, 0.75) -> (0, 0)` in metres forward/left from
the last landed position and heading. Each straight leg starts and ends at
rest, with speed capped at 0.2 m/s. Heading stays fixed. The final stationary
sample lets the executive verify arrival and request PX4 Land.

This reuses the surveyed boundary and ground alignment from `mission.yaml`.
Like the takeoff/hold launcher, it does not run obstacle planning or require
map readiness. Stale telemetry or excessive tracking error stops the course;
the existing executive handles loss of trajectory with HOLD, then landing.
The DroneHacks course remains simulation-only.
