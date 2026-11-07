# Simulate automatic takeoff and hold

This Python ROS 2 node automatically enters Offboard, arms, climbs 1 m above its
initial local position, and holds. It is a one-shot SITL prototype for PX4 v1.17
and the pinned `px4_msgs` release. HOLD keeps PX4 in Offboard; it does not select
PX4's native Hold mode. With a `course` parameter it skips HOLD, tracks a
hard-coded trajectory (TRACKING), then requests PX4 Land (LAND) and finishes in
DONE once PX4 reports landed and disarmed.

## DroneHacks course trajectory (SITL, GPS on)

Start `./scripts/dev run comp --headless` (GPS on), configure the Offboard-loss
parameters below, then:

```bash
ros2 launch flight_executive dronehacks.launch.py            # 1.0 m/s cap
ros2 launch flight_executive dronehacks.launch.py max_speed:=0.5
```

`flight_executive/dronehacks_course.py` holds the route (launch, corridor,
maze, Gate A, tunnel, chicane, Gate B, pad) and the wall map taken from the
course mesh. `flight_executive/trajectory.py` turns waypoints into straight
lines joined by arcs, applies speed and acceleration limits, and smooths the
result in time so acceleration is continuous and jerk is bounded. Each tick
publishes position with velocity and acceleration feedforward, plus yaw and
yawspeed feedforward.

Yaw defaults to `yaw_mode:=tangent`: the vehicle faces along the path, the
speed profile keeps yaw rate under `max_yaw_rate` (default 1.0 rad/s), and the
same smoothing makes yaw rate continuous. The route starts facing north; the
executive refuses to arm if the vehicle heading is more than 20 degrees off.
`yaw_mode:=fixed` holds the takeoff heading instead.

The route is anchored to PX4's local position at takeoff and assumes the
vehicle starts on the launch mark with course north equal to true north. A
horizontal or vertical error over `max_tracking_error` (default 0.4 m) enters
ERROR, and PX4's Offboard-loss action lands. Route clearance is only about
0.07 to 0.15 m from the prop tips at Gate A, Gate B, and maze wall ends; this
needs centimetre-level localization and is not a plan for a real GPS receiver.

## Build in the Dev Container

Run inside the supported non-root Dev Container at `/workspaces/flight-systems`.
Keep build outputs in `ros2_ws` to comply with the parent repository's container
and workspace rules. Do not install ROS or build on the host.

```bash
cd /workspaces/flight-systems
./scripts/dev doctor
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src/px4_msgs ../flight_stack_ws/src/flight_executive --packages-up-to flight_executive --symlink-install
source install/local_setup.bash
colcon test --base-paths src/px4_msgs ../flight_stack_ws/src/flight_executive --packages-select flight_executive
colcon test-result --verbose
```

## Run against SITL

Start one PX4/Gazebo/XRCE workflow using the parent repository's `./scripts/dev`
interface. Inspect `./scripts/dev status` first to avoid competing workflows.
Have QGroundControl or the configured ground-control connection running. PX4's
default arming policy may refuse arming without RC or a ground station.

```bash
./scripts/dev status
./scripts/dev run sitl-ros --headless
```

Before starting this node, use the **SITL PX4 console** to configure and verify
Offboard-loss landing. These settings are for this simulator instance only.

```text
param set COM_OBL_RC_ACT 4
param set COM_OF_LOSS_T 1
param show COM_OBL_RC_ACT
param show COM_OF_LOSS_T
```

With a disarmed, landed SITL vehicle and valid local position, launch from a
ROS-ready container shell with this workspace's overlay sourced:

```bash
ros2 launch flight_executive simulation.launch.py
```

Launching immediately enables automatic arming and ascent once readiness checks
pass. Observe `/flight_executive/state` for WAITING, TAKEOFF, then HOLD. Phase
changes and rejection reasons appear in the node logs.

```bash
ros2 topic echo /flight_executive/state
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
```

The target is `(initial_x, initial_y, initial_z - takeoff_height)` in PX4's NED
frame. It preserves initial yaw. The default height is 1 m, independent of the
estimator's initial z origin; it is not a terrain-relative altitude estimate.
PX4's position controller supplies ascent limits and smoothing.

```bash
ros2 launch flight_executive simulation.launch.py takeoff_height:=1.0 px4_topic_prefix:=/fmu
```

The node parameters `target_system`, `target_component`, `source_system`, and
`source_component` default to 1, 1, 255, and 191. All IDs must be in 1..255. The
node derives DDS topic suffixes from `MESSAGE_VERSION` in the installed message
types. Telemetry subscriptions use best-effort, volatile QoS.

The `simulation_only` parameter defaults to false; the simulation launch file
sets it to true. This is an operator acknowledgement, not a reliable way to
distinguish simulator telemetry from a physical vehicle. Keep this ROS domain
and XRCE agent isolated from hardware. No hardware launch path is provided.

## State and failure behavior

The controller waits up to 30 seconds for fresh disarmed/landed telemetry,
passing preflight checks, and finite valid local position, vertical velocity,
and heading. It streams the initial pose and position-control heartbeat at
20 Hz for 2 seconds, requests Offboard, waits for status confirmation, requests
arming, and waits for armed status before commanding ascent. Each command has
three attempts separated by 1 second; any matching rejection is terminal.
Acknowledgement alone never confirms mode entry or arming.

Heading readiness uses a finite reported heading and PX4's preflight result.
It does not require `heading_good_for_control`, which PX4's magnetometer path
can assert only after its final alignment in flight. A heading reset during
the mission still aborts the executive.

The ascent deadline is 20 seconds from armed confirmation. Reaching HOLD requires
1 continuous second within 0.15 m vertically, 0.25 m horizontally, and vertical
speed below 0.2 m/s. HOLD streams the final target indefinitely.

Status or local-position telemetry older than 1 second, landed telemetry older than
2.5 seconds (PX4 publishes it at 1 Hz), invalid position or
heading, estimator resets, a control-timer gap over 0.5 seconds after preparation,
PX4 failsafe, unexpected disarm, or mode takeover all enter terminal ERROR.
Loss of preflight readiness before arming is also terminal. A timer gap during initial
preparation restarts the 2-second preparation interval. The watchdog uses a
steady clock even when ROS simulated time pauses.

ERROR stops heartbeat, setpoints, and command publications. The node never
re-arms, reacquires Offboard, force-arms, or commands airborne disarm. PX4 owns
recovery using its verified Offboard-loss policy. Terminating the node likewise
stops the heartbeat and should trigger the configured PX4 landing response.
A new process is a new mission; do not restart it to recover an airborne vehicle.
For another simulator run after landing and disarming, select PX4 Hold before
restarting the node (`commander mode auto:loiter` in the SITL console). A previous
Offboard intention without a heartbeat, or Position mode without RC input, can
prevent preflight readiness.

## Hardware prerequisites

PX4 supports the same Offboard heartbeat, position-setpoint, and arm-command
interfaces on hardware. An RC arming-switch flip is not universally required.
A fitted safety button and PX4's health checks still apply. RC availability and
loss behavior depend on configuration; this prototype does not change them or
bypass safety checks.

Indoor position control needs a valid local estimate, for example motion capture
or VIO. Hardware flight also needs an agreed, tested operator takeover method,
vehicle-specific arming review, ceiling clearance, and verified loss behavior.
Hardware operation is outside this prototype's validation scope.

See PX4's [Offboard requirements](https://docs.px4.io/v1.17/en/flight_modes/offboard),
[ROS 2 example](https://docs.px4.io/v1.17/en/ros2/offboard_control), and
[arming configuration](https://docs.px4.io/v1.17/en/advanced_config/prearm_arm_disarm).
