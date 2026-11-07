# Simulate takeoff, hold, and trajectory tracking

This Python ROS 2 node automatically enters Offboard, arms, climbs 1 m above its
initial local position, and holds. It is a one-shot SITL prototype for PX4 v1.17
and the pinned `px4_msgs` release. TRACKING follows custom local NED trajectory
samples. LAND remains reserved without an implementation. HOLD keeps PX4 in
Offboard; it does not select PX4's native Hold mode.

## Build in the Dev Container

Run inside the supported non-root Dev Container at `/workspaces/flight-systems`.
Keep build outputs in `ros2_ws` to comply with the parent repository's container
and workspace rules. Do not install ROS or build on the host.

```bash
cd /workspaces/flight-systems
git -C flight_stack_ws submodule update --init --recursive
./scripts/dev doctor
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base log/flight-stack build --base-paths ../flight_stack_ws/src --build-base build/flight-stack --install-base install/flight-stack --packages-up-to flight_executive --symlink-install
source install/flight-stack/local_setup.bash
colcon --log-base log/flight-stack test --base-paths ../flight_stack_ws/src --build-base build/flight-stack --install-base install/flight-stack --packages-select flight_executive
colcon test-result --test-result-base build/flight-stack --verbose
```

`src/px4_msgs` in the flight-stack repository is a Git submodule pinned to
`86d8239e962f6939e05c3737784f60c02fa884db`, matching the parent simulator's
PX4 v1.17 baseline. The Python import is `px4_msgs`, not `px4_messages`.
Build it with colcon and source the resulting overlay before running the node;
the source checkout alone does not provide the generated Python messages.

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

Status or local-position telemetry older than 1 second, landed telemetry older
than 2.5 seconds (PX4 publishes it only on change or at 1 Hz), invalid position or
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

## Publish tracking setpoints

After takeoff reaches HOLD, publish `flight_executive_msgs/msg/TrajectorySetpoint`
on `/larper/trajectory_in`. A valid fresh sample automatically enters TRACKING.
Samples received in WAITING, TAKEOFF, or ERROR are discarded rather than cached.
The input topic is configurable with the `trajectory_topic` launch argument.

The custom message contains these fields:

```text
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool yaw_valid
float32 yaw
float32 yawspeed
```

Set `header.frame_id` to `local_ned`. All vectors use exactly PX4's local NED
frame and estimator origin, in metres, metres/second, and metres/second squared.
There is no axis conversion, sign flip, origin translation, or TF lookup.
Position is absolute in this local frame, not an offset from takeoff.
The source stamp is informational; freshness uses monotonic receipt time.
All nine vector components must be finite and representable as PX4 float32
values. When `yaw_valid` is true, `yaw` (PX4 heading in radians, clockwise from
north) and `yawspeed` (rad/s feedforward) must be too; otherwise they are ignored.
Invalid samples are ignored and do not refresh the accepted sample's deadline.

For example, after HOLD, stream a stationary target one metre above PX4's local
origin. Adjust the position to match the current local origin before publishing.

```bash
ros2 topic pub --rate 10 /larper/trajectory_in flight_executive_msgs/msg/TrajectorySetpoint "{header: {frame_id: local_ned}, position: {x: 0.0, y: 0.0, z: -1.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}}"
```

The executive repeats the latest accepted position, velocity, and acceleration
unchanged at 20 Hz while its age is at most 1 second. Position control remains
selected; velocity and acceleration are feedforward terms. With `yaw_valid`
false, yaw stays at the HOLD target's yaw and yawspeed is unset. With it true, the
executive forwards the sample's yaw and yawspeed.

If no accepted sample arrives for more than 1 second, TRACKING returns to HOLD.
It captures the current measured position and yaw once and publishes that fixed
pose with zero velocity and acceleration. A new valid sample can resume tracking.
Existing telemetry, estimator-reset, mode-takeover, disarm, and failsafe checks
apply during tracking. Tracking never requests arming or a mode change.

## Fly the hard-coded DroneHacks course

`ros2 launch flight_executive dronehacks.launch.py` starts this executive with
`takeoff_height` equal to the course cruise height, plus the
`flight_trajectory` course node that publishes on `/larper/trajectory_in`. See
[flight_trajectory](../flight_trajectory/README.md) for setup, arguments, and
behaviour.

## Code structure

`flight_executive/state_machine.py` owns mission transitions and shared health
checks. `model.py` contains ROS-independent values. `extensions/takeoff.py` owns
preparation, mode and arm acknowledgments/retries, ascent, and settling.
`extensions/tracking.py` owns the latest trajectory and its receipt deadline.
`flight_executive_node.py` handles ROS subscriptions and PX4 publication.
The separate `flight_executive_msgs` package generates the input message.

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
