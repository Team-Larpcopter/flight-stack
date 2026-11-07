# Simulate takeoff, hold, and trajectory tracking

This Python ROS 2 node waits for the pilot to arm and then switch to Position,
enters Offboard, climbs 1 m above its handoff position, and holds. It is a one-shot SITL prototype for PX4 v1.17
and the pinned `px4_msgs` release. TRACKING follows custom local NED trajectory
samples. LAND hands control to PX4 and DONE confirms touchdown and disarm. HOLD keeps PX4 in
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

After launch, allow at least 2 seconds of heartbeat streaming, arm in SITL,
then switch to Position to authorize Offboard and ascent. The node never arms.
Arming while already in Position requires leaving Position and switching back. Observe `/flight_executive/state` for WAITING, TAKEOFF, then HOLD. Phase
changes and rejection reasons appear in the node logs.

```bash
ros2 topic echo /flight_executive/state
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
```

The target is `(handoff_x, handoff_y, handoff_z - takeoff_height)` in PX4's NED
frame. It preserves yaw at Offboard confirmation. The default height is 1 m, independent of the
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
and heading. It streams the current pose and position-control heartbeat at
20 Hz while waiting for pilot arming, then a subsequent switch into Position.
The switch requires at least 2 seconds of continuous heartbeat; an early switch
is refused terminally. Offboard requests repeat every 0.5 seconds for up to
2 seconds while armed, landed, and in Position. A matching rejection is terminal.
Only observed Offboard confirmation starts ascent and captures its origin.

Heading readiness uses a finite reported heading and PX4's preflight result.
It does not require `heading_good_for_control`, which PX4's magnetometer path
can assert only after its final alignment in flight. A heading reset during
the mission still aborts the executive.

The ascent deadline is 20 seconds from Offboard confirmation. Reaching HOLD requires
1 continuous second within 0.15 m vertically, 0.25 m horizontally, and vertical
speed below 0.2 m/s. HOLD streams its captured target for up to `hold_timeout` (5 seconds by default),
then requests PX4 Land if no valid setpoint arrives.

Status or local-position telemetry older than 1 second, landed telemetry older
than 2.5 seconds (PX4 publishes it only on change or at 1 Hz), invalid position or
heading, estimator resets, a control-timer gap over 0.5 seconds after requesting Offboard,
PX4 failsafe, unexpected disarm, or mode takeover all enter terminal ERROR.
Loss of preflight readiness before Offboard confirmation is also terminal. A timer
gap while waiting for the pilot restarts the 2-second heartbeat warmup. The watchdog uses a
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
string trajectory_id
bool is_final
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
ros2 topic pub --rate 10 /larper/trajectory_in flight_executive_msgs/msg/TrajectorySetpoint "{header: {frame_id: local_ned}, trajectory_id: manual-flight-1, is_final: false, position: {x: 0.0, y: 0.0, z: -1.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}}"
```

The executive repeats the latest accepted position, velocity, and acceleration
unchanged at 20 Hz while its age is at most 1 second. Position control remains
selected; velocity and acceleration are feedforward terms. With `yaw_valid`
false, yaw stays at the HOLD target's yaw and yawspeed is unset. With it true, the
executive forwards the sample's yaw and yawspeed.

If no accepted sample arrives for more than 1 second, TRACKING returns to HOLD.
It captures the current measured position and yaw once and publishes that fixed
pose with zero velocity and acceleration. A new valid sample for the active trajectory can resume tracking during the
5-second HOLD grace period. If none arrives, the executive requests Land.
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
heartbeat preparation, pilot authorization, mode acknowledgments/retries, ascent,
and settling.
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

## Complete a trajectory and land

The custom message also contains `string trajectory_id` and `bool is_final`.
Use one nonempty ID for the entire flight. The first accepted sample claims that
ID; samples from another ID do not replace or refresh the active trajectory.
Set `is_final=true` only for the stationary endpoint, with zero velocity,
acceleration, and yaw rate. Repeat that final sample until the executive reports
LAND. Completion is a property of the common message, not the planner's name.

The executive requires the final position to remain within 0.25 m horizontally
and 0.15 m vertically, with measured 3D speed below 0.2 m/s, for 1 second of fresh
final samples. A changed endpoint, a nonfinal sample, invalid speed, or stale
input resets the dwell. The course publisher supplies a UUID for its process,
marks its COMPLETE endpoint final, and stops streaming when LAND or DONE arrives.

Every HOLD entry starts a monotonic `hold_timeout` deadline. The default is
5 seconds, including after takeoff while waiting for a planner. Stale TRACKING
input first repeats its latest sample through age 1 second, then captures HOLD
at the measured pose; another 5 seconds without accepted input starts LAND.
Receiving valid input before that deadline resumes TRACKING. This timeout lands
even when the planner aborted, so loss of input is not reported as successful
trajectory completion.

LAND sends `VEHICLE_CMD_NAV_LAND` for the current location. It repeats the request
at most three times, 1 second apart, while holding position in Offboard. Accepted
ACKs alone do not complete the handoff. After telemetry confirms PX4 Land mode,
the executive permanently stops its Offboard heartbeat and setpoints for that
mission. It waits for fresh landed and disarmed telemetry, then reports DONE.
No airborne disarm, rearming, or mode reacquisition is attempted. The overall
landing deadline is `landing_timeout`, default 45 seconds. Rejection, timeout,
stale required telemetry, or takeover produces terminal ERROR.

Both launch files expose these deadlines:

```bash
ros2 launch flight_executive dronehacks.launch.py hold_timeout:=5.0 landing_timeout:=45.0
```

`extensions/land.py` owns command retries, mode confirmation, and landing
completion. Landed telemetry remains required during descent; local position
resets after confirmed PX4 handoff or disarm do not invalidate landing success.
This implementation is simulation-only. It does not add the bonus return flight
or persistent mission-frame offsets.

After this interface change, rebuild `flight_executive_msgs`, `flight_executive`,
and `flight_trajectory`, then source the new overlay in every ROS terminal.
