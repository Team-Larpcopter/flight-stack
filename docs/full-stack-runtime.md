# Standalone Jetson flight runtime

The public Ubuntu 22.04 / ROS 2 Humble image now includes Micro XRCE-DDS Agent
v2.4.3 (commit `73622810d984349b80bbac0ef55fc0b694d62222`). Its installation is
appended after the existing Dockerfile steps, preserving their cache. Agent
Fast DDS libraries live under `/opt/xrce-agent`; its wrapper loads them only for
the agent process, without changing ROS middleware library resolution.
[PX4 v1.16's guide](https://docs.px4.io/v1.16/en/ros2/user_guide#setup-the-agent)
uses this agent release. Vendored `px4_msgs` must match the controller firmware.

## Build on Jetson

From this repository, on the Jetson:

```bash
git pull origin main
bash jetson/build.sh
```

Docker reuses existing layers and builds the new agent layer natively.
The agent, DepthAI SDK and C++ workspace packages use `Release` builds. The SDK
and colcon builds are incremental in this checkout. No PC-built binaries, old
integration workspace, organizer image or running container is required.

## Launch graph

`jetson/run_flight_stack.sh` runs one foreground container with host networking
and IPC. Ctrl+C stops the group. An unexpected process exit also stops the group;
PX4 then applies its configured offboard-loss behavior. This helper does not
restart or stop other camera drivers, agents or flight stacks: stop the old owner
before starting it. Only one agent may own the selected transport.

The launch includes:

1. XRCE-DDS Agent, connected over UDP or explicitly mapped serial.
2. Native OAK driver and sqrtVINS, with bundled camera calibration/raw aligned IMU.
3. Body odometry, static CAD mapper and SUPER A* / trajectory planning.
4. A latched goal publisher, using the first observed map localization epoch.
5. Polynomial sampling bridge and planner visualization topics.
6. In `hardware` mode only: external vision to PX4 and the flight executive.

The executive is the sole owner of offboard heartbeat, setpoints and landing
commands. In hardware mode, pilot arming followed by a switch into Position
authorizes a software Offboard request, then automatic takeoff after confirmation; the executive hands control to planner
samples in HOLD/TRACKING and lands after certified arrival or its hold timeout.
The launch itself does not send an arm command.

`shadow` launches no external-vision or executive command publisher. It still
connects the agent and sensors, and samples planner output on the shadow topic.
`start_localization:=false` attaches to an existing camera/sqrtVINS pair instead.
It does not disable the body-odometry adapter; do not run another one concurrently.

## Inputs and frames

Paths passed to the launch refer to this checkout mounted at `/flight-stack`.
Keep your deployment configuration under this checkout (for example ignored
`deployment/`), not in an old integration directory. These are physical setup
values, not missing software dependencies:

- `rig_file`: measured `T_body_imu` and camera geometry for the mapper. The
  body-odometry adapter consumes the same rig as the planner.
- `boundary_config`: ROS parameter YAML for `depth_map`, including the actual
  immutable course enclosure. See [planner integration](planner-executive-integration.md).
- `goal_x`, `goal_y`, `goal_z` (and optional `goal_yaw`): intended destination in map.
- Hardware additionally requires `bridge_params`: the existing
  [PX4 bridge YAML](../src/larp_px4_bridge/README.md), with the measured IMU-to-body
  FRD quaternion and appropriate estimator reset counter. It must agree with the
  FLU body transform in `rig_file`.
- `initial_map_x/y/z`, `initial_map_yaw`: known starting body pose in the CAD map.
- Hardware requires `alignment_verified:=true`, a unique `mission_id`, and
  surveyed `map_heading_ned`, `map_origin_ned_x/y/z` for the planner/PX4 transform.

Bundled navigation, course CAD and estimator files are defaults; each
can be explicitly overridden. The goal remains tied to the initial epoch. A
localization reset needs a deliberate mission restart and updated EV reset counter.
No CAD pose matching or automatic map/PX4 alignment is performed.

## Commands

For a UDP connection, configure the PX4 uXRCE client to target this Jetson on
port 8888 (or `PX4_PORT`). Set `ROS_DOMAIN_ID` to match PX4's `UXRCE_DDS_DOM_ID`.
For a serial connection, set `PX4_TRANSPORT=serial`, `PX4_DEVICE` to the host
UART/USB serial path, and `PX4_BAUD` to its configured baud rate. Only this device
is mapped into the container as `/dev/px4`; no privileged container is used.

After supplying your measured deployment files and actual goal coordinates:

```bash
export ROS_DOMAIN_ID=0   # replace with the controller's configured domain
export PX4_TRANSPORT=serial
export PX4_DEVICE=/dev/serial/by-id/YOUR_CONTROLLER
export PX4_BAUD=921600

bash jetson/run_flight_stack.sh shadow \
  rig_file:=/flight-stack/deployment/rig.json \
  boundary_config:=/flight-stack/deployment/boundary.yaml \
  goal_x:=1.0 goal_y:=0.0 goal_z:=1.0
```

The coordinates above demonstrate syntax, not a validated course goal. For
execution, use your actual goal and surveyed transform values:

```bash
bash jetson/run_flight_stack.sh hardware \
  rig_file:=/flight-stack/deployment/rig.json \
  boundary_config:=/flight-stack/deployment/boundary.yaml \
  bridge_params:=/flight-stack/deployment/bridge.yaml \
  goal_x:="$GOAL_X" goal_y:="$GOAL_Y" goal_z:="$GOAL_Z" \
  initial_map_x:="$START_X" initial_map_y:="$START_Y" initial_map_z:="$START_Z" \
  initial_map_yaw:="$START_YAW" \
  alignment_verified:=true mission_id:="$MISSION_ID" \
  map_heading_ned:="$MAP_HEADING_NED" \
  map_origin_ned_x:="$MAP_ORIGIN_NED_X" \
  map_origin_ned_y:="$MAP_ORIGIN_NED_Y" \
  map_origin_ned_z:="$MAP_ORIGIN_NED_Z"
```

Alternatively source ROS and this checkout's overlay inside a properly configured
container and use `ros2 launch larp_autonomy full_stack.launch.py` with the same
arguments. The host helper supplies transport arguments and USB/serial access.
`BUILD_IMAGE`, `CONTAINER`, and optional `DDS_PROFILE` remain configurable.

PX4 firmware, its uXRCE client, external-vision fusion settings, sensor lever arm,
and offboard-loss failsafe must already be configured. This launch supplies the
companion-side communication and execution path; it does not flash firmware or
silently change flight-controller parameters. The new full launch has not been
qualified by an actual flight.

For the dynamic 5 cm map and eight-pane hardware/shadow launchers, use the
[full mission runtime](full-mission-tmux.md).
