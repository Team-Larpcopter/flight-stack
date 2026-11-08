# EGO as an alternative planner

`larp_ego_planner` vendors the upstream-recommended
[ROS 2 EGO-Swarm port](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/23a8d5a191711dd65633df689bd00f55d4dea8f9)
in single-drone mode. The original
[EGO-Planner repository](https://github.com/ZJU-FAST-Lab/ego-planner) is ROS 1.
The revision, original file hashes, licenses and local adaptations are recorded
in [third-party notices](../src/larp_ego_planner/third_party/NOTICES.md).
The upstream FSM, controller, simulators and swarm transport are excluded.

SUPER remains the default. EGO can now feed the existing flight executive through
`CommittedTrajectory` and `trajectory_bridge`, with the same mission and safety
gates. This is a software integration, not flight qualification. No real vehicle
or Jetson hardware was operated to validate this change.

## Full mission entry points

Build and transfer the supported runtime bundle first, using the existing
[offboard workflow](full-mission-tmux.md). On the Jetson:

```bash
export DEPLOYMENT_CONFIG=$HOME/flight-stack/config
# Validate configuration and installed libraries without starting the mission:
bash ~/flight-stack/deployment/arm64/launch_ego_mission_tmux.sh --check
# Full sensing/planning stack with execution disabled:
bash ~/flight-stack/deployment/arm64/launch_ego_full_system_tmux.sh
# Or the real mission, with the existing pilot arming/Position-mode start gates:
bash ~/flight-stack/deployment/arm64/launch_ego_mission_tmux.sh
```

The entry points create one tiled tmux session with nine panes: Drivers, Foxglove,
sqrtVINS, DDS, LAS, ROG map status, EGO, Mission executive and a manual recorder.
They use the same `mission.yaml`, calibrated rig, surveyed boundaries, goal and
PX4 bridge parameters as SUPER. Hardware mode retains verified alignment,
ground-start checks, takeoff readiness, pilot arming and Position-mode transition,
HOLD/TRACKING gating and certificate freshness. Following the current `main`
configuration, the tmux mission sets `enforce_boundary=False` in the executive;
the planner's hard map boundary and flight envelope remain enabled.
Shadow mode starts no flight executive or external-vision injection, and the
trajectory bridge publishes only `/autonomy/shadow/trajectory_in`. It still
starts sensors and the DDS agent.

The default session is `flight-stack-ego-hardware` or `flight-stack-ego-shadow`.
Explicit launches replace the known competing stack containers/sessions; switch
only while landed and disarmed. `--check` does not replace them. See the
[full mission guide](full-mission-tmux.md#select-ego-for-the-full-mission) for
runtime extraction, custom names, transport and deployment options.

To return to SUPER, use the existing `launch_mission_tmux.sh` or
`launch_full_system_tmux.sh`. The shared launcher also accepts
`launch_system_tmux.sh hardware --planner ego`. The new scripts are included in
both bundle packaging and transfer.

## ROS launch selection

`mission_runtime.launch.py component:=planner planner:=ego` starts the combined
mapper/EGO component; pass the same `mode` and `mission_file` as the other panes.
`planner_mission.launch.py`, `profile_pipeline.launch.py` and
`full_stack.launch.py` also accept `planner:=ego`, defaulting to `super`.
The planner-only launch requires existing camera/VIO inputs and surveyed files:

```bash
ros2 launch larp_autonomy planner_mission.launch.py \
  planner:=ego mode:=shadow static_cad_only:=true \
  rig_file:=/path/to/rig.yaml \
  navigation_config:=/path/to/navigation.yaml \
  cad_prior_file:=/path/to/course_cad_prior.txt \
  boundary_config:=/path/to/boundary.yaml \
  initial_map_x:=0.0 initial_map_y:=0.0 initial_map_z:=0.0 initial_map_yaw:=0.0
```

Replace the example initial pose with the surveyed starting pose. Publish the
existing `NavigationGoal` in `map` on `/autonomy/goal`, with matching localization
epoch and increasing nonzero goal ID. Active planner launch modes still require
`alignment_verified:=true` and a flight-stable `mission_id`. Do not start a second
body adapter or mapper. The full tmux mission uses revisable CAD plus live depth;
the static option above is for explicit immutable-map evaluation.

## Certified EGO backend

The mapper's `planner_backend` parameter selects `super` or `ego`. Both backends
share one observed ROG map and the existing resumable SUPER A* global route.
EGO replaces local trajectory optimization, with its rebound B-spline optimizer
and internal A* still used for collision repair. Route corners seed its local
polynomial initialization. There is no snapshot transfer or second sensor map
in the mission path.

The adapter fixes the start position/velocity/acceleration and terminal stationary
control points inside optimization. Uniform cubic spans convert exactly into
polynomial pieces. Moving replans preserve the old curve through the scheduled
handoff; missing that deadline rejects the new candidate. EGO refuses requests
shorter than 20 cm, so a quintic terminal capture handles those requests under
the same certificate. Finished goal trajectories remain available for the bridge
to signal `is_final` rather than being restarted indefinitely.

The independent whole-curve validator checks position, velocity, acceleration,
continuity, stopping tail, body clearance against occupied **and unknown** voxels,
map extent and flight envelope. Yaw is optimized and validated separately.
Rejected candidates never replace a committed curve. If new geometry invalidates
the current curve, the shared certified-stop routine attempts braking; if that
also fails, the mapper publishes INVALID and the executive's existing certificate
expiry behavior applies. Cancellation clears EGO's command. A backward planner
clock latches the backend off; the bridge independently gates epoch changes.

Mission topics remain `/autonomy/route`, `/autonomy/committed_path` and
`/autonomy/committed_trajectory`. The bridge samples into the executive's existing
`/larper/trajectory_in` interface only in active modes. EGO itself publishes no
PX4 messages. Foxglove and the recorder require no alternate topic layout.

EGO inherits optimizer velocity/acceleration targets from `traj_opt.boundary`
and hard limits/body radius from the existing navigation configuration. It does
not use the standalone diagnostic `ego.yaml` speeds. Optional navigation YAML:

```yaml
ego_planner:
  control_points_distance: 0.1  # metres; allowed 0.03–0.4
  planning_horizon: 0.48       # metres; allowed 0.2–2.0
```

Defaults are 0.1 m control-point spacing and `min(0.6, 8 * optimizer_velocity)`
metres of local route. On `navigation_fine.yaml` this is 0.48 m. Candidate failures
shorten the local horizon without relaxing certificates. Long/tight routes may
still fail to optimize; successful global routing alone does not establish that
EGO can complete the course. Target timing, live-depth behavior on the vehicle,
closed-loop PX4 simulation and flight qualification remain unverified.

## Standalone diagnostic comparison

`ros2 launch larp_ego_planner shadow.launch.py` retains the earlier diagnostic
adapter. It subscribes to an **already running static-map** pipeline with complete
free-space snapshots (`snapshot_include_free:=true`) and matching fresh map leases,
odometry, VIO health and goal. Its separate outputs are
`/autonomy/ego/shadow/trajectory` and `/autonomy/ego/shadow/path`.

This adapter uses `ego_config:=/path/to/ego.yaml`, has soft endpoints and sampled
collision checks, and does not feed the executive or publish certificates. It
blocks unknown/exterior cells, expires stale inputs, handles cancellation and
latches epoch changes. It rejects simulated ROS time because it uses the upstream
clock-dependent warm-start path. The certified backend instead seeds every
optimization explicitly and uses the authoritative caller clock for commands.
Do not attach the diagnostic adapter to dynamic-map leases.

## Build and headless validation

Use the [supported Humble container](workspace.md), with this workspace as the
working directory. Normal Jetson builds include EGO through `larp_navigation`.
A focused development build, with the camera SDK and unchanged VINS executable
excluded, is:

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2
colcon build --base-paths src --packages-up-to larp_autonomy \
  --packages-ignore larp_hardware ov_srvins --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DBUILD_LOCALIZATION_REGRESSION=OFF
source install/local_setup.bash
ctest --test-dir build/larp_navigation --output-on-failure
ctest --test-dir build/larp_ego_planner --output-on-failure
python3 -m pytest -q src/larp_autonomy/test/test_mission_launch.py \
  src/larp_autonomy/test/test_runtime_config.py jetson/offboard/test_mission_startup.py
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 timeout 120s python3 \
  src/larp_navigation/tests/static_pipeline_smoke.py \
  --mapper install/larp_navigation/lib/larp_navigation/depth_map_node \
  --config src/larp_navigation/config/navigation.yaml --planner ego --executive-bridge
git diff --check
```

`BUILD_LOCALIZATION_REGRESSION=OFF` skips only the OpenVINS-dependent localization
regression in focused builds. The supported full build defaults it ON. The EGO
unit tests cover spline conversion, exact stopping boundaries, moving handoffs,
mission endpoint completion, unknown-space rejection, braking, cancellation and
clock rollback. ROS smokes use isolated domains and synthetic inputs. The optional
bridge smoke exercises EGO certificates through the simulation bridge into the
executive input topic, including TAKEOFF/HOLD gating, NED conversion, health
expiry, cancellation and epoch rejection; it starts no executive or PX4 process.
Launcher tests replace Docker and tmux with mocks.

Verified on 2026-11-08 in the supported x86_64 Humble development container:
the three changed packages (`larp_ego_planner`, `larp_navigation`, `larp_autonomy`)
built with `colcon build --packages-select` and the focused-build options above.
All 23 navigation tests, 2 standalone EGO tests, 64 autonomy Python tests and
7 mocked mission-launcher tests passed, as did the EGO-to-executive-input ROS
smoke. The two separately isolated ROS pytest modules were excluded from the
64-test run. Installed launch arguments and shared-library loading, changed
shell/Python syntax, documentation links and first-party `git diff --check` also
passed. The staged upstream import retains its original CRLF and trailing
whitespace, which the full staged whitespace check reports as formatting issues.
The container emitted its existing SciPy/NumPy version warning. This was not a
full camera/VINS or ARM runtime-image build, and nothing was deployed.
