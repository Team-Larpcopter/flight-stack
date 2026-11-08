# ARM64 build host → Jetson deployment

The build host compiles the flight stack in **Release**, under ARM64 QEMU with
Ubuntu 22.04 / ROS 2 Humble. This includes the native camera driver, sqrtVINS,
ROG, SUPER, PX4 bridge/executive, DDS Agent, LAS node and Foxglove.
`BUILD_JOBS` defaults to all host cores (`nproc`). Compiler outputs and SDK
dependencies persist in a named Docker volume for incremental builds. The installed
pinned SDK is reused without fingerprint checks; set `FORCE_SDK_BUILD=1` when
intentionally rebuilding it.

The host transfers the compiled runtime bundle and launch scripts. It does not
start anything remotely. On the Jetson, a separate script assembles a
self-contained image from that bundle, the cached public Humble dependency
image, and the cached GPU runtime/engine. Assembly only copies files into image
layers: **no source compilation, downloads, or TensorRT engine build** occurs on
Jetson. Existing GPU libraries do not make a slow round trip through the host.
After assembly, no integration workspace, skybeta image or GPU-cache folder is
mounted into the running container.

The first host build needs internet to fetch public source/dependencies.
Deployment, Jetson image assembly and runtime can operate without internet.
This is an off-device build, not a claim that an empty host can download
uncached dependencies offline.

## One-time preparation

Install Docker and ARM64 binfmt on the build host:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

The Jetson must have the public dependency image built by `jetson/build.sh`,
`larp-flight-stack:humble`, cached. `BASE_IMAGE` can select an equivalent public
Humble dependency image. A new target without this cache needs a one-time image
transfer (`docker save` / `docker load`); this is separate from routine updates.
It also needs Docker's NVIDIA runtime and host `tmux`.

Cache the existing Jetson LAS runtime and engine once. Set deployment values in
your shell; never put passwords or host identities in Git:

```bash
export SSH_HOST=user@jetson
export GPU_CONTAINER=your-working-las-container
export ENGINE_ON_JETSON=/absolute/path/to/las2_m_320x480_d192_fp16.engine
bash jetson/offboard/collect_gpu_runtime.sh
```

The cache preserves TensorRT 10.3, CUDA 12.6, PyTorch and Python dependencies.
NVIDIA host drivers are provided by the Jetson container runtime. No ONNX export or `trtexec` build occurs. An
incompatible engine fails startup rather than substituting CPU stereo.

## Build and transfer on the host

```bash
bash jetson/offboard/build_and_transfer.sh
```

`BUILD_IMAGE`, `BUILD_VOLUME`, `OUTPUT_DIR` and `REMOTE_DIR` are configurable.
The default target folder is `~/flight-stack/deployment/arm64`. GPU artifacts
stay in `~/flight-stack/deployment/gpu`. The host stops after transfer. It does not assemble the image or launch sensors.

Builds always use `/flight-stack` inside the persistent volume to prevent CMake
cache relocation errors. Do not share the volume between concurrent builds.
The offboard build compiles production targets and performs device-free camera/VINS
checks. Set `RUN_TESTS=1` to also compile and run package tests.

## Launch separately on Jetson

```bash
bash ~/flight-stack/deployment/arm64/launch_localization.sh
```

On first use or after an update, this assembles the runtime image offline. Then
it opens tmux with **one command per pane**: camera, sqrtVINS and Foxglove.
Mouse mode is enabled. LAS, planners, the XRCE agent and PX4 execution remain
off in this launcher. Stop conflicting camera owners before starting it.
`ROS_DOMAIN_ID`, `DDS_PROFILE`, `CONTAINER`, `SESSION`, `GPU_CACHE_DIR`,
`BASE_IMAGE` and `RUNTIME_IMAGE` are configurable. A container using an old image
is rejected rather than silently reusing old binaries; stop/remove it or choose
a new `CONTAINER` before launching an updated image.

## Combined Foxglove view

Connect Foxglove to `ws://<jetson>:8766` and import the transferred
`larpcopter-autonomy-localization.json`:

- Localization: camera preview, sqrtVINS features/pose/path, body pose/path,
  velocity and VIO health when their producers are active.
- Depth mapping: LAS depth preview and the ROG occupied cloud in the map frame,
  with map revision/status details.
- Planning: goal pose, global route, committed path and trajectory details.

Planning/depth views wait for their producers; the default three-pane launcher
does not start them. The read-only bridge exposes `/viz/*`, with throttled
images, map snapshots at most once per second, and planner paths/poses at 5 Hz.
The localization launcher retains the low-latency 64 KB send-buffer limit.
The optional full-stack launch raises it to 8 MB to accommodate map clouds.
Visualization topics are not inputs to planning or vehicle execution.

## Optional full stack

Generate `rig.json` from the surveyed body mount and camera-exported rectified
geometry using `native_calibration --depth`. It must contain `T_body_depth` and
`depth_frame: native_oak_depth_optical`. Put it and `boundary.yaml` in a deployment
configuration directory. Do not use a bench transform for flight.

```bash
DEPLOYMENT_CONFIG=/absolute/path/to/config ROS_DOMAIN_ID=0 \
  bash ~/flight-stack/deployment/arm64/run_full_stack.sh shadow \
  goal_x:=1.0 goal_y:=0.0 goal_z:=1.0
```

This explicitly starts LAS/ROG dynamic mapping, planners, visualization and the
DDS Agent. The hard boundary remains enforced while depth revises occupancy.
Hardware mode additionally requires measured alignment, mission ID, bridge
parameters and PX4 transport configuration; see [full stack](full-stack-runtime.md).

### Fast application updates

The Jetson launchers now extract `stack-runtime.tar.gz` into `runtime/` next to
these scripts and bind-mount the application, ROS and DDS agent files. Normal
bundle updates do not build or export a Docker image. Existing assembled runtime
images can be reused. Only a missing dependency image (or explicit
`REBUILD_DEPENDENCIES=1`) builds the GPU/engine image once.

Stop the old container and tmux session before replacing a deployed bundle.
For the default localization session, after stopping its components:

```bash
tmux kill-session -t flight-stack-runtime
docker stop flight-stack-localization-runtime
docker rm flight-stack-localization-runtime
bash launch_localization.sh
```

Keep the copied deployment folder in place: containers mount its `runtime/`
directory. The original GPU cache/engine is reused, never rebuilt.

### Eight-panel mission startup and timing

`launch_mission_tmux.sh` loads `deployment.env` next to the launcher, including
UART, DDS profile/domain, GPU cache and deployment configuration. Each explicit launch stops and removes existing stack containers and replaces
their tmux sessions. Run launch commands from the SSH shell outside those sessions.
The passive status monitor and unrelated containers are left alone; `--check` on
the takeoff launcher does not stop anything.
Start disarmed and wait for grounded map/PX4 alignment before arming and switching
to Position. Landed telemetry uses the executive's existing 2.5-second allowance;
position and body odometry still need fresh samples.

The full mission defaults to `planner_hold_timeout_s: 15.0` before landing when no
usable planner command arrives (initial planning or recovery). The standalone
takeoff/hold test keeps its own two-second hold. `trajectory_max_age_s: 0.6` is
shared by dynamic mapping and the trajectory bridge to accommodate LAS latency
and the bounded 250 ms A* search. Certificates always expire relative to sensor
capture time, never publication time. Expired data or stale localization still
withholds commands. These timing values can be set in `mission.yaml`.

After a long planner callback, mapping prefers the newest depth frame with an
available capture-time pose. SUPER trajectory sequence numbers use ROS time so a
mapper-only restart does not permanently reject the new stream. This does not
permit restarting localization or changing the frozen flight frame in flight.
Actual Jetson load and end-to-end course execution still need integration testing.

For a prepared small application patch, place `apply_runtime_patch.sh` and
`mission-integration-patch.tar.gz` beside the existing deployment's launchers.
With its stack container stopped, run `bash apply_runtime_patch.sh`. The helper
extracts the patch before installation, preserves device configuration and GPU
assets, and never launches the mission. Normal source builds contain the same
fixes; this patch is only a faster way to update an existing deployment.


### Manual rosbag pane

Both mission and takeoff/hold tmux launchers add a recorder pane with its command
already typed. Press Enter in that pane to start; Ctrl+C finalizes the bag and
returns to its shell. Nothing records automatically. Bags persist under `bags/`
beside the launchers, or the host directory specified by `ROSBAG_DIR`.

The recorder includes all topics (including compressed images), excluding raw
`sensor_msgs/Image` and `stereo_msgs/DisparityImage` topics discovered at startup.
A name filter also excludes the stack's raw-image topics when publishers start
later. It records the existing compressed preview streams without creating more
image encoders. Bags split at 1 GiB; no additional bag compression is applied.

The operator-requested hardware `geofence_padding_m` now defaults to 0.5 m on
all six sides of the executive envelope. With the current course and 0.4 m body
margin, body-position limits are X [-4.2624, 2.0336], Y [-5.6626, 0.6334],
and airborne HOLD/TRACKING Z [-0.1, 1.9] metres in map coordinates. During
TAKEOFF/LAND the lower Z is -0.5 m, or -1.0 m with fresh confirmed ground contact
(the existing 0.5 m ground allowance plus the new padding). This supersedes the
previous executive ceiling; planner/CAD obstacle bounds remain conservative.
Simulation defaults retain zero padding. Set `geofence_padding_m:=0.0` on the
executive to restore its previous envelope.

### Freeze the common origin before mapping

Both production tmux launchers now run `map_origin` in the VINS pane. The body
adapter publishes provisional `vio_odom/base_link` data privately on
`/autonomy/unregistered_odometry`. Before exposing mission odometry, the origin
stage requires fresh healthy odometry and, on hardware, fresh disarmed/landed
PX4 telemetry outside Offboard. It freezes after one second of position/yaw
stability (5 cm / 0.05 rad), with linear and angular speeds below 0.1 m/s and
0.1 rad/s. This stage does not require preflight readiness, avoiding a circular
dependency on PX4 receiving vision. Existing executive readiness checks remain.

At that instant it maps the sampled body position/yaw to the configured start
pose (currently [0,0,0], yaw 0). It applies the same fixed translation and yaw
rotation to subsequent positions, orientations, and world-frame covariance;
body-frame velocities stay unchanged. `/autonomy/odometry` is then the single
registered input to PX4 vision, the executive, planning and Foxglove. Output is
bounded at 50 Hz. `/autonomy/map_origin` retains the chosen source pose for logs.

Hardware dynamic mapping uses `/autonomy/map_update_ready`, which stays false
until the executive publishes a map/PX4 alignment newer than this origin. No
startup depth frames can revise the CAD prior before then. Shadow mode requires
only the stable common origin. The offset never changes during the run; health
loss pauses output/map updates without recentering. Restart the whole stack on
the ground to choose a new origin, not an individual localization component.
