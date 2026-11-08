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
