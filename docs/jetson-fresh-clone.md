# Fresh clone on Jetson

The native production path is OAK stereo/raw aligned IMU → sqrtVINS → optional
CAD/Air-FAR/SUPER planning. ROS 2 Humble runs in Docker. No external IMU
correction file is required. The bundled OAK-D S2 factory geometry targets the
team camera; a different device needs its own factory geometry and mount check.

## Prerequisites

Use a Jetson with Docker, Git, tmux and Internet access. The build pulls the public
multi-architecture `ros:humble-ros-base-jammy` image and installs dependencies;
no organizer/team image or other local workspace is required. `BASE_IMAGE` can
override the base. Native localization and planning do not require CUDA. The
build mounts no camera or serial devices.

## Build from main

```bash
git clone https://github.com/Team-Larpcopter/flight-stack.git
cd flight-stack
bash jetson/build.sh
```

`BUILD_IMAGE` defaults to `larp-flight-stack:humble`. `BUILD_JOBS` defaults to 1
for Jetson RAM limits. Downloads, SDK source and build products stay in ignored
`sdk-source/`, `sdk_install/`, `build_sdk/`, `build/`, `install/`, `log/` directories.
DepthAI 3.10.0 is checksum verified before extraction; CMake fetches its own
pinned dependencies. Re-running resumes the same build. Native and autonomy
checks run after compilation; a failed check exits nonzero.

The package selection includes the camera, estimator, planners, executive and
external-vision bridge, but not legacy CUDA depth experiments. To build those,
use their separate team-image workflow. No Git submodule initialization is needed.

## Create the localization container

No existing container or external DDS/calibration file is required. The launcher
creates `flight-stack-localization` automatically from this checkout and image.
It mounts this clone at `/flight-stack` and uses its `install/` and `sdk_install/`.
It refuses to reuse a container belonging to another checkout. No PX4 serial
device is mounted. ROS domain defaults to 0; set `ROS_DOMAIN_ID` before first
launch to match other vehicle processes. `DDS_PROFILE` is an optional host XML
override for interoperability with a custom DDS setup.

To create without starting anything:

```bash
bash jetson/create_localization_container.sh
```

For multiple checkouts, export unique `CONTAINER` and `SESSION` names. Container
environment is fixed at creation; recreate it explicitly to change the domain.

## Run camera and sqrtVINS

After releasing the camera from any other driver:

```bash
bash jetson/launch_localization_tmux.sh
```

Left pane: native C++ camera driver. Right pane: sqrtVINS, with bundled calibration
and INFO output. Ctrl+C stops each pane; detaching leaves processes running.
Re-running attaches to the existing tmux session. No planner, flight executive,
PX4 bridge, arming or takeoff command is launched by this helper.

In another terminal, inspect the estimator output:

```bash
docker exec -it "${CONTAINER:-flight-stack-localization}" bash -c '
  source /opt/ros/humble/setup.bash
  source /flight-stack/install/setup.bash
  ros2 topic hz /autonomy/odomimu
'
```

Planner/executive setup is documented in
[planner integration](planner-executive-integration.md). Building and starting
localization does not establish flight qualification or validate map alignment.
