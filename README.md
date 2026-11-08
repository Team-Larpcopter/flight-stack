# flight-stack

ROS 2 application packages for Larpcopter flight software. A local checkout is
often named `flight_stack_ws`.

## Jetson quick start (native camera + sqrtVINS)

Clone `main` on the provisioned Jetson and build the native stack:

```bash
git clone https://github.com/Team-Larpcopter/flight-stack.git
cd flight-stack
bash jetson/build.sh
```

This builds the pinned DepthAI SDK, C++ camera driver, sqrtVINS, navigation and
executive packages, then runs hardware-independent checks. It does not start
sensors. Sources are vendored; there are no Git submodules. The build pulls the public multi-architecture ROS 2 Humble image; no existing
team image, integration workspace or calibration file is required.

See [fresh-clone build and launch](docs/jetson-fresh-clone.md) for prerequisites,
container creation and the two-pane camera/sqrtVINS launcher, and
[workspace setup](docs/workspace.md) for package development.

For the complete camera → planner → PX4 runtime, see
[standalone flight runtime](docs/full-stack-runtime.md). The build includes the
XRCE-DDS Agent; `jetson/run_flight_stack.sh` selects shadow or hardware execution.

## Contributing

Branch naming, review, labels for LLM-authored changes, and rebase-and-merge
practice are in [CONTRIBUTING.md](CONTRIBUTING.md). PRs use
[`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md).

## Documentation

- [Simulate automatic takeoff and hold](src/flight_executive/README.md)
- [Fly the hard-coded DroneHacks course](src/flight_trajectory/README.md)
- [Workspace setup](docs/workspace.md)
- [AI assistant setup](docs/ai-assistants.md)
- [Agent / project contract](AGENTS.md)

## Build offboard and deploy to Jetson

Use the [ARM64 emulator build and deployment workflow](docs/offboard-arm64.md) to build
the complete Release stack on a PC, reuse the saved Jetson TensorRT engine, and
transfer the compiled runtime bundle. Jetson assembles a self-contained image
using its cached dependency layers, without compiling code or rebuilding the engine.
It includes localization, live LAS/ROG mapping, planning, DDS/PX4 integration, and
the combined Foxglove layout. The separate tmux launcher starts camera, VINS and
Foxglove in three panes with mouse mode enabled.

For the eight-pane full-system and pilot-triggered PX4 mission launchers, see
[full mission runtime](docs/full-mission-tmux.md).

Foxglove map visualization uses `/viz/autonomy/occupied_chunks` (`MarkerArray`).
At most once per second, only changed spatial chunks are replaced; emptied chunks
send deletions. The 4×4×2 chunk grid retains one message per chunk on separate DDS
writers, allowing Foxglove bridge 3.6 to replay the current map to new clients.
Unchanged maps send no voxel messages. Internal planner map snapshots remain full;
this reduces visualization traffic, not mapper computation. Load the updated flight
layout after upgrading, and restart the Foxglove pane (bridge and visualization
publisher together). The bridge queue cap is 32 for retained chunks; ordinary
single-writer depth-one monitoring streams keep depth one.
