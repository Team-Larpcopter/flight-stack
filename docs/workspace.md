# Workspace setup (flight_stack_ws)

`flight-stack` is a ROS 2 Humble workspace. A local checkout is often named
`flight_stack_ws`.

## Layout

| Path | Role |
| --- | --- |
| `src/` | First-party ROS 2 packages |
| `src/px4_msgs/` | Vendored PX4 messages; revision in `UPSTREAM_REVISION` |
| `src/larp_ego_planner/` | Pinned EGO-Swarm ROS 2 optimizer and diagnostic adapter; [usage](ego-planner.md) |
| `docs/` | Contributor and assistant guidance |
| `.github/` | PR template and GitHub metadata |
| `build/`, `install/`, `log/` | Generated colcon output (do not commit) |

For a fresh Jetson checkout, use [the reproducible container build](jetson-fresh-clone.md).
The commands below assume all package and SDK dependencies are already installed.

## Build and run

```bash
cd /path/to/flight_stack_ws
source /opt/ros/humble/setup.bash
# optional: source any required overlay workspaces
colcon build --symlink-install
source install/local_setup.bash
```

Build one package:

```bash
colcon build --packages-select flight_executive --symlink-install
```

Run a node (example):

```bash
ros2 run flight_executive flight_executive_node
```

Keep colcon outputs under this workspace’s `build/`, `install/`, and `log/`.

When this checkout is nested inside `flight-systems`, follow the parent
repository's container and output-directory rules instead. The
[executive build instructions](../src/flight_executive/README.md#build-in-the-dev-container)
use the local message submodule and keep all generated files under the parent's
`ros2_ws`. Source that build's overlay before importing `px4_msgs`.

## Lint / format

This repo does not yet ship its own pre-commit config. Prefer consistent Python
style (Ruff-compatible) and package-local tests for new first-party code. If this
workspace later adds `.pre-commit-config.yaml` or CI linters, document them here.

## Verification for PRs

Report the smallest useful check set:

1. `colcon build` for touched packages.
2. Package tests if present, or a short `ros2 run` / `ros2 launch` smoke.
3. Note any external runtime dependencies you used for integration checks.
