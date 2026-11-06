# Workspace setup (flight_stack_ws)

`flight-stack` is a ROS 2 Humble workspace. A local checkout is often named
`flight_stack_ws`.

## Layout

| Path | Role |
| --- | --- |
| `src/` | First-party ROS 2 packages |
| `docs/` | Contributor and assistant guidance |
| `.github/` | PR template and GitHub metadata |
| `build/`, `install/`, `log/` | Generated colcon output (do not commit) |

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

## Lint / format

This repo does not yet ship its own pre-commit config. Prefer consistent Python
style (Ruff-compatible) and package-local tests for new first-party code. If this
workspace later adds `.pre-commit-config.yaml` or CI linters, document them here.

## Verification for PRs

Report the smallest useful check set:

1. `colcon build` for touched packages.
2. Package tests if present, or a short `ros2 run` / `ros2 launch` smoke.
3. Note any external runtime dependencies you used for integration checks.
