# flight-stack

ROS 2 application packages for Larpcopter flight software. A local checkout is
often named `flight_stack_ws`.

## Quick start

```bash
cd /path/to/flight_stack_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

See [docs/workspace.md](docs/workspace.md) for layout and verification
expectations.

## Contributing

Branch naming, review, labels for LLM-authored changes, and rebase-and-merge
practice are in [CONTRIBUTING.md](CONTRIBUTING.md). PRs use
[`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md).

## Documentation

- [Workspace setup](docs/workspace.md)
- [AI assistant setup](docs/ai-assistants.md)
- [Agent / project contract](AGENTS.md)
