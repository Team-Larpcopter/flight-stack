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
