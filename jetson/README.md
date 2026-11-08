# Jetson tools

Start with [fresh-clone native build and launch](../docs/jetson-fresh-clone.md).
The legacy organizer-stack helpers below are retained for existing deployments.

Host-side helpers for our DroneHacks drone (`skybeta-15`). On the drone, ROS 2
runs only inside the organisers' container `skybeta-stack` (tmux session
`skybeta`: agent, slam, bridge, competitor). `~/team_ws` on the Jetson is
mounted at `/root/team_ws` inside it. This repo is checked out at
`~/team_ws/src/flight-stack`.

Nothing here touches the organisers' doctor, status services, `drone.env`
identity, PX4 parameters, or `/fmu/in/*`.

## Files

| File | What |
| --- | --- |
| `larp` | Helper CLI, linked as `~/.local/bin/larp`. Run `larp help`. |
| `install.sh` | One-time host setup: tmux, htop, jq, `larp`, Claude Code, Node 22, Codex CLI. Needs internet. |
| `tmux.conf` | Copied to `~/.tmux.conf` if none exists. `Ctrl-b Ctrl-b` sends the prefix to the stack's nested tmux. |
| `integration_image/` | Native integration image (DepthAI 3.10 SDK build, planner deps) and run helpers: `localization_pane.sh camera\|vio\|monitor\|las\|fox`, `launch_localization.sh` (bounded test). |
| `foxglove/larpcopter-autonomy-localization.json` | Foxglove layout for the native path: left camera, depth, 3D sqrtVINS, plots. Bridge and relay: `src/larp_perception`. |

## `larp` commands

| Command | Runs |
| --- | --- |
| `larp status` | hostname, drone.env IDs, Wi-Fi IP, containers, VIO state, `/skybeta/state` |
| `larp shell` | `docker exec -it skybeta-stack bash` (ROS sourced) |
| `larp ros CMD` | one ROS command inside the stack |
| `larp attach` | the stack's tmux (`Ctrl-b d` to leave) |
| `larp build` | `team_build.sh` (colcon build of `~/team_ws`) inside the stack |
| `larp bag` | organisers' `host/run.sh bag` |
| `larp doctor` | organisers' `host/doctor.sh` |
| `larp restart` | restart the stack. **Power-cycle the flight controller afterwards** (skybeta bug #15) |
| `larp work` | host tmux session `larp` for dev work |
| `larp portal` | press ACCEPT on the Stanford Visitor guest portal when HTTPS stops working |

## Camera, depth and Foxglove

The skybeta-era camera node (`larp_oak`), team image, depth benchmarks and
`larp fox` are gone. On the native path: `src/larp_hardware` (camera: stereo, IMU
and the camera's rectified pair), `src/larp_liteanystereo` (Lite Any Stereo depth
on the Jetson GPU) and `src/larp_perception` (read-only Foxglove bridge on 8766
with throttled `/viz` copies, `stack` run commands, benchmarks). See
`src/larp_perception/README.md`. Run everything with `jetson/stack.sh camera|vio|fox|status|stop`
(after `bash jetson/build.sh`); Lite Any Stereo runs in its own GPU container on NVIDIA's Isaac ROS
Humble image.

## Setup on a fresh drone

```bash
cd ~/team_ws/src && git clone https://github.com/Team-Larpcopter/flight-stack.git
~/team_ws/src/flight-stack/jetson/install.sh
claude    # log in yourself
codex     # log in yourself
```

Never commit credentials, auth keys, or Wi-Fi passwords.

## Native sqrtVINS: two-pane tmux launcher

On the Jetson host, with the configured `flight-stack-localization` container
built from this checkout and no other camera/VIO owner running:

```bash
bash jetson/launch_localization_tmux.sh
```

The target OAK-D S2 factory geometry and native IMU topics are bundled in the
repository; no external calibration or IMU correction file is needed. An optional
argument can override the calibration directory **inside the container**. The launcher
creates its own container when absent, starts it if stopped, then creates one tmux window with two
side-by-side panes: camera on the left and sqrtVINS on the right. Both processes
run in the foreground indefinitely; sqrtVINS uses `INFO` verbosity. No monitor,
Foxglove preview, planner, or PX4 process is launched. Ctrl+C stops the process in
its pane. Detaching leaves localization running. Re-running attaches to the same
session without launching another camera or estimator. End that session before
starting a new run with a different calibration.

`CONTAINER`, `SESSION`, `CALIBRATION_DIR`, and `VIO_VERBOSITY` can be set in the
host environment. `WORKSPACE`, `INSTALL_PREFIX`, and `SDK_PREFIX`
are also forwarded when provided. Defaults use this checkout mounted at `/flight-stack` with `install/` and `sdk_install/`.
The helper copied into the container comes from this checkout, so it does not rely
on an older manually installed `localization_pane.sh`.
