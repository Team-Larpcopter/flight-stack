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
| `foxglove/larpcopter-drone.json` | Foxglove layout: L/R cameras + filtered depth (`/viz`), IMU and VIO plots, 3D VIO view, `/skybeta/state`, VIO status. |
| `team_image/Dockerfile` | Team overlay image (`skybeta-team:<VERSION>`): bakes `larp_oak` (`1280×800@30` `mono8` VIO + `800×500@30` stereo depth + CUDA pyramid `depth_filter`) and `larp_viz` into `/root/skybeta_ws`. |
| `oak_test/` | Standalone container harness + monitor for camera/depth/cuVSLAM stress tests. |
| `../src/larp_oak` | ROS package: `oak_node` (direct DepthAI driver: `1280×800@30` `mono8` L/R + `800×500@30` `ROBOTICS` subpixel depth + `200 Hz` IMU) and `depth_filter` (CUDA multi-scale edge-consistent hole fill + temporal filter → `/oak/stereo/filtered/image`). |
| `../src/larp_viz` | ROS package: `viz_relay` republishes downscaled camera and colourised depth JPEGs on `/viz/<topic>/compressed` at 5 Hz. |

## `larp` commands

| Command | Runs |
| --- | --- |
| `larp status` | hostname, drone.env IDs, Wi-Fi IP, containers, VIO state, `/skybeta/state` |
| `larp shell` | `docker exec -it skybeta-stack bash` (ROS sourced) |
| `larp ros CMD` | one ROS command inside the stack |
| `larp attach` | the stack's tmux (`Ctrl-b d` to leave) |
| `larp build` | `team_build.sh` (colcon build of `~/team_ws`) inside the stack |
| `larp image build` / `off` / `status` | build/disable/inspect `skybeta-team:<VERSION>` (`larp_oak` + `depth_filter` + `larp_viz`) |
| `larp fox start [--images]` | read-only Foxglove bridge on port 8766 (no client publish, services or params, rule 5.5). Telemetry only by default; `--images` adds 5 Hz camera + depth copies on `/viz` (see below) |
| `larp fox stop` / `status` | stop the bridge and image relay / show whether they run |
| `larp bag` | organisers' `host/run.sh bag` |
| `larp doctor` | organisers' `host/doctor.sh` |
| `larp restart` | restart the stack. **Power-cycle the flight controller afterwards** (skybeta bug #15) |
| `larp work` | host tmux session `larp` for dev work |
| `larp portal` | press ACCEPT on the Stanford Visitor guest portal when HTTPS stops working |

## Foxglove

```bash
larp fox start                       # telemetry only
larp fox start --images              # + both cameras, 5 Hz, half size, JPEG 50
larp fox start --images --rate 2 --scale 1.0 --quality 80
larp fox start --topics /some/image/compressed   # relay other CompressedImage topics
larp fox start --buffer 262144       # bridge send buffer in bytes (default 131072)
larp fox stop
```

Connect the Foxglove desktop app to `ws://<tailscale-ip>:8766` (8765 belongs to
the organisers' skybeta-status agent), then **Layouts → Import from file →
`jetson/foxglove/larpcopter-drone.json`**.

Full-rate image topics are never served: one OAK JPEG stream is ~250 KB per
frame at 30 Hz (~60 Mbit/s), more than the drone's Wi-Fi uplink, and it cost
the bridge ~50% CPU. The `--images` copies measured ~97 KB/s per camera at
5 Hz; `viz_relay` uses ~22% of one core because it still receives every
30 Hz frame. Any topic containing `/image` is hidden unless it is under `/viz`.

Latest data only: the bridge's send buffer is 128 KB (default 10 MB), so when
the Wi-Fi falls behind it drops old messages instead of queueing seconds of
them. ROS queues are capped at 5 (`/tf_static` needs its 4 static transforms),
and the relay keeps only the newest frame.

The bridge and relay run as `foxglove` and `viz` windows in the stack's tmux,
so they stop whenever the stack restarts; run `larp fox start` again.

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
