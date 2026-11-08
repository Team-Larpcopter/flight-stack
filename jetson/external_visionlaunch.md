# Launch external vision in tmux

Run this on the Jetson host, from the flight-stack checkout. It opens four titled
panes in the `external_visionlaunch` session: Camera + IMU, sqrtVINS,
MicroXRCE-DDS, and External vision. The bridge publishes to
`/fmu/in/vehicle_visual_odometry` on ROS domain 15.

Stop existing camera, VINS, bridge, and agent processes through their current
terminal or launcher first. This launcher refuses to duplicate them and does not
stop processes it did not start. Both localization containers must already be
running. The DDS agent gets a separate temporary container with UART access.

```bash
CALIBRATION_DIR=/integration/evidence/localization-HhiskY64/calibration \
IMU_TO_BODY_FRD='[0.0, 1.0, 0.0, 0.0]' \
bash jetson/external_visionlaunch.sh start
```

The example quaternion is valid only if the source IMU axes align with aircraft
forward-left-up. Set `IMU_TO_BODY_FRD` to the measured IMU-to-aircraft-FRD rotation
in w,x,y,z order. A forward-facing camera alone does not establish the IMU axes.
The launcher requires this setting and the calibration directory explicitly.
It does not arm the aircraft or change PX4 parameters.

## Stop the processes

- Press **Ctrl+C** in a pane to terminate that pane's process and its children.
  Its output remains visible after exit.
- Press **Ctrl+b**, then **d** to detach. All four workloads keep running.
- Run this in another host terminal to stop all four owned workloads:

```bash
bash jetson/external_visionlaunch.sh stop
```

`stop` kills this tmux session, waits for container workers to exit, and reports
an error if it cannot verify cleanup. Killing the tmux session manually also
stops the workers. A lost owner heartbeat triggers cleanup within three seconds,
followed by bounded SIGINT, SIGTERM, and SIGKILL escalation. Existing localization
containers stay running; the temporary agent container removes itself.

An SSH disconnect that only detaches tmux leaves the workloads running. Use
`stop` to terminate them. If one pane has exited, stop the session and start again
to restart all four. Re-running `start` while the session exists only attaches.

```bash
bash jetson/external_visionlaunch.sh status
bash jetson/external_visionlaunch.sh start --detach
```

`--session NAME` selects a different tmux session for start, stop, and status.
Use the same name for all three commands.

## Override the current test setup

| Environment variable | Default |
|---|---|
| `CAMERA_CONTAINER` | `larp-localization-raw-imu` |
| `VIO_CONTAINER` | `larp-localization-ready` |
| `INSTALL_PREFIX` | `/integration/install_clean` |
| `CAMERA_INSTALL_PREFIX` | Same as `INSTALL_PREFIX` |
| `BRIDGE_INSTALL_PREFIX` | `/integration/install_px4_bridge` |
| `SDK_PREFIX` | `/integration/sdk_install` |
| `ROS_DOMAIN_ID` | `15` |
| `XRCE_DEV` | `/dev/ttyTHS1` |
| `XRCE_BAUD` | `921600` |
| `XRCE_REFS` | `~/skybeta/host/dds/agent_refs_local.xml` |
| `AGENT_IMAGE` | `larp-integration:20261107` |

The default paths match the temporary integration setup. After building the
packages in a different mounted workspace, override its install and calibration
paths. No source checkout or image rebuild is needed just to run this launcher.
The controller's DDS domain, UART, baud rate, and participant profile must match
these settings. The launcher checks host resources but does not configure PX4.

## Test cleanup without hardware

```bash
python3 -m unittest discover -s jetson/test -p test_external_visionlaunch.py -v
```

The tests use local sleeping processes. They exercise normal exit, stdin closure,
heartbeat expiration, and cleanup of a child that ignores SIGINT and SIGTERM.
