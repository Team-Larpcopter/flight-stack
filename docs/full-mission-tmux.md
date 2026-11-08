# Eight-pane full system and PX4 mission

Run on the Jetson after offboard build and transfer. Offboard builds compile
production targets by default; `RUN_TESTS=1` also builds/runs the C++ test targets. The host scripts only build
and transfer; these scripts launch separately:

```bash
export DEPLOYMENT_CONFIG=$HOME/flight-stack/config
# Full sensing, mapping, planning and visualization; no executive/PX4 commands:
bash ~/flight-stack/deployment/arm64/launch_full_system_tmux.sh
# Or the real mission (stop the shadow system first):
bash ~/flight-stack/deployment/arm64/launch_mission_tmux.sh
```

Both use one tiled tmux screen, mouse enabled, with eight panes: Drivers,
Foxglove, sqrtVINS, DDS, LAS depth, ROG map, SUPER, Mission executive. Each pane
has one launch command. ROG and SUPER deliberately share one process/map;
the ROG pane displays small map-state messages, avoiding a duplicate map and
large cloud transfers between planning processes. The VINS pane also runs the
body-frame adapter and, in hardware mode, the PX4 external-vision bridge.

Copy `src/larp_autonomy/config/mission.yaml` to the configuration directory,
alongside calibrated `rig.json` and PX4 `bridge.yaml`. Set the surveyed map/PX4
alignment and `alignment_verified: true` for hardware. The nominal course XY
boundary is only appropriate at the matching surveyed placement. The entire
vehicle is bounded below map z=1.8 m, with 0.4 m clearance: commanded body-centre
altitude stays at or below 1.4 m. The executive independently checks measured
position and outgoing setpoints against the same boundary. Ground contact is
allowed during takeoff and landing, but never relaxes the ceiling or XY limits.

The mission matches the successful `sqrtvins-course-053` simulation configuration:
take off 1.0 m, navigate to map `[-1.90688, -0.47244, 1.0]`, yaw 0, then land.
There are no intermediate mission waypoints. In hardware mode the final goal is
published after takeoff reaches HOLD, avoiding global searches and trajectory
optimization while waiting on the ground or ascending. The older short-goal and scripted
course examples remain available but are not used by these launchers.

Start the stack while landed and disarmed in a manual flight mode. Once localization, depth,
and the mapped launch column are ready, the executive streams a ground-position
hold heartbeat. After at least two seconds of continuous heartbeat, arm using
the radio, then switch to **Position**. The software requests Offboard and begins
ascent only after PX4 confirms Offboard. Arming while already in Position requires
switching out and back into Position while still grounded. The software never
arms the vehicle and refuses already-armed, existing Offboard or airborne startup. Leaving Offboard relinquishes control; there is
no automatic re-entry or restart. If a component exits, its pane stays visible.
A new mission requires deliberate container cleanup rather than restarting the
executive over a possibly flying vehicle. PX4's configured Offboard-loss behavior
still applies when commands cease.

`PX4_TRANSPORT=udp4` uses port 8888 by default. For UART/USB use
`PX4_TRANSPORT=serial PX4_DEVICE=/dev/... PX4_BAUD=921600`. `DDS_PROFILE`,
`ROS_DOMAIN_ID`, `XRCE_REFS` (agent participant-reference XML), `RUNTIME_IMAGE`, `CONTAINER`, and `SESSION` are configurable.
Do not run the old localization launcher concurrently: the full system owns the
camera and VINS publishers.

## Depth, map and runtime cost

The runtime uses `navigation_fine.yaml` and `course_cad_prior_fine.txt`: 5 cm
voxels, a revisable CAD prior, and one shared map for SUPER A* and trajectories.
The packaged priors include the updated W1/W2 wall layout; see
[planning-map provenance and regeneration](course-map-prior.md) for geometry,
coordinate registration, and rebuild requirements.
LAS and the mapper both reject metric depth outside 0.3–3.0 m. Invalid/no-return
pixels are unknown; they never count as free-space rays.

New occupancy needs three hit-bearing depth frames. Clearing occupied cells needs
three valid empty-ray observations. Opposing evidence resets the pending streak;
multiple correlated pixels from one frame count once. Pending hits preserve the
previous classification, so a single noisy hit does not block a CAD-free route;
previously unknown cells remain unknown. One byte per voxel stores
both streak directions (roughly 6 MB for the configured map), with constant work
only on touched cells. Hard boundaries cannot be cleared by sensor observations.

The fine-grid global search budget is 250 ms. A native-host actual-CAD check found
the full route in about 188 ms under the 1.8 m ceiling; geometric inspection of
the route confirmed the first divider, Gate A, tunnel and window passages to the
final goal. This is a route check, not an executed flight or target timing result.
Successful routes are cached and revalidated; failed searches back off for 0.5 s
even when depth frames change the map revision. Collision and trajectory checks
remain enabled. Full occupied-map snapshots are capped at 1 Hz; this launcher
omits the unused free-cell snapshot cloud.

## Flight monitoring

Connect Foxglove to `ws://<jetson>:8766` and import
`jetson/foxglove/larpcopter-flight.json` (also transferred with the bundle).
It shows the live occupied map, global route, committed local path, bounded flown
path, vehicle pose/velocity, goal, VIO health, map state and executive state.
Only one left-camera image and one depth image are sent, JPEG at 5 Hz, half size,
quality 60, with latest-only queues and dropping before conversion. LAS's own
preview/disparity JPEGs, right-camera preview, feature clouds, IMU plots and raw
trajectory/map diagnostics are disabled in this flight-monitor profile.

The 1.8 m limit here is the companion planner/executive boundary; this does not
upload or claim to configure PX4's independent onboard geofence parameters.

Each pane runs its launch command from an interactive Bash shell. Ctrl+C stops
that component and returns to the prompt; press Up then Enter to restart it.
Mouse mode stays enabled. Restarting the mission executive still requires its
normal grounded/pilot-transition conditions; it does not adopt an airborne mission.

## Short takeoff/hold test

`bash launch_takeoff_hold_tmux.sh` uses five panes: drivers, Foxglove, VINS,
DDS and executive. It loads an adjacent `deployment.env` when present. The pilot
arms then selects Position; the executive requests Offboard, ascends 0.5 m,
holds for 2 seconds, then requests landing. It starts no planner or trajectory
publisher and intentionally does not require map/obstacle readiness. PX4
preflight, pilot sequencing, telemetry and body-envelope checks remain active.
This does not resolve or qualify unstable VIO.

Run `bash launch_takeoff_hold_tmux.sh --check` for configuration/import checks
without launching sensors or flight publishers. Stop the other stack before
starting this test. Mouse mode and Ctrl+C-to-shell behavior match the full stack.
