# Planner to mission executive

`ros2 launch larp_autonomy planner_mission.launch.py` replaces the hard-coded
course publisher. It starts body-frame odometry, the static CAD mapper/SUPER with its existing A*
frontend, and the polynomial bridge. SUPER A* is the only route backend.
Start the native camera and sqrtVINS separately;
do not also start `observe.launch.py` or another body-odometry adapter.
The course CAD is the intended occupancy map for real flight, as well as profiling.
Depth updates remain disabled and hard boundaries remain mandatory. CAD geometry
does not correct sqrtVINS poses; physical map registration is a separate transform.

Required arguments: `rig_file`, `navigation_config`, `cad_prior_file`, `boundary_config`.
These describe the actual mounting and course;
the launcher does not invent surveyed geometry. Publish `NavigationGoal` on
`/autonomy/goal` in `map` (transient-local), with matching localization epoch.
Set `initial_map_x/y/z` and `initial_map_yaw` to the surveyed starting body pose.

Modes:

- `mode:=shadow` (default): no executive or PX4 command publisher. Samples appear
  on `/autonomy/shadow/trajectory_in`; no mission-state input is needed.
- `mode:=simulation`: starts the simulation executive.
- `mode:=hardware`: explicitly enables the executive's hardware command path.
  This is not flight qualification. Pilot arming followed by Position-mode
  selection authorizes its automatic takeoff sequence. Do not launch for a
  stationary timing test.

Active modes require `alignment_verified:=true`, a nonempty `mission_id`, and
surveyed `map_heading_ned`, `map_origin_ned_x/y/z`. The map heading is clockwise
from PX4 north; map +Y is left, +Z is up. The origin is in PX4 local NED.
External-vision fusion must already be configured; this launch does not inject EV.

The bridge publishes `flight_executive_msgs/TrajectorySetpoint` to
`/larper/trajectory_in` at 50 Hz only in HOLD/TRACKING. The executive remains the
sole publisher of PX4 offboard heartbeat, setpoint and mode/landing commands.
It forwards position, velocity and acceleration in position-control mode.
One mission ID survives replans and goal changes. A localization epoch change
latches output off until restart; stale certificates or unhealthy VIO stop output.
The executive then holds, and lands after its configured hold timeout (default 5 s).

Completion uses `is_final` on repeatedly certified stationary final samples,
not a separate Bool. The executive requires measured arrival (25 cm horizontal,
15 cm vertical, speed below 0.2 m/s for one second) before requesting landing.
Intermediate stops are not completion. Current arrival logic does not check yaw.

## Profiling

Run the planner in shadow mode with localization active and a reachable map goal.
Then in another terminal:

```bash
ros2 run larp_navigation planner_latency_native --ros-args -p duration_seconds:=30.0
```

The profiler only subscribes. It prints counts, received rates, header-age
p50/p95/p99/max and maximum receipt gaps for raw VIO odometry, body odometry,
route, committed trajectory and shadow executive input. Header semantics differ:
odometry is measurement age, route/trajectory is certificate age, executive
input is sampling/publication age. These are not a causally matched end-to-end
planner latency or pure compute duration. Empty topics are reported explicitly.
Use the same ROS domain and clock as the pipeline. An isolated synthetic smoke
validates transport but does not qualify real planner runtime or hardware flight.

## Full live profiling launch

Use `ros2 launch larp_autonomy profile_pipeline.launch.py` with the four required
file arguments above, plus the surveyed initial map pose. This starts the native
OAK driver (30 Hz stereo, 500 Hz raw accel / 400 Hz gyro, bracket alignment,
5 ms exposure cap), bundled raw-IMU sqrtVINS, body odometry and the planner chain.
It always selects shadow mode: no executive, takeoff or PX4 command publishing.
Set `start_localization:=false` to attach to an already running native driver and
sqrtVINS instead of competing for the camera. Use `estimator_config:=...` only
when intentionally overriding the bundled target-camera configuration.

After VIO initialization and publication of a reachable goal, run `planner_latency_native`
in another terminal. It also observes IMU, stereo CameraInfo and corrected pose
alongside propagated odometry. Run this under the complete combined workload;
the synthetic wire smoke does not substitute for live latency measurements.

The bridge's localization epoch guard detects a declared frame/session change.
It does not smooth or compensate visual-update corrections in position or velocity.
Those corrections pass downstream unchanged. The pre-existing body-odometry
adapter separately latches on gross discontinuity (>0.3 m + 5 m/s times dt),
which is far above the previously measured ~2.7 cm correction.

## SUPER A* route frontend

The default backend reuses the vendored SUPER `path_search::Astar`, sharing the
same ROG-Map instance as corridor/trajectory optimization. It requests the full
final goal (no local-horizon success or endpoint relocation), within the supplied
hard-boundary search box. Unknown space remains blocked and candidate edges use
the existing inflated collision queries. Native A* was adapted for lazy node
allocation, immutable priority-queue entries with decrease-key reinsertion, and
steady-clock deadlines that still expire while simulator time is paused.

A successful global route is cached. Goal changes, failed association with the
nearby route, or a geometry revision that invalidates its segments require a new
search. Unchanged valid geometry renews the route without rerunning A*. Failed
searches have a short retry backoff. `global_route.search_budget_ms` defaults to
150 ms and is bounded to 250 ms; partial/timed-out searches do not authorize a
trajectory. SUPER still generates and validates local trajectories. `/autonomy/route` remains available for visualization.

For existing camera and VIO streams, use the full-profile launch with
`start_localization:=false route_backend:=super_astar`. Initialize body/map
alignment at the surveyed start pose before carrying the drone through the course.
The launch attaches to `/autonomy/odomimu` and `/autonomy/poseimu` and never starts
camera or VIO processes in this mode.

## Static CAD scheduling

The mapper publishes a retained `MapSnapshot` only when geometry changes, plus
lightweight `MapState` updates with fresh measured pose and geometry identity.
Routes require healthy, fresh localization and collision-free map geometry.
SUPER replans at 10 Hz and issues certificates at up to 20 Hz. The bridge samples
certified polynomials at 50 Hz and stops output when certificates expire.
These are scheduling settings, not runtime guarantees.

Offline regression (no sensors or executive) is `super_grid_pipeline` under CTest.
Run in an isolated build container; it uses ROS domain 187 and synthetic inputs.
