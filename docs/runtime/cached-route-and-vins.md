# Cached-route mission runtime

`bash launch_mission_tmux.sh` selects the route follower by default. Explicit
`--planner super` and `--planner ego` remain available. The follower uses the
same ROG map, dynamic depth observations, geometry certification, and mission
executive; it replaces trajectory optimization with closed-form quintics.

The current course loads `larp_route_follower/config/course_master_route.xyz`,
precomputed from the 5 cm CAD to the shifted goal (-2.82128, -0.77724, 1.0).
Before takeoff it validates the upcoming section from the projected takeoff position. The executive waits for `/autonomy/route_ready`. No timed
trajectory is advanced during WAITING or TAKEOFF. HOLD/TRACKING starts execution
from measured position. Loss of command certification discards the timed curve,
while retaining the route for recovery. New execution timestamps are assigned
at commit, not at the beginning of search.

A valid committed curve is retained until its last two seconds, so repeated
replanning cannot keep retiming the approach to a corner. Map certification can
still interrupt it immediately. A* edges and trajectory certification use the
same full-body clearance geometry; an inflated-grid route alone is insufficient.

Depth revisions do not trigger whole-course revalidation. The follower checks
its local route section and the commanded curve against the current map; a
blocked local section causes a resumable search. Unaffected distant geometry
stays cached. The map itself retains the existing occupancy and depth policies.

## Timing contract

* Local repair: at most 256 open-queue pops, 64 uncached body-clearance queries,
  40,000 stored nodes and a cooperative 5 ms slice. A search resumes across ticks.
* Smoothing: at most 64 knots, 8 arc samples per corner, linear forward/backward
  speed passes and closed-form quintics. No whole-flight convolution at runtime.
* Candidate generation: one rounded C2 curve from the speed profile. No shorter,
  slower or corner-stop retry cascade. The whole-curve validators retain their limits.
* Replanning retains the current certified curve until replacement succeeds.
  A failed map certification invokes the existing certified-stop path; an
  unavailable command is not replaced by an unchecked shortcut.

These are work limits and cooperative deadlines, **not a hard worst-case
execution-time guarantee**. Individual geometry checks, map integration, route
reconstruction, allocation and Linux scheduling can exceed a time slice. Camera
processing and mapping also need measured timing under the deployed load.

## Master route and local repair

The installed route has 132 samples over 12.86 m, with ordered tunnel entry/exit
and window entry/exit passage indices. Start and final height are 1 m; the CAD
window transit is 0.6948 m to clear the complete 0.3 m body sphere. Geometry is
precomputed offline; live occupied/unknown voxels always override it.

Only forward rejoin samples before the next required passage are eligible.
A single multi-target A* uses 10 cm search steps (the map and body checks remain
5 cm) and `h = min(diagonal_distance_to_rejoin_cell + exact_connector_length +
cached_tail_length)`. Edge cost is distance; no additional clearance field is
computed. It first searches a local region with 0.6 m padding / 3 m forward
route range, then expands to 1.2 m padding / 6 m range if blocked. Completed
route sections are not candidate goals; free space behind the vehicle remains
available for a short retreat. Failed searches back off before retrying.

Progress follows ordered adjacent segments, not global nearest-point matching.
Repair interior points do not advance master progress until the rejoin is
reached. Candidate rejoins require a free onward connector. Search progress can
survive live map revisions; a replacement is revalidated before commitment.
Only upcoming geometry is checked, not the entire CAD tail. Collision-cache
entries are invalidated on each map revision; this version does not yet use
per-voxel dirty-segment notifications.

Trajectory geometry uses the circular fillets and forward/backward speed-pass
approach in `flight_trajectory/trajectory.py`, then C2 quintic interpolation for
bounded local work. It does not reproduce that module's whole-flight raised-
cosine table or its continuous-jerk guarantee. Travel yaw follows the rounded
curve through continuous yaw/rate/acceleration polynomials in up-to-two-second
blocks, rate-limited independently of tiny arc pieces; heading may lag tight turns. The existing bridge
and executive send position, velocity, acceleration, yaw and yaw-rate feedforward
in local NED, the same fields used by `flight_trajectory`.

Regenerate on the development host (numpy/scipy; no runtime dependency):

```bash
python3 src/larp_route_follower/tools/precompute_cad_route.py \
  src/larp_navigation/config/course_cad_prior_fine.txt \
  src/larp_route_follower/config/course_master_route.xyz
```

The artifact is selected by the standard follower launch. A different mission
goal does not use this route: it falls back to the existing generic grid route.
A replacement `follower_route_file` must be sampled at <=15 cm and can specify
ordered passage indices with a `# gates ...` comment. No hash manifest is needed.

## Stationary VINS profiling

Run `bash profile_vins_load.sh 90` from the Jetson deployment directory. Results
are kept in `profiling/<timestamp>/`. This launches camera, VINS, depth, mapping,
follower and visualization in a private network/IPC container and ROS domain.
It does not map PX4 serial, start an XRCE agent, or start a hardware executive.
It leaves operator replay containers alone. Keep the aircraft stationary.

`vins_timing.csv` contains tracking/estimator stage times; `topic_timing.json`
contains arrival times and source-stamp ages for IMU, propagated odometry,
completed visual updates and depth. `resources.jsonl` records container load.
Stationary measurements characterize computation, not flight tracking quality.

The mission VINS launch enables a latest-frame camera policy: image DDS depth
one, stereo synchronization depth two, at most two pending camera observations,
and at most one visual update per drain call. When behind, older eligible images
are skipped in favor of the newest image with IMU coverage. IMU measurements
are retained for integration. Simulation retains the legacy queue policy unless
`low_latency_camera` is explicitly enabled. Periodic VINS logs expose visual
update duration, age, pending images and cumulative skipped images.

If no usable route persists for three seconds after translation stops, the
follower can hold its certified position and sweep yaw +45°, −45°, then back to
its original heading. It continues route search during the sweep and preserves
the yaw handoff when an opening becomes usable. It performs one sweep, then
holds; it does not advance into unseen space. Disable this recovery with
`follower_scan_when_blocked:=false` on the node. Mapping policies are unchanged.

Live mission tracking uses VINS's supported half-resolution camera path, which
scales intrinsics internally. Driver images, LAS inputs, feature-count limits,
IMU noise and online calibration settings are unchanged. `PROFILE_DOWNSAMPLE=1`
selects the same setting in an isolated profiling configuration.

## Local verification (2026-11-08)

Fast-forward closed-loop geometry tests use perfect tracking of the commanded
curve, not a claim about VINS or physical flight performance. The final single-
candidate implementation reached the goal in both fixtures:

| PC geometry fixture | Simulated duration | Max plan call | Failed / blank / unsafe / braking |
|---|---:|---:|---|
| Original CAD + precomputed route | 358.7 s | 4.94 ms | 0 / 0 / 0 / 0 |
| Estimated changed entrance + original precomputed route | 357.4 s | 5.03 ms | 0 / 0 / 0 / 0 |

The changed entrance is defined in `test/fixtures/estimated_entrance.json` and
produced with `tools/estimated_layout_fixture.py`. It is a schematic sensitivity
fixture from the operator sketch, not a surveyed map. It is never installed as
the flight map. The test leaves the tunnel and downstream CAD unchanged.

These local geometry-only tests used the PC's existing Jazzy development headers;
the flight package is separately compiled against Humble in the ARM64 toolchain.
