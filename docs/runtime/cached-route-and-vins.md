# Cached-route mission runtime

`bash launch_mission_tmux.sh` selects the route follower by default. Explicit
`--planner super` and `--planner ego` remain available. The follower uses the
same ROG map, dynamic depth observations, geometry certification, and mission
executive; it replaces trajectory optimization with closed-form quintics.

Before takeoff, it computes and caches the geometric route from the projected
takeoff position. The executive waits for `/autonomy/route_ready`. No timed
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

* Search: at most 256 open-queue pops per call, with a cooperative 5 ms slice.
* Candidate generation: at most twelve candidates; do not start another after
  20 ms has elapsed. The existing whole-curve validators have their own limits.
* Replanning retains the current certified curve until replacement succeeds.
  A failed map certification invokes the existing certified-stop path; an
  unavailable command is not replaced by an unchecked shortcut.

These are work limits and cooperative deadlines, **not a hard worst-case
execution-time guarantee**. Individual geometry checks, map integration, route
reconstruction, allocation and Linux scheduling can exceed a time slice. Camera
processing and mapping also need measured timing under the deployed load.

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
