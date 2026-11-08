# larp_route_follower

A deterministic replacement for SUPER/EGO trajectory optimization, behind the same
`PlannerContract`. Everything else is identical to the current stack: the node is a
copy of `larp_navigation`'s `depth_map_node` with only the planner backend swapped, and
it uses the same mission file, parameters, remappings and topics. Run **one** planner
stack at a time: both publish the same topics.

Nothing outside this package is modified. The map (ROG + CAD prior + launch
clearance), SUPER's global A\* route, the whole-curve validator and the certified stop
are reused read-only from `larp_navigation`'s source tree. Only four SUPER sources are
compiled (A\* and the polynomial containers); no corridor or MINCO optimizer.

## How it plans

Each 100 ms plan tick (the node's existing cadence):

1. **Route section**: from the global route (SUPER A\*, or an optional preloaded
   polyline), take the next `follower_horizon_m` metres from the drone's progress along
   the route (`RouteGuide`: arc-length progress, 2 m association window, 0.5 m
   connector). The section is **cut at the first span that is occupied or unobserved in
   the live map** (minus 0.15 m). If the goal is within reach on a certified straight
   chord, go straight to it.
2. **Shortcut** corners only where the replacement chord passes the certifier's own
   body-sphere check.
3. **Closed-form curve**: quintic Hermite pieces through the waypoints (continuous
   position, velocity, acceleration), corner speed scaled by `cos(turn)`, timed so the
   nominal peak is the configured `traj_opt.boundary.max_vel`, ending at rest. From
   rest with a large heading change it turns first, then moves.
4. **Certify** the whole curve (sphere swept against occupied **and unknown** voxels,
   hard velocity/acceleration limits, yaw rate/acceleration) with the existing
   validator. Attempts: smooth, stop-at-corners, end pulled back 0.1/0.25 m, slower
   timing. Typical plan time: 0.15-0.65 ms.
5. **Consistency**: a live certified curve is kept for `follower_replan_period_s` and
   replans hand over at the current state (no command jump). A failed replan never
   blanks the command.

When nothing new certifies: keep the current curve only while it still certifies in the
latest map (checked every 50 ms); otherwise the certified braking curve; otherwise the
node publishes INVALID, exactly as with SUPER/EGO. A blocked preloaded route is truncated
before the obstacle and the follower falls back to A\* from the current position.

Not modelled explicitly: jerk, thrust and attitude limits (the hard velocity and
acceleration limits are certified; at the configured 0.06 m/s and 0.04 m/s^2 the vehicle
is far inside its envelope).

## Run

```bash
# Every component except the planner runs larp_autonomy's unchanged launch.
ros2 launch larp_route_follower mission_runtime.launch.py component:=planner \
  mode:=shadow mission_file:=/config/mission.yaml \
  follower_horizon_m:=1.5 follower_replan_period_s:=0.4 follower_route_file:=
```

Jetson tmux (same panes/container/recorder as `launch_system_tmux.sh`, planner pane =
follower): copy `scripts/launch_follower_system_tmux.sh` next to the deployed
`launch_system_tmux.sh`, then

```bash
bash launch_follower_system_tmux.sh shadow   # or hardware
```

Optional preloaded route: a text file in the map frame, one `x y z` per line (`#`
comments), ending within 0.3 m of the mission goal; pass `follower_route_file:=<path>`.

## Integration still to do (outside this package, intentionally untouched)

- Add `larp_route_follower` to the package list in
  `jetson/integration_image/build_workspace.sh` (or build it as an overlay) so the
  deployment bundle contains `route_follower_node`.
- Drone test in shadow mode with live camera before hardware mode.

## Tests

`colcon test --packages-select larp_route_follower` (needs the `larp_navigation`
source tree next to this package):

- `hermite`: boundary conditions, C2 continuity, rest at the end, peak speed.
- `follower_planner` (test and flight config): free-space shortcut, stop short of a
  depth-observed wall, follow a corner around a pillar, keep a live curve, preserve the
  committed curve on failure, route file parsing.
- `closed_loop` (test and flight config): whole route around a pillar with depth-like
  sensing as the body moves, at the node's cadence. Flight config: 63.6 s, 0 failures,
  0 blank commands, 0 braking, 0 mm commanded jump between commits, max plan 0.31 ms,
  6 mm from the goal.
- `launch_wrapper`: planner parameters and remappings equal larp_autonomy's, except
  the executable and follower-only parameters.
