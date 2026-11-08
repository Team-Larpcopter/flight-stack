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

The current mission loads the precomputed CAD master route and searches forward reconnections at a fixed 10 Hz, even when the working
route is usable. The previous updated route remains a candidate; shorter clear
connections can replace it. All collision queries use the live ROG map. Completed route sections
are excluded from the rejoin candidates. Required tunnel/window passages stay
ordered. The map, whole-body collision validator and stopping behavior remain
active. See [cached-route runtime](../../docs/runtime/cached-route-and-vins.md)
for the heuristic, bounds, regeneration command and recovery behavior.

Local curves use C2 quintics whose Bernstein controls stay within 3 cm of
checked route chords. Route clearance reserves that extra 3 cm without reducing
the configured body radius. There is one construction and one candidate validation,
no smoothing retry cascade. Replacements preserve position, velocity, acceleration
and yaw derivatives through a future handoff; failures retain the current valid
curve. Unchanged map revisions do not revalidate the same committed curve.

Search has a 2 ms prefix slice plus a 5 ms search slice, 64 collision-query and
256 queue-pop caps. Candidate generation has a cooperative 40 ms deadline and
32-knot cap. Late timer periods are skipped; bridge subscription depth is one.
The default replacement period is 0.1 s, replacing the former two-second-tail
trigger. Planner logs report actual total planning and search times for bag review.
A visual-health Boolean flicker alone no longer clears a curve while body pose and
map are fresh; ground readiness and map-origin alignment gates remain intact.

Work caps are cooperative, not a hard Linux WCET guarantee. This implementation
does not certify thrust/attitude or continuous jerk. It sends the same position,
velocity, acceleration, yaw and yaw-rate fields as `flight_trajectory`.

## Package layout

| Path | What |
|---|---|
| `src/route_follower_node.cpp` | `depth_map_node` copy; only the planner backend differs |
| `include/larp_route_follower/follower_route_planner.hpp` | follower `PlannerContract` backend |
| `include/larp_route_follower/hermite.hpp` | closed-form quintic segments |
| `launch/mission_runtime.launch.py` | one component per call; planner = follower, others = larp_autonomy's |
| `launch/follower_system.launch.py` | whole stack in one `ros2 launch` (no tmux) |
| `scripts/launch_follower_system_tmux.sh` | tmux stack (same panes as `launch_system_tmux.sh`) |
| `scripts/launch_follower_full_system_tmux.sh` | shadow wrapper (like `launch_full_system_tmux.sh`) |
| `scripts/launch_follower_mission_tmux.sh` | hardware wrapper (like `launch_mission_tmux.sh`) |

## Run

Jetson tmux stack, the usual way: copy the three `scripts/launch_follower_*` files next to
the deployed `launch_system_tmux.sh` (they use that folder's `deployment.env`,
`replace_stack.sh` and `runtime_mounts.sh`; or set `DEPLOY_DIR` to it), then

```bash
bash launch_follower_full_system_tmux.sh            # shadow: plans, nothing executed
bash launch_follower_mission_tmux.sh                # hardware: pilot arms, then Position
bash launch_follower_system_tmux.sh shadow --check  # preflight only
```

Inside the runtime container (installed package):

```bash
# Whole stack in one launch, or a subset with components:=depth,map,planner
ros2 launch larp_route_follower follower_system.launch.py mission_file:=/config/mission.yaml mode:=shadow
# One pane/component, same interface as larp_autonomy's mission_runtime.launch.py
ros2 launch larp_route_follower mission_runtime.launch.py component:=planner \
  mode:=shadow mission_file:=/config/mission.yaml
```

Follower arguments (all launch files): `follower_horizon_m` (default 1.5),
`follower_replan_period_s` (0.1), `follower_route_file` (defaults to the installed CAD master route). A route file is a
map-frame text file, one `x y z` per line, sampled at <=15 cm, with optional
`# gates` passage indices. Its endpoint must match the mission goal within 0.1 mm.

## Deployment integration

The standard ARM64 build includes this package, and `package_bundle.sh` copies
all three follower launchers into the deployment folder. Launching another mission
also replaces an existing follower container/session. The follower delegates VINS,
DDS and the executive to the shared runtime, including its current forwarding and
executive fixes. Hardware flight validation remains an operator integration step.

## Tests

`colcon test --packages-select larp_route_follower` (needs the `larp_navigation`
source tree next to this package):

- `hermite`: boundary conditions, C2 continuity, rest at the end, peak speed.
- `follower_planner` (test and flight config): free-space shortcut, stop short of a
  depth-observed wall, follow a corner around a pillar, keep a live curve, preserve the
  committed curve on failure, route file parsing.
- `closed_loop`: fast-forward trajectory tracking against the map, including
  command continuity and whole-body collision checking. Optional CAD and master
  route arguments exercise the complete course.
- `master_route`: forward repair, blocked/cleared map updates, bounded work, ordered
  progress and unchanged-course traversal with zero A* searches.
- `launch_wrapper`: planner parameters and remappings equal larp_autonomy's, except
  the executable and follower-only parameters; the whole-stack launch starts every
  component through the follower runtime.
- `launch_scripts`: scripts parse, run the same panes as `launch_system_tmux.sh`, wrap
  the right mode, and stop before starting anything when deployment helpers are missing.
