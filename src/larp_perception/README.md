# Flight-stack visualization

The read-only Foxglove bridge exposes bounded `/viz/*` views of localization,
LAS depth, the ROG occupancy map, and planning. `full_stack.launch.py` starts
this launch plus `larp_autonomy/planner_visualization` by default.

Import `jetson/foxglove/larpcopter-autonomy-localization.json` and connect to
`ws://<jetson>:8766`. See [the deployment guide](../../docs/offboard-arm64.md).

For a standalone viewer: `ros2 launch larp_perception perception.launch.py`.
Set `viz_tracks:=true` only when feature-track images are needed; rendering them
adds work to the estimator. Feature point clouds and bounded pose paths are
already enabled. The bridge does not permit client topic publishing or services.
