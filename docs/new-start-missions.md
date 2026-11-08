# New-start missions

The operator places the aircraft at old map (1.2, 0, 0), facing 90 degrees right. This becomes a new mission origin (0, 0, 0), yaw zero. No completed mission state is inherited.

Coordinates use `p_new = R @ (p_old - [1.2,0,0])`, with proper rotation `R = [[0,-1,0],[1,0,0],[0,0,1]]`. Sensor/body extrinsics remain unchanged. Map registration starts at zero; PX4 ground alignment uses the existing procedure.

On Jetson, prepare once from the deployed original configuration/assets:

```bash
cd ~/flight-stack-arm64
python3 prepare_new_start.py "$PWD"
```

This creates `config_new_start/mission.yaml`, a rotated CAD preload, a cached guide with a nearby entry for the new task, and independent `config_new_start/fixed_course.yaml`. The original config remains available. The cached guide is guidance only; the live map controls collision checks.

Launch one:

```bash
bash launch_super_new_start_tmux.sh
# Or:
bash launch_fixed_course_new_start_tmux.sh
```

The SUPER-named launcher uses the latest default follower planner, matching the current full mission. Both wrappers select the new configuration after deployment.env is sourced. Fixed-course has no depth/map/planner, as before. Its independent task at 1 m height visits:

```text
0        0         1
4.8261   0         1
4.8261  -4.5863    1
0.77724 -4.5863    1
0.77724 -4.02128   1
```

The transformed goal yaw is +pi/2; fixed-course retains its existing travel-heading behavior. New boundary: [-0.5334,-5.3624,0] to [5.5626,0.7336,1.8].

Validation: deployed runtime config loader and fixed-course construction passed offline on Jetson, including start and goal agreement. No mission was started or flight-tested by this change.
