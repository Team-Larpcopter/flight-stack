# New-start missions

The operator places the aircraft at old map (0, 0, 0), facing 90 degrees right. This becomes a new mission origin (0, 0, 0), yaw zero. No completed mission state is inherited.

All mission geometry is stored in `jetson/offboard/assets/new_start/`: mission config, CAD preload, cached guide, and independent fixed-course waypoints. Coordinates were converted during development and saved. No coordinate-conversion generator is shipped, and build/launch performs no mission-asset rotation or translation. Ordinary live sensor/body/PX4 frame handling remains unchanged.

Packaging copies these static assets directly into `config_new_start/`. On a fresh deployment, copy the aircraft calibration files alongside them (already done on the current Jetson):

```bash
cd ~/flight-stack-arm64
cp config/rig.json config/bridge.yaml config_new_start/
```

Map registration starts at zero; PX4 ground alignment uses the existing procedure. The original mission remains available. The cached guide is guidance only; the live map controls collision checks.

Launch one:

```bash
bash launch_super_new_start_tmux.sh
# Or:
bash launch_fixed_course_new_start_tmux.sh
```

The SUPER-named launcher uses the latest default follower planner, matching the current full mission. Both wrappers select the new configuration after deployment.env is sourced. Fixed-course has no depth/map/planner, as before. Its independent task at 1 m height visits:

```text
0        0         1
0        1.2       1
4.8261   1.2       1
4.8261  -3.3863    1
0.77724 -3.3863    1
0.77724 -2.82128   1
```

The transformed goal yaw is +pi/2; fixed-course keeps yaw zero (physical old-right) during its first sideways leg using `segment_yaws_rad`. Remaining legs use the existing smoothed travel heading. New boundary: [-0.5334,-4.1624,0] to [5.5626,1.9336,1.8].

Validation: deployed runtime config loader and fixed-course construction passed offline on Jetson, including start and goal agreement. No mission was started or flight-tested by this change.
