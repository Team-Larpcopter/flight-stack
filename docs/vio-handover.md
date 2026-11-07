# VIO on the drone: setup, testing and handover

How the camera, IMU, cuVSLAM and sqrtVINS are set up on the skybeta drone, how to
run and test them, how to read the results, and what is still open. Written for
whoever picks this up next. Branch: `hari/localization`.

Contents
1. [What runs where](#1-what-runs-where)
2. [Connecting and the stack](#2-connecting-and-the-stack)
3. [Camera and images (larp_oak)](#3-camera-and-images-larp_oak)
4. [IMU and calibration](#4-imu-and-calibration)
5. [sqrtVINS](#5-sqrtvins)
6. [Testing VIO](#6-testing-vio)
7. [Foxglove](#7-foxglove)
8. [Results so far](#8-results-so-far)
9. [How code gets onto the drone (and how it should)](#9-how-code-gets-onto-the-drone-and-how-it-should)
10. [CPU load and flight safety](#10-cpu-load-and-flight-safety)
11. [Gotchas](#11-gotchas)
12. [Open work](#12-open-work)

---

## 1. What runs where

```
 OAK-D S2 (USB)                         Jetson Orin Nano
 ─────────────                          ─────────────────────────────────────────────────────────
 left/right mono 1280x800@30 ──┐        docker skybeta-stack  (organisers' stack, our team image)
 on-device undistortion  ──────┤          tmux "skybeta": agent | slam | bridge | competitor | foxglove | viz
 on-device stereo depth  ──────┤          slam window:  larp_oak (camera node)  ──► /oak/*
 BNO086 IMU 200 Hz  ───────────┘                        cuVSLAM  ──► /visual_slam/tracking/odometry ──► PX4
                                                        depth_filter ──► /oak/stereo/filtered/image
                                          bridge:      flight interface (/skybeta/*) ◄──► PX4 (TELEM2)
                                        docker larp-sqrtvins (ours, capped 2 CPU / 1.5 GB)
                                                        sqrtVINS ──► /ov_srvins/*   (does NOT feed PX4)
                                        docker larp-bag      (only during vio_test.sh, 1 CPU)
                                        Foxglove bridge ws://<drone-ip>:8766 (read-only) + /viz/* image relay
```

- **cuVSLAM** (NVIDIA, organisers' choice) is what PX4 flies on today.
- **sqrtVINS** (rpng/sqrtVINS, an OpenVINS fork, LGPL-3.0) runs next to it for
  comparison. It does not publish anything PX4 reads.
- Everything talks over ROS 2 Humble on the stack's `ROS_DOMAIN_ID` with a
  localhost-only DDS profile (`~/skybeta/host/dds/fastdds_localhost.xml`), so
  ROS traffic never goes over Wi-Fi.

## 2. Connecting and the stack

```bash
ssh dronehacks@<drone-ip>        # Tailscale IP, or <hostname>.local on the same network
larp status                      # containers, VIO state, /skybeta/state
larp attach                      # stack tmux (Ctrl-b d to leave); windows below
larp shell                       # bash inside the stack, ROS sourced
larp ros ros2 topic hz /oak/imu/data
larp doctor                      # organisers' health check
larp restart                     # restart the whole stack (see warning)
larp work                        # host tmux session for dev work, survives SSH drops
```

`larp` is `jetson/larp` (installed to `~/.local/bin/larp` by `jetson/install.sh`).

Stack tmux windows: **agent** (organisers' status agent), **slam** (larp_oak camera
node, cuVSLAM, depth filter), **bridge** (flight interface to PX4), **competitor**
(team code: `team_build.sh` then `competitor.launch.py`), **foxglove**, **viz**.

> **⚠️ Every stack stop needs a flight-controller power cycle** (PX4 bug #15: TELEM2
> goes silent after the stack restarts). Restarting only the camera node (below) does
> not touch the PX4 link and needs no power cycle.

> **⚠️ The competitor window runs the organisers' example mission** (a 5 m square
> once armed and in Position mode). Keep props off / do not arm during hand tests.

Rules we keep: never publish to `/fmu/in/*`, never touch the organisers' doctor,
status services, tailscaled or `drone.env` identity, never ssh into other teams'
drones, never commit credentials.

### Restarting only the camera node

The launch file respawns `oak_node` (3 s delay) when it exits. After editing it:

```bash
docker exec skybeta-stack bash -c 'kill -9 $(pgrep -f lib/larp_oak/oak_node)'
```

Use `kill -9`: a SIGINT made the OAK crash on shutdown and left the process stuck
in USB I/O (state `D`) until killed. If Ubuntu's `apport` then burns a core
processing a crash dump, `sudo pkill -x apport`.

## 3. Camera and images (larp_oak)

`src/larp_oak/larp_oak/oak_node.py` replaces the organisers' depthai_ros_driver.
One depthai v3 pipeline, everything at 30 Hz (measured: all six streams hold 30 Hz):

| Topic | What | Used by |
|---|---|---|
| `/oak/{left,right}/image_raw` + `camera_info` | 1280x800 mono8, **distorted**; camera_info = `rational_polynomial` (8 coeffs) from the device calibration | cuVSLAM |
| `/oak/{left,right}/image_undistorted` + `/image_undistorted/camera_info` | 1280x800 mono8, **undistorted on the camera** (warp engine, full lens model); pinhole, D = 0, K = same as raw | sqrtVINS |
| `/oak/stereo/image_raw` + `camera_info` | 800x500 depth, uint16 mm, aligned to rectified left | depth filter, mapping |
| `/oak/stereo/filtered/image` | depth after the Jetson CUDA filter | planning / viz |
| `/oak/imu/data` | 200 Hz accel + gyro, frame `oak_imu_frame` | both VIOs |
| `/tf_static` | `oak_parent_frame` → rgb/left/right optical frames + `oak_imu_frame` | everyone |

Parameters (`ros2 run larp_oak oak_node --ros-args -p ...`): `fps` (30),
`vio_width/height` (1280x800), `stereo_width/height` (800x500), `depth_preset`
(ROBOTICS), `subpixel`, `imu_hz` (200), `imu_mode` (`raw`), `publish_depth`,
`vio_undistorted` (true).

**Why sqrtVINS gets separately undistorted images:** sqrtVINS only supports
pinhole-radtan or equidistant lenses; the OAK lens needs the 8-coefficient
rational model (radtan fit error: mean 1.9 px, max 24 px). So the distortion is
removed upstream and sqrtVINS sees a pinhole camera with zero distortion.

**Why on the camera, not on the Jetson:** the first version undistorted in a
Python node on the Jetson (`larp_localization/undistort.py`). It ran at 14-19 Hz
and dropped *different* frames on each side, so sqrtVINS paired left/right images
from different moments. Fine when still, fatal on a fast turn (square1 diverged
to kilometres). On-camera undistortion gives both sides from the same capture
(left/right stamps within 0.02 ms) at the full 30 Hz and costs the Jetson
nothing but the extra publish. Checked against OpenCV's full-model undistortion:
within ~0.25 px in the centre, ~1-1.5 px in the corners (the device's warp mesh
is coarser). If the device rejects the undistortion output, the node logs an
error and restarts without it; the Python node can then be used again
(`SQRTVINS_HOST_UNDISTORT=true`).

**Timestamps:** device timestamps are host-synced `CLOCK_MONOTONIC`, shifted into
ROS time. The monotonic→ROS offset is refreshed once a second (per-message clock
reads used to give the two images of one capture slightly different stamps).

**Cost:** each full-size image stream is ~31 MB/s through Python and DDS. The two
undistorted streams are extra; set `vio_undistorted:=false` when sqrtVINS is not
running (see [section 10](#10-cpu-load-and-flight-safety)).

## 4. IMU and calibration

Nothing is hard-coded: geometry comes from the camera's own factory calibration
at start-up, and remaining sensor errors are estimated online by the filter.

### IMU orientation (the bug that made sqrtVINS drift a metre while still)

- The OAK-D S2 has a BNO086. depthai offers RAW and CALIBRATED reports. On this
  unit CALIBRATED = RAW with y and z negated, and both read |g| = 9.16 m/s².
- The camera's factory calibration includes the IMU→camera transform
  (`calib.getImuToCameraExtrinsics(CAM_B)`), defined for the **RAW** axes
  (≈ diag(1, −1, −1), t ≈ (7.6, 0, −0.15) cm from the left camera).
- larp_oak uses **RAW** (`imu_mode: raw`) and publishes `oak_imu_frame` in TF
  from that factory transform: `T_parent_imu = T_parent_left · T_left_imu`.
- Previously the TF was a nominal URDF pose. Combined with RAW data, gravity
  came out pointing *up* in the camera frame. sqrtVINS drifted ~1.1 m while
  sitting still; with the factory transform a 3-minute static run drifted 2.5 mm.
- **Check it:** at start-up larp_oak logs the mean accel over ~1 s in the IMU
  frame and in the left optical frame. With the camera level and facing forward,
  the optical-frame value should be ≈ (0, −9.2, 0) (optical y points down, so
  gravity reads −y). Seen: `[-0.355 -9.149 0.282]`.

### Accelerometer scale and bias

- |g| reads **9.16 instead of 9.81** (7% low) in both modes. Not corrected by us.
- sqrtVINS absorbs it as an accelerometer **bias** (≈ −0.65 m/s², estimated at
  init and updated online, stable across runs). It can also estimate a scale
  matrix online (`calib_imu_intrinsics: true`, not enabled yet).
- The real fix is an IMU calibration: a multi-position accelerometer
  calibration and an Allan-variance run (also gives real noise values).

### IMU noise values

`config/oak_d_s2/kalibr_imu_chain.yaml`: borrowed from the organisers' cuVSLAM
config, inflated (×2 white noise, ×10 random walk). **These are guesses**, not
measured. An Allan-variance recording of the still drone (2-3 h) replaces them.

### Camera calibration in sqrtVINS

- Intrinsics and camera↔IMU extrinsics come from the device calibration, via
  `tools/dump_oak_calib.py` (reads `/oak/*/camera_info` + TF from the running
  stack into `oak_calib_<drone>.json`) and `tools/oak_to_kalibr.py --undistorted`
  (writes `kalibr_imucam_chain.yaml` etc.). Re-run both for another drone/camera.
- **Online camera calibration is OFF** (`estimator_config_fixedcalib.yaml`, the
  default). With it on (lift1), sqrtVINS moved both cameras ~6 cm at its first
  update (implied baseline 16 cm instead of 7.5 cm), triangulated points behind
  the camera and diverged. The factory calibration is good (0.0 px vertical
  stereo error measured), so we trust it. The camera-IMU **time offset** is still
  estimated online (settles around −9 ms).

## 5. sqrtVINS

Package `src/larp_localization`:

| Path | What |
|---|---|
| `launch/sqrtvins.launch.py` | sqrtVINS (`ov_srvins run_subscribe_msckf`, namespace `ov_srvins`); `undistort:=true` adds the old Python undistorter |
| `config/oak_d_s2/estimator_config_fixedcalib.yaml` | **default**: factory calibration fixed |
| `config/oak_d_s2/estimator_config.yaml` | same, but online camera calibration on (do not use) |
| `config/oak_d_s2/kalibr_imucam_chain.yaml` / `kalibr_imu_chain.yaml` | cameras + IMU, generated (section 4) |
| `patches/` | two patches against sqrtVINS 30fafc8 (below) |
| `scripts/sqrtvins_on_drone.sh` | start / stop / logs / status / bag / bagstop on the drone |
| `scripts/vio_test.sh` | one-command test (section 6) |
| `tools/` | `stationary_test.py` (record + summarise), `compare_vio.py` (aligned comparison), calibration tools |
| `foxglove/larpcopter-localization.json` | Foxglove layout |

**Outputs:** `/ov_srvins/odomimu` (nav_msgs/Odometry at IMU rate, frame `global`
→ `imu`), `poseimu`, `pathimu`, `points_msckf`, `points_slam`, `trackhist`. The
`global` frame is gravity-aligned with arbitrary yaw, starting where sqrtVINS
initialised; it is not connected to `odom`/`map` in TF (that is why Foxglove
needs the sqrtVINS 3D panel to follow `global`).

**Initialisation:** static. Keep the drone still for a few seconds after start;
it then uses zero-velocity updates (`try_zupt`) only at the beginning.

### Building on the drone (one-time, ~20 min)

sqrtVINS is not in the team image (see [section 9](#9-how-code-gets-onto-the-drone-and-how-it-should)).
It lives in `~/team_ws/sqrtvins_ws` (with a `COLCON_IGNORE` so the stack's
`team_build.sh` does not try to compile it at every start):

```bash
cd ~/team_ws/sqrtvins_ws/src
git clone https://github.com/rpng/sqrtVINS && git -C sqrtVINS checkout 30fafc8
# larp_localization: copy from the repo (src/larp_localization) into src/ first
git -C sqrtVINS apply ../larp_localization/patches/sqrtvins-30fafc8-fstream.patch
git -C sqrtVINS apply ../larp_localization/patches/sqrtvins-30fafc8-zupt-cleanup.patch
docker run --rm --memory=4g -v ~/team_ws/sqrtvins_ws:/root/sqrtvins_ws --entrypoint bash skybeta-team:$(cat ~/skybeta/VERSION) -c \
 'source /opt/ros/humble/setup.bash && cd /root/sqrtvins_ws && \
  MAKEFLAGS=-j2 colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBoost_DIR=/usr/lib/cmake/Boost-1.80.0'
```

- `-DBoost_DIR=...1.80.0`: the image has Boost 1.80 next to Ubuntu's 1.74;
  without it the link fails (`find_parent_path_size`).
- `-j2` at 4 GB: `-j3` runs out of memory.
- **fstream patch:** missing `#include <fstream>` (fails on this compiler).
- **zupt-cleanup patch:** a real sqrtVINS bug. When a zero-velocity update is
  accepted, `VioManager` returns before the only place the feature database is
  cleaned, so a still drone leaks ~60 MB/s and is OOM-killed after ~40 s.

Python-only changes (larp_localization tools, scripts, configs) need no rebuild:
copy the file into `~/team_ws/sqrtvins_ws/src/larp_localization/...` (the
container mounts the workspace; `vio_test.sh` and `SQRTVINS_CONFIG` read configs
from `src`, a plain `start` uses the installed share copy).

### Running

```bash
S=~/team_ws/sqrtvins_ws/src/larp_localization/scripts
$S/sqrtvins_on_drone.sh start      # container larp-sqrtvins (2 CPU, 1.5 GB cap)
$S/sqrtvins_on_drone.sh status     # odometry rate + CPU per process
$S/sqrtvins_on_drone.sh logs
$S/sqrtvins_on_drone.sh stop
SQRTVINS_CONFIG=estimator_config.yaml $S/sqrtvins_on_drone.sh start   # pick another config
```

## 6. Testing VIO

One command does everything:

```bash
~/team_ws/sqrtvins_ws/src/larp_localization/scripts/vio_test.sh NAME SECONDS [options]
#   --no-fox-reset   keep the Foxglove session (default: restart bridge + image relay)
#   --no-bag         skip the rosbag
#   --online-calib   use the online-camera-calibration config (for comparison only)
```

1. Starts a rosbag (`larp-bag` container), resets Foxglove, restarts sqrtVINS.
2. **Keep the drone still until it prints `GO`.**
3. Records `SECONDS` of both odometries, then prints a summary and the comparison.
4. Saves to `~/team_ws/sqrtvins_ws/results/<NAME>_<time>/`:
   `ov_srvins_odomimu.csv`, `visual_slam_tracking_odometry.csv` (columns
   `recv_s, stamp_s, x, y, z, yaw_deg, vx, vy, vz, lag_ms`), `summary.json`,
   `compare.json`, `sqrtvins.log`, `bag/` (mcap).

The results folder is owned by root (written from the containers). Read it
normally; to delete, use `docker exec larp-sqrtvins rm -rf /root/sqrtvins_ws/results/<dir>`.
Copy to a laptop: `scp -r dronehacks@<drone-ip>:team_ws/sqrtvins_ws/results/<dir> .`

### Test ladder

| Test | Command | Motion | Pass |
|---|---|---|---|
| Static | `vio_test.sh static 180` | none | drift a few mm |
| Lift | `vio_test.sh lift 20` | lift ~0.4 m straight up, hold, put back on the same spot | peak ≈ lift height, end ≈ 0 |
| Square | `vio_test.sh square 60` | carry around a 1-2 m square, end on the start mark | end within a few % of the path |
| Fast | `vio_test.sh fast 60` | quick moves and turns | no divergence |
| Long | `vio_test.sh loop 180` | 2-3 min loop | drift growth over time |

Mark the start spot on the floor. Keep the camera facing texture (not a blank
wall). Turn the Foxglove image relay off for fair cuVSLAM numbers (section 10).

### What the numbers mean

| Field | Meaning |
|---|---|
| `drift_m_end` | distance from the start at the end. For a closed loop this **is** the drift |
| `drift_m_max` | furthest from the start. Static: worst wander. Lift: should equal the lift |
| `yaw_drift_deg` | heading change start→end (unwrapped, so a full turn is ±360). The two VIOs use different frames: compare magnitudes |
| `rate_hz`, `max_gap_ms` | output rate and longest gap. cuVSLAM gaps > 100 ms mean it is starved |
| `drift_rate_mm_per_min`, `lag_ms_median` | static tests only. sqrtVINS lag is meaningless (see gotchas) |
| `between.rms_m / max_m / final_m` | disagreement between the two tracks after the best rotation+translation alignment |
| `alignment_yaw_deg` | heading difference between the two world frames (arbitrary) |
| `path_length_m` | total distance travelled; sqrtVINS's is inflated by jitter at 190 Hz |

Neither VIO is ground truth. When they disagree, look at the run second by
second (CSV) and at `sqrtvins.log` (it prints its state, biases, calibration and
time offset at every update). The bag allows replaying the exact run.

### Bags

`vio_test.sh` records an mcap bag: undistorted stereo, camera_info (raw and
undistorted), IMU, `/tf_static`, both odometries. ~56 MB/s (60 s ≈ 3.4 GB).
Raw images are not recorded (four full streams load the Jetson too much), so a
bag can replay sqrtVINS but not cuVSLAM. `ros2 bag info <dir>/bag` to inspect;
the organisers' flight-debug set is `larp bag`.

## 7. Foxglove

```bash
larp fox start --images     # bridge ws://<drone-ip>:8766 + /viz/* images at 5 Hz, half size
larp fox start              # telemetry only (no images: lighter)
larp fox status | larp fox stop
```

- Port **8766** (8765 is the organisers' status agent). Read-only bridge: no
  publishing, services or parameters from the client.
- Full-rate camera topics are never served (one stream is more than the Wi-Fi
  uplink); the relay publishes JPEG copies on `/viz/<topic>/compressed`.
- In Foxglove: *Open connection* → *Foxglove WebSocket* → `ws://<drone-ip>:8766`,
  then *Layout* → *Import from file* → `src/larp_localization/foxglove/larpcopter-localization.json`.
- Layout: left/right images, raw and filtered depth; a 3D panel for cuVSLAM
  (follows `odom`) and one for sqrtVINS (follows `global`, shows `pathimu` and
  the feature clouds); position plots for both; IMU accel/gyro plots; raw
  `/visual_slam/status` and `/skybeta/state`.
- If a 3D panel complains about a missing frame: sqrtVINS's `global` frame only
  exists while sqrtVINS runs, and it is not linked to `odom`; set each panel's
  display frame to its own estimator's frame.
- `vio_test.sh` restarts the bridge before each run so old paths are cleared;
  Foxglove reconnects by itself (or reconnect manually).

## 8. Results so far

skybeta-15, hand-held, cuVSLAM on raw images:

| Run | Setup | sqrtVINS | cuVSLAM |
|---|---|---|---|
| static (3 min) | after IMU fix | 2.5 mm end, 11 mm max, yaw 0.03° | 3.6-5.8 mm |
| lift1 (0.4 m) | online camera calibration | peak 2.25 m, end 1.63 m ✗ | peak 0.39 m, end 1.0 cm |
| lift2 (0.4 m) | fixed calibration | peak 0.41 m, end 2.6 cm; agrees with cuVSLAM to 1.1 cm RMS | peak 0.39 m, end 0.8 cm |
| square1 (60 s) | fixed calib, Python undistort | lost tracking on a ~300°/s turn, diverged to km ✗ | 0.21 m end after a 365° loop |
| square2 (60 s) | on-camera undistortion + bag | tracked the full turn, **0.60 m end** (~8.6% of path; 0.32 m of it vertical) | **starved** (7-10 Hz, 0.2-0.5 s frame gaps), jumped 3.5 m on the fast turn ✗ |

Raw data: `~/team_ws/sqrtvins_ws/results/` on the drone (square2 =
`square1_032536`, which has a bag).

## 9. How code gets onto the drone (and how it should)

### Today (three places, which is the problem)

| Code | Where it runs from | Updated by |
|---|---|---|
| larp_oak, larp_viz | **baked into the team image** (`jetson/team_image/Dockerfile` → `/root/skybeta_ws`, needed because `run_slam.sh` sets `SKYBETA_UNDERLAY_ONLY=1`) **and** built into `~/team_ws/install` by `team_build.sh` at every stack start (symlink-install, overrides the image copy: the running `oak_node` is `/root/team_ws/install/larp_oak/...`) | `larp image build` (slow) / editing `~/team_ws/src/flight-stack` + restarting the node (fast) |
| flight-stack checkout on the drone | `~/team_ws/src/flight-stack` | `git` on the drone (was left on `hari/jetson-tools` with local edits during this work) |
| sqrtVINS + larp_localization | `~/team_ws/sqrtvins_ws` (separate workspace, mounted into `larp-sqrtvins`) | manual build (section 5), files copied in by hand |

Consequences: the image copy of larp_oak is stale (it predates the IMU and
undistortion changes); if `team_build.sh` ever fails, the stack silently falls
back to that stale copy. Edits made on the drone are easy to lose. The image
has to be rebuilt for larp_oak changes to stick, and sqrtVINS is not in it at all.

### What it should be

1. **Heavy, rarely-changing C++ goes in the image, once.** Add a stage to
   `jetson/team_image/Dockerfile` that clones sqrtVINS at the pinned commit,
   applies `src/larp_localization/patches/*.patch`, and builds `ov_core` and
   `ov_srvins` (`-DBoost_DIR=/usr/lib/cmake/Boost-1.80.0`,
   `MAKEFLAGS=-j2`). Only rebuilt when the sqrtVINS commit or the patches change
   (~20 min, on the Jetson; never on a 14 GB laptop without a memory cap).
2. **Our fast-changing Python stays in `~/team_ws` as a symlink-install overlay**
   (larp_oak, larp_localization, larp_viz): edit, restart the node, done. No image
   rebuild for Python changes. `team_build.sh` builds them at stack start (ament_python,
   seconds).
3. **One source of truth on the drone:** `~/team_ws/src/flight-stack` is a git
   checkout of a branch. Push from a laptop, `git pull` on the drone, restart
   the node. Never leave uncommitted edits there.
4. **Keep the image copy in step** or remove the duplicate: rebuild the image
   (`larp image build`) whenever larp_oak changes in a way the stack must keep
   if `team_build.sh` fails, or make the slam window always source the team_ws
   overlay and drop larp_oak from the image.
5. **Launch sqrtVINS from the stack** (a tmux window or the slam launch, behind
   a flag) instead of the separate `larp-sqrtvins` container, once it runs from
   the image. Until then the container is fine and keeps it capped.

## 10. CPU load and flight safety

The Jetson is close to its limit. Measured cuVSLAM rate (it should be 22-30 Hz):

| Running | cuVSLAM |
|---|---|
| stack only | ~22 Hz |
| + sqrtVINS + Python undistorter + Foxglove images | ~9 Hz |
| + on-camera undistortion + sqrtVINS + bag + Foxglove images (square2) | 7-10 Hz, frame gaps up to 0.5 s → lost track |

**Before any flight:** cuVSLAM must have the CPU. Stop sqrtVINS and bags,
`larp fox start` without `--images`, and run larp_oak with
`vio_undistorted:=false`, unless PX4 is switched to sqrtVINS (which first needs
the timestamp fix and divergence detection below). Check
`larp ros ros2 topic hz /visual_slam/tracking/odometry` before arming.

## 11. Gotchas

- **sqrtVINS header stamps are float32-quantised** (128 s steps at current Unix
  time). Positions are fine; the tools use receive time instead. Must be fixed
  before sqrtVINS could feed PX4.
- **sqrtVINS has no divergence detection or reset**: once lost, it integrates
  the IMU into kilometres. A wrapper must watch speed / covariance / disagreement.
- `pkill -f PATTERN` over ssh can kill your own ssh command if the pattern is in
  it; use `pgrep` with a specific path, or `kill -9 <pid>`.
- OpenCV YAML configs must start with `%YAML:1.0`; a comment above it breaks parsing.
- `ros2 topic hz` from inside the stack (`larp ros ...`) is reliable; a Python
  probe with `spin_once` under-counts high-rate topics (showed 143 Hz for the 200 Hz IMU).
- Don't put new colcon workspaces under `~/team_ws` without a `COLCON_IGNORE`:
  `team_build.sh` builds everything below it at stack start (it once tried to
  compile sqrtVINS there, starving cuVSLAM and blocking the competitor window).
- Laptop: building sqrtVINS without a memory cap froze it (no swap). Use
  `docker run --memory=4g` and `-j1`/`-j2`. Simulator on the laptop: ≤ 5 Hz images.
- Branch names: `hari/<feature>`; no assistant co-author lines.

## 12. Open work

1. **Fair comparison**: measure cuVSLAM rate per configuration; rerun square with
   images off in Foxglove so cuVSLAM is not starved.
2. **Offline replay** of the square2 bag on a laptop to tune sqrtVINS (noise,
   features, `calib_imu_intrinsics` for the 7% scale) without new hand tests.
3. **IMU calibration**: Allan variance (noise) and multi-position accelerometer
   calibration (scale, bias).
4. **sqrtVINS for flight**: fix output stamps, divergence detection + auto-reset,
   publish in the frame and rate PX4's vision input expects.
5. **Image integration** (section 9): sqrtVINS into the team image, one git
   checkout on the drone, rebuild the image so its larp_oak is current.
6. Height drift (0.32 m in square2): check after the IMU scale work.
