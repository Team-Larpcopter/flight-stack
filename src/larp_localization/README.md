# larp_localization: sqrtVINS on the OAK-D S2

Visual-inertial odometry with [sqrtVINS](https://github.com/rpng/sqrtVINS)
(RPNG, University of Delaware, LGPL-3.0; a square-root-filter fork of OpenVINS),
running next to cuVSLAM for comparison. It does not feed PX4 yet.

```
larp_oak ── /oak/{left,right}/image_raw (1280x800 mono8, 30 Hz) ──▶ larp_undistort
         ── /oak/{left,right}/camera_info (rational_polynomial)  ──▶   │
         ── /oak/imu/data (200 Hz) ─────────────────────────────┐      ▼
                                                                 │  /oak/{left,right}/image_undistorted
                                                                 ▼      (pinhole, zero distortion)
                                                      ov_srvins (run_subscribe_msckf)
                                                         └─▶ /ov_srvins/odomimu, poseimu, pathimu
```

## Why an undistortion node

sqrtVINS (like OpenVINS) models only pinhole-radtan and equidistant lenses. The
OAK-D S2 factory calibration is an 8-coefficient rational model. Fitted to this
camera's calibration (`tools/oak_to_kalibr.py`, 1280x800):

| Model fitted to the OAK lens | Mean error | p95 | Max (corners) |
|---|---|---|---|
| radtan (k1, k2, p1, p2) | 1.9 px | 2.9 px | 18-24 px |
| equidistant (k1..k4) | 0.7-1.3 px | 1.5-3.0 px | 6-8 px |

Both are too far off for feature tracking, so `larp_undistort` removes the
distortion exactly (OpenCV, same 8 coefficients; checked to 0.0 px against the
depthai model in `test/test_calibration.py`) and sqrtVINS sees a pinhole camera
with zero distortion. The undistortion keeps K, the image size and the optical
frame, so the camera-IMU extrinsics are unchanged.

## Files

| Path | What |
|---|---|
| `larp_localization/undistort.py` | `undistort` node: rational -> pinhole remap per camera |
| `launch/sqrtvins.launch.py` | undistort + `ov_srvins/run_subscribe_msckf` with the OAK config |
| `config/oak_d_s2/oak_calib_skybeta15.json` | Factory calibration of the skybeta-15 OAK-D S2 (K, D, TF) |
| `config/oak_d_s2/kalibr_imucam_chain.yaml` | Generated: cameras (pinhole, zero distortion, undistorted topics) and T_cam_imu |
| `config/oak_d_s2/kalibr_imu_chain.yaml` | Generated: IMU noise (**starting guesses**, see below) |
| `config/oak_d_s2/estimator_config.yaml` | sqrtVINS settings, from sqrtVINS `config/euroc_mav` with the overrides below |
| `tools/dump_oak_calib.py` | Read K, D and TF from the running ROS graph (read-only) into a JSON |
| `tools/oak_to_kalibr.py` | JSON -> Kalibr-format configs; `--undistorted` for the node above |

Regenerate for another camera:

```bash
python3 tools/dump_oak_calib.py oak_calib.json      # with larp_oak running; read-only
python3 tools/oak_to_kalibr.py oak_calib.json config/oak_d_s2 --undistorted
```

## Settings (vs sqrtVINS `euroc_mav`)

`euroc_mav` is the maintained config for this sqrtVINS version; `rs_d455` is
missing required keys (`use_imuavg`, `use_rk4int`, the `init_*` initializer set)
and makes the node exit with "unable to parse all parameters".

| Key | Value | Why |
|---|---|---|
| `use_stereo` / `max_cameras` | true / 2 | OAK-D S2 stereo pair |
| `track_frequency` | 31 | camera at 30 Hz |
| `downsample_cameras` | false | 1280x800; set true (640x400) if CPU-bound |
| `num_opencv_threads` | 2 | leave cores for the rest of the stack |
| `init_max_reproj` / `init_min_feat` | 2.0 / 30 | values the euroc config recommends for stereo |
| `record_init_pose` / `record_init_timing` | false | do not write logs to /tmp on the drone |
| `try_zupt` / `zupt_only_at_beginning` | true / true | zero-velocity updates on the pad |
| `calib_cam_extrinsics` / `_intrinsics` / `_timeoffset` | true | refine the nominal IMU pose and timing online |

## Known gaps

- **IMU noise is a guess.** `kalibr_imu_chain.yaml` uses the cuVSLAM values from
  skybeta, inflated x2 (white noise) and x10 (random walk) as OpenVINS
  recommends. Replace with an Allan-variance calibration of this BNO086.
- **IMU extrinsics** come from the OAK's factory IMU->camera calibration, which
  larp_oak publishes as the oak_imu_frame TF (raw IMU axes). The earlier nominal
  URDF pose had gravity flipped relative to the cameras and made sqrtVINS drift
  ~1.1 m while still; with the factory pose a 3-minute static test drifted 2.5 mm.
- **Accelerometer scale:** |g| reads 9.16 m/s^2 at rest (~7% low). sqrtVINS
  absorbs it as an accelerometer bias; a proper IMU calibration is the real fix.
- **Output stamps:** sqrtVINS runs in float32 and its odometry header stamps are
  quantised to 128 s at current Unix time. Positions are fine; fix the stamps
  before feeding PX4.
- **Not yet run on drone data.** Built (x86 laptop) and started with this config:
  subscribes to `/oak/imu/data` and `/oak/{left,right}/image_undistorted`, publishes
  `/ov_srvins/{odomimu,poseimu,pathimu,points_slam,points_msckf,trackhist}`.
- **Known upstream issue:** `run_subscribe_msckf` segfaults while shutting down
  (`~ROS2Visualizer` / image_transport). Harmless at exit, but it hides error
  messages; run it with `stdbuf -o0` when debugging start-up.

## Testing on the drone

```bash
~/team_ws/sqrtvins_ws/src/larp_localization/scripts/vio_test.sh square1 90
```
Restarts sqrtVINS (keep still until GO), records sqrtVINS and cuVSLAM, prints
drift and the aligned comparison (`tools/compare_vio.py`). The workspace lives in
`~/team_ws/sqrtvins_ws` with a `COLCON_IGNORE` so the stack's team_build skips it;
build it with `--base-paths src`.

## Building

sqrtVINS is C++ (Eigen-heavy). Build it in its own workspace:

```bash
mkdir -p ~/sqrtvins_ws/src && cd ~/sqrtvins_ws/src
git clone https://github.com/rpng/sqrtVINS.git
git -C sqrtVINS checkout 30fafc8
git -C sqrtVINS apply ../../src/larp_localization/patches/sqrtvins-30fafc8-fstream.patch   # see below
git -C sqrtVINS apply ../../src/larp_localization/patches/sqrtvins-30fafc8-zupt-cleanup.patch
cd .. && source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --executor sequential --packages-up-to ov_srvins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ARUCO_TAGS=OFF \
  -DBoost_DIR=/usr/lib/cmake/Boost-1.80.0    # on the drone (skybeta image), see below
```

`patches/sqrtvins-30fafc8-zupt-cleanup.patch` fixes a memory leak: when a
zero-velocity update is accepted, VioManager returned before the only place the
feature database is cleaned, so a stationary drone grew it without bound
(~60 MB/s, OOM-killed after ~40 s). The patch cleans measurements older than the
oldest clone in both ZUPT branches, as the normal path does.

`patches/sqrtvins-30fafc8-fstream.patch` adds `#include <fstream>` to four
sqrtVINS files that use `std::ofstream` without it. x86 builds pick the header up
transitively; the Jetson (aarch64, L4T R36) build fails with "aggregate
'std::ofstream outfile' has incomplete type". This is our only change to
sqrtVINS (LGPL-3.0: the modification is published here).

The NVIDIA base image ships Boost 1.80 headers in /usr/include and Boost 1.80
libraries in /usr/lib, next to Ubuntu's 1.74 libraries and CMake config. CMake
picks 1.74 by default and the executables then fail to link
(`undefined reference to boost::filesystem::path::find_parent_path_size()`).
Build on the drone with `-DBoost_DIR=/usr/lib/cmake/Boost-1.80.0`.

On a laptop without swap, run this inside a memory-capped container
(`docker run --memory=4g --memory-swap=4g ...`): two parallel compiler jobs
exhausted 14 GB of RAM and froze the machine. On the drone it would go into the
team image (aarch64) once it is validated.
