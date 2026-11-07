# Native localization and fixed-map navigation integration

The integration branch combines the existing Jetson camera/localization tooling
with patched sqrtVINS, ROG-Map, Air-FAR and SUPER. Dependencies are vendored as
ordinary source directories, without Git submodules. `references/`, builds,
SDK downloads and hardware evidence are excluded from Git.

## Current architecture

OAK performs stereo lens undistortion. The native C++ acquisition node publishes
1280x800 stereo at 30 Hz, paired at average mid-exposure time, with a 0.5 ms pair
skew gate. It publishes raw accel/gyro independently. Optional VIO IMU alignment
applies an explicit raw-axis accelerometer correction once, then interpolates
between bracketing acceleration samples at gyro timestamps. It never extrapolates
and rejects brackets wider than 10 ms. Current bench requests are 500 Hz accel,
400 Hz gyro and a candidate 5 ms exposure cap; these are not universally tuned
flight settings. IMU noise parameters still require characterization.

`native_calibration.py` generates OpenCV-compatible Kalibr files from SDK
camera-to-IMU exports and actual image projections, converting centimetres to
metres. Intrinsics/extrinsics stay fixed; temporal offset is estimated. A generated
bench frame is explicitly distinguished from a surveyed aircraft mount. No CAD
pose matcher is used.

The body adapter handles frame and lever-arm conversions and supports explicit
initial position/yaw in the CAD coordinate system. Fixed-CAD planning requires an
explicit geofence parameter file. Air-FAR's route feeds SUPER's geometric frontend;
the trajectory bridge samples its certified stopping polynomial. Depth is computed on
the camera (800x500, rectified left), filtered on the Jetson by
`larp_perception` and published on `/stereo/depth/image` for `depth_map_node`;
live map updates from it are not yet validated on hardware.

`px4_bench_node` publishes external vision and certified setpoints only with
explicit enablement, fresh disarmed status and healthy VIO. Setpoints additionally
require verified map/NED alignment. It sends no arming, mode-change or offboard
heartbeat messages. The existing flight executive remains simulation-only and
must not be launched on hardware. PX4 messages match the hardware image's
release/1.16 revision recorded in `src/px4_msgs/UPSTREAM_REVISION`.

## Verification on 2026-11-07

- Patched sqrtVINS and native camera/PX4 targets built on Jetson ARM / ROS Humble.
- Three native ARM tests passed: frames, command admission, IMU alignment.
- Thirteen autonomy Python tests passed locally, including an OpenCV FileStorage
  regression check for generated camera calibration.
- A 23-second diagnostic window received about 28.5 Hz updated VIO poses,
  approximately 48 ms median / 50 ms p95 pose capture age, and approximately
  400 Hz aligned IMU. These are brief bench observations, not flight qualification.
- Corrected container DDS configuration enabled pose reception inside Foxglove.
  Use host networking, shared host IPC, the vehicle's DDS profile and ROS domain.
- Fixed shutdown teardown of global ROS objects after a reproducible exit crash.
- PX4 UART telemetry was received and reported disarmed without failsafe. Actual
  external-vision fusion and planner-to-PX4 consumption have not been verified.
- Planner compilation was blocked by missing ROS PCL packages; dependencies were
  installed in an isolated image, but a successful full planner build/run on the
  Jetson is still outstanding.

Moving accuracy, surveyed airframe mounting, map/PX4 registration, reset handling,
full-system latency under planner load and real flight remain unqualified.

## Running localization

`jetson/integration_image/Dockerfile` fixes inherited Boost header/library mismatch
and installs planner dependencies. `build_sdk.sh` builds pinned DepthAI 3.10;
include its install `lib` directory in `LD_LIBRARY_PATH`.

`launch_localization.sh WORKSPACE SECONDS` runs a bounded calibration/localization
check inside an isolated container, saving fresh calibration and evidence. Set
`ROS_DOMAIN_ID` to the vehicle's configured domain and `DDS_PROFILE` to its host
Fast DDS XML file. It refuses to run beside known competing camera owners.

For indefinite foreground tmux operation use `localization_pane.sh camera`,
`localization_pane.sh vio`, and `localization_pane.sh monitor` in separate panes
inside the correctly configured container. Set `CALIBRATION_DIR` for the VIO pane
to the generated native calibration directory. `WORKSPACE`, `INSTALL_PREFIX`,
`SDK_PREFIX`, `IMU_CALIBRATION_FILE` and `VIO_VERBOSITY` are configurable. No
host-specific calibration artifacts are embedded in these new launch helpers.

Use `jetson/foxglove/larpcopter-autonomy-localization.json` for the new topics.
Pose/path frame is `global`, at the IMU origin. Images use optional `/viz` preview
relays; previews do not feed the estimator. The existing `larp` CLI and historical
localization launch files are retained from the hardware branch and target its
legacy stack; they are not the native entry point described here.
