# Native localization and fixed-map navigation integration

The integration branch combines the existing Jetson camera/localization tooling
with patched sqrtVINS, ROG-Map and SUPER. Dependencies are vendored as
ordinary source directories, without Git submodules. `references/`, builds,
SDK downloads and hardware evidence are excluded from Git.

## Current architecture

OAK performs stereo lens undistortion. The native C++ acquisition node publishes
1280x800 stereo at 30 Hz, paired at average mid-exposure time, with a 0.5 ms pair
skew gate. It publishes raw accel/gyro independently. Optional VIO IMU alignment
preserves uncorrected raw-axis SI measurements and interpolates
between bracketing acceleration samples at gyro timestamps. It never extrapolates
and rejects brackets wider than 10 ms. Current bench requests are 500 Hz accel,
400 Hz gyro and a candidate 5 ms exposure cap; these are not universally tuned
flight settings. IMU noise parameters still require characterization.

`native_calibration.py` generates OpenCV-compatible Kalibr files from SDK
camera-to-IMU exports and actual image projections, converting centimetres to
metres. Factory intrinsics/extrinsics initialize the estimator; the restored upstream
EuRoC defaults refine them and the temporal offset online. A generated
bench frame is explicitly distinguished from a surveyed aircraft mount. No CAD
pose matcher is used.

The body adapter handles frame and lever-arm conversions and supports explicit
initial position/yaw in the CAD coordinate system. Fixed-CAD planning requires an
explicit geofence parameter file. SUPER's A* route feeds its geometric frontend;
the trajectory bridge samples its certified stopping polynomial. Depth acquisition
and map updates are disabled in the current hardware path.

`px4_bench_node` publishes external vision and certified setpoints only with
explicit enablement, fresh disarmed status and healthy VIO. Setpoints additionally
require verified map/NED alignment. It sends no arming, mode-change or offboard
heartbeat messages. The executive defaults to disabled and requires explicit simulation or hardware
enablement; see [planner integration](planner-executive-integration.md). PX4 messages match the hardware image's
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
- The later planner integration passed 134 Python and 16 C++/ROS checks on Jetson,
  including synthetic valid routes/trajectories and stale-localization rejection.
  Live combined-system performance remains a separate qualification step.

Moving accuracy, surveyed airframe mounting, map/PX4 registration, reset handling,
full-system latency under planner load and real flight remain unqualified.

## Estimator defaults

The OAK estimator templates now match the vendored sqrtVINS EuRoC estimator
configuration except for `track_frequency: 31.0`, matching the OAK 30 Hz
camera with a 1 Hz scheduling margin (EuRoC uses 21 for its 20 Hz cameras).
The current native ROS 2 callback processes every paired frame without using this
frequency throttle; the value remains correct for paths that honor it.
Gravity is 9.81 m/s² and is no longer imported from the
accelerometer correction profile. Online camera calibration is enabled, ZUPT is
disabled, and the initial accelerometer-bias prior is
0.05. The historical `estimator_config_fixedcalib.yaml` filename is retained for
compatibility but now also contains those same settings. Device geometry, sensor
topics/rates and IMU noise values are separate from this estimator-tuning reset.
The native driver now applies no local accelerometer bias/scale correction;
sqrtVINS estimates biases online. No local IMU correction file is required.
Timestamp alignment still interpolates acceleration between bracketing samples;
raw does not mean integer sensor counts or unaligned accel/gyro packets. Existing generated
calibration directories must be regenerated or have their estimator YAML updated;
changing the source templates does not rewrite deployed evidence.

## Building and running from this repository

Use [the fresh-clone workflow](jetson-fresh-clone.md). It pulls a public Humble
image, downloads checksum-pinned DepthAI, and builds this checkout's own `install/`
and `sdk_install/`. No separate integration folder, prepared container, team
image, IMU correction file or manually generated camera calibration is required.

`bash jetson/launch_localization_tmux.sh` creates its container when absent and
runs the native camera and sqrtVINS in foreground panes. It verifies that an
existing container mounts this same checkout. The bundled OAK-D S2 configuration
is the default. `CALIBRATION_DIR` can intentionally override it for another rig.

Use `jetson/foxglove/larpcopter-autonomy-localization.json` for localization topics.
Pose/path frame is `global`, at the IMU origin. The legacy `larp` CLI targets the
organizer stack and is not required for this native entry point.
