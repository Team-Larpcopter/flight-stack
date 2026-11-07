# Native hardware integration

`oak_native_node` uses DepthAI 3.10 for on-device stereo undistortion and C++
ROS publication. On-device stereo depth is on by default (`publish_depth`):
800x500 16UC1 mm on `/native_oak/depth/image_raw`, aligned to the rectified left
camera; the Jetson filter and Foxglove viewing are in `../larp_perception`.
Dynamic stereo auto-calibration is off unless `auto_calibration` selects it. Device access defaults off. Factory exports
and explicit raw-axis accelerometer correction are required for localization.
Stereo pairs use their average mid-exposure timestamp, with a 0.5 ms skew gate.

Raw acceleration and gyro remain separately available with original timestamps.
Optional `align_imu=true` supplies sqrtVINS with acceleration interpolated between
bracketing measured samples at gyro timestamps. It never extrapolates and rejects
brackets wider than 10 ms. It applies the provided raw-axis bias/scale correction
exactly once; gyro bias remains estimated by VIO. This alignment still needs moving
hardware validation.

Bench captures found that firmware reports are limited by the slower enabled IMU
stream: 250/200 Hz requests delivered about 200 Hz; 500/400 Hz requests delivered
about 400 Hz, with some upstream acceleration reports omitted. Two IMU nodes are
not supported by this device. Neither requested rates nor these loaded diagnostic
captures establish production latency. A 5 ms exposure limit is a candidate for
motion testing, not a universally qualified setting.

`px4_bench_node` connects body odometry and certified planner samples to PX4.
Output requires explicit enablement, fresh DISARMED status and healthy VIO. It
publishes no arming, mode-change or offboard-heartbeat messages. Vision is labelled
arbitrary-heading FRD unless surveyed NED alignment is explicitly supplied.
Setpoints require that alignment and an unexpired collision certificate. This is
a disarmed interface test, not a flight controller or flight-qualified deployment.

Use the PX4 message revision recorded in `../px4_msgs/UPSTREAM_REVISION`; it matches
the current hardware image. The original simulation checkout used a newer schema.
Runtime must include the chosen SDK installation's `lib` directory in
`LD_LIBRARY_PATH` (or install the SDK into a configured system library directory).

2026-11-07: native camera and PX4 targets and patched sqrtVINS compiled on ARM.
Three native ARM tests passed. A short native stereo/IMU + sqrtVINS run produced
approximately 28.5 Hz updated poses with 50 ms p95 capture age. The corrected
shared-IPC/DDS container configuration also delivered poses to Foxglove.
Moving accuracy, aircraft mounting and the complete planner/PX4 path remain
unverified; see `../../docs/autonomy-integration.md` for the integration checkpoint.
