# sqrtVINS to PX4

This ROS 2 Humble package forwards `/autonomy/odomimu` (`nav_msgs/msg/Odometry`)
to `/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`).
It uses the sqrtVINS publisher contract, not arbitrary ROS odometry conventions.
Position is in gravity-aligned, arbitrary-heading `global`; twist and orientation
error covariance are in `imu`. The Hamilton quaternion maps IMU to global.

A fixed world rotation `A = diag(1,-1,-1)` makes Z down. The required mounting
rotation `B` maps IMU coordinates into aircraft body FRD. The output obeys
`p_out=A p`, `R_out=A R B.T`, and `v_out=B v`. Covariances rotate as `T C T.T`
before extracting their diagonal. All-zero covariance blocks become NaN.
The output uses `POSE_FRAME_FRD` and `VELOCITY_FRAME_BODY_FRD`.
It does not claim geographic north alignment.

## Build and launch

From the flight-stack workspace root, with matching PX4 messages available:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select larp_px4_bridge
source install/local_setup.bash
ros2 launch larp_px4_bridge bridge.launch.py params_file:=/absolute/path/bridge.yaml
```

Example YAML, valid only when the source IMU axes already align with aircraft FLU:

```yaml
vins_px4_bridge:
  ros__parameters:
    imu_to_body_frd_quaternion_wxyz: [0.0, 1.0, 0.0, 0.0]
    input_topic: /autonomy/odomimu
    output_topic: /fmu/in/vehicle_visual_odometry
    publish_rate_hz: 50.0
    max_sample_age_s: 0.2
    world_frame: global
    imu_frame: imu
    reset_counter: 0
```

Measure the actual mounting. The node refuses to start without a quaternion.
The quaternion is wxyz, mapping vectors from IMU axes to aircraft FRD axes.
No temporary integration workspace or calibration evidence path is required.

The 50 Hz timer consumes only the newest sample once. Missing input stops output.
Old, future, duplicate and backward timestamps are rejected. Sample timestamps
remain unchanged in microseconds; publication timestamps use ROS time. PX4 uXRCE
synchronizes the clocks, so do not apply another offset. If old float32 sqrtVINS
builds quantize epoch timestamps, fix the publisher; do not replace capture time
with receipt time. After a clock rollback restart the bridge after synchronization.

The reported position and velocity remain at the IMU origin. Configure PX4's
`EKF2_EV_POS_X/Y/Z` sensor offsets for that point. No lever-arm correction is
applied here. Increment `reset_counter` modulo 256 when restarting an estimator
whose world origin or heading changed, then restart the bridge with that value.
The Odometry message has no reset flag, so resets cannot be inferred reliably.

PX4 fusion configuration is separate. `EKF2_EV_CTRL` bits 0, 1, 2, 3 enable
horizontal position, vertical position, velocity and yaw respectively. Select
only the measurements you intend to fuse. Inspect timing and innovations before
setting `EKF2_EV_DELAY`, and choose the height reference deliberately. This package
does not change parameters or arm the aircraft. Avoid competing publishers on
the same visual-odometry topic. Match the ROS domain to the FC's DDS domain.

[PX4 external position guidance](https://docs.px4.io/v1.16/en/ros/external_position_estimation)
recommends 30–50 Hz.
[VehicleOdometry](https://docs.px4.io/v1.16/en/msg_docs/VehicleOdometry)
defines the frames and variances. Use px4_msgs matching the firmware release.

## Tests

```bash
python3 -m pytest src/larp_px4_bridge/test
colcon test --packages-select larp_px4_bridge
colcon test-result --verbose
python3 src/larp_px4_bridge/test/ros_smoke.py
```
