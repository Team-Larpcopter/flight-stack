# Localization and controller frame contract

Notation: `T_A_B` maps coordinates expressed in B into A; its translation is the
origin of B expressed in A. Distances are metres and angles are radians. Camera
factory extrinsic translations returned by DepthAI are centimetres and must be
converted once. A frame name alone never establishes a surveyed transform.

| Boundary | Required convention | Evidence/check |
| --- | --- | --- |
| OAK camera images | Optical: +X right, +Y down, +Z forward | Pinhole image projection must match per-frame K and actual undistortion. |
| OAK raw IMU | Physical raw sensor axes | Factory IMU-to-camera transform; do not reuse it for a different IMU report-axis convention. Preserve accel/gyro timestamps separately until alignment is validated. |
| Factory `getImuToCameraExtrinsics(left)` | `T_left_imu`, translation in cm | Invert to `T_imu_left`, then convert translation to metres once. |
| sqrtVINS `T_imu_cam` | Camera coordinates into IMU | Its parser internally inverts this to the IMU-to-camera state. Do not supply an already inverted calibration. |
| sqrtVINS internal attitude | JPL quaternion, global to IMU | `quat_2_Rot(q)` is world-to-IMU. |
| sqrtVINS ROS attitude | Same numeric xyzw, ROS Hamilton interpretation: IMU to global | Verified using the actual vendored JPL conversion in rotation tests. Do not invert again. |
| sqrtVINS ROS position | IMU origin in `global` | Not the airframe origin. |
| sqrtVINS ROS twist | Linear and angular velocity in child `imu` | Fast propagator explicitly rotates global linear velocity into IMU. |
| Body adapter | Pose of FLU body in launch-aligned `map`; twist in body FLU | Apply the measured IMU-to-body rotation and lever arm. Startup yaw removal establishes arbitrary launch heading, not true north. |
| Planner | Positions/orientations in that same map | Map, CAD, geofence and goals need the same registered origin/yaw. Camera depth uses the rectified optical transform, not blindly the raw camera transform. |
| PX4 external vision | Explicit pose-frame enum; body FRD convention | Unknown map heading must not be labeled NED. NED requires declared/validated heading registration, or use a separately validated arbitrary-heading FRD fusion contract. |
| PX4 trajectory setpoint | Absolute position in PX4 local NED | Requires map-to-PX4 translation, heading and reset handling. A direct ENU swap is incorrect for arbitrary launch FLU. |

For a validated map +X heading `h` clockwise from north:

- `R_NED_map = Rz(h) diag(1,-1,-1)`.
- `p_NED = R_NED_map p_map + origin_NED`.
- `R_NED_bodyFRD = R_NED_map R_map_bodyFLU diag(1,-1,-1)`.
- Body velocity and angular velocity use `diag(1,-1,-1)`.
- PX4 `VehicleOdometry.q` is Hamilton **wxyz**, unlike ROS message field order xyzw.

Covariances must follow the quantity's frame. The sqrtVINS ROS publisher reorders
its position/attitude blocks but leaves attitude error in local IMU coordinates;
the body adapter explicitly handles this mixed representation. ROS output pose
covariance is then in map coordinates. PX4 documents orientation variance in the
body frame, so rotate that covariance into body FRD before taking its diagonal.
PX4 cannot receive the full cross-covariance through `VehicleOdometry`.

## Validation status

Local tests verify JPL/Hamilton interpretation on all axes, noncommuting
roll/pitch/yaw, FLU/FRD signs, map/NED heading and translation, covariance rotations,
and existing IMU lever-arm compensation. Factory/live frame and moving tests are
still required. No measured camera-to-airframe mounting transform has yet been
qualified, and the native prototype is not connected to the planner or FC.
