# larp_perception

Foxglove for the drone, the `stack` run commands and the perception benchmarks
for the native OAK path (camera: `larp_hardware/oak_native_node`, depth:
`larp_liteanystereo`).

## Data flow

```
OAK-D S2 (on the camera)                         Jetson
 left/right mono, 30 Hz
  ├─ 1280x800 undistorted ─────────────► /native_oak/{left,right}/image_undistorted ─► sqrtVINS
  └─ 800x500 ─► stereo engine,
               rectification only ─────► /native_oak/rectified/{left,right}/image
                                            │
                                            ▼  larp_liteanystereo (TensorRT, GPU)
                                         /las_depth/depth/image_raw = /stereo/depth/image ─► planner
 BNO086 IMU ───────────────────────────► /native_oak/{acceleration,gyro}_raw, /autonomy/imu/data

 viz_relay: /viz/<topic>/compressed (5 Hz, half size, JPEG) + throttled pose/IMU/clouds
   ─► foxglove_bridge :8766 (read-only, /viz only) ─► Foxglove
```

## Running (on the drone)

```bash
git pull                    # main, from the flight-stack checkout
bash jetson/build.sh        # main's build (includes larp_perception and larp_liteanystereo)

jetson/stack.sh camera      # camera + rectified pair + Lite Any Stereo depth (camera off: no depth)
jetson/stack.sh vio         # sqrtVINS (bundled calibration; keep still a few seconds)
jetson/stack.sh fox         # Foxglove on ws://<drone-ip>:8766 (--tracks: sqrtVINS feature tracks)
jetson/stack.sh status      # what runs + last log line;  logs camera|las|vio|fox;  stop [part]
```

Each command starts its part in the background and returns; logs are in
`log/stack/` of the checkout. Camera, sqrtVINS and Foxglove run in main's
`flight-stack-localization` container; Lite Any Stereo runs in `flight-stack-las`
(same checkout, GPU, same ROS domain) on NVIDIA's public Isaac ROS Humble image
(CUDA, TensorRT 10.3, PyTorch; already on skybeta drones, pulled if missing).
Nothing is built for it and the TensorRT engine is not rebuilt:
it is saved in `larp_liteanystereo/models`. Only one program can own the camera:
stop other camera drivers first (e.g. `jetson/launch_localization_tmux.sh`).

## Foxglove

Connect to `ws://<drone-ip>:8766` and import
`jetson/foxglove/larpcopter-autonomy-localization.json`.

- **Read-only bridge** serving **only `/viz/*`**: nothing reaches Wi-Fi at full rate.
- **Newest data only, no lag build-up**: queue depth 1 and a 64 KB send buffer, so a
  slow link drops old messages instead of queueing seconds of them.
- **Throttled copies** (`viz_relay`): images (left camera, depth colourised red near
  / blue far / black none; `viz_right`, `viz_rectified`, `viz_tracks` optional) as
  5 Hz half-size JPEG; sqrtVINS pose 10 Hz, IMU 20 Hz, feature clouds 2 Hz forwarded
  as raw bytes; a bounded path built from the poses.
- **Not slowing sqrtVINS**: it only computes its IMU-rate odometry and its
  feature-track image when someone subscribes. The relay uses the per-frame pose
  instead, and the track image is opt-in. Measured: sqrtVINS 28.8 Hz bare, 17.4 Hz
  with a full-rate viewer, about 25 Hz with this one.
- Depth previews: `/viz/stereo/depth/image/compressed` (the planner's depth) and
  `/viz/las_depth/{depth,disparity}/image/compressed` (from the depth node).

## Benchmarks

`python3 tools/bench.py 20 --out FILE.json` (in the localization container, ROS sourced): per topic rate, p95/max gap and
header-stamp age at arrival; depth fill ratio and temporal noise; CPU per process.
For image topics prefer their `camera_info` (one per image, cheap): Python receiving
full images reads low on a busy Jetson.

Measured on skybeta-15 (`stack camera`, 20 s): undistorted stereo 29.8/29.9 Hz,
rectified pair 29.8/30.0 Hz, Lite Any Stereo depth 28.6 Hz arriving 85 ms after
capture, IMU 399 Hz.

## Tests

`python3 -m pytest test` (also run by `jetson/build.sh`): relay conversions and benchmark
statistics.
