# larp_perception

Depth and viewing for the native OAK-D S2 path: the Jetson depth filter, the
Foxglove bridge + low-bandwidth image relay, and the perception benchmarks.

The camera itself is `larp_hardware/oak_native_node` (it owns the device). It
computes depth **on the camera** by default; this package filters it on the
Jetson and makes everything viewable over Wi-Fi.

## Data flow

```
OAK-D S2 (on the camera)                      Jetson
 left/right mono sensors, 30 Hz
  ├─ 1280x800 undistorted ──────────────────► /native_oak/{left,right}/image_undistorted ─► sqrtVINS
  └─ 800x500 ─► StereoDepth (LR check,
                subpixel, rectified-left) ──► /native_oak/depth/image_raw (16UC1 mm)
                                              /native_oak/depth/camera_info
                                                 │
                                                 ▼  depth_filter (CUDA)
                                              /stereo/depth/image (16UC1 mm)  ─► larp_navigation depth_map_node
                                              /stereo/depth/camera_info
 BNO086 IMU ────────────────────────────────► /native_oak/{acceleration,gyro}_raw, /autonomy/imu/data

 viz_relay: /viz/<topic>/compressed (5 Hz, half size, JPEG) ─► foxglove_bridge :8766 (read-only) ─► Foxglove
```

### Depth on the camera (`oak_native_node`, `publish_depth` default **true**)

| Parameter | Default | |
|---|---|---|
| `publish_depth` | `true` | second 800x500 output per mono camera into the on-device StereoDepth engine |
| `depth_width` / `depth_height` | 800 / 500 | same aspect as the 1280x800 VIO stream |
| `depth_preset` | `ROBOTICS` | `DEFAULT`, `HIGH_DETAIL`, `DENSITY`, `ACCURACY`, `FACE` |
| `depth_subpixel` | `true` | without it a 7x7 median runs on the camera |
| `auto_calibration` | `OFF` | DepthAI 3.x otherwise recalibrates the stereo pair at every start and may **flash** it to the camera; `ON_START` / `CONTINUOUS` opt in |

Left-right check on; depth aligned to the **rectified left** camera; on-device
post-filters off (spatial/temporal/speckle at this size throttle every camera
stream), so filtering runs on the Jetson. StereoDepth inputs are non-blocking:
a slow depth stage drops depth frames, never VIO frames. Depth timestamps are
mid-exposure capture time on the same clock as the stereo pair and IMU (what
`depth_map_node` needs to look up the pose).

`camera_info` describes the depth image itself: `K` from the frame's own
transformation (rectified intrinsics at 800x500), `R` = left rectification
rotation (left optical → depth frame), `P[0][3] = -fx * baseline`. Frame
`native_oak_depth_optical`. With the first depth frame the node writes the same
geometry to `<factory_calibration_out>.depth.json`;
`larp_autonomy.native_calibration --depth` turns it into `T_body_depth` and
`depth_frame` in `rig.json` (`T_body_depth = T_body_imu · T_imu_left · Rᵀ`).

Why on the camera: measured on the Jetson, on-device depth holds 30 Hz at
800x500 with all streams on USB 3 at no Jetson cost; Jetson stereo was VPI CUDA
11.5 fps at 1280x800 (and competes with sqrtVINS for the GPU), OpenCV SGBM 5 fps,
OpenCV BM 26 fps with 16% valid pixels.

### Depth filter (`depth_filter`)

1. 2x-binned disparity speckle filter (CPU, ~1.5 ms): removes phantom-obstacle
   blobs without fragmenting real surfaces.
2. Multi-scale edge-consistent hole fill + smoothing (TorchScript CUDA): coarse
   cells fill only when bracketed on opposite sides by matching depths, so
   obstacle edges are not smeared across gaps.
3. Temporal EMA (alpha 0.3, reset on jumps > 150 mm), holding dropouts up to
   4 frames.

Parameters: `input_topic`, `input_info_topic`, `output_topic`,
`output_info_topic` (defaults above), `speckle_size_px`, `speckle_max_diff_px`,
`spatial`, `temporal`, `temporal_alpha`, `temporal_delta_mm`,
`temporal_persist`. Logs its own rate and per-frame time every 10 s. Header
and camera_info are passed through unchanged.

## Running (inside the integration container)

Same container and environment as the localization panes
(`jetson/integration_image/localization_pane.sh`):

```bash
bash /integration/localization_pane.sh camera       # camera: stereo + IMU + depth (owns the OAK)
bash /integration/localization_pane.sh vio          # sqrtVINS (CALIBRATION_DIR=...)
bash /integration/localization_pane.sh perception   # depth filter + Foxglove bridge + /viz relay
bash /integration/localization_pane.sh bench 30     # benchmark the running setup
```

`perception` passes extra arguments to the launch, e.g.
`... perception foxglove:=false` (filter only) or `... perception viz_rate_hz:=2.0`.

## Foxglove

Connect the Foxglove app to `ws://<drone-ip>:8766` and import
`jetson/foxglove/larpcopter-autonomy-localization.json` (stereo pair, sqrtVINS
feature tracks, filtered + raw depth, 3D sqrtVINS view, position/velocity/IMU
plots).

Kept from the skybeta setup, because they mattered on the drone's Wi-Fi:

- **Read-only bridge**: no client publishing, services or parameters.
- **No full-rate images over Wi-Fi**: one full-rate camera stream (~60 Mbit/s as
  JPEG) is more than the uplink and cost the bridge ~50% CPU. Any topic
  containing `/image` (and `trackhist`) is hidden unless it is a `/viz` copy.
- **`/viz` copies**: `viz_relay` re-encodes the stereo pair, `/autonomy/trackhist`
  and both depth streams (colourised: red near, blue far, black no depth,
  0.3–6 m) at 5 Hz, half size, JPEG; about 100 KB/s per image. Frames between
  publications are dropped before any conversion. It still deserialises every
  30 Hz frame it subscribes to (~20% of a core in the skybeta setup).
- **Latest data only**: 128 KB bridge send buffer (default 10 MB) and ROS queues
  capped at 5, so a slow link drops old messages instead of queueing seconds.

## Benchmarks

`tools/bench.py [SECONDS] [--out FILE]` (or the `bench` pane) reports per topic:
rate, p95/max gap, header-stamp age at arrival (median/p95); for depth: fill
ratio, median depth and temporal noise (still scene); CPU per process (camera,
depth filter, relay, bridge, sqrtVINS) and load. Python receives every full-size
image, so on a loaded Jetson its image rates can read lower than C++ subscribers
see.

## IMU calibration

`tools/imu_six_face.py` is the guided six-face accelerometer calibration. It reads
the native node's raw, uncorrected `/native_oak/acceleration_raw` and
`/native_oak/gyro_raw`, fits bias and scale per axis on the full gravity vectors
(hand-held tilt does not bias it) and writes the JSON `oak_native_node` takes as
`imu_calibration_file`. Rest the drone still on each of its six sides when asked.

## Tests

`python3 -m pytest test` (no camera or ROS graph): relay image conversions,
six-face fit with tilted faces, benchmark statistics.
