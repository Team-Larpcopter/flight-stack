# `larp_neural_depth`: TensorRT Fast-FoundationStereo Depth Node

Standalone ROS 2 package for running NVIDIA's **Fast-FoundationStereo** (`C-Fast-FoundationStereo`) in **TensorRT FP16** on the Jetson Orin Nano using the native OAK-D S2 rectified stereo pair (`/native_oak/left/image_undistorted` and `/native_oak/right/image_undistorted`).

Designed so it **does not touch** `larp_hardware`, `larp_perception`, or the camera/CUDA `depth_filter` pipeline. Both can run simultaneously for side-by-side comparison in Foxglove.

---

## Why This Fixes the OAK Depth Issues
1. **Zero empty spaces / holes:** Uses an EdgeNeXt monocular+stereo backbone and iterative ConvGRU refinement (`valid_iters=8`) to produce **100% dense** disparity even on blank walls and inside the dark tunnel.
2. **Close range `< 0.5m` down to `0.15m`:** Searches up to `max_disp=416` on downscaled `480×320` or `640×400` pairs (where `0.20 m` is only ~110 px disparity), avoiding the OAK hardware 96-px disparity ceiling.
3. **100% GPU TensorRT pipeline:** Transfers `mono8` images to GPU once, runs bilinear resize + 3-channel expansion + ImageNet normalization on CUDA, and executes the TensorRT FP16 engine via `execute_async_v3`.

---

## Topics

| Direction | Topic | Type | Description |
| :--- | :--- | :--- | :--- |
| **Sub** | `/native_oak/left/image_undistorted` | `sensor_msgs/Image` | Left rectified mono8 image |
| **Sub** | `/native_oak/right/image_undistorted` | `sensor_msgs/Image` | Right rectified mono8 image |
| **Sub** | `/native_oak/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera intrinsics ($f_x$) |
| **Sub** | `/native_oak/depth/camera_info` | `sensor_msgs/CameraInfo` | Stereo baseline ($B = -P_{0,3}/f_x$) |
| **Pub** | `/neural_depth/depth/image_raw` | `sensor_msgs/Image` | **16UC1 metric depth in mm** |
| **Pub** | `/neural_depth/depth/camera_info` | `sensor_msgs/CameraInfo` | Scaled CameraInfo matching neural depth |
| **Pub** | `/neural_depth/disparity/image_raw` | `sensor_msgs/Image` | 32FC1 disparity in pixels |
| **Pub** | `/viz/neural_depth/depth/image/compressed` | `sensor_msgs/CompressedImage` | 5 Hz JET colorized depth for Foxglove |
| **Pub** | `/viz/neural_depth/disparity/image/compressed` | `sensor_msgs/CompressedImage` | 5 Hz TURBO colorized disparity for Foxglove |

Because `foxglove_bridge` in `larp_perception` serves `^/viz/.*$`, the `/viz/neural_depth/...` streams appear in Foxglove automatically without restarting or reconfiguring `foxglove_bridge`.

---

## Quick Start (Sync from Laptop -> Build & Run on Jetson)

### 1. Sync from Laptop to Jetson (`~/neural_depth_ws`)
This copies `larp_neural_depth` into a separate folder `~/neural_depth_ws` on the Jetson without touching `~/depth_ws` or changing any Git branches on the Jetson:
```bash
./src/larp_neural_depth/scripts/neural_depth_ws.sh sync dronehacks@skybeta-15
```

### 2. Build the TensorRT FP16 Engine on Jetson (One-time)
Downloads `c_fast_foundationstereo_bp2_dynbatch_dynhw.onnx` and compiles `/models/fast_foundationstereo_320x480_fp16.engine`:
```bash
~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh build-engine 320 480
```

### 3. Build the ROS 2 Package on Jetson
```bash
~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh build
```

### 4. Run Side-by-Side with the Existing Stack
```bash
# Foreground (shows live Hz and median/p95 ms latency):
~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh run 320 480

# Or background daemon container (larp-neural-depth):
~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh start 320 480
```

### 5. View in Foxglove
Import the layout from:
- Laptop Downloads: `~/Downloads/larpcopter-neural-depth-comparison.json`
- Repo: `jetson/foxglove/larpcopter-neural-depth-comparison.json`
