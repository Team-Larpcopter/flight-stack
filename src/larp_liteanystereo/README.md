# larp_liteanystereo

Stereo depth on the Jetson GPU with **Lite Any Stereo V2** (LAS2-M, TensorRT FP16),
from the OAK's own rectified stereo pair.

- Upstream: [TomTomTommi/LiteAnyStereo](https://github.com/TomTomTommi/LiteAnyStereo)
  (MIT), pinned at `8c97bd4`; weights `LAS2_M.pth` from
  [huggingface.co/tomtomtommi/LiteAnyStereoV2](https://huggingface.co/tomtomtommi/LiteAnyStereoV2).
- Saved engine: `models/las2_m_320x480_d192_fp16.engine` (480x320 input, max
  disparity 192, built for the Orin Nano / TensorRT 10.3; see `models/README.md`).

## Node: `las_stereo_node`

| | |
|---|---|
| In | `/native_oak/rectified/{left,right}/image` + `camera_info` (800x500, rectified and hardware-synced on the camera; baseline from the right `P[0][3]`) |
| Out | `/las_depth/depth/image_raw` (16UC1 mm) + `camera_info`, `/las_depth/disparity/image_raw` (32FC1 px, only with subscribers), 5 Hz `/viz/las_depth/{depth,disparity}/image/compressed` |
| Primary | `publish_as_primary:=true` also publishes `/stereo/depth/image` + `camera_info` (what `larp_navigation`'s `depth_map_node` reads) |
| Frame | `native_oak_depth_optical` = the rectified left camera. `larp_autonomy.native_calibration --depth <factory>.rectified.json` puts it in `rig.json` (`T_body_depth`). |

Inputs go to the engine as 3-channel 0-255 images (LAS normalizes inside the
network). The node also supports `rectify_stereo:=true` with the 1280x800
undistorted pair (rectification on the Jetson) for cameras without the
rectified output (`las_depth.launch.py input:=undistorted`).

```bash
ros2 launch larp_liteanystereo las_depth.launch.py [publish_as_primary:=true] [input:=rectified|undistorted]
```

Measured on skybeta-15: engine 18 ms per frame (`trtexec`, 56/s); in the node
~21 ms per pair and **28.6 Hz** end to end on the camera's 30 Hz rectified pair,
depth arriving 85 ms after capture. On the 1280x800 pair rectified on the Jetson
it reached only 10-16 Hz (Python receiving two 1 MB images per pair).

## Rebuilding the engine (`scripts/las_ws.sh`)

Only needed after a JetPack / TensorRT change or for another model size; the saved
engine is used otherwise.

```bash
las_ws.sh export m 320 480 192    # laptop (docker): ONNX, needs CPU torch + torchvision + timm
scp las_models/las2_m_320x480_d192.onnx <drone>:<checkout>/src/larp_liteanystereo/models/
las_ws.sh engine m 320 480 192    # Jetson: trtexec FP16 in the Isaac ROS image (~10 min)
```

## Running

`jetson/stack.sh camera` starts the camera and this node (GPU container
`flight-stack-las` on NVIDIA's public Isaac ROS Humble image, which has CUDA,
TensorRT 10.3 and PyTorch; nothing to build).

An engine only loads on the same GPU type and TensorRT version: rebuild after a
JetPack / TensorRT change. Sizes `s|m|l|h`; width and height must be divisible by 32.
