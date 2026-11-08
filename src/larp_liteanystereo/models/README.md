# Saved TensorRT engine

| File | What |
|---|---|
| `las2_m_320x480_d192_fp16.engine` | Lite Any Stereo V2, size M, 480x320 input, max disparity 192, FP16 |

sha256 `7e46974de48cef55dca200468053b5e1f2cd0f038fbb08f9324f26f30b40aa57`.

Built 2026-11-07 on skybeta-15 (Jetson Orin Nano 8 GB, L4T R36.5, TensorRT 10.3.0, in
`larp-integration:20261107`) with `scripts/las_ws.sh engine m 320 480 192` (568 s) from
`las2_m_320x480_d192.onnx` (sha256 `7e1355918ba1db289aa2fdabd96a43f97652f213bca9a10bdf7f1d008da40931`,
exported with `las_ws.sh export`, upstream commit 8c97bd4, weights `LAS2_M.pth`).
Inputs `left` / `right` 1x3x320x480 float, raw 0-255 RGB (normalized inside the network);
output `disparity` 1x1x320x480.

`trtexec` on the drone (busy Jetson, load 8.5): **18.0 ms median, 56 inferences/s**.
During the build some tactics were skipped for lack of free GPU memory; rebuilding
with the Jetson idle may be slightly faster.

**A TensorRT engine only loads on the same GPU type and TensorRT version**: rebuild it
after a JetPack/TensorRT change or on other hardware. `stack.sh` mounts this folder
as `/models`, so the node loads it from here directly.
