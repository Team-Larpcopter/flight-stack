# Reused TensorRT engine

The deployment uses `las2_m_320x480_d192_fp16.engine`, LAS2-M, input 480x320,
maximum disparity 192, FP16. It was built on the target Jetson Orin Nano with
TensorRT 10.3.0. The engine is an external deployment artifact, not a Git file.

SHA-256: `7e46974de48cef55dca200468053b5e1f2cd0f038fbb08f9324f26f30b40aa57`.
ONNX SHA-256: `7e1355918ba1db289aa2fdabd96a43f97652f213bca9a10bdf7f1d008da40931`.
Upstream LiteAnyStereo commit: `8c97bd4`, weights `LAS2_M.pth`.

`jetson/offboard/collect_gpu_runtime.sh` retrieves the existing engine and its
working GPU runtime. Neither the build nor deployment scripts rebuild engines.
A GPU/TensorRT mismatch fails startup rather than silently selecting CPU stereo.
