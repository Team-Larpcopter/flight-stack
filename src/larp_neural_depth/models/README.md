# Saved TensorRT engines

| File | What |
|---|---|
| `fast_foundationstereo_320x480_fp16.engine` | Fast-FoundationStereo, 480x320 input, max disparity 160, 4 iterations, FP16 |
| `ffs_320x480_d160_i4.yaml` | the model config it was exported with (`normalize: true`: feed ImageNet-normalized input) |

sha256 `60ab6b237d50a33d459cd21c4984c04c2b547f2fbc01c299a81f95cbee2a9f9a` (engine).

Built 2026-11-07 on skybeta-15 (Jetson Orin Nano 8 GB, L4T R36.5, TensorRT 10.3.0, in
`larp-integration:20261107`) from `ffs_320x480_d160_i4.onnx` with
`scripts/build_trt_engine.sh`. **A TensorRT engine only loads on the same GPU type and
TensorRT version**: rebuild it (same script) after a JetPack/TensorRT change or on
other hardware. Copy it to the models folder the node uses:

```bash
cp src/larp_neural_depth/models/fast_foundationstereo_320x480_fp16.engine ~/neural_depth_ws/models/
```
