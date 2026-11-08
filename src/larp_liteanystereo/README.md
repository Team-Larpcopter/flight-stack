# Lite Any Stereo depth

Consumes the native OAK rectified, synchronized stereo pair and runs the saved
LAS2-M TensorRT engine. Publishes 16UC1 depth in millimetres with scaled
CameraInfo. `publish_as_primary:=true` publishes `/stereo/depth/image` and
`/stereo/depth/camera_info` for ROG dynamic mapping. Input capture timestamps and
the `native_oak_depth_optical` frame are preserved.

Run `ros2 launch larp_liteanystereo las_depth.launch.py publish_as_primary:=true`
in the GPU runtime image. The engine defaults to
`/models/las2_m_320x480_d192_fp16.engine`. Missing/incompatible engines fail
startup; there is no automatic rebuild or fallback in the production node.

See [offboard deployment](../../docs/offboard-arm64.md) and
[engine provenance](models/README.md). Depth-to-body calibration must be generated
using `native_calibration --depth` with the native camera's rectified geometry.
