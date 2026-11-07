"""neural_stereo_node: Standalone Jetson TensorRT neural stereo depth node.

Subscribes to the native OAK stereo streams without touching any existing nodes:
  - /native_oak/left/image_undistorted   (or /oak/left/image_rect)
  - /native_oak/right/image_undistorted  (or /oak/right/image_rect)
  - /native_oak/left/camera_info         (for fx)
  - /native_oak/depth/camera_info        (optional: for exact stereo rectification R and baseline)

Publishes on completely separate topics so it can run side-by-side with the
camera/CUDA depth filter:
  - /neural_depth/depth/image_raw                 (sensor_msgs/Image, 16UC1 mm)
  - /neural_depth/depth/camera_info               (sensor_msgs/CameraInfo)
  - /neural_depth/disparity/image_raw             (sensor_msgs/Image, 32FC1 px)
  - /viz/neural_depth/depth/image/compressed      (sensor_msgs/CompressedImage, 5 Hz JET JPEG for Foxglove)
  - /viz/neural_depth/disparity/image/compressed  (sensor_msgs/CompressedImage, 5 Hz TURBO JPEG for Foxglove)

Because foxglove_bridge in perception.launch.py whitelists `^/viz/.*$`, anything
published under `/viz/neural_depth/...` is automatically streamed to Foxglove!
"""

import array
import glob
import json
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .trt_stereo_engine import (
    OpenCvSgbmFallbackEngine,
    TensorRtSingleStereoEngine,
    disparity_to_depth_mm,
)

LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)
OUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE
)
VIZ_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE
)


def decode_image(msg: Image) -> np.ndarray | None:
    if msg.encoding in ("mono8", "8UC1"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return arr[:, : msg.width].copy()
    if msg.encoding in ("bgr8", "rgb8", "8UC3"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if msg.encoding == "bgr8" else img.copy()
    return None


def colorize_depth_jet(depth_u16: np.ndarray, min_m: float, max_m: float, scale: float = 1.0) -> np.ndarray:
    """Colorize 16UC1 mm depth identical to larp_perception/viz_relay.py (near=red, far=blue, 0=black)."""
    depth_m = depth_u16.astype(np.float32) / 1000.0
    if scale != 1.0 and scale > 0.0:
        valid = (depth_m > 0) & np.isfinite(depth_m)
        d = np.where(valid, depth_m, 0.0).astype(np.float32)
        sz = (max(1, int(depth_m.shape[1] * scale)), max(1, int(depth_m.shape[0] * scale)))
        tot = cv2.resize(d, sz, interpolation=cv2.INTER_AREA)
        cnt = cv2.resize(valid.astype(np.float32), sz, interpolation=cv2.INTER_AREA)
        depth_m = np.where(cnt > 0.25, tot / np.maximum(cnt, 1e-6), 0.0).astype(np.float32)
    valid = np.isfinite(depth_m) & (depth_m > 0)
    clipped = np.clip(np.nan_to_num(depth_m), min_m, max_m)
    scaled = ((max_m - clipped) / max(1e-6, max_m - min_m) * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    color[~valid] = 0
    return color


def colorize_disparity_turbo(disp: np.ndarray, max_disp: float = 160.0) -> np.ndarray:
    valid = np.isfinite(disp) & (disp > 0.25)
    clipped = np.clip(np.nan_to_num(disp), 0.0, max_disp)
    scaled = (clipped / max(1.0, max_disp) * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


class NeuralStereoNode(Node):
    def __init__(self):
        super().__init__("neural_stereo_node")
        dyn = ParameterDescriptor(dynamic_typing=True)
        dp = self.declare_parameter

        # Topics
        self.left_topic = str(dp("left_topic", "/native_oak/left/image_undistorted", dyn).value)
        self.right_topic = str(dp("right_topic", "/native_oak/right/image_undistorted", dyn).value)
        self.left_info_topic = str(dp("left_info_topic", "/native_oak/left/camera_info", dyn).value)
        self.right_info_topic = str(dp("right_info_topic", "/native_oak/right/camera_info", dyn).value)
        self.depth_info_topic = str(dp("depth_info_topic", "/native_oak/depth/camera_info", dyn).value)

        self.out_depth_topic = str(dp("output_depth_topic", "/neural_depth/depth/image_raw", dyn).value)
        self.out_info_topic = str(dp("output_info_topic", "/neural_depth/depth/camera_info", dyn).value)
        self.out_disp_topic = str(dp("output_disp_topic", "/neural_depth/disparity/image_raw", dyn).value)
        self.viz_depth_topic = str(dp("viz_depth_topic", "/viz/neural_depth/depth/image/compressed", dyn).value)
        self.viz_disp_topic = str(dp("viz_disp_topic", "/viz/neural_depth/disparity/image/compressed", dyn).value)
        self.publish_as_primary = bool(dp("publish_as_primary", False, dyn).value)
        self.primary_depth_topic = str(dp("primary_depth_topic", "/stereo/depth/image", dyn).value)
        self.primary_info_topic = str(dp("primary_info_topic", "/stereo/depth/camera_info", dyn).value)

        # Model / Engine settings
        default_engine = os.environ.get(
            "NEURAL_STEREO_ENGINE",
            os.path.expanduser("~/neural_depth_ws/models/fast_foundationstereo_320x480_fp16.engine"),
        )
        self.engine_path = str(dp("engine_path", default_engine, dyn).value)
        self.target_h = int(dp("target_height", 320, dyn).value)
        self.target_w = int(dp("target_width", 480, dyn).value)
        self.output_h = int(dp("output_height", 0, dyn).value)  # 0 = keep target_h
        self.output_w = int(dp("output_width", 0, dyn).value)   # 0 = keep target_w

        # Depth & stereo geometry
        self.baseline_m = float(dp("baseline_m", 0.075, dyn).value)
        self.default_fx = float(dp("default_fx", 800.0, dyn).value)
        self.min_depth_m = float(dp("min_depth_m", 0.15, dyn).value)  # handles < 0.5m down to 15cm!
        self.max_depth_m = float(dp("max_depth_m", 8.0, dyn).value)
        self.remove_invisible = bool(dp("remove_invisible", True, dyn).value)
        self.rectify_stereo = bool(dp("rectify_stereo", True, dyn).value)
        self.extrinsics_glob = str(
            dp("extrinsics_glob", "/integration/evidence/*/factory.json.extrinsics.json", dyn).value
        )

        # Foxglove /viz settings
        self.viz_enable = bool(dp("viz_enable", True, dyn).value)
        self.viz_rate_hz = float(dp("viz_rate_hz", 5.0, dyn).value)
        self.viz_min_m = float(dp("viz_min_depth_m", 0.2, dyn).value)
        self.viz_max_m = float(dp("viz_max_depth_m", 6.0, dyn).value)
        self.viz_quality = int(dp("viz_jpeg_quality", 60, dyn).value)

        # The Fast-FoundationStereo ONNX export (cfg normalize: true) expects ImageNet-normalized
        # inputs: on a rectified bench pair raw 0-255 input was off by ~29 px median vs SGBM,
        # normalized input agreed to 0.3 px.
        self.normalize_input = bool(dp("normalize_input", True, dyn).value)

        # Load TensorRT engine (or fallback if not built yet)
        self.backend_name = "unknown"
        try:
            self.runner = TensorRtSingleStereoEngine(
                self.engine_path, target_height=self.target_h, target_width=self.target_w,
                normalize=self.normalize_input,
            )
            self.target_h = self.runner.target_h
            self.target_w = self.runner.target_w
            self.backend_name = f"TensorRT ({os.path.basename(self.engine_path)} @ {self.target_w}x{self.target_h})"
            self.get_logger().info(f"Loaded {self.backend_name}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Could not load TensorRT engine '{self.engine_path}' ({exc}); "
                f"using OpenCV SGBM fallback at {self.target_w}x{self.target_h} until engine is built."
            )
            self.runner = OpenCvSgbmFallbackEngine(self.target_h, self.target_w)
            self.backend_name = f"OpenCV-SGBM-Fallback ({self.target_w}x{self.target_h})"

        # Publishers
        self.pub_depth = self.create_publisher(Image, self.out_depth_topic, OUT_QOS)
        self.pub_info = self.create_publisher(CameraInfo, self.out_info_topic, OUT_QOS)
        self.pub_disp = self.create_publisher(Image, self.out_disp_topic, OUT_QOS)
        self.pub_viz_depth = self.create_publisher(CompressedImage, self.viz_depth_topic, VIZ_QOS)
        self.pub_viz_disp = self.create_publisher(CompressedImage, self.viz_disp_topic, VIZ_QOS)
        self.pub_primary_depth = None
        self.pub_primary_info = None
        if self.publish_as_primary and self.primary_depth_topic != self.out_depth_topic:
            self.pub_primary_depth = self.create_publisher(Image, self.primary_depth_topic, OUT_QOS)
            self.pub_primary_info = self.create_publisher(CameraInfo, self.primary_info_topic, OUT_QOS)
            self.get_logger().info(
                f"Also publishing primary navigation depth on {self.primary_depth_topic} (+ {self.primary_info_topic})"
            )

        # State
        self.fx_orig = None
        self.left_info = None
        self.right_info = None
        self.depth_info = None
        self.rect_map_l = None
        self.rect_map_r = None
        self.rect_shape = None
        self.r_l_r = self._load_stereo_r_l_r()
        self.left_buf = deque(maxlen=4)
        self.right_buf = deque(maxlen=4)
        self.latest_pair = None
        self.pair_event = threading.Event()
        self.lock = threading.Lock()
        self.last_viz_time = 0.0
        self.infer_times_ms = []
        self.running = True

        # Subscriptions
        self.create_subscription(CameraInfo, self.left_info_topic, self._on_left_info, LATEST_QOS)
        self.create_subscription(CameraInfo, self.right_info_topic, self._on_right_info, LATEST_QOS)
        self.create_subscription(CameraInfo, self.depth_info_topic, self._on_depth_info, LATEST_QOS)
        self.create_subscription(Image, self.left_topic, self._on_left, LATEST_QOS)
        self.create_subscription(Image, self.right_topic, self._on_right, LATEST_QOS)

        # Background worker thread so slow/fast inference never blocks ROS subscription callbacks
        self.worker = threading.Thread(target=self._inference_loop, daemon=True)
        self.worker.start()
        self.create_timer(5.0, self._report_stats)

    def _load_stereo_r_l_r(self) -> np.ndarray:
        """Load R_L_R (rotation from right optical to left optical) from OAK extrinsics export."""
        # Default OAK-D S2 factory R_L_R = R_imu_L^T @ R_imu_R (~0.7 deg relative rotation)
        default_t_imu_l = np.array([
            [0.9999218583106995, -0.009144069626927376, 0.008521977812051773],
            [-0.00914827361702919, -0.9999580979347229, 0.00045461239642463624],
            [0.008517463691532612, -0.0005325372330844402, -0.9999635815620422],
        ], dtype=np.float64)
        default_t_imu_r = np.array([
            [0.9999282956123352, -0.011861786246299744, 0.0016274235676974058],
            [-0.011879357509315014, -0.9998661875724792, 0.01124940998852253],
            [0.001493767835199833, -0.01126793585717678, -0.9999353885650635],
        ], dtype=np.float64)
        r_l_r = default_t_imu_l.T @ default_t_imu_r
        try:
            candidates = sorted(glob.glob(self.extrinsics_glob), key=os.path.getmtime, reverse=True)
            for path in candidates:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                cam_imu = data.get("camera_to_imu_cm", {})
                if "left" in cam_imu and "right" in cam_imu:
                    rl = np.array(cam_imu["left"], dtype=np.float64)[:3, :3]
                    rr = np.array(cam_imu["right"], dtype=np.float64)[:3, :3]
                    r_l_r = rl.T @ rr
                    self.get_logger().info(f"Loaded stereo R_L_R extrinsics from {path}")
                    break
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Using built-in OAK factory R_L_R ({exc})")
        return r_l_r

    def _on_left_info(self, msg: CameraInfo):
        self.left_info = msg
        if msg.k[0] > 0:
            self.fx_orig = float(msg.k[0])

    def _on_right_info(self, msg: CameraInfo):
        self.right_info = msg

    def _on_depth_info(self, msg: CameraInfo):
        self.depth_info = msg
        if msg.k[0] > 0 and msg.p[3] != 0:
            self.baseline_m = abs(float(msg.p[3]) / float(msg.k[0]))

    def _stamp_ns(self, msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_left(self, msg: Image):
        with self.lock:
            self.left_buf.append((self._stamp_ns(msg), msg))
            self._match_locked()

    def _on_right(self, msg: Image):
        with self.lock:
            self.right_buf.append((self._stamp_ns(msg), msg))
            self._match_locked()

    def _match_locked(self):
        # Match newest pair within 5 ms (oak_native_node pairs within 0.5 ms and stamps identically)
        while self.left_buf and self.right_buf:
            lt, lmsg = self.left_buf[-1]
            # Find closest right frame
            best_idx = -1
            best_dt = 1_000_000_000
            for idx, (rt, _) in enumerate(self.right_buf):
                dt = abs(lt - rt)
                if dt < best_dt:
                    best_dt = dt
                    best_idx = idx
            if best_dt <= 5_000_000:  # 5 ms tolerance
                _, rmsg = self.right_buf[best_idx]
                self.left_buf.clear()
                self.right_buf.clear()
                self.latest_pair = (lmsg, rmsg)
                self.pair_event.set()
                return
            # Otherwise drop older side
            if self.left_buf[0][0] < self.right_buf[0][0]:
                self.left_buf.popleft()
            else:
                self.right_buf.popleft()

    def _inference_loop(self):
        while self.running and rclpy.ok():
            if not self.pair_event.wait(timeout=0.5):
                continue
            self.pair_event.clear()
            with self.lock:
                pair = self.latest_pair
                self.latest_pair = None
            if pair is None:
                continue
            try:
                self._process_pair(pair[0], pair[1])
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Inference error: {exc}", throttle_duration_sec=5.0)

    def _ensure_rectify_maps(self, orig_w: int, orig_h: int):
        """Build stereo rectification maps warping undistorted L/R into native_oak_depth_optical."""
        if not self.rectify_stereo:
            return
        if self.left_info is None or self.right_info is None or self.depth_info is None:
            return
        key = (orig_w, orig_h, self.target_w, self.target_h)
        if self.rect_shape == key and self.rect_map_l is not None and self.rect_map_r is not None:
            return

        kl = np.array(self.left_info.k, dtype=np.float64).reshape(3, 3)
        kr = np.array(self.right_info.k, dtype=np.float64).reshape(3, 3)
        r_rect_l = np.array(self.depth_info.r, dtype=np.float64).reshape(3, 3)
        if np.allclose(r_rect_l, 0.0):
            r_rect_l = np.eye(3, dtype=np.float64)
        r_rect_r = r_rect_l @ self.r_l_r

        # Target rectified intrinsics at (target_w, target_h) aligned with depth_info
        sx = float(self.target_w) / float(self.depth_info.width)
        sy = float(self.target_h) / float(self.depth_info.height)
        kd = np.array(self.depth_info.k, dtype=np.float64).reshape(3, 3)
        p_rect = np.array([
            [kd[0, 0] * sx, 0.0, kd[0, 2] * sx, 0.0],
            [0.0, kd[1, 1] * sy, kd[1, 2] * sy, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ], dtype=np.float64)

        size = (self.target_w, self.target_h)
        self.rect_map_l = cv2.initUndistortRectifyMap(kl, None, r_rect_l, p_rect, size, cv2.CV_16SC2)
        self.rect_map_r = cv2.initUndistortRectifyMap(kr, None, r_rect_r, p_rect, size, cv2.CV_16SC2)
        self.rect_shape = key
        self.get_logger().info(
            f"Built stereo rectification maps ({orig_w}x{orig_h} -> {self.target_w}x{self.target_h} "
            f"in {self.depth_info.header.frame_id or 'native_oak_depth_optical'})"
        )

    def _process_pair(self, lmsg: Image, rmsg: Image):
        t0 = time.perf_counter()
        left = decode_image(lmsg)
        right = decode_image(rmsg)
        if left is None or right is None:
            return

        orig_h, orig_w = left.shape[:2]
        self._ensure_rectify_maps(orig_w, orig_h)
        if self.rect_map_l is not None and self.rect_map_r is not None:
            left = cv2.remap(left, self.rect_map_l[0], self.rect_map_l[1], cv2.INTER_LINEAR)
            right = cv2.remap(right, self.rect_map_r[0], self.rect_map_r[1], cv2.INTER_LINEAR)
            # Images are now rectified at (target_h, target_w) with depth_info's horizontal FOV
            fx_orig = float(self.depth_info.k[0]) * (float(orig_w) / float(self.depth_info.width))
        else:
            fx_orig = self.fx_orig if self.fx_orig is not None else (self.default_fx * (orig_w / 1280.0))

        # Run GPU TensorRT stereo inference
        disp = self.runner.infer(left, right)

        out_h = self.output_h if self.output_h > 0 else self.target_h
        out_w = self.output_w if self.output_w > 0 else self.target_w

        depth_u16, _ = disparity_to_depth_mm(
            disp,
            fx_orig=fx_orig,
            baseline_m=self.baseline_m,
            orig_w=orig_w,
            orig_h=orig_h,
            out_w=out_w,
            out_h=out_h,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
            remove_invisible=self.remove_invisible,
        )

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.infer_times_ms.append(dt_ms)

        # Publish 16UC1 metric depth (aligned to depth_info frame so depth_map_node accepts it directly)
        out_frame = (
            self.depth_info.header.frame_id
            if (self.depth_info is not None and self.depth_info.header.frame_id)
            else "native_oak_depth_optical"
        )
        d_msg = Image()
        d_msg.header.stamp = lmsg.header.stamp
        d_msg.header.frame_id = out_frame
        d_msg.height, d_msg.width = depth_u16.shape
        d_msg.encoding = "16UC1"
        d_msg.is_bigendian = 0
        d_msg.step = d_msg.width * 2
        d_msg.data = array.array("B", depth_u16.tobytes())
        self.pub_depth.publish(d_msg)

        # Publish scaled CameraInfo (matching depth_map_node's frame/R/P contract)
        ci = self._build_camera_info(d_msg.header, orig_w, orig_h, out_w, out_h, fx_orig)
        self.pub_info.publish(ci)

        # If configured as the primary depth method, also publish on /stereo/depth/image + /stereo/depth/camera_info
        if self.pub_primary_depth is not None and self.pub_primary_info is not None:
            self.pub_primary_depth.publish(d_msg)
            self.pub_primary_info.publish(ci)

        # Publish 32FC1 disparity if anyone subscribes
        if self.pub_disp.get_subscription_count() > 0:
            disp_f32 = np.ascontiguousarray(disp, dtype=np.float32)
            disp_msg = Image()
            disp_msg.header = lmsg.header
            disp_msg.height, disp_msg.width = disp_f32.shape
            disp_msg.encoding = "32FC1"
            disp_msg.is_bigendian = 0
            disp_msg.step = disp_msg.width * 4
            disp_msg.data = array.array("B", disp_f32.tobytes())
            self.pub_disp.publish(disp_msg)

        # Publish throttled JPEG previews on /viz/neural_depth/... for Foxglove
        now = time.monotonic()
        if self.viz_enable and (now - self.last_viz_time) >= (1.0 / max(0.5, self.viz_rate_hz)):
            self.last_viz_time = now
            color_depth = colorize_depth_jet(depth_u16, self.viz_min_m, self.viz_max_m)
            ok, jpg = cv2.imencode(".jpg", color_depth, [cv2.IMWRITE_JPEG_QUALITY, self.viz_quality])
            if ok:
                cmsg = CompressedImage()
                cmsg.header = lmsg.header
                cmsg.format = "jpeg"
                cmsg.data = array.array("B", jpg.tobytes())
                self.pub_viz_depth.publish(cmsg)

            color_disp = colorize_disparity_turbo(disp, max_disp=160.0)
            ok2, jpg2 = cv2.imencode(".jpg", color_disp, [cv2.IMWRITE_JPEG_QUALITY, self.viz_quality])
            if ok2:
                cmsg2 = CompressedImage()
                cmsg2.header = lmsg.header
                cmsg2.format = "jpeg"
                cmsg2.data = array.array("B", jpg2.tobytes())
                self.pub_viz_disp.publish(cmsg2)

    def _build_camera_info(
        self, header, orig_w: int, orig_h: int, out_w: int, out_h: int, fx_orig: float
    ) -> CameraInfo:
        ci = CameraInfo()
        ci.header = header
        ci.width = out_w
        ci.height = out_h
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        if self.depth_info is not None and self.depth_info.width > 0 and self.depth_info.height > 0:
            sx = float(out_w) / float(self.depth_info.width)
            sy = float(out_h) / float(self.depth_info.height)
            k = list(self.depth_info.k)
            fx, fy, cx, cy = k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy
            ci.r = list(self.depth_info.r)
        elif self.left_info is not None:
            sx = float(out_w) / float(orig_w)
            sy = float(out_h) / float(orig_h)
            k = list(self.left_info.k)
            fx, fy, cx, cy = k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy
            ci.r = list(self.left_info.r)
        else:
            sx = float(out_w) / float(orig_w)
            sy = float(out_h) / float(orig_h)
            fx = fx_orig * sx
            fy = fx_orig * sy
            cx = out_w * 0.5
            cy = out_h * 0.5
            ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, -fx * self.baseline_m, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _report_stats(self):
        if not self.infer_times_ms:
            self.get_logger().info(f"[{self.backend_name}] waiting for stereo pair on {self.left_topic}...")
            return
        arr = np.array(self.infer_times_ms)
        self.infer_times_ms.clear()
        hz = len(arr) / 5.0
        self.get_logger().info(
            f"[{self.backend_name}] {hz:.1f} Hz | latency median {np.median(arr):.1f} ms, "
            f"p95 {np.percentile(arr, 95):.1f} ms"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NeuralStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
