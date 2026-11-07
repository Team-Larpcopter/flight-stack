"""depth_filter: Jetson CUDA post-processing for the OAK-D S2's on-device depth.

The OAK-D S2's on-device post-processing filters throttle the VIO camera streams
at decimation 1 (800x500), so the native camera node (larp_hardware
oak_native_node) runs only the hardware left-right check (+ subpixel) on the
camera and streams raw 800x500 depth here.

Pipeline:
  1. 2x-binned disparity speckle filter (CPU, ~1.5 ms):
     Bins 2x2 max disparity to bridge 1-2 px low-texture perforations so real
     walls/furniture aren't fragmented and wiped out, runs cv2.filterSpeckles
     on 4x fewer pixels to kill phantom-obstacle blobs, and gates full-res depth.
  2. Multi-scale bracketed spatial smoothing + hole-filling (TorchScript CUDA):
     2x/4x/8x edge-consistent pyramid where empty coarse cells fill ONLY when
     bracketed on opposite sides (L+R, U+D, or diagonals) by matching depths,
     filling low-texture interior holes cleanly without smearing obstacle edges
     across gates, tunnels, or windows.
  3. Temporal EMA + bounded persistence (CUDA):
     Exponential smoothing (alpha=0.4) reset on jumps > 150 mm, holding dropouts
     for up to 4 frames (~133 ms at 30 Hz).

Subscribes: input_topic  (default /native_oak/depth/image_raw, 16UC1 mm) + input_info_topic
Publishes:  output_topic (default /stereo/depth/image, 16UC1 mm) + output_info_topic
            (defaults are what larp_navigation depth_map_node reads). Header stamp,
            frame and camera_info are passed through unchanged.
"""

import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

LATEST = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
OUT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE)


def _build_cuda_module(torch, alpha: float, delta_mm: float, persist: float):
    import torch.nn.functional as F

    class _PyramidFilter(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.alpha = float(alpha)
            self.delta_mm = float(delta_mm)
            self.persist = float(persist)

        def _down(self, img: torch.Tensor, min_cnt: float, rel_tol: float, abs_tol: float) -> torch.Tensor:
            ph = img.shape[2] % 2
            pw = img.shape[3] % 2
            if ph != 0 or pw != 0:
                img = F.pad(img, (0, pw, 0, ph), mode="constant", value=0.0)
            v = (img > 0).float()
            cnt = F.avg_pool2d(v, 2, 2) * 4.0
            mean = (F.avg_pool2d(img, 2, 2) * 4.0) / cnt.clamp_min(1.0)
            pos_inf = torch.where(img > 0, img, torch.full_like(img, 1e9))
            zmin = -F.max_pool2d(-pos_inf, 2, 2)
            zmax = F.max_pool2d(img, 2, 2)
            ok = (cnt >= min_cnt) & ((zmax - zmin) <= (rel_tol * mean + abs_tol))
            return torch.where(ok, mean, torch.zeros_like(mean))

        def _bracket_smooth(self, c: torch.Tensor) -> torch.Tensor:
            cp = F.pad(c, (1, 1, 1, 1), mode="constant", value=0.0)
            H = c.shape[2]
            W = c.shape[3]
            u = cp[:, :, 0:H, 1 : W + 1]
            dn = cp[:, :, 2 : H + 2, 1 : W + 1]
            lf = cp[:, :, 1 : H + 1, 0:W]
            rt = cp[:, :, 1 : H + 1, 2 : W + 2]
            ul = cp[:, :, 0:H, 0:W]
            ur = cp[:, :, 0:H, 2 : W + 2]
            dl = cp[:, :, 2 : H + 2, 0:W]
            dr = cp[:, :, 2 : H + 2, 2 : W + 2]

            nbs = torch.cat([u, dn, lf, rt, ul, ur, dl, dr, c], dim=1)
            nv = (nbs > 0).float()
            cnt = nv.sum(dim=1, keepdim=True)
            mean = nbs.sum(dim=1, keepdim=True) / cnt.clamp_min(1.0)
            n_inf = torch.where(nbs > 0, nbs, torch.full_like(nbs, 1e9))
            n_ninf = torch.where(nbs > 0, nbs, torch.full_like(nbs, -1e9))
            consistent = (n_ninf.max(dim=1, keepdim=True).values - n_inf.min(dim=1, keepdim=True).values) <= (
                0.06 * mean + 60.0
            )

            lr_ok = (lf > 0) & (rt > 0) & ((lf - rt).abs() <= (0.06 * 0.5 * (lf + rt) + 60.0))
            ud_ok = (u > 0) & (dn > 0) & ((u - dn).abs() <= (0.06 * 0.5 * (u + dn) + 60.0))
            d1_ok = (ul > 0) & (dr > 0) & ((ul - dr).abs() <= (0.06 * 0.5 * (ul + dr) + 60.0))
            d2_ok = (ur > 0) & (dl > 0) & ((ur - dl).abs() <= (0.06 * 0.5 * (ur + dl) + 60.0))
            fill_ok = (c > 0) | ((cnt >= 3.0) & consistent & (lr_ok | ud_ok | d1_ok | d2_ok))

            ref = torch.where(c > 0, c, mean)
            w = nv * torch.exp(-(((nbs - ref) / (0.035 * ref + 20.0)) ** 2))
            wsum = w.sum(dim=1, keepdim=True)
            sm = (w * nbs).sum(dim=1, keepdim=True) / wsum.clamp_min(1e-6)
            return torch.where(fill_ok & (wsum > 0.5), sm, torch.zeros_like(c))

        def _up(self, coarse: torch.Tensor, h: int, w: int) -> torch.Tensor:
            v = (coarse > 0).float()
            num = F.interpolate(coarse, size=(h, w), mode="bilinear", align_corners=False)
            den = F.interpolate(v, size=(h, w), mode="bilinear", align_corners=False)
            bilin = torch.where(den > 0.55, num / den.clamp_min(1e-6), torch.zeros_like(num))
            # Prevent "flying pixels" (veil points in 3D occupancy maps) across depth jumps:
            # if neighboring coarse cells jump > 6% + 60mm, snap to nearest valid depth instead of blending!
            cp = F.pad(coarse, (1, 1, 1, 1), mode="replicate")
            pos_inf = torch.where(cp > 0, cp, torch.full_like(cp, 1e9))
            zmin = -F.max_pool2d(-pos_inf, 3, 1)
            zmax = F.max_pool2d(cp, 3, 1)
            edge = (zmax > 0) & ((zmax - zmin) > (0.06 * zmin + 60.0))
            edge_up = F.interpolate(edge.float(), size=(h, w), mode="nearest") > 0.5
            near = F.interpolate(coarse, size=(h, w), mode="nearest")
            return torch.where(edge_up, near, bilin)

        def forward(self, d_in: torch.Tensor, avg_in: torch.Tensor, age_in: torch.Tensor):
            H = d_in.shape[2]
            W = d_in.shape[3]
            c2 = self._down(d_in, 1.0, 0.05, 50.0)
            c4 = self._down(c2, 1.0, 0.06, 60.0)
            c8 = self._down(c4, 1.0, 0.06, 60.0)
            c16 = self._down(c8, 1.0, 0.07, 70.0)
            c32 = self._down(c16, 1.0, 0.07, 70.0)

            c32 = self._bracket_smooth(c32)
            c16_fill = self._up(c32, c16.shape[2], c16.shape[3])
            c16 = self._bracket_smooth(torch.where(c16 > 0, c16, c16_fill))
            c8_fill = self._up(c16, c8.shape[2], c8.shape[3])
            c8 = self._bracket_smooth(torch.where(c8 > 0, c8, c8_fill))
            c4_fill = self._up(c8, c4.shape[2], c4.shape[3])
            c4 = self._bracket_smooth(torch.where(c4 > 0, c4, c4_fill))
            c2_fill = self._up(c4, c2.shape[2], c2.shape[3])
            c2 = self._bracket_smooth(torch.where(c2 > 0, c2, c2_fill))
            c2_up = self._up(c2, H, W)

            # Edge-preserving full-res merge: smooth valid pixels where consistent with c2_up,
            # preserve crisp fine silhouettes, and fill interior holes from c2_up.
            valid_in = d_in > 0
            agree = valid_in & (c2_up > 0) & ((d_in - c2_up).abs() <= (0.04 * d_in + 35.0))
            d = torch.where(agree, 0.45 * d_in + 0.55 * c2_up, torch.where(valid_in, d_in, c2_up))

            v = d > 0
            jump = v & (avg_in > 0) & ((d - avg_in).abs() > self.delta_mm)
            blend = v & (~jump) & (avg_in > 0)
            avg = torch.where(blend, self.alpha * d + (1.0 - self.alpha) * avg_in, avg_in)
            avg = torch.where(v & (jump | (avg == 0)), d, avg)
            age = torch.where(v, torch.zeros_like(age_in), age_in + 1.0)
            avg = torch.where(age > self.persist, torch.zeros_like(avg), avg)
            out_i32 = avg.round().clamp(0.0, 65535.0).to(torch.int32)
            return out_i32, avg, age

    return _PyramidFilter()


class DepthFilter(Node):
    def __init__(self):
        super().__init__("depth_filter")
        dp = self.declare_parameter
        self.speckle_size = int(dp("speckle_size_px", 200).value)
        self.speckle_diff = float(dp("speckle_max_diff_px", 2.0).value)
        self.use_spatial = bool(dp("spatial", True).value)
        self.use_temporal = bool(dp("temporal", True).value)
        self.alpha = float(dp("temporal_alpha", 0.3).value)
        self.delta_mm = float(dp("temporal_delta_mm", 150.0).value)
        self.persist = int(dp("temporal_persist", 4).value)
        self.fallback_baseline_m = float(dp("baseline_m", 0.0755).value)
        self.fb = None  # fx * baseline (mm * px)
        self.info = None
        self.avg = self.age = None
        self.pin_in = None
        self.gpu_mod = None
        self.traced_shape = None
        self.times = []
        try:
            import torch

            self.torch = torch
            self.dev = torch.device("cuda") if torch.cuda.is_available() else None
        except ImportError:
            self.torch, self.dev = None, None
        self.get_logger().info(f"multi-scale spatial/temporal on {'CUDA' if self.dev else 'CPU fallback'}")
        topic = lambda name, default: str(dp(name, default).value)  # noqa: E731
        self.pub = self.create_publisher(Image, topic("output_topic", "/stereo/depth/image"), OUT_QOS)
        self.pub_info = self.create_publisher(
            CameraInfo, topic("output_info_topic", "/stereo/depth/camera_info"), OUT_QOS
        )
        self.create_subscription(
            CameraInfo, topic("input_info_topic", "/native_oak/depth/camera_info"), self._on_info, LATEST
        )
        self.create_subscription(Image, topic("input_topic", "/native_oak/depth/image_raw"), self._on_depth, LATEST)
        self.create_timer(10.0, self._report)

    def _on_info(self, msg):
        self.info = msg
        fx = msg.k[0]
        baseline = -msg.p[3] / fx if fx and msg.p[3] else self.fallback_baseline_m
        self.fb = fx * baseline * 1000.0

    def _speckle_2x(self, depth):
        h, w = depth.shape
        h2, w2 = h // 2, w // 2
        crop = depth[: h2 * 2, : w2 * 2]
        with np.errstate(divide="ignore"):
            disp = np.where(crop > 0, (self.fb * 16.0) / crop.astype(np.float32), 0.0)
        d16 = np.clip(disp, 0, 32767).astype(np.int16)
        d2 = d16.reshape(h2, 2, w2, 2).max(axis=(1, 3))
        diff16 = int(self.speckle_diff * 16)
        cv2.filterSpeckles(d2, 0, max(16, self.speckle_size // 4), diff16)
        d2_up = np.repeat(np.repeat(d2, 2, axis=0), 2, axis=1)
        keep = (d16 > 0) & (d2_up > 0) & (np.abs(d16.astype(np.int32) - d2_up.astype(np.int32)) <= diff16 * 2)
        out = np.zeros_like(depth)
        out[: h2 * 2, : w2 * 2] = np.where(keep, crop, 0)
        return out

    def _ensure_gpu(self, shape):
        if self.traced_shape == shape and self.gpu_mod is not None:
            return
        torch = self.torch
        h, w = shape
        eager = _build_cuda_module(torch, self.alpha, self.delta_mm, self.persist).to(self.dev).eval()
        dummy = torch.zeros((1, 1, h, w), dtype=torch.float32, device=self.dev)
        with torch.no_grad():
            traced = torch.jit.trace(eager, (dummy, dummy, dummy))
            self.gpu_mod = torch.jit.optimize_for_inference(traced)
            # Warm up kernels
            for _ in range(3):
                self.gpu_mod(dummy, dummy, dummy)
            torch.cuda.synchronize()
        self.avg = torch.zeros_like(dummy)
        self.age = torch.zeros_like(dummy)
        self.pin_in = torch.empty((1, 1, h, w), dtype=torch.float32, pin_memory=True)
        self.traced_shape = shape
        self.get_logger().info(f"traced CUDA pyramid filter for {w}x{h}")

    def _gpu(self, depth):
        torch = self.torch
        self._ensure_gpu(depth.shape)
        np.copyto(self.pin_in.numpy()[0, 0], depth, casting="unsafe")
        d_gpu = self.pin_in.to(self.dev, non_blocking=True)
        with torch.no_grad():
            out_i32, self.avg, self.age = self.gpu_mod(d_gpu, self.avg, self.age)
        return out_i32[0, 0].cpu().numpy().astype(np.uint16)

    def _on_depth(self, msg):
        if self.fb is None or msg.encoding not in ("16UC1", "mono16"):
            return
        t0 = time.perf_counter()
        depth = np.frombuffer(msg.data, dtype="<u2").reshape(msg.height, msg.step // 2)[:, : msg.width]
        out = self._speckle_2x(depth)
        if self.dev is not None and (self.use_spatial or self.use_temporal):
            out = self._gpu(out)
        img = Image()
        img.header = msg.header
        img.height, img.width = out.shape
        img.encoding = "16UC1"
        img.step = out.shape[1] * 2
        img.data = array.array("B", out.tobytes())  # rclpy fast path
        self.pub.publish(img)
        if self.info is not None:
            self.info.header = msg.header
            self.pub_info.publish(self.info)
        self.times.append((time.perf_counter() - t0) * 1000)

    def _report(self):
        if self.times:
            t = np.array(self.times)
            self.get_logger().info(
                f"{len(t) / 10.0:.1f} Hz, {np.median(t):.1f} ms median, {np.percentile(t, 95):.1f} ms p95"
            )
        self.times = []


def main(args=None):
    rclpy.init(args=args)
    node = DepthFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
