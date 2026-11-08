"""TensorRT stereo inference for larp_liteanystereo.

TensorRtSingleStereoEngine runs a single-ONNX stereo engine (inputs left/right
1x3xHxW float, output disparity) in FP16 on the GPU: one host->GPU copy per image,
resize + 3-channel expansion (+ optional ImageNet normalization) on CUDA.
OpenCvSgbmFallbackEngine is a CPU stand-in when no engine file exists.
"""

import logging
import os
import time

import cv2
import numpy as np

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


class TensorRtSingleStereoEngine:
    """Zero-copy GPU TensorRT 10.x runner for single-ONNX stereo models
    (Lite Any Stereo V2 and other left/right -> disparity exports).

    Accepts mono8 or bgr8/rgb8 numpy images, transfers to GPU once, performs
    bilinear resize + 3-channel expansion + ImageNet normalization on CUDA via
    PyTorch, executes the TensorRT FP16 context asynchronously on the current
    CUDA stream, and returns disparity at the target resolution.
    """

    def __init__(self, engine_path: str, target_height: int = 320, target_width: int = 480, normalize: bool = False):
        import tensorrt as trt
        import torch
        import torch.nn.functional as F

        self.trt = trt
        self.torch = torch
        self.F = F
        self.normalize = normalize
        self.logger = trt.Logger(trt.Logger.WARNING)

        if not os.path.isfile(engine_path):
            raise FileNotFoundError(f"TensorRT engine not found: {engine_path}")

        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(self.logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f"Failed to deserialize TensorRT engine {engine_path} "
                f"(TensorRT version {trt.__version__}). Rebuild with trtexec --fp16."
            )

        self.context = self.engine.create_execution_context()
        self.dev = torch.device("cuda")
        self.cuda_stream = torch.cuda.Stream(device=self.dev)

        self.in_names = []
        self.out_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.in_names.append(name)
            else:
                self.out_names.append(name)

        # Inspect engine input shape; if static, adopt its HxW
        in_shape = tuple(self.engine.get_tensor_shape(self.in_names[0]))
        if len(in_shape) == 4 and in_shape[2] > 0 and in_shape[3] > 0:
            self.target_h, self.target_w = int(in_shape[2]), int(in_shape[3])
        else:
            self.target_h, self.target_w = int(target_height), int(target_width)

        self.mean = torch.tensor(IMAGENET_MEAN, dtype=torch.float32, device=self.dev).view(1, 3, 1, 1)
        self.std = torch.tensor(IMAGENET_STD, dtype=torch.float32, device=self.dev).view(1, 3, 1, 1)

        # Pre-bind input shapes and allocate output buffers
        self.in_dtypes = {}
        for name in self.in_names:
            shape = (1, 3, self.target_h, self.target_w)
            self.context.set_input_shape(name, shape)
            self.in_dtypes[name] = self._trt_to_torch_dtype(self.engine.get_tensor_dtype(name))

        self.outputs = {}
        for name in self.out_names:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = self._trt_to_torch_dtype(self.engine.get_tensor_dtype(name))
            self.outputs[name] = torch.empty(shape, device=self.dev, dtype=dtype)
            self.context.set_tensor_address(name, int(self.outputs[name].data_ptr()))

        # Warm up 3 iterations
        dummy = np.zeros((self.target_h, self.target_w), dtype=np.uint8)
        for _ in range(3):
            self.infer(dummy, dummy)
        torch.cuda.synchronize()

    def _trt_to_torch_dtype(self, dt):
        trt = self.trt
        torch = self.torch
        mapping = {
            trt.DataType.FLOAT: torch.float32,
            trt.DataType.HALF: torch.float16,
            trt.DataType.BF16: torch.bfloat16,
            trt.DataType.INT32: torch.int32,
            trt.DataType.INT8: torch.int8,
            trt.DataType.BOOL: torch.bool,
        }
        return mapping.get(dt, torch.float32)

    def _preprocess_gpu(self, img: np.ndarray, dtype):
        torch = self.torch
        F = self.F
        t = torch.from_numpy(img).to(self.dev, non_blocking=True).float()
        if t.ndim == 2:
            t = t.unsqueeze(0).unsqueeze(0)  # (1, 1, H, W)
            if t.shape[2] != self.target_h or t.shape[3] != self.target_w:
                t = F.interpolate(t, size=(self.target_h, self.target_w), mode="bilinear", align_corners=False)
            t = t.expand(1, 3, self.target_h, self.target_w)
        else:
            t = t[:, :, :3].permute(2, 0, 1).unsqueeze(0)  # (1, 3, H, W)
            if t.shape[2] != self.target_h or t.shape[3] != self.target_w:
                t = F.interpolate(t, size=(self.target_h, self.target_w), mode="bilinear", align_corners=False)
        if self.normalize:
            t = (t / 255.0 - self.mean) / self.std
        return t.to(dtype).contiguous()

    def infer(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Run stereo inference and return disparity at (target_h, target_w) in target-width pixels."""
        torch = self.torch
        with torch.no_grad(), torch.cuda.stream(self.cuda_stream):
            left_name = self.in_names[0]
            right_name = self.in_names[1]
            t_left = self._preprocess_gpu(left, self.in_dtypes[left_name])
            t_right = self._preprocess_gpu(right, self.in_dtypes[right_name])

            self.context.set_tensor_address(left_name, int(t_left.data_ptr()))
            self.context.set_tensor_address(right_name, int(t_right.data_ptr()))

            if not self.context.execute_async_v3(self.cuda_stream.cuda_stream):
                raise RuntimeError("TensorRT execute_async_v3 failed")

            disp_t = self.outputs[self.out_names[0]].float()
            disp_gpu = disp_t.squeeze().clamp_min(0.0)
        self.cuda_stream.synchronize()
        return disp_gpu.cpu().numpy()


class OpenCvSgbmFallbackEngine:
    """Fast SGBM + WLS/hole-fill fallback when no TensorRT engine file is present yet."""

    def __init__(self, target_height: int = 320, target_width: int = 480, max_disp: int = 160):
        self.target_h = int(target_height)
        self.target_w = int(target_width)
        self.max_disp = (int(max_disp) // 16) * 16
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.max_disp,
            blockSize=5,
            P1=8 * 25,
            P2=32 * 25,
            disp12MaxDiff=2,
            uniquenessRatio=8,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def infer(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        if left.ndim == 3:
            left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        if right.ndim == 3:
            right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        if left.shape[:2] != (self.target_h, self.target_w):
            left = cv2.resize(left, (self.target_w, self.target_h), interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, (self.target_w, self.target_h), interpolation=cv2.INTER_AREA)
        disp = self.sgbm.compute(left, right).astype(np.float32) / 16.0
        return np.clip(disp, 0.0, None)


def disparity_to_depth_mm(
    disp: np.ndarray,
    fx_orig: float,
    baseline_m: float,
    orig_w: int,
    orig_h: int,
    out_w: int = 0,
    out_h: int = 0,
    min_depth_m: float = 0.15,
    max_depth_m: float = 10.0,
    remove_invisible: bool = True,
) -> tuple[np.ndarray, float]:
    """Convert disparity at (H_net, W_net) to 16UC1 depth in millimeters.

    Returns (depth_mm_uint16, scale_x_out_over_orig).
    """
    h_net, w_net = disp.shape[:2]
    out_w = out_w or w_net
    out_h = out_h or h_net

    # Mask left border non-overlapping region where (x - disp) < 0 in right camera
    if remove_invisible:
        xx = np.arange(w_net, dtype=np.float32)[None, :]
        invalid = (xx - disp) < 0.0
        disp = np.where(invalid, 0.0, disp)

    # Disparity at w_net corresponds to focal length fx_net = fx_orig * (w_net / orig_w)
    fx_net = float(fx_orig) * (float(w_net) / float(orig_w))
    fb_mm = fx_net * float(baseline_m) * 1000.0

    with np.errstate(divide="ignore", invalid="ignore"):
        depth_mm = np.where(disp > 0.25, fb_mm / disp, 0.0)

    min_mm = float(min_depth_m) * 1000.0
    max_mm = float(max_depth_m) * 1000.0
    depth_mm = np.where((depth_mm >= min_mm) & (depth_mm <= max_mm), depth_mm, 0.0)

    if (out_h, out_w) != (h_net, w_net):
        depth_mm = cv2.resize(depth_mm, (out_w, out_h), interpolation=cv2.INTER_NEAREST)

    depth_u16 = np.clip(np.round(depth_mm), 0, 65535).astype(np.uint16)
    return depth_u16, float(out_w) / float(orig_w)
