"""Time stereo depth on the Jetson: VPI (GPU / other backends) vs OpenCV (CPU).

Run on the Jetson HOST (VPI's Python bindings are there), while the stack runs,
so the GPU is shared with cuVSLAM as it would be in flight:

    python3 jetson_stereo_bench.py PAIR_DIR [ITERS]

PAIR_DIR holds left.png / right.png (a rectified mono pair from grab_pair.py).
Each size is produced by resizing that pair, so timings are representative
but quality at upscaled sizes is not. maxdisp is chosen so that the minimum
depth is ~0.3 m with the OAK-D S2 7.5 cm baseline.
Prints one JSON line per (method, size): ms per frame, fps, valid-pixel ratio.
"""

import json
import sys
import time

import cv2
import numpy as np

SIZES = [(640, 400), (800, 500), (1024, 640), (1280, 800)]
# fx ~ 0.625 * width for this lens (measured fx 250 at 400 px wide) ->
# disparity at 0.3 m = fx * 0.075 / 0.3; rounded up to a multiple of 16.
MAXDISP = {640: 112, 800: 128, 1024: 160, 1280: 208}


def timeit(fn, iters):
    fn()  # warm-up (allocations, JIT)
    t = time.perf_counter()
    for _ in range(iters):
        out = fn()
    ms = (time.perf_counter() - t) / iters * 1000
    return ms, out


def report(method, size, ms, valid, extra=None):
    row = {
        "method": method,
        "size": f"{size[0]}x{size[1]}",
        "ms": round(ms, 1),
        "fps": round(1000 / ms, 1),
        "valid": round(valid, 3),
    }
    row.update(extra or {})
    print(json.dumps(row), flush=True)


def bench_vpi(left, right, size, iters):
    import vpi

    maxdisp = MAXDISP[size[0]]
    results = []
    for name, backend in (
        ("vpi_cuda", vpi.Backend.CUDA),
        ("vpi_ofa", getattr(vpi.Backend, "OFA", None)),
        ("vpi_ofa_pva_vic", None),
    ):
        try:
            if name == "vpi_ofa_pva_vic":
                backend = vpi.Backend.OFA | vpi.Backend.PVA | vpi.Backend.VIC
            if backend is None:
                raise RuntimeError("backend not in this VPI build")
            stream = vpi.Stream()
            with stream:
                fmt = vpi.Format.Y16_ER
                lv = vpi.asimage(left).convert(fmt, scale=1, backend=vpi.Backend.CUDA)
                rv = vpi.asimage(right).convert(fmt, scale=1, backend=vpi.Backend.CUDA)
                stream.sync()

                def run(lv=lv, rv=rv, backend=backend, stream=stream):
                    with stream:
                        d = vpi.stereodisp(lv, rv, backend=backend, window=5, maxdisp=maxdisp)
                    stream.sync()
                    return d

                ms, disp = timeit(run, iters)
                arr = disp.cpu().astype(np.float32) / 32.0  # Q10.5 fixed point
                results.append((name, ms, float((arr > 0).mean()), {"maxdisp": maxdisp}))
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"method": name, "size": f"{size[0]}x{size[1]}",
                              "error": str(e)[:160]}), flush=True)  # fmt: skip
    for r in results:
        report(r[0], size, r[1], r[2], r[3])


def bench_opencv(left, right, size, iters):
    nd = MAXDISP[size[0]]
    bm = cv2.StereoBM_create(numDisparities=nd, blockSize=15)
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=nd, blockSize=5, P1=8 * 25, P2=32 * 25,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )  # fmt: skip
    for name, m in (("opencv_bm_cpu", bm), ("opencv_sgbm3way_cpu", sgbm)):
        ms, disp = timeit(lambda m=m: m.compute(left, right), max(3, iters // 5))
        report(name, size, ms, float((disp > 0).mean()), {"maxdisp": nd})


def main():
    pair = sys.argv[1]
    iters = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    L0 = cv2.imread(f"{pair}/left.png", cv2.IMREAD_GRAYSCALE)
    R0 = cv2.imread(f"{pair}/right.png", cv2.IMREAD_GRAYSCALE)
    print(json.dumps({"pair": f"{L0.shape[1]}x{L0.shape[0]}", "cv2": cv2.__version__,
                      "threads": cv2.getNumThreads()}), flush=True)  # fmt: skip
    for size in SIZES:
        L = cv2.resize(L0, size, interpolation=cv2.INTER_LINEAR)
        R = cv2.resize(R0, size, interpolation=cv2.INTER_LINEAR)
        bench_vpi(L, R, size, iters)
        bench_opencv(L, R, size, iters)


if __name__ == "__main__":
    main()
