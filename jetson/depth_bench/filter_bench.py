"""Depth post-processing on the Jetson: CPU (OpenCV/numpy) vs GPU (PyTorch CUDA).

    python3 filter_bench.py DEPTH.npz OUT_DIR

Run inside the running stack container (torch + CUDA there, GPU shared with
cuVSLAM as in flight). Replays captured raw depth frames through filter chains
and reports per-frame time, process CPU, and quality before/after (fill,
flicker, temporal noise), and writes a colourised frame per chain.

Filters (mirroring the OAK's on-device post-processing):
  speckle   cv2.filterSpeckles on disparity: removes small isolated blobs that
            differ from their surroundings (the "phantom obstacle" killer).
            CPU only: it needs connected components.
  median    5x5 median (hardware on the OAK already does 7x7; optional extra)
  spatial   edge-preserving smoothing + small-hole fill: 5x5 normalised
            convolution weighted by depth similarity (bilateral-like)
  temporal  per-pixel exponential average over frames, reset on big changes,
            keeps a pixel alive for a few frames if it drops out (persistence)
"""

import json
import os
import sys
import time

import cv2
import numpy as np

DATA, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
z = np.load(DATA)
FRAMES = z["depth"]  # N,H,W uint16 mm
FB = float(z["fx"]) * float(z["baseline_m"]) * 1000.0  # disparity(px) = FB / depth(mm)
SPECKLE_SIZE = 200  # px: blobs smaller than this are removed
SPECKLE_DIFF = 2.0  # px of disparity: max step inside one blob
ALPHA, DELTA_MM, PERSIST = 0.4, 150, 3


def to_disp16(depth):
    with np.errstate(divide="ignore"):
        d = np.where(depth > 0, FB / depth.astype(np.float32), 0)
    return np.clip(d * 16, 0, 32767).astype(np.int16)  # 4 fractional bits, like OpenCV SGBM


def speckle_cpu(depth):
    disp = to_disp16(depth)
    cv2.filterSpeckles(disp, 0, SPECKLE_SIZE, int(SPECKLE_DIFF * 16))
    out = depth.copy()
    out[disp == 0] = 0
    return out


def spatial_cpu(depth):
    d = depth.astype(np.float32)
    valid = (d > 0).astype(np.float32)
    acc = np.zeros_like(d)
    wsum = np.zeros_like(d)
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            n = np.roll(np.roll(d, dy, 0), dx, 1)
            nv = np.roll(np.roll(valid, dy, 0), dx, 1)
            ref = np.where(d > 0, d, n)
            w = nv * np.exp(-(((n - ref) / (0.03 * ref + 1e-3)) ** 2))
            acc += w * n
            wsum += w
    out = np.where(wsum > 0.5, acc / np.maximum(wsum, 1e-6), 0)
    return out.astype(np.uint16)


class TemporalCPU:
    def __init__(self):
        self.avg = None
        self.age = None

    def __call__(self, depth):
        d = depth.astype(np.float32)
        if self.avg is None:
            self.avg, self.age = d.copy(), np.zeros(d.shape, np.int32)
        v = d > 0
        jump = v & (self.avg > 0) & (np.abs(d - self.avg) > DELTA_MM)
        blend = v & ~jump & (self.avg > 0)
        self.avg[blend] = ALPHA * d[blend] + (1 - ALPHA) * self.avg[blend]
        fresh = v & (jump | (self.avg == 0))
        self.avg[fresh] = d[fresh]
        self.age[v] = 0
        self.age[~v] += 1
        self.avg[self.age > PERSIST] = 0
        return self.avg.astype(np.uint16)


def chain_cpu(names):
    temporal = TemporalCPU()

    def run(depth):
        out = depth
        for n in names:
            if n == "speckle":
                out = speckle_cpu(out)
            elif n == "median":
                out = cv2.medianBlur(out, 5)
            elif n == "spatial":
                out = spatial_cpu(out)
            elif n == "temporal":
                out = temporal(out)
        return out

    return run


def chain_gpu(names):
    import torch
    import torch.nn.functional as F

    dev = torch.device("cuda")
    state = {"avg": None, "age": None}

    def spatial(d):
        valid = (d > 0).float()
        pad = F.pad(d[None, None], (2, 2, 2, 2), mode="replicate")
        vpad = F.pad(valid[None, None], (2, 2, 2, 2))
        nb = F.unfold(pad, 5).view(25, *d.shape)
        nv = F.unfold(vpad, 5).view(25, *d.shape)
        # empty centre pixel: compare each neighbour with itself (weight 1), as on the CPU,
        # so small holes are filled from valid neighbours
        ref = torch.where(d > 0, d, nb)
        w = nv * torch.exp(-(((nb - ref) / (0.03 * ref + 1e-3)) ** 2))
        wsum = w.sum(0)
        return torch.where(wsum > 0.5, (w * nb).sum(0) / wsum.clamp_min(1e-6), torch.zeros_like(d))

    def temporal(d):
        if state["avg"] is None:
            state["avg"], state["age"] = d.clone(), torch.zeros_like(d)
        avg, age = state["avg"], state["age"]
        v = d > 0
        jump = v & (avg > 0) & ((d - avg).abs() > DELTA_MM)
        blend = v & ~jump & (avg > 0)
        avg = torch.where(blend, ALPHA * d + (1 - ALPHA) * avg, avg)
        avg = torch.where(v & (jump | (avg == 0)), d, avg)
        age = torch.where(v, torch.zeros_like(age), age + 1)
        avg = torch.where(age > PERSIST, torch.zeros_like(avg), avg)
        state["avg"], state["age"] = avg, age
        return avg

    def run(depth):
        out = speckle_cpu(depth) if "speckle" in names else depth
        g = torch.from_numpy(out.astype(np.float32)).to(dev, non_blocking=True)
        for n in names:
            if n == "spatial":
                g = spatial(g)
            elif n == "temporal":
                g = temporal(g)
        return g.round().clamp(0, 65535).to(torch.int32).cpu().numpy().astype(np.uint16)

    run(FRAMES[0])  # warm-up: CUDA context, kernels
    torch.cuda.synchronize()
    state["avg"] = None
    return run


def quality(frames):
    s = np.stack(frames).astype(np.float32)
    valid = s > 0
    h, w = s.shape[1:]
    out = {
        "fill_all": round(float(valid.mean()), 3),
        "fill_centre": round(float(valid[:, h // 4 : 3 * h // 4, w // 4 : 3 * w // 4].mean()), 3),
        "flicker": round(float(((valid.mean(0) > 0.05) & (valid.mean(0) < 0.95)).mean()), 3),
    }
    stable = valid.mean(0) >= 0.9
    mean = np.nanmean(np.where(valid, s, np.nan), 0)
    std = np.nanstd(np.where(valid, s, np.nan), 0)
    for lo, hi in ((0.7, 1.3), (1.6, 2.4), (2.5, 3.5)):
        sel = stable & (mean >= lo * 1000) & (mean < hi * 1000)
        if sel.sum() > 50:
            out[f"noise_cm_{(lo + hi) / 2:.0f}m"] = round(float(np.nanmedian(std[sel])) / 10, 1)
    return out


def colorize(d):
    m = d.astype(np.float32) / 1000.0
    c = cv2.applyColorMap(((6 - np.clip(m, 0.3, 6)) / 5.7 * 255).astype(np.uint8), cv2.COLORMAP_JET)
    c[m <= 0] = 0
    return c


CHAINS = [
    ("raw", "cpu", []),
    ("cpu_speckle", "cpu", ["speckle"]),
    ("cpu_speckle_median", "cpu", ["speckle", "median"]),
    ("cpu_speckle_spatial", "cpu", ["speckle", "spatial"]),
    ("cpu_speckle_spatial_temporal", "cpu", ["speckle", "spatial", "temporal"]),
    ("gpu_speckle(cpu)_spatial", "gpu", ["speckle", "spatial"]),
    ("gpu_speckle(cpu)_spatial_temporal", "gpu", ["speckle", "spatial", "temporal"]),
    ("gpu_spatial_temporal", "gpu", ["spatial", "temporal"]),
]


def main():
    only = sys.argv[3] if len(sys.argv) > 3 else ""
    print(json.dumps({"frames": int(len(FRAMES)), "size": f"{FRAMES.shape[2]}x{FRAMES.shape[1]}",
                      "fx": float(z["fx"]), "baseline_m": float(z["baseline_m"])}), flush=True)  # fmt: skip
    for name, kind, names in CHAINS:
        if only and not (name.startswith(only) or name == "raw"):
            continue
        try:
            fn = chain_gpu(names) if kind == "gpu" else chain_cpu(names)
            outs, times = [], []
            c0, t0 = os.times(), time.perf_counter()
            for f in FRAMES:
                s = time.perf_counter()
                outs.append(fn(f))
                times.append((time.perf_counter() - s) * 1000)
            wall = time.perf_counter() - t0
            c1 = os.times()
            cpu = (c1.user - c0.user + c1.system - c0.system) / wall * 100
            ms = float(np.median(times))
            row = {"chain": name, "ms_median": round(ms, 2), "ms_p95": round(float(np.percentile(times, 95)), 2),
                   "max_hz": round(1000 / ms, 1) if ms else None,
                   "cpu_pct_one_core_while_busy": round(cpu, 0),
                   "cpu_pct_one_core_at_30hz": round(cpu * min(1.0, ms / 33.3), 0)}  # fmt: skip
            row.update(quality(outs))
            cv2.imwrite(os.path.join(OUT, f"{name}.png"), colorize(outs[len(outs) // 2]))
            print(json.dumps(row), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"chain": name, "error": str(e)[:300]}), flush=True)


if __name__ == "__main__":
    main()
