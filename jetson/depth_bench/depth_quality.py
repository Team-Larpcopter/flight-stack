"""Depth quality per StereoDepth setting, in our target layout (stack STOPPED).

    python3 depth_quality.py OUT_DIR [SECONDS]

Layout: CAM_B/CAM_C @ 30 fps; 1280x800 GRAY8 -> host; 800x500 -> StereoDepth
(decimation 1) -> host. For each setting:
  * dumps the preset's resolved configuration (what the preset actually sets)
  * rate and size of the depth stream
  * fill: share of pixels with a depth value (mean over frames), plus centre
    vs border, because stereo always loses a strip at the left edge
  * noise on a still scene: per-pixel temporal std of depth, as a % of depth,
    for pixels valid in >= 90 % of frames, overall and near 1 / 2 / 3 m
  * flicker: share of pixels that switch between valid and invalid
  * writes OUT_DIR/<setting>.png (colourised: red near, blue far, black none,
    0.3-6 m) and OUT_DIR/left.png (the camera image for reference)
"""

import json
import os
import sys
import time

import cv2
import depthai as dai
import numpy as np

OUT = sys.argv[1]
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
P = dai.node.StereoDepth.PresetMode
FULL, STEREO, FPS = (1280, 800), (800, 500), 30

SETTINGS = [
    # name, preset, turn post-processing filters off?
    ("default_nofilters", P.DEFAULT, True),
    ("fast_density", P.FAST_DENSITY, False),
    ("fast_accuracy", P.FAST_ACCURACY, False),
    ("robotics", P.ROBOTICS, False),
    ("robotics_nofilters", P.ROBOTICS, True),
]


def g(obj, name):
    v = getattr(obj, name, None)
    return v if isinstance(v, (int, float, bool)) or v is None else str(v)


def dump(cfg):
    a, cm, pp = cfg.algorithmControl, cfg.costMatching, cfg.postProcessing
    return {
        "lr_check": g(a, "enableLeftRightCheck"),
        "lr_threshold": g(a, "leftRightCheckThreshold"),
        "subpixel": g(a, "enableSubpixel"),
        "extended_disparity": g(a, "enableExtended"),
        "confidence_threshold": g(cm, "confidenceThreshold"),
        "median": g(pp, "median"),
        "decimation": g(pp.decimationFilter, "decimationFactor"),
        "speckle": g(pp.speckleFilter, "enable"),
        "spatial": g(pp.spatialFilter, "enable"),
        "spatial_hole_fill_radius": g(pp.spatialFilter, "holeFillingRadius"),
        "temporal": g(pp.temporalFilter, "enable"),
        "threshold_mm": [g(pp.thresholdFilter, "minRange"), g(pp.thresholdFilter, "maxRange")],
        "hole_filling": g(getattr(pp, "holeFilling", None), "enable"),
        "adaptive_median": g(getattr(pp, "adaptiveMedianFilter", None), "enable"),
    }


def colorize(d_mm):
    m = d_mm.astype(np.float32) / 1000.0
    valid = m > 0
    s = ((6.0 - np.clip(m, 0.3, 6.0)) / 5.7 * 255).astype(np.uint8)
    c = cv2.applyColorMap(s, cv2.COLORMAP_JET)
    c[~valid] = 0
    return c


def stats(frames):
    st = np.stack(frames).astype(np.float32)  # N,H,W mm
    valid = st > 0
    fill = valid.mean(axis=(1, 2))
    h, w = st.shape[1:]
    centre = valid[:, h // 4 : 3 * h // 4, w // 4 : 3 * w // 4].mean()
    stable = valid.mean(axis=0) >= 0.9
    flicker = float(((valid.mean(axis=0) > 0.05) & (valid.mean(axis=0) < 0.95)).mean())
    masked = np.where(valid, st, np.nan)
    mean = np.nanmean(masked, axis=0)
    std = np.nanstd(masked, axis=0)
    rel = (std / mean)[stable]
    out = {
        "fill_mean": round(float(fill.mean()), 3),
        "fill_centre": round(float(centre), 3),
        "flicker": round(flicker, 3),
        "noise_pct_median": round(float(np.nanmedian(rel)) * 100, 2) if rel.size else None,
        "range_m_p5_p50_p95": [round(float(x) / 1000, 2) for x in np.nanpercentile(mean[stable], [5, 50, 95])]
        if stable.any() else None,
    }  # fmt: skip
    for lo, hi in ((0.7, 1.3), (1.6, 2.4), (2.5, 3.5)):
        sel = stable & (mean >= lo * 1000) & (mean < hi * 1000)
        if sel.sum() > 50:
            out[f"noise_cm_at_{(lo + hi) / 2:.0f}m"] = round(float(np.nanmedian(std[sel])) / 10, 1)
    return out


def run(name, preset, nofilters):
    res = {"setting": name}
    with dai.Pipeline() as p:
        outs = {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            cam = p.create(dai.node.Camera).build(sock, sensorFps=FPS)
            outs[side] = (
                cam.requestOutput(FULL, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
                cam.requestOutput(STEREO, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
            )
        st = p.create(dai.node.StereoDepth).build(outs["left"][1], outs["right"][1], presetMode=preset)
        st.setLeftRightCheck(True)
        for inp in ("left", "right", "inputLeft", "inputRight"):  # v3 names first
            port = getattr(st, inp, None)
            if port is not None and hasattr(port, "setBlocking"):
                port.setBlocking(False)
        pp = st.initialConfig.postProcessing
        pp.decimationFilter.decimationFactor = 1
        if nofilters:
            for f in ("spatialFilter", "temporalFilter", "speckleFilter"):
                getattr(pp, f).enable = False
        res["config"] = dump(st.initialConfig)
        qd = st.depth.createOutputQueue(maxSize=4, blocking=False)
        ql = outs["left"][0].createOutputQueue(maxSize=2, blocking=False)
        qr = outs["right"][0].createOutputQueue(maxSize=2, blocking=False)  # realistic load
        p.start()
        time.sleep(3)
        qd.tryGetAll()
        frames, stamps, left = [], [], None
        end = time.monotonic() + SECONDS
        while time.monotonic() < end:
            for m in qd.tryGetAll():
                frames.append(m.getFrame().copy())
                stamps.append(m.getTimestampDevice().total_seconds())
            for m in ql.tryGetAll():
                left = m.getFrame()
            qr.tryGetAll()
            time.sleep(0.002)
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
    res["hz"] = round((len(stamps) - 1) / span, 1) if span else 0
    res["size"] = f"{frames[0].shape[1]}x{frames[0].shape[0]}" if frames else None
    if frames:
        res.update(stats(frames))
        cv2.imwrite(os.path.join(OUT, f"{name}.png"), colorize(frames[len(frames) // 2]))
    if left is not None and not os.path.exists(os.path.join(OUT, "left.png")):
        cv2.imwrite(os.path.join(OUT, "left.png"), left)
    return res


def main():
    os.makedirs(OUT, exist_ok=True)
    for s in SETTINGS:
        try:
            print(json.dumps(run(*s)), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"setting": s[0], "error": str(e)[:300]}), flush=True)
        time.sleep(3)
    print(json.dumps({"setting": "DONE"}), flush=True)


if __name__ == "__main__":
    main()
