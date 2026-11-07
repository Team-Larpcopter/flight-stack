"""Filtered depth in the larp_oak layout, plus a calibration check (stack STOPPED).

    python3 filters_test.py OUT_DIR [SECONDS]

Layout: CAM_B/CAM_C @ 30 fps; 1280x800 GRAY8 -> host (VIO); 800x500 ->
StereoDepth (NON-BLOCKING inputs, decimation 1) -> depth + rectified pair.
Per case: rates of the VIO images and depth, fill (all / centre / without the
left stereo band), where the holes are (left band, too close, elsewhere),
noise on a still scene, and an epipolar check on the rectified pair: ORB
matches between rectified left/right should lie on the same row, so the median
|dy| is the calibration's vertical error in pixels.
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

CASES = [
    # name, spatial, speckle, temporal, extended disparity
    ("A_robotics_all_filters", True, True, False, False),
    ("B_speckle_median_no_spatial", False, True, False, False),
    ("C_B_plus_extended_disparity", False, True, False, True),
]


def colorize(d_mm):
    m = d_mm.astype(np.float32) / 1000.0
    s = ((6.0 - np.clip(m, 0.2, 6.0)) / 5.8 * 255).astype(np.uint8)
    c = cv2.applyColorMap(s, cv2.COLORMAP_JET)
    c[m <= 0] = 0
    return c


def epipolar(left, right):
    orb = cv2.ORB_create(3000)
    k1, d1 = orb.detectAndCompute(left, None)
    k2, d2 = orb.detectAndCompute(right, None)
    if d1 is None or d2 is None:
        return None
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(d1, d2)
    dy, dx = [], []
    for m in matches:
        (x1, y1), (x2, y2) = k1[m.queryIdx].pt, k2[m.trainIdx].pt
        if 0 < x1 - x2 < 200 and abs(y1 - y2) < 20:  # plausible stereo match
            dy.append(abs(y1 - y2))
            dx.append(x1 - x2)
    if len(dy) < 30:
        return {"matches": len(dy)}
    dy = np.array(dy)
    return {
        "matches": int(len(dy)),
        "dy_median_px": round(float(np.median(dy)), 2),
        "dy_p90_px": round(float(np.percentile(dy, 90)), 2),
        "within_1px": round(float((dy <= 1.0).mean()), 3),
    }


def run(name, spatial, speckle, temporal, extended):
    res = {"case": name}
    with dai.Pipeline() as p:
        cams = {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            cam = p.create(dai.node.Camera).build(sock, sensorFps=FPS)
            cams[side] = (
                cam.requestOutput(FULL, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
                cam.requestOutput(STEREO, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
            )
        st = p.create(dai.node.StereoDepth).build(cams["left"][1], cams["right"][1], presetMode=P.ROBOTICS)
        st.setLeftRightCheck(True)
        st.setExtendedDisparity(extended)
        st.left.setBlocking(False)
        st.right.setBlocking(False)
        pp = st.initialConfig.postProcessing
        pp.decimationFilter.decimationFactor = 1
        pp.spatialFilter.enable = spatial
        pp.speckleFilter.enable = speckle
        pp.temporalFilter.enable = temporal
        res["config"] = {
            "median": str(pp.median), "spatial": spatial, "speckle": speckle,
            "temporal": temporal, "extended": extended,
            "confidence": st.initialConfig.costMatching.confidenceThreshold,
        }  # fmt: skip
        q = {
            "left_full": cams["left"][0].createOutputQueue(maxSize=2, blocking=False),
            "right_full": cams["right"][0].createOutputQueue(maxSize=2, blocking=False),
            "depth": st.depth.createOutputQueue(maxSize=4, blocking=False),
            "rect_left": st.rectifiedLeft.createOutputQueue(maxSize=2, blocking=False),
            "rect_right": st.rectifiedRight.createOutputQueue(maxSize=2, blocking=False),
        }
        p.start()
        time.sleep(3)
        for x in q.values():
            x.tryGetAll()
        stamps = {k: [] for k in q}
        frames, last = [], {}
        end = time.monotonic() + SECONDS
        while time.monotonic() < end:
            for k, x in q.items():
                for m in x.tryGetAll():
                    stamps[k].append(m.getTimestampDevice().total_seconds())
                    if k == "depth":
                        frames.append(m.getFrame().copy())
                    elif k.startswith("rect") or k == "left_full":
                        last[k] = m.getFrame()
            time.sleep(0.002)
    for k, ts in stamps.items():
        gaps = sorted(b - a for a, b in zip(ts, ts[1:], strict=False))
        span = ts[-1] - ts[0] if len(ts) > 1 else 0
        res[k + "_hz"] = round((len(ts) - 1) / span, 1) if span else 0
        res[k + "_max_gap_ms"] = round(gaps[-1] * 1000, 1) if gaps else None
    if frames:
        s = np.stack(frames).astype(np.float32)
        valid = s > 0
        h, w = s.shape[1:]
        band = int(w * 0.12)  # left stereo band (no overlap with the right camera)
        res["fill_all"] = round(float(valid.mean()), 3)
        res["fill_centre"] = round(float(valid[:, h // 4 : 3 * h // 4, w // 4 : 3 * w // 4].mean()), 3)
        res["fill_without_left_band"] = round(float(valid[:, :, band:].mean()), 3)
        holes = ~valid
        res["holes_in_left_band_pct"] = round(float(holes[:, :, :band].sum() / max(holes.sum(), 1)) * 100, 1)
        stable = valid.mean(axis=0) >= 0.9
        res["flicker"] = round(float(((valid.mean(0) > 0.05) & (valid.mean(0) < 0.95)).mean()), 3)
        mean = np.nanmean(np.where(valid, s, np.nan), axis=0)
        std = np.nanstd(np.where(valid, s, np.nan), axis=0)
        for lo, hi in ((0.7, 1.3), (1.6, 2.4), (2.5, 3.5)):
            sel = stable & (mean >= lo * 1000) & (mean < hi * 1000)
            if sel.sum() > 50:
                res[f"noise_cm_at_{(lo + hi) / 2:.0f}m"] = round(float(np.nanmedian(std[sel])) / 10, 1)
        if stable.any():
            res["range_m_p5_p50_p95"] = [round(float(x) / 1000, 2) for x in np.nanpercentile(mean[stable], [5, 50, 95])]
        cv2.imwrite(os.path.join(OUT, f"{name}.png"), colorize(frames[len(frames) // 2]))
    if "rect_left" in last and "rect_right" in last:
        res["epipolar"] = epipolar(last["rect_left"], last["rect_right"])
        cv2.imwrite(os.path.join(OUT, "rect_left.png"), last["rect_left"])
        cv2.imwrite(os.path.join(OUT, "rect_right.png"), last["rect_right"])
    if "left_full" in last:
        cv2.imwrite(os.path.join(OUT, "left_full.png"), last["left_full"])
    return res


def main():
    os.makedirs(OUT, exist_ok=True)
    for c in CASES:
        try:
            print(json.dumps(run(*c)), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"case": c[0], "error": str(e)[:300]}), flush=True)
        time.sleep(3)
    print(json.dumps({"case": "DONE"}), flush=True)


if __name__ == "__main__":
    main()
