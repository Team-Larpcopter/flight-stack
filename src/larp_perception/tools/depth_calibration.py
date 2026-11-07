#!/usr/bin/env python3
"""Check and (optionally) improve the OAK stereo calibration with DepthAI's
Dynamic Calibration (DCL), with a before/after depth comparison.

    depth_calibration.py OUT_DIR [--seconds 60] [--mode DEFAULT] [--apply] [--flash]
    depth_calibration.py OUT_DIR --restore BACKUP.json

Opens the camera itself: stop the camera node first (only one program can own
the OAK). What it does:

1. Backs up the current and the factory calibration to OUT_DIR (always).
2. BEFORE: records ~3 s of depth (same settings as oak_native_node: 800x500,
   ROBOTICS, LR check, subpixel, rectified left) with the camera STILL, and
   reports fill ratio, temporal noise and, with --plane, a plane fit in the
   central ROI (point it at a flat wall; RMS residual and distance).
3. DCL: collects images while you SLOWLY move the drone over a textured scene
   (objects at several depths, no blank walls) for up to --seconds, printing
   coverage, then reports the proposed calibration: rotation change between
   the cameras, data and calibration confidence. It never changes intrinsics or
   the baseline (keepCameraCenters).
4. With --apply: applies the new calibration to this session only (RAM), asks
   you to put the camera back at the same still view and records AFTER, so you
   see whether depth actually improved. Nothing is written to the camera.
5. With --flash (implies --apply): after the comparison, writes the new
   calibration to the camera's EEPROM (asks for confirmation). The factory
   calibration is stored separately and is not touched; --restore BACKUP.json
   (or the factory backup) puts any saved calibration back.

After a flash: restart the camera node, regenerate the VIO calibration (its
right-camera pose comes from this calibration) and re-run the depth bench.
Everything printed is also written to OUT_DIR/report.json.
"""

import argparse
import json
import os
import sys
import time

import depthai as dai
import numpy as np


def depth_metrics(frames, plane_roi=None):
    a = np.stack(frames).astype(np.float32)  # mm, 0 = none
    valid = a > 0
    out = {"frames": len(frames), "fill_ratio": round(float(valid.mean()), 4)}
    steady = valid.mean(axis=0) >= 0.9
    if steady.any():
        m = np.where(valid, a, np.nan)
        std = np.nanstd(m, axis=0)[steady]
        out["temporal_noise_mm_median"] = round(float(np.median(std)), 2)
        out["temporal_noise_pct_median"] = round(float(np.median(std / np.nanmean(m, axis=0)[steady]) * 100), 3)
    if plane_roi is not None:
        k, (x0, y0, x1, y1) = plane_roi
        d = np.nanmedian(np.where(valid, a, np.nan), axis=0)[y0:y1, x0:x1] / 1000.0
        v, u = np.mgrid[y0:y1, x0:x1]
        ok = np.isfinite(d)
        if ok.sum() > 100:
            z = d[ok]
            pts = np.c_[(u[ok] - k[0][2]) * z / k[0][0], (v[ok] - k[1][2]) * z / k[1][1], z]
            c = pts.mean(0)
            normal = np.linalg.svd(pts - c)[2][2]
            res = (pts - c) @ normal
            out["plane"] = {
                "points": int(ok.sum()),
                "rms_mm": round(float(np.sqrt((res**2).mean()) * 1000), 2),
                "p95_abs_mm": round(float(np.percentile(np.abs(res), 95) * 1000), 2),
                "centre_distance_m": round(float(np.median(z)), 4),
                "tilt_deg": round(float(np.degrees(np.arccos(abs(normal[2])))), 2),
            }
    return out


def describe(obj, depth=0):
    """Best-effort plain dict of a DepthAI result object (fields differ by version)."""
    if obj is None or depth > 3:
        return None
    if isinstance(obj, (int, float, str, bool)):
        return obj
    if isinstance(obj, (list, tuple)):
        return [describe(x, depth + 1) for x in obj][:16]
    if isinstance(obj, dict):
        return {str(k): describe(v, depth + 1) for k, v in obj.items()}
    out = {}
    for name in dir(obj):
        if name.startswith("_") or name in ("newCalibration", "currentCalibration"):
            continue
        try:
            v = getattr(obj, name)
        except Exception:  # noqa: BLE001
            continue
        if callable(v):
            continue
        out[name] = describe(v, depth + 1)
    return out or str(obj)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=60.0, help="max DCL data collection time")
    ap.add_argument("--mode", default="DEFAULT",
                    choices=["DEFAULT", "OPTIMIZE_PERFORMANCE", "OPTIMIZE_SPEED", "STATIC_SCENERY", "RELAXED_COVERAGE"])
    ap.add_argument("--apply", action="store_true", help="apply to this session and measure AFTER")
    ap.add_argument("--flash", action="store_true", help="write to the camera EEPROM after the comparison")
    ap.add_argument("--plane", action="store_true", help="plane fit in the central ROI (point at a flat wall)")
    ap.add_argument("--restore", help="flash this saved calibration JSON back and exit")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    report = {"args": vars(a), "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def save():
        json.dump(report, open(os.path.join(a.out, "report.json"), "w"), indent=2, default=str)

    if a.restore:
        dev = dai.Device()
        cal = dai.CalibrationHandler(a.restore)
        dev.readCalibration().eepromToJsonFile(os.path.join(a.out, f"before_restore_{int(time.time())}.json"))
        if input(f"Flash {a.restore} to the camera? type YES: ") == "YES":
            dev.flashCalibration(cal)
            print("restored. Restart the camera node.")
        return

    with dai.Pipeline() as p:
        p.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
        dev = p.getDefaultDevice()
        current = dev.readCalibration()
        current.eepromToJsonFile(os.path.join(a.out, "backup_current.json"))
        try:
            dev.readFactoryCalibration().eepromToJsonFile(os.path.join(a.out, "backup_factory.json"))
        except Exception as e:  # noqa: BLE001
            print("factory calibration not readable:", e)
        print(f"backups written to {a.out}/backup_current.json and backup_factory.json")
        dev.setCalibration(current)

        cams, depth_in = {}, {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            cam = p.create(dai.node.Camera).build(sock, sensorFps=30.0)
            cams[side] = cam.requestFullResolutionOutput()
            depth_in[side] = cam.requestOutput((800, 500), type=dai.ImgFrame.Type.GRAY8, fps=30.0)
        st = p.create(dai.node.StereoDepth).build(depth_in["left"], depth_in["right"],
                                                  presetMode=dai.node.StereoDepth.PresetMode.ROBOTICS)
        st.setLeftRightCheck(True)
        st.setSubpixel(True)
        st.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        pp = st.initialConfig.postProcessing
        pp.decimationFilter.decimationFactor = 1
        for f in ("spatialFilter", "temporalFilter", "speckleFilter"):
            getattr(pp, f).enable = False
        dq = st.depth.createOutputQueue(maxSize=4, blocking=False)

        dyn = p.create(dai.node.DynamicCalibration)
        cams["left"].link(dyn.left)
        cams["right"].link(dyn.right)
        ctrl = dyn.inputControl.createInputQueue()
        q_cal = dyn.calibrationOutput.createOutputQueue()
        q_cov = dyn.coverageOutput.createOutputQueue()
        q_met = dyn.metricsOutput.createOutputQueue()
        p.start()

        k = current.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 800, 500)
        roi = (k, (300, 175, 500, 325)) if a.plane else None

        def grab(n=90):
            frames, t0 = [], time.time()
            while len(frames) < n and time.time() - t0 < 10:
                m = dq.tryGet()
                if m is not None:
                    frames.append(m.getFrame().copy())
                else:
                    time.sleep(0.005)
            return frames

        input("\nBEFORE: point the camera at a still, textured scene"
              + (" (flat wall in the centre)" if a.plane else "") + ", hold it still, press Enter...")
        time.sleep(1.0)
        dq.tryGetAll()
        report["before"] = depth_metrics(grab(), roi)
        print("before:", json.dumps(report["before"]))
        save()

        def metrics(cal, label):
            """DCL's confidence in a calibration, judged on the images collected so far."""
            ctrl.send(dai.DynamicCalibrationControl.computeCalibrationMetrics(cal))
            met = q_met.get()
            report[label] = {"calibrationConfidence": met.calibrationConfidence, "dataConfidence": met.dataConfidence}
            print(f"{label}:", report[label])

        input("\nDCL: press Enter, then SLOWLY move the drone over a textured scene "
              "(near and far objects, cover the whole field of view)...")
        ctrl.send(dai.DynamicCalibrationControl.setPerformanceMode(getattr(dai.DynamicCalibrationControl.PerformanceMode, a.mode)))
        ctrl.send(dai.DynamicCalibrationControl.resetData())
        ctrl.send(dai.DynamicCalibrationControl.startCalibration())
        result, t0, last = None, time.time(), 0.0
        while time.time() - t0 < a.seconds:
            cov = q_cov.tryGet()
            if cov is not None and time.time() - last > 1.0:
                last = time.time()
                print(f"  coverage {cov.meanCoverage * 100 if cov.meanCoverage <= 1 else cov.meanCoverage:.0f}%  "
                      f"data acquired {getattr(cov, 'dataAcquired', '?')}", flush=True)
            res = q_cal.tryGet()
            if res is not None:
                print("  DCL:", res.info)
                if res.calibrationData is not None:
                    result = res
                    break
            time.sleep(0.02)
        ctrl.send(dai.DynamicCalibrationControl.stopCalibration())
        metrics(current, "current_metrics")
        if result is None:
            report["dcl"] = "no calibration produced (more coverage / texture / time needed)"
            print(report["dcl"])
            save()
            return
        data = result.calibrationData
        report["dcl"] = {"info": result.info, "result": describe(data)}
        new = data.newCalibration
        new.eepromToJsonFile(os.path.join(a.out, "proposed.json"))
        print("proposed calibration saved:", os.path.join(a.out, "proposed.json"))
        print("difference / confidence:", json.dumps(report["dcl"]["result"], default=str)[:1500])
        metrics(new, "proposed_metrics")
        save()

        if not (a.apply or a.flash):
            print("\nNot applied (use --apply to compare depth before/after, --flash to keep it).")
            return
        ctrl.send(dai.DynamicCalibrationControl.applyCalibration(new))  # session only
        input("\nAFTER: put the camera back at the same still view as BEFORE, press Enter...")
        time.sleep(1.0)
        dq.tryGetAll()
        report["after"] = depth_metrics(grab(), roi)
        print("before:", json.dumps(report["before"]))
        print("after: ", json.dumps(report["after"]))
        save()
        if a.flash:
            if input("Write the new calibration to the camera EEPROM? type YES: ") == "YES":
                dev.flashCalibration(new)
                report["flashed"] = True
                print("flashed. Backups:", a.out, "| restore with --restore", os.path.join(a.out, "backup_current.json"))
            else:
                report["flashed"] = False
            save()


if __name__ == "__main__":
    sys.exit(main())
