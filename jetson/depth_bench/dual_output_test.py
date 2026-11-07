"""Feasibility: full-res mono to host for VIO + 800x500 on-device stereo depth.

Camera layout per case (stack STOPPED, camera free):
    CAM_B/CAM_C @ 30 fps
      output A: 1280x800 GRAY8 -> host                (VIO)
      output B:  800x500 GRAY8 -> StereoDepth -> host (depth)
    IMU accel+gyro 200 Hz -> host

    python3 dual_output_test.py [SECONDS]

Prints one JSON line per case: per-stream rate (device timestamps), size, worst
gap, device CPU and temperature. Cases vary only the depth post-processing,
to find full-size 800x500 depth without dropping below 20-30 Hz.
"""

import json
import sys
import time

import depthai as dai

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
P = dai.node.StereoDepth.PresetMode
FULL = (1280, 800)
STEREO = (800, 500)
FPS = 30

CASES = [
    # name, preset, decimation factor (None = preset's own), filters off?
    ("default_preset", P.DEFAULT, None, False),
    ("default_decim1", P.DEFAULT, 1, False),
    ("default_decim1_nofilters", P.DEFAULT, 1, True),
    ("fast_density_decim1", P.FAST_DENSITY, 1, False),
    ("robotics_decim1_nofilters", P.ROBOTICS, 1, True),
]


def configure(stereo, decim, nofilters, notes):
    cfg = stereo.initialConfig
    pp = cfg.postProcessing
    if decim is not None:
        pp.decimationFilter.decimationFactor = decim
        notes.append(f"decimation={decim}")
    if nofilters:
        for name in ("spatialFilter", "temporalFilter", "speckleFilter"):
            f = getattr(pp, name, None)
            if f is not None and hasattr(f, "enable"):
                f.enable = False
                notes.append(f"{name}=off")
        th = getattr(pp, "thresholdFilter", None)
        if th is not None:
            notes.append("threshold kept")


def run_case(name, preset, decim, nofilters):
    res = {"case": name}
    notes = []
    with dai.Pipeline() as p:
        cams = {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B),
                           ("right", dai.CameraBoardSocket.CAM_C)):  # fmt: skip
            cam = p.create(dai.node.Camera).build(sock, sensorFps=FPS)
            cams[side] = (
                cam.requestOutput(FULL, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
                cam.requestOutput(STEREO, type=dai.ImgFrame.Type.GRAY8, fps=FPS),
            )
        st = p.create(dai.node.StereoDepth).build(cams["left"][1], cams["right"][1], presetMode=preset)
        st.setLeftRightCheck(True)
        try:
            configure(st, decim, nofilters, notes)
        except Exception as e:  # noqa: BLE001
            notes.append(f"config error: {str(e)[:120]}")
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        outs = {
            "left_full": cams["left"][0],
            "right_full": cams["right"][0],
            "depth": st.depth,
            "imu": imu.out,
        }
        qs = {k: o.createOutputQueue(maxSize=8, blocking=False) for k, o in outs.items()}
        p.start()
        dev = p.getDefaultDevice()
        res["usb"] = str(dev.getUsbSpeed())
        time.sleep(3.0)
        for q in qs.values():
            q.tryGetAll()
        stamps = {k: [] for k in qs}
        sizes = {}
        nbytes = {k: 0 for k in qs}
        end = time.monotonic() + SECONDS
        while time.monotonic() < end:
            got = False
            for k, q in qs.items():
                for m in q.tryGetAll():
                    got = True
                    if k == "imu":
                        for pkt in m.packets:
                            stamps[k].append(pkt.gyroscope.getTimestampDevice().total_seconds())
                        continue
                    stamps[k].append(m.getTimestampDevice().total_seconds())
                    sizes[k] = f"{m.getWidth()}x{m.getHeight()}"
                    nbytes[k] += len(m.getData())
            if not got:
                time.sleep(0.001)
        for k, ts in stamps.items():
            ts = sorted(ts)
            gaps = sorted(b - a for a, b in zip(ts, ts[1:], strict=False))
            span = ts[-1] - ts[0] if len(ts) > 1 else 0
            res[k] = {
                "hz": round((len(ts) - 1) / span, 1) if span else 0,
                "size": sizes.get(k),
                "max_gap_ms": round(gaps[-1] * 1000, 1) if gaps else None,
                "MBps": round(nbytes[k] / SECONDS / 1e6, 1) if k != "imu" else None,
            }
        try:
            res["leon_css_cpu"] = round(dev.getLeonCssCpuUsage().average * 100, 1)
            res["leon_mss_cpu"] = round(dev.getLeonMssCpuUsage().average * 100, 1)
            res["chip_temp_c"] = round(dev.getChipTemperature().average, 1)
        except Exception as e:  # noqa: BLE001
            res["stats_error"] = str(e)[:120]
    res["notes"] = notes
    return res


def main():
    for case in CASES:
        try:
            print(json.dumps(run_case(*case)), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"case": case[0], "error": str(e)[:400]}), flush=True)
        time.sleep(3)
    print(json.dumps({"case": "DONE"}), flush=True)


if __name__ == "__main__":
    main()
