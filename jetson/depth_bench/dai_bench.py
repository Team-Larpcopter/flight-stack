"""Pure-depthai throughput benchmark (no ROS): is the limit the OAK or the ROS driver?

Run with the camera free (skybeta stack stopped), inside the team image:
    python3 dai_bench.py [SECONDS]

For each case: build a depthai v3 pipeline, open host queues (non-blocking or
blocking, small or large), drain them for SECONDS, and report per-stream rates
from DEVICE timestamps, frame sizes, the device's own CPU load (LEON CSS/MSS),
chip temperature and the negotiated USB speed. Also tries the neural depth
nodes to see whether this device (RVC2) supports them.
"""

import json
import sys
import time

import depthai as dai

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
P = dai.node.StereoDepth.PresetMode

CASES = [
    # name, size, fps, mode, preset, queue blocking, queue size
    ("cams_only_1280_60", (1280, 800), 60, "cams", None, False, 4),
    ("cams_only_1280_120", (1280, 800), 120, "cams", None, False, 4),
    ("stereo_1280_60", (1280, 800), 60, "stereo", P.DEFAULT, False, 4),
    ("stereo_1024_60", (1024, 640), 60, "stereo", P.DEFAULT, False, 4),
    ("stereo_1024_30", (1024, 640), 30, "stereo", P.DEFAULT, False, 4),
    ("stereo_1024_30_blockingq", (1024, 640), 30, "stereo", P.DEFAULT, True, 30),
    ("stereo_800_60", (800, 500), 60, "stereo", P.DEFAULT, False, 4),
    ("stereo_640_60", (640, 400), 60, "stereo", P.DEFAULT, False, 4),
    ("depth_only_1024_60", (1024, 640), 60, "depth_only", P.DEFAULT, False, 4),
    ("stereo_1024_60_fast_density", (1024, 640), 60, "stereo", P.FAST_DENSITY, False, 4),
    ("stereo_1024_60_fast_accuracy", (1024, 640), 60, "stereo", P.FAST_ACCURACY, False, 4),
    ("stereo_1280_60_fast_density", (1280, 800), 60, "stereo", P.FAST_DENSITY, False, 4),
]


def run_case(name, size, fps, mode, preset, blocking, qsize):
    res = {"case": name, "size": f"{size[0]}x{size[1]}", "req_fps": fps}
    with dai.Pipeline() as p:
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        lo = left.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=fps)
        ro = right.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=fps)
        outs = {}
        if mode == "cams":
            outs = {"left": lo, "right": ro}
        else:
            st = p.create(dai.node.StereoDepth).build(lo, ro, presetMode=preset)
            st.setLeftRightCheck(True)
            outs = {"depth": st.depth}
            if mode == "stereo":
                outs.update(rect_left=st.rectifiedLeft, rect_right=st.rectifiedRight)
        queues = {k: o.createOutputQueue(maxSize=qsize, blocking=blocking) for k, o in outs.items()}
        p.start()
        dev = p.getDefaultDevice()
        res["usb_speed"] = str(dev.getUsbSpeed())
        res["platform"] = str(dev.getPlatformAsString())
        time.sleep(2.0)  # warm-up
        for q in queues.values():
            q.tryGetAll()
        stamps = {k: [] for k in queues}
        shapes = {}
        end = time.monotonic() + SECONDS
        while time.monotonic() < end:
            got = False
            for k, q in queues.items():
                for m in q.tryGetAll():
                    got = True
                    stamps[k].append(m.getTimestampDevice().total_seconds())
                    shapes[k] = f"{m.getWidth()}x{m.getHeight()}"
            if not got:
                time.sleep(0.001)
        for k, ts in stamps.items():
            gaps = sorted(b - a for a, b in zip(ts, ts[1:], strict=False))
            span = ts[-1] - ts[0] if len(ts) > 1 else 0
            res[k] = {
                "hz": round((len(ts) - 1) / span, 1) if span else 0,
                "size": shapes.get(k),
                "max_gap_ms": round(gaps[-1] * 1000, 1) if gaps else None,
            }
        try:
            res["leon_css_cpu"] = round(dev.getLeonCssCpuUsage().average * 100, 1)
            res["leon_mss_cpu"] = round(dev.getLeonMssCpuUsage().average * 100, 1)
            res["chip_temp_c"] = round(dev.getChipTemperature().average, 1)
        except Exception as e:  # noqa: BLE001
            res["device_stats_error"] = str(e)
    return res


def try_node(node_name):
    """Can this device build and start a neural depth node at all?"""
    try:
        with dai.Pipeline() as p:
            getattr(dai.node, node_name)  # exists in the library?
            if node_name == "NeuralDepth":
                p.create(dai.node.NeuralDepth).build(
                    p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B).requestFullResolutionOutput(),
                    p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C).requestFullResolutionOutput(),
                )
            p.start()
            time.sleep(2)
        return "started"
    except Exception as e:  # noqa: BLE001
        return f"failed: {str(e)[:200]}"


def main():
    for case in CASES:
        try:
            print(json.dumps(run_case(*case)), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"case": case[0], "error": str(e)[:300]}), flush=True)
        time.sleep(2)
    print(json.dumps({"case": "NeuralDepth_support", "result": try_node("NeuralDepth")}), flush=True)


if __name__ == "__main__":
    main()
