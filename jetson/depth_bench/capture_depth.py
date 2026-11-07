"""Capture raw 800x500 depth frames in the larp_oak layout (stack STOPPED).

    python3 capture_depth.py OUT.npz [SECONDS]

Camera: 1280x800 GRAY8 -> host (consumed, as in flight) + 800x500 -> StereoDepth
(ROBOTICS preset, LR check, hardware 7x7 median, decimation 1, speckle /
spatial / temporal OFF). Saves depth frames (uint16 mm), device timestamps,
and the depth camera's fx for depth <-> disparity conversion.
"""

import sys
import time

import depthai as dai
import numpy as np

OUT = sys.argv[1]
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
P = dai.node.StereoDepth.PresetMode

with dai.Pipeline() as p:
    outs = {}
    for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
        cam = p.create(dai.node.Camera).build(sock, sensorFps=30)
        outs[side] = (
            cam.requestOutput((1280, 800), type=dai.ImgFrame.Type.GRAY8, fps=30),
            cam.requestOutput((800, 500), type=dai.ImgFrame.Type.GRAY8, fps=30),
        )
    st = p.create(dai.node.StereoDepth).build(outs["left"][1], outs["right"][1], presetMode=P.ROBOTICS)
    st.setLeftRightCheck(True)
    st.left.setBlocking(False)
    st.right.setBlocking(False)
    pp = st.initialConfig.postProcessing
    pp.decimationFilter.decimationFactor = 1
    for f in ("spatialFilter", "temporalFilter", "speckleFilter"):
        getattr(pp, f).enable = False
    qd = st.depth.createOutputQueue(maxSize=8, blocking=False)
    ql = outs["left"][0].createOutputQueue(maxSize=2, blocking=False)
    qr = outs["right"][0].createOutputQueue(maxSize=2, blocking=False)
    p.start()
    dev = p.getDefaultDevice()
    calib = dev.readCalibration()
    # depth is aligned to the rectified right camera by default; intrinsics at 800x500
    fx = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 800, 500)[0][0]
    baseline_m = calib.getBaselineDistance(useSpecTranslation=False) / 100.0
    time.sleep(3)
    qd.tryGetAll()
    frames, stamps = [], []
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        for m in qd.tryGetAll():
            frames.append(m.getFrame().copy())
            stamps.append(m.getTimestampDevice().total_seconds())
        ql.tryGetAll()
        qr.tryGetAll()
        time.sleep(0.002)

np.savez_compressed(OUT, depth=np.stack(frames), stamps=np.array(stamps), fx=fx, baseline_m=baseline_m)
span = stamps[-1] - stamps[0]
print(f"saved {len(frames)} frames {frames[0].shape[1]}x{frames[0].shape[0]} "
      f"@ {(len(frames) - 1) / span:.1f} Hz, fx={fx:.1f} px, baseline={baseline_m * 100:.2f} cm")  # fmt: skip
