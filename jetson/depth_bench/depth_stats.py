"""Depth granularity on live frames: what distances can the camera actually report?

    python3 depth_stats.py SECONDS [DEPTH_TOPIC] [INFO_TOPIC]

Collects depth frames, then prints one JSON object with: frame size and rate,
fill ratio (valid pixels), percentiles of the measured distances, the distinct
depth values the camera produced, and the step between adjacent possible
values near 0.5/1/2/3/4/6 m (that step is the depth resolution at that range).
"""

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def main():
    seconds = float(sys.argv[1])
    depth_topic = sys.argv[2] if len(sys.argv) > 2 else "/oak/stereo/image_raw"
    info_topic = sys.argv[3] if len(sys.argv) > 3 else "/oak/stereo/camera_info"
    rclpy.init()
    node = rclpy.create_node("larp_depth_stats")
    frames, stamps, info = [], [], {}

    def on_depth(msg):
        if msg.encoding not in ("16UC1", "mono16"):
            info["encoding"] = msg.encoding
            return
        a = np.frombuffer(msg.data, dtype="<u2").reshape(msg.height, msg.step // 2)
        frames.append(a[:, : msg.width].copy())
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_info(msg):
        info.update(fx=msg.k[0], width=msg.width, height=msg.height, frame=msg.header.frame_id)

    subs = [
        node.create_subscription(Image, depth_topic, on_depth, qos_profile_sensor_data),
        node.create_subscription(CameraInfo, info_topic, on_info, qos_profile_sensor_data),
    ]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {"frames": len(frames), "info": info}
    if frames:
        stack = np.stack(frames)
        valid = stack[stack > 0].astype(np.float64) / 1000.0  # metres
        out["size"] = f"{stack.shape[2]}x{stack.shape[1]}"
        span = stamps[-1] - stamps[0]
        out["hz"] = round((len(stamps) - 1) / span, 1) if span else None
        out["fill_ratio"] = round(float((stack > 0).mean()), 3)
        if valid.size:
            out["range_m"] = {
                f"p{p}": round(float(np.percentile(valid, p)), 3) for p in (1, 5, 50, 95, 99)
            }
            uniq = np.unique(valid)
            out["distinct_values"] = int(uniq.size)
            steps = {}
            for target in (0.5, 1.0, 2.0, 3.0, 4.0, 6.0):
                i = np.searchsorted(uniq, target)
                if 0 < i < uniq.size:
                    steps[f"{target}m"] = round(float(uniq[i] - uniq[i - 1]), 4)
            out["observed_step_m"] = steps
    fx = info.get("fx")
    if fx:
        fb = fx * 0.075  # OAK-D S2 baseline 7.5 cm
        out["theory"] = {
            "fx_px": round(fx, 1),
            "step_1px_disparity_m": {
                f"{z}m": round(z * z / fb, 3) for z in (0.5, 1.0, 2.0, 3.0, 4.0, 6.0)
            },
            "step_subpixel_1_8px_m": {
                f"{z}m": round(z * z / fb / 8, 3) for z in (0.5, 1.0, 2.0, 3.0, 4.0, 6.0)
            },
            "min_depth_m_disp95": round(fb / 95, 3),
        }
    print(json.dumps(out))
    del subs
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
