#!/usr/bin/env python3
"""Read the OAK calibration from the running ROS graph (read-only) into CALIB.json.

    dump_oak_calib.py OUT.json [--seconds 10]

Takes one /oak/left/camera_info and /oak/right/camera_info (published by
larp_oak from the device EEPROM) and the static TF oak_parent_frame ->
oak_{left,right}_camera_optical_frame and oak_imu_frame. Does not open the
camera, so it can run next to the live pipeline.
"""

import argparse
import json
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


def tf_to_matrix(t):
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (p.x, p.y, p.z)
    return T.tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("larp_dump_oak_calib")
    infos = {}
    for side in ("left", "right"):
        node.create_subscription(
            CameraInfo, f"/oak/{side}/camera_info",
            lambda m, s=side: infos.setdefault(s, m), qos_profile_sensor_data,
        )  # fmt: skip
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end and len(infos) < 2:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}
    for side, m in infos.items():
        out[side] = {
            "K": np.array(m.k).reshape(3, 3).tolist(),
            "D": list(m.d)[:8],
            "model": m.distortion_model,
            "size": [m.width, m.height],
            "frame": m.header.frame_id,
        }
    for key, child in (
        ("T_parent_left", "oak_left_camera_optical_frame"),
        ("T_parent_right", "oak_right_camera_optical_frame"),
        ("T_parent_imu", "oak_imu_frame"),
    ):
        out[key] = tf_to_matrix(buf.lookup_transform("oak_parent_frame", child, Time()).transform)
    json.dump(out, open(a.out, "w"), indent=2)
    print(f"wrote {a.out}: {sorted(out)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
