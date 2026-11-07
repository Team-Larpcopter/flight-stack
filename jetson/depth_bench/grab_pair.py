"""Save one rectified stereo pair from the live stack as PNGs (run inside the stack).

    python3 grab_pair.py OUT_DIR [LEFT_TOPIC] [RIGHT_TOPIC]
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def to_gray(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    if msg.encoding in ("mono8", "8UC1"):
        return a[:, : msg.width].copy()
    a = a[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)


def main():
    out = sys.argv[1]
    lt = sys.argv[2] if len(sys.argv) > 2 else "/oak/left/image_rect"
    rt = sys.argv[3] if len(sys.argv) > 3 else "/oak/right/image_rect"
    rclpy.init()
    node = rclpy.create_node("larp_grab_pair")
    got = {}
    for name, topic in (("left", lt), ("right", rt)):
        node.create_subscription(
            Image, topic, lambda m, n=name: got.__setitem__(n, m), qos_profile_sensor_data
        )
    end = time.monotonic() + 10
    while time.monotonic() < end and len(got) < 2:
        rclpy.spin_once(node, timeout_sec=0.1)
    for name, msg in got.items():
        cv2.imwrite(f"{out}/{name}.png", to_gray(msg))
        print(name, msg.width, msg.height, msg.encoding)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
