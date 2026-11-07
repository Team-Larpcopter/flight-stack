"""Measure topic rates, sizes and stamp gaps for a fixed window (header stamps).

    python3 rates.py SECONDS TOPIC[:TYPE] ...    TYPE: info (CameraInfo, default) | image

Prints one JSON object: {topic: {hz, n, p95_gap_ms, max_gap_ms, size}}.
"""

import json
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def main():
    seconds = float(sys.argv[1])
    specs = [a.split(":") + ["info"] for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("larp_depth_bench_rates")
    stamps = {s[0]: [] for s in specs}
    sizes = {}

    def cb(topic):
        def f(msg):
            stamps[topic].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            sizes[topic] = f"{msg.width}x{msg.height}"

        return f

    subs = [
        node.create_subscription(
            Image if kind == "image" else CameraInfo, topic, cb(topic), qos_profile_sensor_data
        )
        for topic, kind, *_ in specs
    ]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}
    for topic, ts in stamps.items():
        ts = sorted(set(ts))
        gaps = sorted((b - a) * 1000 for a, b in zip(ts, ts[1:], strict=False))
        span = ts[-1] - ts[0] if len(ts) > 1 else 0
        out[topic] = {
            "hz": round((len(ts) - 1) / span, 1) if span else 0,
            "n": len(ts),
            "p95_gap_ms": round(gaps[int(0.95 * (len(gaps) - 1))], 1) if gaps else None,
            "max_gap_ms": round(gaps[-1], 1) if gaps else None,
            "size": sizes.get(topic),
        }
    print(json.dumps(out))
    del subs
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
