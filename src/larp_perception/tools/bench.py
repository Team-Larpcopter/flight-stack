#!/usr/bin/env python3
"""Benchmark the native stereo / depth / IMU / VIO / Foxglove streams.

    python3 bench.py [SECONDS] [--out FILE.json] [--topics T1 T2 ...] [--no-cpu]

Run ROS-sourced on the same domain/DDS profile as the camera (ideally in the same
container, so the CPU sampling sees the processes). For each topic: messages,
rate, p95/max gap between messages, and the header-stamp age at arrival (median
/ p95; how old the data is when it reaches a subscriber). For 16UC1 depth
topics also: fill ratio (pixels with depth), median depth, and temporal noise
(per-pixel std over the run, median over pixels valid in >= 90% of frames;
meaningful only with a still camera and scene). Per-process CPU (% of one core)
for the camera, depth filter, relay, bridge and sqrtVINS processes.

Python receives every full-size image, so on a loaded Jetson its rates can read
lower than what C++ subscribers get; the depth filter logs its own rate.
"""

import argparse
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPICS = [
    "/native_oak/left/image_undistorted",
    "/native_oak/right/image_undistorted",
    "/native_oak/depth/image_raw",
    "/stereo/depth/image",
    "/neural_depth/depth/image_raw",
    "/autonomy/imu/data",
    "/native_oak/acceleration_raw",
    "/native_oak/gyro_raw",
    "/autonomy/poseimu",
    "/viz/native_oak/left/image_undistorted/compressed",
    "/viz/stereo/depth/image/compressed",
    "/viz/neural_depth/depth/image/compressed",
]
PROCESSES = ["oak_native_node", "depth_filter", "neural_stereo_node", "viz_relay", "foxglove_bridge", "run_subscribe_msckf"]
MAX_DEPTH_FRAMES = 150  # frames kept per depth topic for the noise estimate


class Bench(Node):
    def __init__(self, topics):
        super().__init__("perception_bench")
        self.lock = threading.Lock()
        self.data = {t: {"recv": [], "age": []} for t in topics}
        self.depth = {}
        self.missing = []
        deadline = time.time() + 5.0  # wait for discovery before looking up types
        while True:
            types = dict(self.get_topic_names_and_types())
            if all(t in types for t in topics) or time.time() > deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        for t in topics:
            if t not in types:
                self.missing.append(t)
                continue
            cls = get_message(types[t][0])
            self.create_subscription(cls, t, lambda m, t=t: self.on(t, m), qos_profile_sensor_data)

    def on(self, topic, msg):
        now = time.time()
        rec = self.data[topic]
        with self.lock:
            rec["recv"].append(now)
            h = getattr(msg, "header", None)
            if h is not None and (h.stamp.sec or h.stamp.nanosec):
                rec["age"].append((now - (h.stamp.sec + h.stamp.nanosec * 1e-9)) * 1000.0)
            if getattr(msg, "encoding", "") == "16UC1":
                frames = self.depth.setdefault(topic, [])
                d = np.frombuffer(msg.data, dtype="<u2").reshape(msg.height, msg.step // 2)[:, : msg.width]
                if len(frames) < MAX_DEPTH_FRAMES:
                    frames.append(d.copy())


def cpu_sample(seconds):
    """% of one core per process name (sum over matching pids) over `seconds`."""

    def snapshot():
        out = {}
        for pid in filter(str.isdigit, os.listdir("/proc")):
            try:
                cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="ignore")
                st = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
            except OSError:
                continue
            for name in PROCESSES:
                if name in cmd and "bench.py" not in cmd:
                    out[(name, pid)] = int(st[11]) + int(st[12])  # utime + stime (ticks)
        return out

    hz = os.sysconf("SC_CLK_TCK")
    a = snapshot()
    time.sleep(seconds)
    b = snapshot()
    usage = {}
    for key, ticks in b.items():
        if key in a:
            usage[key[0]] = usage.get(key[0], 0.0) + (ticks - a[key]) / hz / seconds * 100.0
    return {k: round(v, 1) for k, v in usage.items()}


def stats(recv, age):
    if len(recv) < 2:
        return {"msgs": len(recv)}
    t = np.array(recv)
    gaps = np.diff(t) * 1000.0
    out = {
        "msgs": len(t),
        "hz": round((len(t) - 1) / (t[-1] - t[0]), 2),
        "gap_p95_ms": round(float(np.percentile(gaps, 95)), 1),
        "gap_max_ms": round(float(gaps.max()), 1),
    }
    if age:
        out["age_median_ms"] = round(float(np.median(age)), 1)
        out["age_p95_ms"] = round(float(np.percentile(age, 95)), 1)
    return out


def depth_stats(frames):
    a = np.stack(frames).astype(np.float32)  # mm, 0 = no depth
    valid = a > 0
    fill = valid.mean(axis=(1, 2))
    out = {
        "size": f"{a.shape[2]}x{a.shape[1]}",
        "fill_ratio": round(float(fill.mean()), 3),
        "median_depth_m": round(float(np.median(a[valid])) / 1000.0, 3) if valid.any() else None,
    }
    steady = valid.mean(axis=0) >= 0.9
    if len(frames) >= 10 and steady.any():
        masked = np.where(valid, a, np.nan)
        std = np.nanstd(masked, axis=0)[steady]
        rel = (std / np.nanmean(masked, axis=0)[steady]) * 100.0
        out["temporal_noise_mm_median"] = round(float(np.median(std)), 1)
        out["temporal_noise_pct_median"] = round(float(np.median(rel)), 2)
        out["steady_pixel_ratio"] = round(float(steady.mean()), 3)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float, default=20.0)
    ap.add_argument("--out")
    ap.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS)
    ap.add_argument("--no-cpu", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    node = Bench(a.topics)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)  # discovery
    with node.lock:
        for rec in node.data.values():
            rec["recv"].clear()
            rec["age"].clear()
    cpu = {} if a.no_cpu else cpu_sample(a.seconds)
    if a.no_cpu:
        time.sleep(a.seconds)
    with node.lock:
        report = {
            "seconds": a.seconds,
            "topics": {t: stats(r["recv"], r["age"]) for t, r in node.data.items() if t not in node.missing},
            "missing_topics": node.missing,
            "depth": {t: depth_stats(f) for t, f in node.depth.items() if f},
            "cpu_percent_of_one_core": cpu,
            "load_average": os.getloadavg(),
        }
    print(json.dumps(report, indent=2))
    if a.out:
        json.dump(report, open(a.out, "w"), indent=2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
