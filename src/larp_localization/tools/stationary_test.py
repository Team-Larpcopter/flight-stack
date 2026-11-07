#!/usr/bin/env python3
"""Stationary VIO test: log sqrtVINS and cuVSLAM odometry side by side and summarise.

    stationary_test.py SECONDS OUT_DIR [--topics /ov_srvins/odomimu /visual_slam/tracking/odometry]

Keep the drone still for the whole run. Writes OUT_DIR/<name>.csv per topic
(t_s, x, y, z, yaw_deg, vx, vy, vz, stamp lag) and OUT_DIR/summary.json, and
prints the summary. Read-only: only subscribes.

Summary per topic:
  rate_hz, max_gap_ms        output rate and the longest pause
  drift_m@10s/30s/60s/end    distance from the first pose (a still drone should stay ~0)
  drift_rate_mm_per_min      linear fit of drift over the run
  jitter_mm                  per-axis std of position about a 1 s moving average
  yaw_drift_deg              final minus first yaw
  lag_ms_median              receive time minus header stamp (pipeline latency, approx.)
"""

import argparse
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def summarise(rows):
    if len(rows) < 3:
        return {"samples": len(rows), "note": "too few messages"}
    a = np.array(rows, dtype=float)  # t, x, y, z, yaw, vx, vy, vz, lag
    t, p, yaw, lag = a[:, 0] - a[0, 0], a[:, 1:4], np.unwrap(np.radians(a[:, 4])), a[:, 8]
    d = np.linalg.norm(p - p[0], axis=1)
    gaps = np.diff(t)
    out = {
        "samples": len(rows),
        "duration_s": round(float(t[-1]), 1),
        "rate_hz": round(float((len(t) - 1) / t[-1]), 1) if t[-1] > 0 else None,
        "max_gap_ms": round(float(gaps.max() * 1000), 1),
        "drift_m_end": round(float(d[-1]), 4),
        "drift_m_max": round(float(d.max()), 4),
        "drift_rate_mm_per_min": round(float(np.polyfit(t, d, 1)[0] * 60000), 1),
        "yaw_drift_deg": round(float(np.degrees(yaw[-1] - yaw[0])), 3),
        "lag_ms_median": round(float(np.median(lag)), 1),
    }
    for s in (10, 30, 60):
        i = np.searchsorted(t, s)
        if i < len(t):
            out[f"drift_m@{s}s"] = round(float(d[i]), 4)
    win = max(3, int(len(t) / max(t[-1], 1e-6)))  # ~1 s window
    if len(p) > win:
        kernel = np.ones(win) / win
        smooth = np.stack([np.convolve(p[:, k], kernel, mode="same") for k in range(3)], 1)
        resid = (p - smooth)[win:-win]
        out["jitter_mm_xyz"] = [round(float(v) * 1000, 2) for v in resid.std(axis=0)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", type=float)
    ap.add_argument("out")
    ap.add_argument("--topics", nargs="+", default=["/ov_srvins/odomimu", "/visual_slam/tracking/odometry"])
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("larp_stationary_test")
    data = {t: [] for t in a.topics}

    def cb(topic):
        def f(m):
            now = node.get_clock().now().nanoseconds * 1e-9
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, v = m.pose.pose.position, m.twist.twist.linear
            data[topic].append([st, p.x, p.y, p.z, yaw_deg(m.pose.pose.orientation), v.x, v.y, v.z, (now - st) * 1000])

        return f

    subs = [node.create_subscription(Odometry, t, cb(t), qos_profile_sensor_data) for t in a.topics]
    print(f"recording {a.seconds:.0f} s from {a.topics} -- keep the drone still", flush=True)
    end, next_note = time.monotonic() + a.seconds, time.monotonic() + 10
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() > next_note:
            print("  " + ", ".join(f"{t}: {len(r)} msgs" for t, r in data.items()), flush=True)
            next_note += 10
    summary = {}
    for topic, rows in data.items():
        name = topic.strip("/").replace("/", "_")
        with open(os.path.join(a.out, f"{name}.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp_s", "x", "y", "z", "yaw_deg", "vx", "vy", "vz", "lag_ms"])
            w.writerows(rows)
        summary[topic] = summarise(rows)
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=2)
    print(json.dumps(summary, indent=2))
    del subs
    rclpy.shutdown()


if __name__ == "__main__":
    main()
