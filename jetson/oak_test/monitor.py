"""Sustained-run monitor for the larp_oak pipeline: one JSON line every PERIOD s.

    python3 monitor.py DURATION_S [PERIOD_S] [LOG_DIR]

Per interval: topic rates (header stamps) and worst gaps for the VIO images,
raw and filtered depth, IMU and VIO odometry; latest vo_state; CPU % of one
core per process (from /proc, this container's processes); GPU load; RAM;
SoC temperature; and the number of oak_node device errors / respawns seen in
the launch log so far. Ends with a summary line.
"""

import json
import os
import re
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

DURATION = float(sys.argv[1])
PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
LOG_DIR = sys.argv[3] if len(sys.argv) > 3 else "/root/team_ws/oak_test"
TOPICS = {
    "left": ("/oak/left/camera_info", CameraInfo),
    "right": ("/oak/right/camera_info", CameraInfo),
    "depth_raw": ("/oak/stereo/camera_info", CameraInfo),
    "depth_filtered": ("/oak/stereo/filtered/camera_info", CameraInfo),
    "imu": ("/oak/imu/data", Imu),
    "vio": ("/visual_slam/tracking/odometry", Odometry),
}
PROCS = {
    "oak_node": "larp_oak/oak_node",
    "depth_filter": "larp_oak/depth_filter",
    "vslam": "component_container_mt",
    "foxglove": "foxglove_bridge",
    "viz_relay": "viz_relay",
}
GPU_LOAD = "/sys/devices/platform/bus@0/17000000.gpu/load"
HZ = os.sysconf("SC_CLK_TCK")


def proc_ticks():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
            for name, pat in PROCS.items():
                if pat in cmd:
                    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
                    out[name] = out.get(name, 0) + int(f[11]) + int(f[12])  # utime + stime
        except (OSError, IndexError, ValueError):
            pass
    return out


def read(path, default=None):
    try:
        with open(path, "rb") as f:
            return f.read().decode(errors="ignore").strip()
    except Exception:  # noqa: BLE001  (some sysfs nodes error on read)
        return default


def soc_temp():
    temps = []
    for z in range(10):
        t = read(f"/sys/devices/virtual/thermal/thermal_zone{z}/temp")
        if t and t.lstrip("-").isdigit():
            temps.append(int(t) / 1000)
    return max(temps) if temps else None


def oak_errors():
    n = 0
    log = os.path.join(LOG_DIR, "launch.log")
    if os.path.exists(log):
        txt = open(log, errors="ignore").read()
        n = len(re.findall(r"OAK device error|has crashed|process has died", txt))
    return n


def main():
    rclpy.init()
    node = rclpy.create_node("larp_oak_monitor")
    stamps = {k: [] for k in TOPICS}
    state = {"vo_state": None}

    def cb(key):
        def f(msg):
            stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        return f

    subs = [node.create_subscription(t, topic, cb(k), qos_profile_sensor_data) for k, (topic, t) in TOPICS.items()]
    if VisualSlamStatus is not None:
        subs.append(
            node.create_subscription(
                VisualSlamStatus, "/visual_slam/status",
                lambda m: state.__setitem__("vo_state", int(m.vo_state)), 10,
            )
        )  # fmt: skip
    # Allow 2 s for DDS discovery before beginning timed intervals, then clear initial buffer
    warm = time.monotonic() + 2.0
    while time.monotonic() < warm:
        rclpy.spin_once(node, timeout_sec=0.05)
    for k in stamps:
        stamps[k] = []
    start = time.monotonic()
    t_prev, ticks_prev = time.monotonic(), proc_ticks()
    history = []
    while time.monotonic() - start < DURATION:
        end = time.monotonic() + PERIOD
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        now, ticks = time.monotonic(), proc_ticks()
        row = {"t_s": round(now - start), "vo_state": state["vo_state"]}
        for k, ts in stamps.items():
            ts = sorted(set(ts))
            gaps = [b - a for a, b in zip(ts, ts[1:], strict=False)]
            row[f"{k}_hz"] = round(len(ts) / PERIOD, 1)
            row[f"{k}_max_gap_ms"] = round(max(gaps) * 1000) if gaps else None
            stamps[k] = []
        row["cpu_pct"] = {
            k: round((ticks.get(k, 0) - ticks_prev.get(k, 0)) / HZ / (now - t_prev) * 100)
            for k in PROCS
            if k in ticks
        }
        load = read(GPU_LOAD)
        row["gpu_pct"] = round(int(load) / 10) if load and load.isdigit() else None
        mem = dict(line.split(":", 1) for line in open("/proc/meminfo") if ":" in line)
        row["ram_used_mb"] = round(
            (int(mem["MemTotal"].split()[0]) - int(mem["MemAvailable"].split()[0])) / 1024
        )
        row["soc_temp_c"] = soc_temp()
        row["load1"] = float(open("/proc/loadavg").read().split()[0])
        row["oak_errors_total"] = oak_errors()
        print(json.dumps(row), flush=True)
        history.append(row)
        t_prev, ticks_prev = now, ticks
    keys = ["left_hz", "right_hz", "depth_raw_hz", "depth_filtered_hz", "imu_hz", "vio_hz"]
    summary = {"summary": True, "intervals": len(history)}
    for k in keys:
        vals = [r[k] for r in history]
        summary[k] = {"min": min(vals), "mean": round(sum(vals) / len(vals), 1)} if vals else None
    summary["vo_state_not_tracking_intervals"] = sum(1 for r in history if r["vo_state"] != 1)
    summary["oak_errors_total"] = history[-1]["oak_errors_total"] if history else None
    summary["max_soc_temp_c"] = max((r["soc_temp_c"] or 0) for r in history) if history else None
    print(json.dumps(summary), flush=True)
    del subs
    rclpy.shutdown()


if __name__ == "__main__":
    main()
