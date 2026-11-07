#!/usr/bin/env python3
"""Guided 6-face accelerometer calibration for the OAK IMU.

    python3 imu_six_face.py [--g 9.7996] [--out DIR]

Run (ROS sourced, same domain/DDS profile) while larp_hardware oak_native_node
runs: it reads /native_oak/acceleration_raw and /native_oak/gyro_raw, which are
the BNO086 RAW axes and uncorrected (the correction is only applied on
/autonomy/imu/data), so the fit is never of already-corrected data. The output
JSON (axes "raw", accel_bias, accel_scale, g) is the format oak_native_node's
imu_calibration_file expects.

For each of the 6 faces you rest the drone on that side and press Enter; it waits
until the drone is still, records 5 s, and prints the mean accel and |a|. Faces can be done in any order; each must put one IMU axis
roughly along gravity (up or down). At the end, per axis:
    reading = s * true + b   ->   s = (up - down) / (2 g),  b = (up + down) / 2
refined by a least-squares fit of |(a - b) / s| = g over the full vectors of all
faces (does not assume level faces, so hand-held tilt does not bias it), and
|a| per face before/after the correction. Also prints the gyro bias (mean over
all still periods; only meaningful if the faces were really still). Writes
DIR/imu_six_face_<time>.json.
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import qos_profile_sensor_data


class Rec(Node):
    def __init__(self, accel_topic, gyro_topic):
        super().__init__("imu_six_face")
        self.lock = threading.Lock()
        self.buf = {"a": [], "g": []}  # (monotonic t, x, y, z) per stream
        qos = qos_profile_sensor_data
        self.create_subscription(Vector3Stamped, accel_topic, lambda m: self.on(m, "a"), qos)
        self.create_subscription(Vector3Stamped, gyro_topic, lambda m: self.on(m, "g"), qos)

    def on(self, m, key):
        with self.lock:
            b = self.buf[key]
            b.append((time.monotonic(), m.vector.x, m.vector.y, m.vector.z))
            if len(b) > 8000:
                del b[:2000]

    def window(self, seconds, key):
        with self.lock:
            b = np.array(self.buf[key])
        return b[b[:, 0] > time.monotonic() - seconds][:, 1:] if len(b) else np.zeros((0, 3))


def fit(faces, g):
    """Per-axis start from the up/down pairs, then least squares of |(a - b) / s| = g
    over the full vectors of all faces. Returns scale s and bias b (raw axes)."""
    s, b = np.ones(3), np.zeros(3)
    for ax in range(3):
        up = [f["mean"][ax] for f in faces if f["axis"] == ax and f["sign"] == "+"]
        dn = [f["mean"][ax] for f in faces if f["axis"] == ax and f["sign"] == "-"]
        if up and dn:
            s[ax], b[ax] = (up[0] - dn[0]) / (2 * g), (up[0] + dn[0]) / 2
    A = np.array([f["mean"] for f in faces])
    p = np.r_[s, b]

    def resid(p):
        return np.linalg.norm((A - p[3:]) / p[:3], axis=1) - g

    for _ in range(50):
        r = resid(p)
        J = np.stack([(resid(p + d) - r) / 1e-6 for d in np.eye(6) * 1e-6], axis=1)
        p = p - np.linalg.lstsq(J, r, rcond=None)[0]
    return p[:3], p[3:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g", type=float, default=9.7996, help="local gravity (Stanford ~9.7996)")
    ap.add_argument("--out", default="/integration/evidence")
    ap.add_argument("--accel-topic", default="/native_oak/acceleration_raw")
    ap.add_argument("--gyro-topic", default="/native_oak/gyro_raw")
    a = ap.parse_args()
    rclpy.init()
    node = Rec(a.accel_topic, a.gyro_topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(2)
    if len(node.window(2, "a")) < 100 or len(node.window(2, "g")) < 100:
        sys.exit(f"no {a.accel_topic} / {a.gyro_topic}: is oak_native_node running?")
    print("IMU OK (raw axes, uncorrected)")
    faces, still_g = [], []
    names = ["1st", "2nd", "3rd", "4th", "5th", "6th"]
    while len(faces) < 6:
        input(f"\n[{names[len(faces)]} face] Rest the drone on a new side (one axis up/down), let go, press Enter...")
        print("  waiting for it to be still", end="", flush=True)
        t0, still = time.monotonic(), False
        while time.monotonic() - t0 < 30:
            wa, wg = node.window(1.0, "a"), node.window(1.0, "g")
            if len(wa) > 100 and len(wg) > 100 and np.linalg.norm(wg.mean(0)) < 0.03 and wa.std(0).max() < 0.15:
                still = True
                break
            print(".", end="", flush=True)
            time.sleep(0.5)
        if not still:
            print(" -- not still after 30 s, try again")
            continue
        print(" recording 5 s", end="", flush=True)
        time.sleep(5)
        w, wg = node.window(5.0, "a"), node.window(5.0, "g")
        mean = w.mean(0)
        axis = int(np.argmax(np.abs(mean)))
        tilt = np.degrees(np.arccos(min(1.0, abs(mean[axis]) / np.linalg.norm(mean))))
        sign = "+" if mean[axis] > 0 else "-"
        dup = [f for f in faces if f["axis"] == axis and f["sign"] == sign]
        print(f"\n  a = {np.round(mean, 3)}  |a| = {np.linalg.norm(mean):.3f}  -> {sign}{'xyz'[axis]} up "
              f"(tilt {tilt:.1f} deg)")
        if dup:
            print("  same side as an earlier face: ignored, use a different side"); continue
        if tilt > 15:
            print("  tilted more than 15 deg from an axis: ignored, rest it flatter"); continue
        faces.append({"axis": axis, "sign": sign, "mean": mean.tolist(), "n": int(len(w)), "tilt_deg": float(tilt)})
        still_g.append(wg)
        done = sorted(f"{f['sign']}{'xyz'[f['axis']]}" for f in faces)
        print(f"  ok ({len(faces)}/6 done: {', '.join(done)})")
    s, b = fit(faces, a.g)
    gyro_bias = np.concatenate(still_g).mean(0)
    print("\n=== result (raw IMU axes) ===")
    print(f"accel scale  s = {np.round(s, 4)}   (1.0 = perfect; true = (reading - b) / s)")
    print(f"accel bias   b = {np.round(b, 4)} m/s^2")
    print(f"gyro bias      = {np.round(gyro_bias, 5)} rad/s = {np.round(np.degrees(gyro_bias), 3)} deg/s")
    print("face   |a| before  |a| after   (target %.4f)" % a.g)
    for f in faces:
        m = np.array(f["mean"])
        print(f"{f['sign']}{'xyz'[f['axis']]}     {np.linalg.norm(m):8.3f}  {np.linalg.norm((m - b) / s):8.3f}")
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f"imu_six_face_{time.strftime('%Y%m%d_%H%M%S')}.json")
    json.dump({"axes": "raw", "g": a.g, "accel_scale": s.tolist(), "accel_bias": b.tolist(),
               "gyro_bias": gyro_bias.tolist(), "faces": faces,
               "source": "imu_six_face.py, /native_oak/acceleration_raw, full-vector least-squares fit"},
              open(path, "w"), indent=2)
    print("saved", path, "(use it as oak_native_node imu_calibration_file)")


if __name__ == "__main__":
    main()
