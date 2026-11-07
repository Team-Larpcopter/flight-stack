#!/usr/bin/env python3
"""Guided 6-face accelerometer calibration for the OAK IMU (and a check of the
BNO086's own calibration).

    python3 imu_six_face.py [--mode calibrated|raw] [--g 9.7996] [--out DIR]

Run inside the stack (ROS sourced) while larp_oak publishes /oak/imu/data and
/oak/imu/accuracy. --mode must match larp_oak's imu_mode: in "calibrated" mode
the samples are converted back to the RAW axes (y, z negated, measured on
skybeta-15), so the result applies to raw mode.

For each of the 6 faces you rest the drone on that side and press Enter; it waits
until the drone is still, records 5 s, and prints the mean accel, |a| and the
BNO086 accuracy flags. Faces can be done in any order; each must put one IMU axis
roughly along gravity (up or down). At the end, per axis:
    reading = s * true + b   ->   s = (up - down) / (2 g),  b = (up + down) / 2
and |a| per face before/after the correction. Also prints the gyro bias (mean
over all still periods). Writes DIR/imu_six_face_<time>.json.
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
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray

ACC_NAMES = ["unreliable", "low", "medium", "high"]


class Rec(Node):
    def __init__(self, flip):
        super().__init__("imu_six_face")
        self.flip = np.array([1.0, -1.0, -1.0]) if flip else np.ones(3)
        self.lock = threading.Lock()
        self.buf = []  # (t, ax, ay, az, gx, gy, gz)
        self.acc = (0, 0)
        self.create_subscription(Imu, "/oak/imu/data", self.on_imu, 400)
        self.create_subscription(UInt8MultiArray, "/oak/imu/accuracy", self.on_acc, 10)

    def on_imu(self, m):
        a = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]) * self.flip
        g = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]) * self.flip
        with self.lock:
            self.buf.append((time.monotonic(), *a, *g))
            if len(self.buf) > 4000:
                del self.buf[:1000]

    def on_acc(self, m):
        self.acc = tuple(m.data)

    def window(self, seconds):
        with self.lock:
            b = np.array(self.buf)
        return b[b[:, 0] > time.monotonic() - seconds] if len(b) else b


def flags(acc):
    return f"chip accuracy: accel {ACC_NAMES[acc[0]]}, gyro {ACC_NAMES[acc[1]]}" if len(acc) == 2 else "chip accuracy: ?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="calibrated", choices=["calibrated", "raw", "uncalibrated"])
    ap.add_argument("--g", type=float, default=9.7996, help="local gravity (Stanford ~9.7996)")
    ap.add_argument("--out", default="/root/team_ws/imu_cal")
    a = ap.parse_args()
    rclpy.init()
    node = Rec(flip=a.mode != "raw")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(2)
    if len(node.window(2)) < 100:
        sys.exit("no /oak/imu/data: is the stack running?")
    print(f"IMU OK ({a.mode} mode, converted to raw axes). {flags(node.acc)}")
    faces, still_g = [], []
    names = ["1st", "2nd", "3rd", "4th", "5th", "6th"]
    while len(faces) < 6:
        input(f"\n[{names[len(faces)]} face] Rest the drone on a new side (one axis up/down), let go, press Enter...")
        print("  waiting for it to be still", end="", flush=True)
        t0, still = time.monotonic(), False
        while time.monotonic() - t0 < 30:
            w = node.window(1.0)
            if len(w) > 100 and np.linalg.norm(w[:, 4:7].mean(0)) < 0.03 and w[:, 1:4].std(0).max() < 0.15:
                still = True
                break
            print(".", end="", flush=True)
            time.sleep(0.5)
        if not still:
            print(" -- not still after 30 s, try again")
            continue
        print(" recording 5 s", end="", flush=True)
        time.sleep(5)
        w = node.window(5.0)
        mean = w[:, 1:4].mean(0)
        axis = int(np.argmax(np.abs(mean)))
        tilt = np.degrees(np.arccos(min(1.0, abs(mean[axis]) / np.linalg.norm(mean))))
        sign = "+" if mean[axis] > 0 else "-"
        dup = [f for f in faces if f["axis"] == axis and f["sign"] == sign]
        print(f"\n  a = {np.round(mean, 3)}  |a| = {np.linalg.norm(mean):.3f}  -> {sign}{'xyz'[axis]} up "
              f"(tilt {tilt:.1f} deg)  {flags(node.acc)}")
        if dup:
            print("  same side as an earlier face: ignored, use a different side"); continue
        if tilt > 15:
            print("  tilted more than 15 deg from an axis: ignored, rest it flatter"); continue
        faces.append({"axis": axis, "sign": sign, "mean": mean.tolist(), "n": int(len(w)), "tilt_deg": float(tilt),
                      "accuracy": list(node.acc)})
        still_g.append(w[:, 4:7])
        done = sorted(f"{f['sign']}{'xyz'[f['axis']]}" for f in faces)
        print(f"  ok ({len(faces)}/6 done: {', '.join(done)})")
    # fit
    s, b = np.ones(3), np.zeros(3)
    for ax in range(3):
        up = [f["mean"][ax] for f in faces if f["axis"] == ax and f["sign"] == "+"]
        dn = [f["mean"][ax] for f in faces if f["axis"] == ax and f["sign"] == "-"]
        if not up or not dn:
            print(f"axis {'xyz'[ax]}: missing its up or down face, cannot fit"); continue
        s[ax] = (up[0] - dn[0]) / (2 * a.g)
        b[ax] = (up[0] + dn[0]) / 2
    gyro_bias = np.concatenate(still_g).mean(0)
    print("\n=== result (raw IMU axes) ===")
    print(f"accel scale  s = {np.round(s, 4)}   (1.0 = perfect; true = (reading - b) / s)")
    print(f"accel bias   b = {np.round(b, 4)} m/s^2")
    print(f"gyro bias      = {np.round(gyro_bias, 5)} rad/s = {np.round(np.degrees(gyro_bias), 3)} deg/s")
    print("face   |a| before  |a| after   (target %.4f)" % a.g)
    for f in faces:
        m = np.array(f["mean"])
        print(f"{f['sign']}{'xyz'[f['axis']]}     {np.linalg.norm(m):8.3f}  {np.linalg.norm((m - b) / s):8.3f}")
    print(f"final {flags(node.acc)}")
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f"imu_six_face_{time.strftime('%Y%m%d_%H%M%S')}.json")
    json.dump({"mode_recorded": a.mode, "axes": "raw", "g": a.g, "accel_scale": s.tolist(), "accel_bias": b.tolist(),
               "gyro_bias": gyro_bias.tolist(), "faces": faces, "final_accuracy": list(node.acc)},
              open(path, "w"), indent=2)
    print("saved", path)


if __name__ == "__main__":
    main()
