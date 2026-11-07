#!/usr/bin/env python3
"""Check the IMU->camera rotation (the axis flips) against real motion in a bag.

    check_imu_camera_rotation.py BAG_DIR [--chain kalibr_imucam_chain.yaml] [--step 3]

Independent of any calibration file: for pairs of left images `step` frames
apart, the camera rotation is measured from the images (KLT tracks + essential
matrix, pinhole K from the undistorted camera_info) and the IMU rotation by
integrating the gyro over the same interval. For a rigid camera+IMU,
    rotvec_cam = R_cam_imu @ rotvec_imu
so the best-fit R_cam_imu (Kabsch over all pairs with enough rotation) is the
measured IMU->camera rotation. It is compared with the R from T_cam_imu in the
config (cam0), and the camera-IMU time offset is scanned as well.

A wrong axis flip shows up as an angle of ~90-180 deg between measured and
configured; a correct config agrees to ~1 deg. Needs a bag with real rotation
about all three axes (a hand-carried square with turns is enough; a still bag
is not). Uses /oak/left/image_undistorted, its camera_info and /oak/imu/data.
"""

import argparse
import re

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu

LEFT, INFO, IMU = "/oak/left/image_undistorted", "/oak/left/image_undistorted/camera_info", "/oak/imu/data"


def read_bag(path, step):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[LEFT, INFO, IMU]))
    imgs, imu, K, n = [], [], None, 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == IMU:
            m = deserialize_message(data, Imu)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            imu.append((t, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        elif topic == INFO and K is None:
            K = np.array(deserialize_message(data, CameraInfo).k).reshape(3, 3)
        elif topic == LEFT:
            n += 1
            if n % step:
                continue
            m = deserialize_message(data, Image)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            imgs.append((t, np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width)))
    return imgs, np.array(imu), K


def visual_rotation(a, b, K):
    """Rotation of the camera from image a to image b, as a rotvec in camera-a coordinates."""
    p0 = cv2.goodFeaturesToTrack(a, 400, 0.01, 12)
    if p0 is None or len(p0) < 30:
        return None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p0, None, winSize=(21, 21), maxLevel=4)
    good = st.ravel() == 1
    if good.sum() < 30:
        return None
    E, mask = cv2.findEssentialMat(p0[good], p1[good], K, cv2.RANSAC, 0.999, 1.0)
    if E is None or E.shape != (3, 3):
        return None
    inl, R, _, _ = cv2.recoverPose(E, p0[good], p1[good], K, mask=mask)
    if inl < 30:
        return None
    # recoverPose: x_b = R x_a + t, i.e. R = R_b_a; the camera's own rotation is R_a_b = R.T
    return cv2.Rodrigues(R.T)[0].ravel()


def gyro_rotation(imu, t0, t1):
    """Integrated gyro rotation from t0 to t1 as a rotvec in IMU-at-t0 coordinates."""
    sel = (imu[:, 0] >= t0) & (imu[:, 0] <= t1)
    if sel.sum() < 3:
        return None
    w, t = imu[sel, 1:4], imu[sel, 0]
    R = np.eye(3)
    for k in range(len(t) - 1):
        R = R @ cv2.Rodrigues(w[k] * (t[k + 1] - t[k]))[0]
    return cv2.Rodrigues(R)[0].ravel()


def kabsch(a, b):
    """R minimising sum |b_i - R a_i|^2."""
    U, _, Vt = np.linalg.svd(b.T @ a)
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ S @ Vt


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def config_R_cam_imu(path):
    txt = open(path).read()
    block = txt[txt.index("cam0:"):txt.index("cam1:")]
    rows = re.findall(r"- \[([^\]]+)\]", block)[:3]
    return np.array([[float(v) for v in r.split(",")][:3] for r in rows])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--chain", help="kalibr_imucam_chain.yaml to compare against (cam0)")
    ap.add_argument("--step", type=int, default=3, help="image pairs this many frames apart")
    ap.add_argument("--min-deg", type=float, default=2.0, help="ignore pairs rotating less than this")
    a = ap.parse_args()
    imgs, imu, K = read_bag(a.bag, a.step)
    print(f"{len(imgs)} images (every {a.step}th), {len(imu)} IMU samples, K fx={K[0, 0]:.1f}")
    pairs = []
    for (ta, ia), (tb, ib) in zip(imgs, imgs[1:]):
        rv = visual_rotation(ia, ib, K)
        if rv is not None and np.degrees(np.linalg.norm(rv)) >= a.min_deg:
            pairs.append((ta, tb, rv))
    print(f"{len(pairs)} image pairs with >= {a.min_deg} deg of rotation")
    best = None
    for dt_ms in range(-60, 61, 5):  # camera-IMU time offset scan
        dt = dt_ms / 1000.0
        cam, gyr = [], []
        for ta, tb, rv in pairs:
            g = gyro_rotation(imu, ta + dt, tb + dt)
            if g is not None:
                cam.append(rv)
                gyr.append(g)
        cam, gyr = np.array(cam), np.array(gyr)
        R = kabsch(gyr, cam)
        res = np.degrees(np.linalg.norm(cam - gyr @ R.T, axis=1))
        if best is None or np.median(res) < best[1]:
            best = (dt_ms, float(np.median(res)), R, cam, gyr, res)
    dt_ms, med, R, cam, gyr, res = best
    np.set_printoptions(precision=4, suppress=True)
    print(f"best camera-IMU time offset: {dt_ms:+d} ms (IMU time = camera time + offset)")
    print(f"per-pair residual after fit: median {med:.2f} deg, p90 {np.percentile(res, 90):.2f} deg")
    span = np.degrees(np.abs(gyr).sum(0))
    print(f"rotation excited about IMU x/y/z (sum of |rotvec|, deg): {np.round(span, 0)}")
    print("measured R_cam_imu (from motion):\n", R)
    print("  rounded:", np.round(R).astype(int).tolist())
    if a.chain:
        Rc = config_R_cam_imu(a.chain)
        print("configured R_cam_imu (cam0, T_cam_imu):\n", Rc)
        print("  rounded:", np.round(Rc).astype(int).tolist())
        d = angle_deg(R.T @ Rc)
        print(f"ANGLE BETWEEN MEASURED AND CONFIGURED: {d:.2f} deg -> "
              + ("axes OK" if d < 5 else "AXES DISAGREE: check the IMU frame / flips"))


if __name__ == "__main__":
    main()
