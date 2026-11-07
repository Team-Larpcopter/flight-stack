"""larp_oak: OAK-D S2 driver for VIO at full resolution plus on-device depth.

Replaces the organisers' depthai_ros_driver for our stack. The stock driver can
only send the stereo sockets to the host as bgr8 and cannot give one camera two
outputs, so cuVSLAM and depth could not both have good resolution. Measured on
skybeta-15 (jetson/depth_bench): this layout holds 30 Hz on every stream.

Camera pipeline (on the OAK):
    CAM_B (left) / CAM_C (right) mono sensors @ fps
      output A: vio_width x vio_height GRAY8 -> host     /oak/{left,right}/image_raw
      output B: stereo_width x stereo_height GRAY8 -> StereoDepth (non-blocking
                inputs, so a slow depth stage can never stall output A)
                -> depth uint16 mm, aligned to the rectified left camera
                                                         /oak/stereo/image_raw
      output C (vio_undistorted, default on): vio size GRAY8, undistorted ON the
                device (warp engine, full rational model) -> host
                                                         /oak/{left,right}/image_undistorted
                pinhole camera_info, D = 0, K from the frame's own transformation.
                Left/right come from the same synced sensor capture, so the pair
                stays in step (the host-side Python undistorter dropped different
                frames on each side and ran at ~17 Hz). If the device rejects it,
                the node logs an error and restarts without output C.
    IMU accelerometer + gyroscope @ imu_hz               /oak/imu/data

StereoDepth: preset (default ROBOTICS) with hardware left-right check and 7x7
median, decimation 1 and the software post-processing filters (speckle,
spatial, temporal) OFF: at full size they slow every camera output. Filtering
runs on the Jetson instead (larp_depth_filter).

Also publishes CameraInfo for every image from the device calibration
(rational_polynomial, first 8 coefficients) and static TF from the
calibration, using the same frame names and layout as depthai_ros_driver
(oak_rgb_camera_optical_frame at oak_parent_frame, left/right/imu optical
frames relative to it), so cuVSLAM and the rest of the stack are unchanged.

Timestamps: depthai's host-synced timestamps are std::chrono::steady_clock,
i.e. CLOCK_MONOTONIC; they are shifted into ROS time with the current
monotonic -> ROS offset.

On a device error the node exits non-zero; the launch file respawns it.
"""

import array
import math
import threading
import time

import depthai as dai
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import UInt8MultiArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE
)
IMU_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)  # fmt: skip

# Optical frame orientation relative to a body-style frame (x fwd, y left, z up),
# as depthai_ros_driver publishes it: quaternion xyzw (0.5, -0.5, 0.5, -0.5).
OPTICAL_Q = (0.5, -0.5, 0.5, -0.5)


def quat_from_matrix(m):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def matrix_from_quat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


class OakNode(Node):
    def __init__(self):
        super().__init__("oak")
        dp = self.declare_parameter
        self.fps = float(dp("fps", 30.0).value)
        self.vio_size = (int(dp("vio_width", 1280).value), int(dp("vio_height", 800).value))
        self.stereo_size = (int(dp("stereo_width", 800).value), int(dp("stereo_height", 500).value))
        self.preset = dp("depth_preset", "ROBOTICS").value
        self.subpixel = bool(dp("subpixel", True).value)
        self.imu_hz = int(dp("imu_hz", 200).value)
        # "raw" (default): BNO086 RAW accel/gyro, in the sensor axes the device's
        # factory IMU->camera extrinsics are defined for (used for the IMU TF below).
        # "calibrated" outputs the same values with y and z negated (measured on
        # skybeta-15) and the same |g| (9.16 m/s^2 at rest, ~7% low: an accelerometer
        # scale error the filter must absorb), so it does NOT match those extrinsics.
        self.imu_mode = dp("imu_mode", "raw").value
        self.imu_check = []  # first ~5 s of IMU samples for the start-up gravity check
        self.r_left_imu = None  # rotation IMU -> left optical, for that check
        self.parent = dp("parent_frame", "oak_parent_frame").value
        self.prefix = dp("tf_prefix", "oak").value
        self.publish_depth = bool(dp("publish_depth", True).value)
        self.vio_undistorted = bool(dp("vio_undistorted", True).value)
        f = self.prefix
        self.frames = {
            "rgb": f"{f}_rgb_camera_optical_frame",
            "left": f"{f}_left_camera_optical_frame",
            "right": f"{f}_right_camera_optical_frame",
            "imu": f"{f}_imu_frame",
        }
        self.pub = {
            "left": self.create_publisher(Image, "/oak/left/image_raw", IMAGE_QOS),
            "right": self.create_publisher(Image, "/oak/right/image_raw", IMAGE_QOS),
            "depth": self.create_publisher(Image, "/oak/stereo/image_raw", IMAGE_QOS),
            "left_info": self.create_publisher(CameraInfo, "/oak/left/camera_info", IMAGE_QOS),
            "right_info": self.create_publisher(CameraInfo, "/oak/right/camera_info", IMAGE_QOS),
            "depth_info": self.create_publisher(CameraInfo, "/oak/stereo/camera_info", IMAGE_QOS),
            "imu": self.create_publisher(Imu, "/oak/imu/data", IMU_QOS),
            # BNO086 report accuracy [accel, gyro] at 10 Hz: 0 unreliable, 1 low, 2 medium, 3 high
            "imu_accuracy": self.create_publisher(UInt8MultiArray, "/oak/imu/accuracy", 10),
        }
        if self.vio_undistorted:
            for side in ("left", "right"):
                self.pub[f"{side}_u"] = self.create_publisher(Image, f"/oak/{side}/image_undistorted", IMAGE_QOS)
                self.pub[f"{side}_u_info"] = self.create_publisher(
                    CameraInfo, f"/oak/{side}/image_undistorted/camera_info", IMAGE_QOS
                )
        self.tf = StaticTransformBroadcaster(self)
        keys = ("left", "right", "left_u", "right_u", "depth", "imu") if self.vio_undistorted else (
            "left", "right", "depth", "imu")
        self.stats = {k: 0 for k in keys}
        self.failed = None
        self._offset_ns, self._offset_at = 0, -(10**18)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self._report)

    # ------------------------------------------------------------------ helpers
    def _ros_stamp(self, ts):
        """depthai host timestamp (timedelta on CLOCK_MONOTONIC) -> builtin Time."""
        # Offset refreshed once a second, not per message: per-message clock reads add
        # microseconds of jitter, so left/right frames of one capture got different stamps.
        now_mono = time.monotonic_ns()
        if now_mono - self._offset_at > 1_000_000_000:
            self._offset_ns = self.get_clock().now().nanoseconds - time.monotonic_ns()
            self._offset_at = now_mono
        ns = int(ts.total_seconds() * 1e9) + self._offset_ns
        return rclpy.time.Time(nanoseconds=ns).to_msg()

    def _camera_info(self, calib, socket, size, frame):
        k = calib.getCameraIntrinsics(socket, size[0], size[1])
        d = list(calib.getDistortionCoefficients(socket))[:8]
        d += [0.0] * (8 - len(d))
        info = CameraInfo()
        info.header.frame_id = frame
        info.width, info.height = size
        info.distortion_model = "rational_polynomial"
        info.d = [float(x) for x in d]
        info.k = [float(x) for row in k for x in row]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0][0], k[0][1], k[0][2], 0.0, k[1][0], k[1][1], k[1][2], 0.0, 0.0, 0.0, 1.0, 0.0]
        info.p = [float(x) for x in info.p]
        return info

    def _transform(self, child, t_xyz, q_xyzw):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t_xyz
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q_xyzw
        return tf

    def _publish_tf(self, calib):
        """Optical frames from calibration, laid out like depthai_ros_driver:
        rgb optical at the parent origin; T_parent_cam = T_parent_rgb * T_rgb_cam."""
        r_po = matrix_from_quat(OPTICAL_Q)  # parent <- rgb optical
        tfs = [self._transform(self.frames["rgb"], (0.0, 0.0, 0.0), OPTICAL_Q)]
        T_parent = {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            # getCameraExtrinsics(src, dst) maps points from src to dst: T_dst_src.
            e = np.array(calib.getCameraExtrinsics(sock, dai.CameraBoardSocket.CAM_A))  # T_rgb_cam, cm
            r = r_po @ e[:3, :3]
            t = r_po @ (e[:3, 3] / 100.0)
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = r, t
            T_parent[side] = T
            tfs.append(self._transform(self.frames[side], tuple(float(x) for x in t), quat_from_matrix(r)))
        # IMU: from the device's factory IMU->camera calibration, which is defined for
        # the RAW sensor axes (imu_mode raw). With it, the resting accelerometer reads
        # ~(0, -9.2, 0) in the left optical frame (y down), i.e. gravity is consistent
        # with the cameras. The previous nominal URDF pose put gravity on +y (flipped),
        # which broke IMU fusion (sqrtVINS drifted ~1 m while still).
        T_imu = None
        try:
            e = np.array(calib.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B))  # T_left_imu, cm
            if e.shape == (4, 4) and abs(np.linalg.det(e[:3, :3]) - 1.0) < 1e-3:
                T_left_imu = e.copy()
                T_left_imu[:3, 3] /= 100.0
                T_imu = T_parent["left"] @ T_left_imu
                self.r_left_imu = T_left_imu[:3, :3]
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warning(f"IMU extrinsics unavailable ({ex})")
        if T_imu is None:
            self.get_logger().warning("no IMU calibration on the device: using the nominal URDF IMU pose")
            tfs.append(self._transform(self.frames["imu"], (-0.002, -0.037948, -0.00079), OPTICAL_Q))
        else:
            tfs.append(self._transform(
                self.frames["imu"], tuple(float(x) for x in T_imu[:3, 3]), quat_from_matrix(T_imu[:3, :3])
            ))  # fmt: skip
            self.get_logger().info(
                "IMU pose from factory calibration (IMU -> left camera):\n"
                + np.array2string(T_left_imu, precision=4, suppress_small=True)
            )
        self.tf.sendTransform(tfs)
        for tf in tfs[1:]:
            t, q = tf.transform.translation, tf.transform.rotation
            self.get_logger().info(
                f"TF {self.parent} -> {tf.child_frame_id}: t=({t.x:.6f}, {t.y:.6f}, {t.z:.6f}) "
                f"q=({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})"
            )

    def _image(self, frame, stamp, frame_id, encoding, step):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        # array('B') takes rclpy's fast path; bytes are validated element by element (~150 ms/MB)
        msg.data = array.array("B", frame.tobytes())
        return msg

    # ------------------------------------------------------------------ device
    def _build(self, p):
        cams = {}
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            cam = p.create(dai.node.Camera).build(sock, sensorFps=self.fps)
            cams[side] = (
                cam.requestOutput(self.vio_size, type=dai.ImgFrame.Type.GRAY8, fps=self.fps),
                cam.requestOutput(self.stereo_size, type=dai.ImgFrame.Type.GRAY8, fps=self.fps),
            ) + ((
                cam.requestOutput(self.vio_size, type=dai.ImgFrame.Type.GRAY8, fps=self.fps,
                                  enableUndistortion=True),
            ) if self.vio_undistorted else ())  # fmt: skip
        preset = getattr(dai.node.StereoDepth.PresetMode, self.preset)
        st = p.create(dai.node.StereoDepth).build(cams["left"][1], cams["right"][1], presetMode=preset)
        st.setLeftRightCheck(True)
        st.setSubpixel(self.subpixel)
        st.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        st.left.setBlocking(False)
        st.right.setBlocking(False)
        pp = st.initialConfig.postProcessing
        if not self.subpixel:
            pp.median = dai.MedianFilter.KERNEL_7x7
        pp.decimationFilter.decimationFactor = 1
        for name in ("spatialFilter", "temporalFilter", "speckleFilter"):
            getattr(pp, name).enable = False
        imu = p.create(dai.node.IMU)
        modes = {
            "raw": [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            "calibrated": [dai.IMUSensor.ACCELEROMETER_CALIBRATED, dai.IMUSensor.GYROSCOPE_CALIBRATED],
            "uncalibrated": [dai.IMUSensor.ACCELEROMETER_UNCALIBRATED, dai.IMUSensor.GYROSCOPE_UNCALIBRATED],
        }
        if self.imu_mode not in modes:
            self.get_logger().error(f"unknown imu_mode {self.imu_mode!r}, using raw")
            self.imu_mode = "raw"
        sensors = modes[self.imu_mode]
        imu.enableIMUSensor(sensors, self.imu_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        q = {
            "left": cams["left"][0].createOutputQueue(maxSize=2, blocking=False),
            "right": cams["right"][0].createOutputQueue(maxSize=2, blocking=False),
            "depth": st.depth.createOutputQueue(maxSize=2, blocking=False),
            "imu": imu.out.createOutputQueue(maxSize=50, blocking=False),
        }
        if self.vio_undistorted:
            for side in ("left", "right"):
                q[f"{side}_u"] = cams[side][2].createOutputQueue(maxSize=2, blocking=False)
        return q

    def _undistorted_info(self, m, frame):
        """Pinhole CameraInfo from an undistorted frame's own transformation."""
        tr = m.getTransformation()
        k = [[float(x) for x in row] for row in tr.getIntrinsicMatrix()]
        d = [float(x) for x in tr.getDistortionCoefficients()]
        if any(abs(x) > 1e-9 for x in d):
            self.get_logger().warn(f"undistorted frame reports distortion {d[:8]}: publishing D=0 anyway")
        info = CameraInfo()
        info.header.frame_id = frame
        info.width, info.height = m.getWidth(), m.getHeight()
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [x for row in k for x in row]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0][0], k[0][1], k[0][2], 0.0, k[1][0], k[1][1], k[1][2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f"{frame} undistorted on device: {info.width}x{info.height} "
            f"fx={k[0][0]:.3f} fy={k[1][1]:.3f} cx={k[0][2]:.3f} cy={k[1][2]:.3f}"
        )
        return info

    def _run(self):
        # self.failed is what main() watches; set it only once we give up
        err = self._run_once()
        if err and self.vio_undistorted and not self.stop.is_set() and not self.got_undistorted:
            self.get_logger().error(f"pipeline with on-device undistortion failed ({err}); retrying without it")
            self.vio_undistorted = False
            self.stats = {k: 0 for k in ("left", "right", "depth", "imu")}
            err = self._run_once()
        if err:
            self.get_logger().error(f"OAK device error, exiting for respawn: {err}")
            self.failed = err

    def _run_once(self):
        """Run the pipeline until stopped; returns an error string or None."""
        self.got_undistorted = False
        err = None
        try:
            with dai.Pipeline() as p:
                q = self._build(p)
                p.start()
                dev = p.getDefaultDevice()
                calib = dev.readCalibration()
                self.get_logger().info(
                    f"OAK {dev.getDeviceId()} usb={dev.getUsbSpeed()} vio={self.vio_size} "
                    f"stereo={self.stereo_size} @ {self.fps} Hz, depth preset {self.preset}"
                )
                self._publish_tf(calib)
                infos = {
                    "left": self._camera_info(calib, dai.CameraBoardSocket.CAM_B, self.vio_size, self.frames["left"]),
                    "right": self._camera_info(calib, dai.CameraBoardSocket.CAM_C, self.vio_size, self.frames["right"]),
                }  # fmt: skip
                depth_info = None
                u_infos = {}
                while not self.stop.is_set() and p.isRunning():
                    busy = False
                    for side in ("left", "right"):
                        for m in q[side].tryGetAll():
                            busy = True
                            stamp = self._ros_stamp(m.getTimestamp())
                            img = m.getFrame()
                            self.pub[side].publish(
                                self._image(img, stamp, self.frames[side], "mono8", img.shape[1])
                            )
                            infos[side].header.stamp = stamp
                            self.pub[f"{side}_info"].publish(infos[side])
                            self.stats[side] += 1
                    for side in ("left", "right") if self.vio_undistorted else ():
                        for m in q[f"{side}_u"].tryGetAll():
                            busy = True
                            self.got_undistorted = True
                            stamp = self._ros_stamp(m.getTimestamp())
                            img = m.getFrame()
                            if side not in u_infos:
                                u_infos[side] = self._undistorted_info(m, self.frames[side])
                            self.pub[f"{side}_u"].publish(
                                self._image(img, stamp, self.frames[side], "mono8", img.shape[1])
                            )
                            u_infos[side].header.stamp = stamp
                            self.pub[f"{side}_u_info"].publish(u_infos[side])
                            self.stats[f"{side}_u"] += 1
                    for m in q["depth"].tryGetAll():
                        busy = True
                        if not self.publish_depth:
                            continue
                        stamp = self._ros_stamp(m.getTimestamp())
                        d = m.getFrame()
                        if depth_info is None:
                            depth_info = self._depth_info(m, calib, d.shape)
                        self.pub["depth"].publish(
                            self._image(d, stamp, self.frames["left"], "16UC1", d.shape[1] * 2)
                        )
                        depth_info.header.stamp = stamp
                        self.pub["depth_info"].publish(depth_info)
                        self.stats["depth"] += 1
                    for m in q["imu"].tryGetAll():
                        busy = True
                        for pkt in m.packets:
                            self._publish_imu(pkt)
                    if not busy:
                        time.sleep(0.001)
                if not self.stop.is_set():
                    err = "pipeline stopped running"
        except Exception as e:  # noqa: BLE001
            err = f"{type(e).__name__}: {e}"
        return err

    def _depth_info(self, msg, calib, shape):
        h, w = shape
        try:
            k = msg.getTransformation().getIntrinsicMatrix()
        except Exception:  # noqa: BLE001
            k = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, w, h)
        baseline_m = calib.getBaselineDistance(useSpecTranslation=False) / 100.0
        info = CameraInfo()
        info.header.frame_id = self.frames["left"]
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(x) for row in k for x in row]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fx = float(k[0][0])
        # P[3] = -fx * baseline: stereo convention, lets consumers recover the baseline.
        info.p = [fx, float(k[0][1]), float(k[0][2]), -fx * baseline_m,
                  float(k[1][0]), float(k[1][1]), float(k[1][2]), 0.0, 0.0, 0.0, 1.0, 0.0]  # fmt: skip
        self.get_logger().info(f"depth {w}x{h} fx={fx:.1f} baseline={baseline_m * 100:.2f} cm")
        return info

    def _publish_imu(self, pkt):
        a, g = pkt.acceleroMeter, pkt.gyroscope
        msg = Imu()
        msg.header.stamp = self._ros_stamp(g.getTimestamp())
        msg.header.frame_id = self.frames["imu"]
        msg.orientation_covariance[0] = -1.0  # no orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g.x, g.y, g.z
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a.x, a.y, a.z
        self.pub["imu"].publish(msg)
        self.stats["imu"] += 1
        if self.stats["imu"] % max(1, self.imu_hz // 10) == 0:
            self.pub["imu_accuracy"].publish(UInt8MultiArray(data=[int(a.accuracy), int(g.accuracy)]))
        if self.imu_check is not None:
            self.imu_check.append((a.x, a.y, a.z, g.x, g.y, g.z, int(a.accuracy), int(g.accuracy)))
            if len(self.imu_check) >= 5 * self.imu_hz:  # ~5 s
                self._gravity_check(np.array(self.imu_check))
                self.imu_check = None

    def _gravity_check(self, acc):
        """Log the resting accelerometer vector once: |a| should be ~9.81 m/s^2, and in
        the left optical frame (x right, y down, z forward) a level camera reads ~(0, -9.81, 0)."""
        mean, gyro = acc[:, :3].mean(axis=0), acc[:, 3:6].mean(axis=0)
        # BNO086 report accuracy: 0 unreliable, 1 low, 2 medium, 3 high
        acc_q, gyro_q = np.bincount(acc[:, 6].astype(int), minlength=4), np.bincount(acc[:, 7].astype(int), minlength=4)
        text = (f"IMU check ({self.imu_mode}, {len(acc)} samples): |a| = {np.linalg.norm(mean):.3f} m/s^2, "
                f"mean a = {np.round(mean, 3)}, mean gyro = {np.round(gyro, 4)} rad/s (IMU frame), "
                f"accuracy counts [unrel, low, med, high] accel {acc_q.tolist()} gyro {gyro_q.tolist()}")
        if self.r_left_imu is not None:
            text += f", a in left optical frame = {np.round(self.r_left_imu @ mean, 3)}"
        self.get_logger().info(text)

    def _report(self):
        s = ", ".join(f"{k} {v / 10.0:.1f} Hz" for k, v in self.stats.items())
        self.get_logger().info(f"rates: {s}")
        self.stats = {k: 0 for k in self.stats}

    def shutdown(self):
        self.stop.set()
        self.thread.join(timeout=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = OakNode()
    code = 0
    try:
        while rclpy.ok() and node.failed is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.failed:
            code = 1
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
