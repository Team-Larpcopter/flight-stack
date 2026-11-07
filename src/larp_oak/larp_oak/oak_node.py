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
        self.parent = dp("parent_frame", "oak_parent_frame").value
        self.prefix = dp("tf_prefix", "oak").value
        self.publish_depth = bool(dp("publish_depth", True).value)
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
        }
        self.tf = StaticTransformBroadcaster(self)
        self.stats = {k: 0 for k in ("left", "right", "depth", "imu")}
        self.failed = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self._report)

    # ------------------------------------------------------------------ helpers
    def _ros_stamp(self, ts):
        """depthai host timestamp (timedelta on CLOCK_MONOTONIC) -> builtin Time."""
        offset_ns = self.get_clock().now().nanoseconds - time.monotonic_ns()
        ns = int(ts.total_seconds() * 1e9) + offset_ns
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
        for side, sock in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            # getCameraExtrinsics(src, dst) maps points from src to dst: T_dst_src.
            e = np.array(calib.getCameraExtrinsics(sock, dai.CameraBoardSocket.CAM_A))  # T_rgb_cam, cm
            r = r_po @ e[:3, :3]
            t = r_po @ (e[:3, 3] / 100.0)
            tfs.append(self._transform(self.frames[side], tuple(float(x) for x in t), quat_from_matrix(r)))
        # IMU: depthai_ros_driver uses the model's nominal pose; keep it identical
        # (cuVSLAM runs without IMU fusion here).
        tfs.append(self._transform(self.frames["imu"], (-0.002, -0.037948, -0.00079), OPTICAL_Q))
        self.tf.sendTransform(tfs)
        for tf in tfs[1:3]:
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
            )
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
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], self.imu_hz
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        q = {
            "left": cams["left"][0].createOutputQueue(maxSize=2, blocking=False),
            "right": cams["right"][0].createOutputQueue(maxSize=2, blocking=False),
            "depth": st.depth.createOutputQueue(maxSize=2, blocking=False),
            "imu": imu.out.createOutputQueue(maxSize=50, blocking=False),
        }
        return q

    def _run(self):
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
                    self.failed = "pipeline stopped running"
        except Exception as e:  # noqa: BLE001
            self.failed = f"{type(e).__name__}: {e}"
        if self.failed:
            self.get_logger().error(f"OAK device error, exiting for respawn: {self.failed}")

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
