"""larp_undistort: OAK stereo images -> undistorted pinhole images for sqrtVINS.

sqrtVINS (like OpenVINS) only models pinhole-radtan and equidistant lenses. The
OAK-D S2 factory calibration is a rational model that neither fits well
(measured: radtan mean 1.9 px / max 24 px, equidistant mean 0.7-1.3 px / max
8 px over the 1280x800 image; tools/oak_to_kalibr.py), so this node removes
the distortion exactly with OpenCV and sqrtVINS runs on zero-distortion images.

For each side: subscribes /oak/<side>/image_raw + camera_info, builds the
undistortion map once (cv2.initUndistortRectifyMap with the original K as the
new camera matrix: no rotation, same optical frame, same size), and publishes
/oak/<side>/image_undistorted + camera_info (K unchanged, D = 0, plumb_bob).
The frame_id and stamps are passed through, so TF and timing are unchanged.
"""

import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

IN_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
OUT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE)


class Side:
    def __init__(self, node, name):
        self.node, self.name = node, name
        self.maps = None
        self.info_out = None
        self.pub = node.create_publisher(Image, f"/oak/{name}/image_undistorted", OUT_QOS)
        self.pub_info = node.create_publisher(CameraInfo, f"/oak/{name}/image_undistorted/camera_info", OUT_QOS)
        node.create_subscription(CameraInfo, f"/oak/{name}/camera_info", self.on_info, IN_QOS)
        node.create_subscription(Image, f"/oak/{name}/image_raw", self.on_image, IN_QOS)
        self.times = []

    def on_info(self, msg):
        if self.maps is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        size = (msg.width, msg.height)
        self.maps = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_16SC2)
        out = CameraInfo()
        out.width, out.height = msg.width, msg.height
        out.distortion_model = "plumb_bob"
        out.d = [0.0] * 5
        out.k = list(msg.k)
        out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        out.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        out.p = [float(v) for v in out.p]
        self.info_out = out
        self.node.get_logger().info(
            f"{self.name}: undistortion map {size[0]}x{size[1]} from {msg.distortion_model} "
            f"({len(msg.d)} coeffs), fx={K[0, 0]:.1f}"
        )

    def on_image(self, msg):
        if self.maps is None or msg.encoding != "mono8":
            return
        t0 = time.perf_counter()
        src = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        dst = cv2.remap(src, self.maps[0], self.maps[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img = Image()
        img.header = msg.header
        img.height, img.width = dst.shape
        img.encoding = "mono8"
        img.step = dst.shape[1]
        img.data = array.array("B", dst.tobytes())  # rclpy fast path
        self.pub.publish(img)
        self.info_out.header = msg.header
        self.pub_info.publish(self.info_out)
        self.times.append((time.perf_counter() - t0) * 1000)


class Undistort(Node):
    def __init__(self):
        super().__init__("undistort")
        self.sides = [Side(self, "left"), Side(self, "right")]
        self.create_timer(10.0, self.report)

    def report(self):
        parts = []
        for s in self.sides:
            if s.times:
                t = np.array(s.times)
                parts.append(f"{s.name} {len(t) / 10.0:.1f} Hz {np.median(t):.1f} ms")
            s.times = []
        if parts:
            self.get_logger().info(", ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = Undistort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
