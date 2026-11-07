"""Low-bandwidth copies of camera and depth streams for Foxglove over Wi-Fi.

Full-rate OAK streams are far more than the drone's Wi-Fi uplink (a raw
1024x640 depth image is ~39 MB/s, one JPEG camera stream ~60 Mbit/s). This
node republishes them at ``rate_hz``, downscaled by ``scale``, as JPEG
``sensor_msgs/CompressedImage`` on ``/viz`` + <input topic> + ``/compressed``
(``/viz`` + <input topic> for inputs that are already compressed).

Three kinds of input:

* ``topics``        CompressedImage (JPEG), re-encoded smaller
* ``raw_topics``    Image, mono8/bgr8/rgb8 (e.g. the rectified stereo pair)
* ``depth_topics``  Image, 16UC1 (mm) or 32FC1 (m), colourised: near = red,
                    far = blue, no depth = black, scaled to
                    [``min_depth_m``, ``max_depth_m``]

Frames between publications are dropped before any conversion, so the CPU
cost is only paid at ``rate_hz``. Viewing only: nothing in flight reads /viz.

Non-image topics for Foxglove are throttled too, so the bridge never forwards
anything at full rate (400 Hz IMU, IMU-rate odometry) and Wi-Fi never queues:

* ``throttle_topics``  "topic@hz" entries, any message type: the newest message
                       is forwarded as raw serialized bytes (no decoding) to
                       /viz + topic, at most hz times per second.
* ``path_pose_topic``  a PoseWithCovarianceStamped / PoseStamped / Odometry
                       topic; a bounded path (``path_max_points``, one point
                       per 1/``path_hz`` s) is published on /viz + topic +
                       /path at ``path_hz``. Replaces estimator paths that
                       resend the whole, ever-growing trajectory every update.
"""

import array
from collections import deque

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped

#: Keep only the newest frame: a viewer never wants a queued, stale one.
LATEST_ONLY = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)

RAW_CHANNELS = {"mono8": 1, "8UC1": 1, "bgr8": 3, "rgb8": 3, "8UC3": 3}


def encode(image, scale, quality):
    """Resize by ``scale`` and JPEG-encode. None on failure."""
    if scale != 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return encoded.tobytes() if ok else None


def shrink_jpeg(data, scale, quality):
    """Decode a JPEG, resize by ``scale`` and re-encode. None if undecodable."""
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    return None if image is None else encode(image, scale, quality)


def image_to_array(msg):
    """sensor_msgs/Image -> numpy array (no cv_bridge), or None if unsupported."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in RAW_CHANNELS:
        ch = RAW_CHANNELS[msg.encoding]
        image = data.reshape(msg.height, msg.step)[:, : msg.width * ch]
        image = image.reshape(msg.height, msg.width, ch) if ch == 3 else image
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else image
    if msg.encoding in ("16UC1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)
        return rows[:, : msg.width].astype(np.float32) / 1000.0  # mm -> m
    if msg.encoding == "32FC1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 4)
        return rows[:, : msg.width].copy()
    return None


def shrink_depth(depth_m, scale):
    """Downscale depth like the OAK's decimation: mean of the VALID pixels only.

    Resizing the colourised image instead averages holes (black) into their
    neighbours and makes the view look far emptier than the depth is.
    """
    valid = (depth_m > 0) & np.isfinite(depth_m)
    d = np.where(valid, depth_m, 0).astype(np.float32)
    size = (max(1, int(depth_m.shape[1] * scale)), max(1, int(depth_m.shape[0] * scale)))
    total = cv2.resize(d, size, interpolation=cv2.INTER_AREA)
    count = cv2.resize(valid.astype(np.float32), size, interpolation=cv2.INTER_AREA)
    return np.where(count > 0.25, total / np.maximum(count, 1e-6), 0).astype(np.float32)


def colorize_depth(depth_m, min_m, max_m):
    """Metres -> BGR: near red, far blue, invalid (0, NaN, out of range) black."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    clipped = np.clip(np.nan_to_num(depth_m), min_m, max_m)
    scaled = ((max_m - clipped) / (max_m - min_m) * 255.0).astype(np.uint8)  # near -> 255
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    color[~valid] = 0
    return color


class VizRelay(Node):
    def __init__(self):
        super().__init__("viz_relay")
        number = ParameterDescriptor(dynamic_typing=True)  # accept 5 or 5.0
        strings = ParameterDescriptor(dynamic_typing=True)  # allow empty lists
        topics = self.declare_parameter("topics", [""], strings).value
        raw_topics = self.declare_parameter("raw_topics", [""], strings).value
        depth_topics = self.declare_parameter("depth_topics", [""], strings).value
        self.period = 1.0 / float(self.declare_parameter("rate_hz", 5.0, number).value)
        self.scale = float(self.declare_parameter("scale", 0.5, number).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 50, number).value)
        self.min_m = float(self.declare_parameter("min_depth_m", 0.3, number).value)
        self.max_m = float(self.declare_parameter("max_depth_m", 6.0, number).value)
        self.prefix = self.declare_parameter("prefix", "/viz").value.rstrip("/")
        throttle = self.declare_parameter("throttle_topics", [""], strings).value
        path_topic = self.declare_parameter("path_pose_topic", "").value
        self.path_period = 1.0 / float(self.declare_parameter("path_hz", 5.0, number).value)
        self.path_points = deque(maxlen=int(self.declare_parameter("path_max_points", 1500, number).value))
        self.last = {}
        self._keep = []
        self._pending = {}  # throttled topic -> (topic, rate) until its type is known
        for entry in filter(None, throttle):
            topic, _, hz = entry.partition("@")
            self._pending[topic] = float(hz or 5.0)
        self._path_topic = path_topic
        self._path_pub = None
        self._discovery = None
        if self._pending or path_topic:
            self._discovery = self.create_timer(1.0, self._discover)  # topics may appear later
        for topic in filter(None, topics):
            self._add(CompressedImage, topic, self.prefix + topic, self._compressed)
        for topic in filter(None, raw_topics):
            self._add(Image, topic, self.prefix + topic + "/compressed", self._raw)
        for topic in filter(None, depth_topics):
            self._add(Image, topic, self.prefix + topic + "/compressed", self._depth)
        self.get_logger().info(
            f"{1.0 / self.period:.1f} Hz, scale {self.scale}, JPEG quality {self.quality}, "
            f"depth {self.min_m}-{self.max_m} m"
        )

    def _discover(self):
        """Subscribe to throttled / path topics once their type is known."""
        types = dict(self.get_topic_names_and_types())
        for topic, hz in list(self._pending.items()):
            if topic in types:
                cls = get_message(types[topic][0])
                out = self.create_publisher(cls, self.prefix + topic, 1)
                period = 1.0 / hz
                self._keep.append(self.create_subscription(
                    cls, topic, lambda raw, t=topic, p=out, dt=period: self._throttle(t, p, raw, dt),
                    LATEST_ONLY, raw=True))
                del self._pending[topic]
                self.get_logger().info(f"{topic} -> {self.prefix + topic} at {hz:g} Hz (raw)")
        if self._path_topic and self._path_pub is None and self._path_topic in types:
            cls = get_message(types[self._path_topic][0])
            self._path_pub = self.create_publisher(Path, self.prefix + self._path_topic + "/path", 1)
            self._keep.append(self.create_subscription(cls, self._path_topic, self._pose, LATEST_ONLY))
            self.create_timer(self.path_period, self._publish_path)
            self.get_logger().info(f"{self._path_topic} -> {self.prefix + self._path_topic}/path")
        if self._discovery and not self._pending and (not self._path_topic or self._path_pub is not None):
            self._discovery.cancel()

    def _throttle(self, topic, publisher, raw, period):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last.get(topic, 0.0) >= period:
            self.last[topic] = now
            publisher.publish(raw)  # serialized bytes: forwarded without decoding

    def _pose(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last.get("__path", 0.0) < self.path_period:
            return
        self.last["__path"] = now
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        p = PoseStamped()
        p.header = msg.header
        p.pose = pose
        self.path_points.append(p)

    def _publish_path(self):
        if self.path_points:
            path = Path()
            path.header = self.path_points[-1].header
            path.poses = list(self.path_points)
            self._path_pub.publish(path)

    def _add(self, msg_type, topic, out_topic, convert):
        out = self.create_publisher(CompressedImage, out_topic, 1)
        self._keep.append(
            self.create_subscription(
                msg_type, topic, lambda m, t=topic, p=out: self.relay(t, p, m, convert), LATEST_ONLY
            )
        )
        self.get_logger().info(f"{topic} -> {out_topic}")

    def _compressed(self, msg):
        return shrink_jpeg(bytes(msg.data), self.scale, self.quality)

    def _raw(self, msg):
        image = image_to_array(msg)
        return None if image is None else encode(image, self.scale, self.quality)

    def _depth(self, msg):
        depth = image_to_array(msg)
        if depth is None or depth.dtype != np.float32:
            return None
        if self.scale != 1.0:
            depth = shrink_depth(depth, self.scale)
        return encode(colorize_depth(depth, self.min_m, self.max_m), 1.0, self.quality)

    def relay(self, topic, publisher, msg, convert):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last.get(topic, 0.0) < self.period:
            return
        self.last[topic] = now
        data = convert(msg)
        if data is None:
            self.get_logger().warn(
                f"{topic}: cannot convert encoding {getattr(msg, 'encoding', 'jpeg')}",
                throttle_duration_sec=10.0,
            )
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = array.array("B", data)  # rclpy fast path
        publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VizRelay()
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
