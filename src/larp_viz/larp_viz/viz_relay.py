"""Low-bandwidth copies of the camera streams for Foxglove over Wi-Fi.

The OAK-D JPEG streams are ~250 KB per frame at 30 Hz (~60 Mbit/s each),
more than the drone's Wi-Fi uplink. This node republishes each input
``sensor_msgs/CompressedImage`` at ``rate_hz``, downscaled by ``scale`` and
re-encoded at ``jpeg_quality``, on ``/viz`` + the input topic. Frames between
publications are dropped before decoding, so the CPU cost is only paid at
``rate_hz``. Viewing only: nothing in flight reads ``/viz``.
"""

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

#: Keep only the newest frame: a viewer never wants a queued, stale one.
LATEST_ONLY = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)

DEFAULT_TOPICS = [
    "/oak/left/image_raw/compressed",
    "/oak/right/image_raw/compressed",
]


def shrink_jpeg(data, scale, quality):
    """Decode a JPEG, resize by ``scale`` and re-encode. None if undecodable."""
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        return None
    if scale != 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return encoded.tobytes() if ok else None


class VizRelay(Node):
    def __init__(self):
        super().__init__("viz_relay")
        # Accept 5 or 5.0 (and 1 or 1.0) from the command line.
        number = ParameterDescriptor(dynamic_typing=True)
        topics = self.declare_parameter("topics", DEFAULT_TOPICS).value
        self.period = 1.0 / float(self.declare_parameter("rate_hz", 5.0, number).value)
        self.scale = float(self.declare_parameter("scale", 0.5, number).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 50, number).value)
        self.prefix = self.declare_parameter("prefix", "/viz").value.rstrip("/")
        self.last = {}
        self._keep = []
        for topic in topics:
            out = self.create_publisher(CompressedImage, self.prefix + topic, 1)
            self._keep.append(
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, t=topic, p=out: self.relay(t, p, msg),
                    LATEST_ONLY,
                )
            )
            self.get_logger().info(f"{topic} -> {self.prefix + topic}")
        self.get_logger().info(
            f"{1.0 / self.period:.1f} Hz, scale {self.scale}, JPEG quality {self.quality}"
        )

    def relay(self, topic, publisher, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last.get(topic, 0.0) < self.period:
            return
        self.last[topic] = now
        data = shrink_jpeg(bytes(msg.data), self.scale, self.quality)
        if data is None:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = data
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
