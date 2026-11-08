import cv2
import numpy as np
from sensor_msgs.msg import Image

from larp_perception.viz_relay import colorize_depth, image_to_array, shrink_jpeg


def make_image(array, encoding):
    msg = Image()
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = array.strides[0]
    msg.data = array.tobytes()
    return msg


def test_shrink_halves_size_and_compresses():
    image = (np.random.default_rng(0).random((640, 1024, 3)) * 255).astype(np.uint8)
    ok, big = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    small = shrink_jpeg(big.tobytes(), 0.5, 50)
    decoded = cv2.imdecode(np.frombuffer(small, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    assert decoded.shape[:2] == (320, 512)
    assert len(small) < len(big.tobytes()) / 4


def test_garbage_returns_none():
    assert shrink_jpeg(b"not a jpeg", 0.5, 50) is None


def test_mono8_round_trip():
    mono = np.arange(640 * 1024, dtype=np.uint32).reshape(640, 1024).astype(np.uint8)
    assert np.array_equal(image_to_array(make_image(mono, "mono8")), mono)


def test_depth_mm_to_metres():
    depth = np.full((4, 6), 1500, dtype=np.uint16)
    out = image_to_array(make_image(depth, "16UC1"))
    assert out.dtype == np.float32 and np.allclose(out, 1.5)


def test_colorize_marks_invalid_black_and_near_differs_from_far():
    depth = np.array([[0.0, 0.5, 5.0]], dtype=np.float32)
    color = colorize_depth(depth, 0.3, 6.0)
    assert (color[0, 0] == 0).all()
    assert not np.array_equal(color[0, 1], color[0, 2])
