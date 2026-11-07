import cv2
import numpy as np

from larp_viz.viz_relay import shrink_jpeg


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
