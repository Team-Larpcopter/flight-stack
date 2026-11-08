"""Output rays must describe the rectified left frame consumed by ROG."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from larp_liteanystereo.stereo_depth_node import StereoDepthNode


def test_scaled_left_depth_projection():
    info = CameraInfo()
    info.width, info.height = 800, 500
    info.k = [500., 0., 399.5, 0., 500., 249.5, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    holder = SimpleNamespace(depth_info=info, left_info=info, baseline_m=.075)
    header = Header(frame_id='native_oak_depth_optical')
    header.stamp.sec = 123
    result = StereoDepthNode._build_camera_info(holder, header, 800, 500, 480, 320, 500.)
    assert result.header == header
    assert (result.width, result.height) == (480, 320)
    assert list(result.k) == pytest.approx([300., 0., 239.5, 0., 320., 159.5, 0., 0., 1.])
    assert result.p[3] == 0.  # depth is in the left frame, not a right camera baseline
    assert result.p[0] == result.k[0] and result.p[2] == result.k[2]
