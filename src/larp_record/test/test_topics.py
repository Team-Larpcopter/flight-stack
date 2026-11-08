"""Check which topics the recorder skips."""

import re

import pytest
from larp_record.topics import exclude_regex

RAW_IMAGES = [
    '/stereo/left/image',
    '/stereo/right/image',
    '/camera/depth/image_raw',
    '/camera/depth/image_visual',
    '/oak/rgb/image_rect',
    '/stereo/left/image/theora',
]
COMPRESSED_IMAGES = [
    '/stereo/left/image/compressed',
    '/camera/depth/image_visual/compressed',
    '/camera/depth/image_raw/compressedDepth',
]
ALWAYS_KEPT = [
    '/fmu/out/vehicle_status_v1',
    '/fmu/in/trajectory_setpoint',
    '/larper/trajectory_in',
    '/stereo/imu',
    '/stereo/left/camera_info',
    '/stereo/left/image/camera_info',
    '/stereo/left/image/compressed/camera_info',
    '/tf_static',
    '/imagery/status',
]


def excluded(regex, topic):
    # rosbag2 versions differ between search and full match; require both agree.
    search = re.search(regex, topic) is not None
    assert search == (re.fullmatch(regex, topic) is not None), topic
    return search


@pytest.mark.parametrize('topic', RAW_IMAGES)
def test_compressed_mode_skips_raw_images(topic):
    assert excluded(exclude_regex('compressed'), topic)


@pytest.mark.parametrize('topic', COMPRESSED_IMAGES + ALWAYS_KEPT)
def test_compressed_mode_keeps_everything_else(topic):
    assert not excluded(exclude_regex('compressed'), topic)


@pytest.mark.parametrize('topic', COMPRESSED_IMAGES)
def test_none_mode_skips_compressed_images(topic):
    assert excluded(exclude_regex('none'), topic)


@pytest.mark.parametrize('topic', ALWAYS_KEPT)
def test_none_mode_keeps_non_images(topic):
    assert not excluded(exclude_regex('none'), topic)


def test_raw_mode_excludes_nothing():
    assert exclude_regex('raw') == ''


def test_extra_regex_is_added():
    regex = exclude_regex('compressed', '^/fmu/out/wind$')
    assert excluded(regex, '/fmu/out/wind')
    assert excluded(regex, '/stereo/left/image')
    assert not excluded(regex, '/fmu/out/vehicle_status_v1')
    assert exclude_regex('raw', '^/a$') == '^/a$'


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        exclude_regex('jpeg')
