"""Check the recorder's arguments and the ros2 bag command it builds."""

from pathlib import Path

import pytest
from larp_record.record import bag_command, depth_command, parse_args


def test_defaults_skip_raw_images_and_keep_one_file():
    cmd = bag_command(parse_args([]), Path('/bags/x'), 'sqlite3')
    assert cmd[:5] == ['ros2', 'bag', 'record', '-a', '--include-hidden-topics']
    assert cmd[cmd.index('-o') + 1] == '/bags/x'
    assert cmd[cmd.index('-s') + 1] == 'sqlite3'
    assert '-x' in cmd
    assert '-d' not in cmd and '--compression-mode' not in cmd


def test_options_reach_ros2_bag():
    args = parse_args(['--images', 'raw', '--split-seconds', '60', '--zstd'])
    cmd = bag_command(args, Path('/bags/x'), 'mcap')
    assert '-x' not in cmd
    assert cmd[cmd.index('-d') + 1] == '60'
    assert cmd[cmd.index('--compression-format') + 1] == 'zstd'


def test_depth_republisher_targets_compressed_depth_topic():
    cmd = depth_command('/camera/depth/image_raw')
    assert 'in:=/camera/depth/image_raw' in cmd
    assert 'out/compressedDepth:=/camera/depth/image_raw/compressedDepth' in cmd


@pytest.mark.parametrize('argv', [
    ['--name', 'bad/name'],
    ['--duration', '0'],
    ['--split-seconds', 'x'],
    ['--images', 'jpeg'],
])
def test_bad_arguments_are_rejected(argv):
    with pytest.raises(SystemExit):
        parse_args(argv)
