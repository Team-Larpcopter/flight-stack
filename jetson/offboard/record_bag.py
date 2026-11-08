#!/usr/bin/env python3
"""Record the stack on demand, retaining compressed images and excluding raw images."""
import os
from pathlib import Path
import re
import time
from datetime import datetime

# Covers this stack's image publishers even if their panes start after recording.
RAW_IMAGE_NAMES = r'(^|/)(image|image_raw|image_undistorted|image_rect|image_rect_color|image_rect_mono|trackhist)$'


def main():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('bag_image_discovery')
    raw = set()
    until = time.monotonic() + 1.0
    try:
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
            raw.update(name for name, types in node.get_topic_names_and_types()
                       if any(t in ('sensor_msgs/msg/Image', 'stereo_msgs/msg/DisparityImage') for t in types))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    exclude = '|'.join([RAW_IMAGE_NAMES, *(f'^{re.escape(name)}$' for name in sorted(raw))])
    root = Path('/bags')
    root.mkdir(parents=True, exist_ok=True)
    output = root / datetime.now().strftime('flight_%Y%m%d_%H%M%S_%f')
    print(f'Recording to {output}; Ctrl+C stops and finalizes the bag.', flush=True)
    print('Raw images excluded; compressed images and all other discovered topics included.', flush=True)
    os.execvp('ros2', ['ros2', 'bag', 'record', '--all', '--include-hidden-topics',
                      '--exclude', exclude, '--output', str(output),
                      '--max-cache-size', '33554432', '--max-bag-size', '1073741824'])


if __name__ == '__main__':
    main()
