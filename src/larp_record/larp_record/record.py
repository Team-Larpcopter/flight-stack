"""Record every ROS 2 topic to a rosbag, with images compressed.

    ros2 run larp_record record [--name NAME] [--duration S] ...

Bags go to <bag-dir>/<UTC start>[-NAME]/; --bag-dir defaults to $LARP_BAG_DIR,
else ~/larp_bags. Ctrl-C stops recording and closes the bag. Topics are picked
up as they appear, so start it before or after the simulator or flight launch.

Raw image topics are skipped (see larp_record.topics); their /compressed copies
are kept. Metric depth has no compressed copy from the camera bridge, so this
script republishes --depth-topic as <depth-topic>/compressedDepth while it runs.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from larp_record.topics import IMAGE_MODES, exclude_regex

# rosbag2 needs time to flush its cache and write metadata.yaml after SIGINT.
CLOSE_TIMEOUT_S = 20.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog='ros2 run larp_record record',
        description='Record all ROS 2 topics to a rosbag, with raw images replaced by '
                    'compressed copies. Ctrl-C stops recording.')
    parser.add_argument('--name', default='', help='suffix for the bag folder, e.g. dronehacks')
    parser.add_argument('--bag-dir', default=os.environ.get('LARP_BAG_DIR', '~/larp_bags'),
                        help='parent folder for bags (default: $LARP_BAG_DIR or ~/larp_bags)')
    parser.add_argument('--duration', type=positive_int, metavar='S',
                        help='stop automatically after S seconds')
    parser.add_argument('--images', choices=IMAGE_MODES, default='compressed',
                        help='compressed: skip raw images (default); none: skip all images; '
                             'raw: keep raw images too (large)')
    parser.add_argument('--exclude', default='', metavar='REGEX',
                        help='extra regex of topics to skip')
    parser.add_argument('--depth-topic', default='/camera/depth/image_raw',
                        help='depth image to republish as compressedDepth; "" disables')
    parser.add_argument('--storage', choices=('auto', 'mcap', 'sqlite3'), default='auto',
                        help='auto: mcap if its plugin is installed, else sqlite3')
    parser.add_argument('--split-seconds', type=positive_int, metavar='S',
                        help='start a new bag file every S seconds')
    parser.add_argument('--zstd', action='store_true',
                        help='zstd-compress each bag file when it closes')
    parser.add_argument('--min-free-gb', type=float, default=5.0,
                        help='refuse to start with less free disk (default: 5)')
    args = parser.parse_args(argv)
    if args.name and not all(c.isalnum() or c in '._-' for c in args.name):
        parser.error('--name may contain only letters, digits, ".", "_" and "-"')
    return args


def positive_int(text):
    if not text.isdigit() or int(text) < 1:
        raise argparse.ArgumentTypeError(f'must be a whole number >= 1, not {text!r}')
    return int(text)


def storage_plugin(requested):
    if requested != 'auto':
        return requested
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory('rosbag2_storage_mcap')
        return 'mcap'
    except PackageNotFoundError:
        return 'sqlite3'


def bag_command(args, bag, storage):
    cmd = ['ros2', 'bag', 'record', '-a', '--include-hidden-topics', '-o', str(bag),
           '-s', storage]
    regex = exclude_regex(args.images, args.exclude)
    if regex:
        cmd += ['-x', regex]
    if args.split_seconds:
        cmd += ['-d', str(args.split_seconds)]
    if args.zstd:
        cmd += ['--compression-mode', 'file', '--compression-format', 'zstd']
    return cmd


def depth_command(depth_topic):
    return ['ros2', 'run', 'image_transport', 'republish', 'raw', 'compressedDepth',
            '--ros-args', '-r', '__node:=record_depth_compress', '-r', f'in:={depth_topic}',
            '-r', f'out/compressedDepth:={depth_topic}/compressedDepth']


def start(cmd, **kwargs):
    # Own session: a terminal Ctrl-C reaches the child once, through stop(),
    # and the `ros2` wrapper and the real binary are signalled together.
    return subprocess.Popen(cmd, start_new_session=True, **kwargs)


def signal_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def main(argv=None):
    args = parse_args(argv)
    parent = Path(args.bag_dir).expanduser()
    parent.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(parent).free / 2**30
    if free_gb < args.min_free_gb:
        sys.exit(f'error: only {free_gb:.1f} GiB free in {parent}; need {args.min_free_gb:g}')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    bag = parent / (f'{stamp}-{args.name}' if args.name else stamp)
    storage = storage_plugin(args.storage)

    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    depth = None
    if args.images == 'compressed' and args.depth_topic:
        # PNG of quantised inverse depth: precise up close, coarser past ~10 m.
        # Its output is hidden: Humble's republisher segfaults harmlessly on exit.
        depth = start(depth_command(args.depth_topic),
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print(f'Recording all topics to {bag} ({storage}, images: {args.images})', flush=True)
    print('Press Ctrl-C to stop.', flush=True)
    recorder = start(bag_command(args, bag, storage), stdin=subprocess.DEVNULL)
    deadline = time.monotonic() + args.duration if args.duration else None
    try:
        while recorder.poll() is None and not stopping:
            if deadline and time.monotonic() >= deadline:
                break
            time.sleep(0.2)
        status = stop(recorder)
    finally:
        if depth:
            signal_group(depth, signal.SIGKILL)
            depth.wait()
    return finish(bag, storage, status)


def stop(recorder):
    if recorder.poll() is None:
        signal_group(recorder, signal.SIGINT)
        try:
            recorder.wait(CLOSE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            print(f'warning: recorder did not close within {CLOSE_TIMEOUT_S:.0f} s; killing it',
                  file=sys.stderr)
            signal_group(recorder, signal.SIGKILL)
            recorder.wait()
    return recorder.returncode


def finish(bag, storage, status):
    if not (bag / 'metadata.yaml').exists() and any(bag.glob('*.db3')):
        # The recorder died without closing the bag (for example, its .db3 was
        # opened while recording). Rebuild metadata so it is still playable.
        print('Recorder did not close the bag cleanly; running ros2 bag reindex.',
              file=sys.stderr)
        subprocess.run(['ros2', 'bag', 'reindex', '-s', storage, str(bag)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    if not (bag / 'metadata.yaml').exists():
        print(f'error: no usable bag in {bag}', file=sys.stderr)
        return status or 1
    print(f'\nSaved bag: {bag}')
    info = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True,
                          check=False)
    print('\n'.join(info.stdout.splitlines()[:8]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
