#!/usr/bin/env bash
# Record all ROS 2 topics to a rosbag, skipping raw images; their /compressed
# copies are kept. Ctrl-C stops recording and saves the bag.
#
#   scripts/record.sh [name]
#
# Bags go to $LARP_BAG_DIR/<UTC start>[-name], default ~/larp_bags.
# Source ROS and the flight-stack overlay first so custom messages record.
set -euo pipefail

out="${LARP_BAG_DIR:-$HOME/larp_bags}/$(date -u +%Y%m%dT%H%M%SZ)${1:+-$1}"
mkdir -p "$(dirname "$out")"
echo "Recording to $out (Ctrl-C to stop)"
exec ros2 bag record -a --include-hidden-topics -s sqlite3 -o "$out" \
  -x '^(?!.*(/camera_info|/compressed|/compressedDepth)$).*/image(_[^/]*)?(/.*)?$'
