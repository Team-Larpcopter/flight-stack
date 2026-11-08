#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/sdk_install/lib:${LD_LIBRARY_PATH:-}"
command -v MicroXRCEAgent >/dev/null || { echo 'Rebuild image with bash jetson/build.sh (new agent layer required)' >&2; exit 2; }
exec ros2 launch larp_autonomy full_stack.launch.py "$@"
