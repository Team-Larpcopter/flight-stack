#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /flight-stack/install/setup.bash
exec "$@"
