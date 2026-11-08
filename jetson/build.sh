#!/usr/bin/env bash
# Host entry point: builds source in an isolated container with no devices.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
image=${BUILD_IMAGE:-larp-flight-stack:humble}
base=${BASE_IMAGE:-ros:humble-ros-base-jammy}
docker build --build-arg "BASE=$base" -f "$root/jetson/integration_image/Dockerfile" -t "$image" "$root"
exec docker run --rm --init --network host \
  -v "$root:/flight-stack" -w /flight-stack -e "BUILD_JOBS=${BUILD_JOBS:-1}" \
  "$image" bash jetson/integration_image/build_workspace.sh
