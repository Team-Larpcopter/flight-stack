#!/usr/bin/env bash
set -euo pipefail
# Run inside the toolchain image, using this flight-stack checkout.
# Downloads/builds dependencies only; no device access is needed.
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source_dir=${1:-$root/sdk-source/depthai-core-v3.10.0}
build_dir=${2:-$root/build_sdk}
install_dir=${3:-$root/sdk_install}
cmake -S "$source_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX="$install_dir" \
  -DDEPTHAI_BUILD_EXAMPLES=OFF -DDEPTHAI_BUILD_TESTS=OFF \
  -DDEPTHAI_BUILD_PYTHON=OFF -DDEPTHAI_BUILD_BETA=OFF \
  -DDEPTHAI_OPENCV_SUPPORT=OFF -DDEPTHAI_XTENSOR_SUPPORT=OFF \
  -DDEPTHAI_ENABLE_APRIL_TAG=OFF -DDEPTHAI_ENABLE_PROTOBUF=OFF \
  -DDEPTHAI_ENABLE_REMOTE_CONNECTION=OFF -DDEPTHAI_ENABLE_EVENTS_MANAGER=OFF \
  -DDEPTHAI_ENABLE_MP4V2=OFF -DDEPTHAI_ENABLE_CURL=OFF \
  -DDEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=OFF \
  -DDEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF -DDEPTHAI_ENABLE_DEVICE_RVC3_FW=OFF
cmake --build "$build_dir" --parallel 1
cmake --install "$build_dir"
