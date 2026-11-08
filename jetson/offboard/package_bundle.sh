#!/usr/bin/env bash
# Package locally compiled ARM64 artifacts; reuse base/GPU layers on the target.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
out=${OUTPUT_DIR:-$root/deployment/arm64}
image=${BUILD_IMAGE:-larp-flight-stack:arm64-toolchain}
volume=${BUILD_VOLUME:-flight-stack-arm64-build}
mkdir -p "$out"
# ROS includes Foxglove and its Python/message support. Agent carries isolated DDS libraries.
docker run --rm --platform linux/arm64 -v "$volume:/flight-stack" "$image" \
  tar -cf - -C / flight-stack/install flight-stack/sdk_install flight-stack/jetson \
    opt/ros/humble opt/xrce-agent usr/local/bin/MicroXRCEAgent | \
  gzip -1 > "$out/stack-runtime.tar.gz"
cp "$root/jetson/offboard/Dockerfile" "$out/Dockerfile"
cp "$root/jetson/offboard/"{launch_localization.sh,load_image.sh,run_full_stack.sh} "$out/"
cp "$root/jetson/foxglove/larpcopter-autonomy-localization.json" "$out/"
echo "Packaged ARM64 runtime bundle: $out"
