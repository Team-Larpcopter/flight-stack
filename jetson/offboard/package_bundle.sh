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
cp "$root/jetson/offboard/"{launch_localization.sh,load_image.sh,runtime_mounts.sh,replace_stack.sh,run_full_stack.sh,launch_system_tmux.sh,launch_full_system_tmux.sh,launch_mission_tmux.sh,launch_ego_mission_tmux.sh,launch_ego_full_system_tmux.sh,launch_triangle_tmux.sh,launch_takeoff_hold_tmux.sh,launch_fixed_course_tmux.sh} "$out/"
cp "$root/src/larp_route_follower/scripts/"launch_follower_*tmux.sh "$out/"
cp "$root/jetson/foxglove/larpcopter-autonomy-localization.json" "$out/"
cp "$root/jetson/foxglove/larpcopter-flight.json" "$out/"
cp "$root/src/larp_autonomy/config/mission.yaml" "$out/mission.example.yaml"
echo "Packaged ARM64 runtime bundle: $out"
