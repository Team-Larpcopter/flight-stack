#!/usr/bin/env bash
# Compile on an x86 host under ARM64/QEMU; package for offline Jetson assembly.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
toolchain=${BUILD_IMAGE:-larp-flight-stack:arm64-toolchain}
volume=${BUILD_VOLUME:-flight-stack-arm64-build}
out=${OUTPUT_DIR:-$root/deployment/arm64}
jobs=${BUILD_JOBS:-$(nproc)}
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo 'BUILD_JOBS must be positive' >&2; exit 2; }
mkdir -p "$out"
context=$(mktemp -d)
trap 'rm -rf "$context"' EXIT
# Keep native Dockerfile layers intact; only offboard DDS compilation uses all cores.
sed "s/--parallel 1/--parallel $jobs/" "$root/jetson/integration_image/Dockerfile" > "$context/Dockerfile"
docker build --platform linux/arm64 -t "$toolchain" "$context"
docker volume create "$volume" >/dev/null
tar -C "$root" --exclude='*.engine' --exclude='__pycache__' --exclude='.pytest_cache' \
  -cf "$out/source.tar" src jetson
docker run --rm -i --platform linux/arm64 -v "$volume:/flight-stack" "$toolchain" \
  bash -c 'rm -rf /flight-stack/src /flight-stack/jetson; tar -xf - -C /flight-stack' < "$out/source.tar"
docker run --rm --platform linux/arm64 --network host \
  -v "$volume:/flight-stack" -e BUILD_JOBS="$jobs" -e FORCE_SDK_BUILD="${FORCE_SDK_BUILD:-0}" -e RUN_TESTS="${RUN_TESTS:-0}" -w /flight-stack \
  "$toolchain" bash jetson/integration_image/build_workspace.sh
OUTPUT_DIR="$out" BUILD_IMAGE="$toolchain" BUILD_VOLUME="$volume" bash "$root/jetson/offboard/package_bundle.sh"
# Validate the installed artifacts in an independent image without source/build mounts.
cp "$out/stack-runtime.tar.gz" "$context/stack-runtime.tar.gz"
printf '%s\n' "FROM $toolchain" 'ADD stack-runtime.tar.gz /' \
  'ENV LD_LIBRARY_PATH=/flight-stack/sdk_install/lib' \
  'ENTRYPOINT ["/flight-stack/jetson/offboard/entrypoint.sh"]' > "$context/Dockerfile"
docker build --platform linux/arm64 --network none -t larp-flight-stack:arm64-verify "$context"
docker run --rm --platform linux/arm64 --network none larp-flight-stack:arm64-verify bash -ec \
  'ros2 pkg executables larp_hardware; ros2 pkg executables larp_liteanystereo; ros2 pkg executables larp_perception; command -v MicroXRCEAgent; ros2 launch larp_autonomy visualization_runtime.launch.py --show-args; bash /flight-stack/jetson/integration_image/smoke_localization.sh'
echo "Built and checked ARM64 bundle: $out/stack-runtime.tar.gz"
