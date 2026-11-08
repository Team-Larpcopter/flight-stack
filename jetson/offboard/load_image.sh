#!/usr/bin/env bash
# Jetson-side image assembly only: no source compilation, downloads or engine build.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cache=${GPU_CACHE_DIR:-$here/../gpu}
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
base=${BASE_IMAGE:-larp-flight-stack:humble}
cd "$here"
docker image inspect "$base" >/dev/null
# Hard links keep the cached GPU archive out of the network transfer and avoid copying it.
context=$(mktemp -d "$here/.assembly.XXXXXXXX")
trap 'rm -rf "$context"' EXIT
for file in stack-runtime.tar.gz Dockerfile; do ln "$here/$file" "$context/$file"; done
ln "$cache/gpu-runtime.tar.gz" "$context/gpu-runtime.tar.gz"
ln "$cache/engine" "$context/engine"
docker build --pull=false --network none --build-arg BASE="$base" -t "$image" "$context"
touch "$here/.assembled"
