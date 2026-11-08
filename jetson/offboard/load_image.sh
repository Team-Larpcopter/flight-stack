#!/usr/bin/env bash
# Extract application updates; build the GPU dependency image only when absent.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cache=${GPU_CACHE_DIR:-$here/../gpu}
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
base=${BASE_IMAGE:-larp-flight-stack:humble}
cd "$here"
if [[ ! -f runtime/.extracted || stack-runtime.tar.gz -nt runtime/.extracted ]]; then
  # Never replace libraries underneath a running container.
  for id in $(docker ps -q); do
    if docker inspect --format '{{range .Mounts}}{{println .Source}}{{end}}' "$id" | grep -Fqx "$here/runtime/flight-stack"; then
      echo 'Stop the container using this runtime before updating it.' >&2; exit 2
    fi
  done
  mkdir -p runtime
  tar -xzf stack-runtime.tar.gz -C runtime --no-same-owner
  touch runtime/.extracted
fi
# Existing assembled images also work: the mounted runtime overrides their app.
if [[ ${REBUILD_DEPENDENCIES:-0} == 1 ]] || ! docker image inspect "$image" >/dev/null 2>&1; then
  docker image inspect "$base" >/dev/null
  context=$(mktemp -d "$here/.assembly.XXXXXXXX")
  trap 'rm -rf "$context"' EXIT
  cp "$here/Dockerfile" "$context/Dockerfile"
  ln "$cache/gpu-runtime.tar.gz" "$context/gpu-runtime.tar.gz"
  ln "$cache/engine" "$context/engine"
  docker build --pull=false --network none --build-arg BASE="$base" -t "$image" "$context"
fi
