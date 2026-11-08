#!/usr/bin/env bash
# One-time target-side cache of the working GPU runtime; no engine compilation.
set -euo pipefail
: "${SSH_HOST:?Set SSH_HOST to user@jetson}"
: "${GPU_CONTAINER:?Set GPU_CONTAINER to the working LAS container}"
: "${ENGINE_ON_JETSON:?Set ENGINE_ON_JETSON to the saved engine absolute path}"
[[ $GPU_CONTAINER =~ ^[a-zA-Z0-9_.-]+$ ]] || exit 2
[[ $ENGINE_ON_JETSON =~ ^/[a-zA-Z0-9_./-]+$ ]] || exit 2
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cache=${REMOTE_GPU_DIR:-flight-stack/deployment/gpu}
[[ $cache =~ ^[a-zA-Z0-9_./-]+$ && $cache != /* && $cache != *..* ]] || exit 2
remote=$(ssh "$SSH_HOST" mktemp -d /tmp/flight-gpu-export.XXXXXXXX)
scp "$root/jetson/offboard/export_gpu_runtime.py" "$SSH_HOST:$remote/export.py"
ssh "$SSH_HOST" "set -e; docker cp '$remote/export.py' '$GPU_CONTAINER:$remote.py'; docker exec '$GPU_CONTAINER' python3 '$remote.py' '$remote.tar.gz'; mkdir -p ~/'$cache'; docker cp '$GPU_CONTAINER:$remote.tar.gz' ~/'$cache/gpu-runtime.tar.gz'; docker cp '$GPU_CONTAINER:$remote.tar.gz.json' ~/'$cache/gpu-runtime.json'; cp '$ENGINE_ON_JETSON' ~/'$cache/engine'; cd ~/'$cache'; docker inspect --format '{{.Image}}' '$GPU_CONTAINER' > source-image.txt"
echo "GPU runtime cached on Jetson at ~/$cache; it stays there across deployments."
