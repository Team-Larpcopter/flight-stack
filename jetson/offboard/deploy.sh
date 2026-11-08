#!/usr/bin/env bash
# Transfer only; assembly/launching is a separate Jetson-side step.
set -euo pipefail
: "${SSH_HOST:?Set SSH_HOST to user@jetson}"
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
out=${OUTPUT_DIR:-$root/deployment/arm64}
remote=${REMOTE_DIR:-flight-stack/deployment/arm64}
[[ $remote =~ ^[a-zA-Z0-9_./-]+$ && $remote != /* && $remote != *..* ]] || {
  echo 'REMOTE_DIR must be a relative directory under the SSH user home' >&2; exit 2;
}
ssh "$SSH_HOST" "mkdir -p ~/'$remote'"
scp "$out/stack-runtime.tar.gz" "$out/Dockerfile" \
  "$out/launch_localization.sh" "$out/runtime_mounts.sh" "$out/load_image.sh" "$out/run_full_stack.sh" \
  "$out/launch_system_tmux.sh" "$out/launch_full_system_tmux.sh" "$out/launch_mission_tmux.sh" "$out/launch_takeoff_hold_tmux.sh" \
  "$out/larpcopter-autonomy-localization.json" "$out/larpcopter-flight.json" "$out/mission.example.yaml" "$SSH_HOST:$remote/"
echo "Transferred to ~/$remote. Run bash ~/$remote/launch_localization.sh on Jetson separately."
