#!/usr/bin/env bash
# Run on Jetson after transfer: one command per tmux pane, mouse enabled.
set -euo pipefail
container=${CONTAINER:-flight-stack-localization-runtime}
session=${SESSION:-flight-stack-runtime}
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ -f $here/stack-runtime.tar.gz ]]; then
  if [[ ! -f $here/.assembled || $here/stack-runtime.tar.gz -nt $here/.assembled || $here/Dockerfile -nt $here/.assembled ]] || ! docker image inspect "$image" >/dev/null 2>&1; then
    bash "$here/load_image.sh"
  fi
fi
command -v tmux >/dev/null || { echo 'tmux is required on the Jetson host' >&2; exit 1; }
if ! docker container inspect "$container" >/dev/null 2>&1; then
  dds=()
  if [[ -n ${DDS_PROFILE:-} ]]; then
    dds=(-v "$(realpath "$DDS_PROFILE"):/etc/flight-stack/dds.xml:ro"
         -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml)
  fi
  docker create --init --name "$container" --network host --ipc host \
    --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb "${dds[@]}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$image" sleep infinity >/dev/null
fi
actual=$(docker inspect --format '{{.Image}}' "$container")
expected=$(docker image inspect --format '{{.Id}}' "$image")
[[ $actual == "$expected" ]] || { echo "Container $container uses an older image; choose a new CONTAINER or remove it after stopping its processes." >&2; exit 2; }
docker start "$container" >/dev/null
if tmux has-session -t "$session" 2>/dev/null; then
  tmux set-option -t "$session" mouse on
  exec tmux attach-session -t "$session"
fi
pane() {
  printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
    bash /flight-stack/jetson/offboard/localization_pane.sh "$1"
}
tmux new-session -d -s "$session" -n localization "bash --noprofile --norc -i"
tmux set-option -t "$session" mouse on
tmux set-option -t "$session" remain-on-exit off
tmux split-window -h -t "$session:localization" "bash --noprofile --norc -i"
tmux split-window -v -t "$session:localization.1" "bash --noprofile --norc -i"
parts=(camera vins foxglove)
for i in 0 1 2; do
  tmux send-keys -t "$session:localization.$i" -l "$(pane "${parts[i]}")"
  tmux send-keys -t "$session:localization.$i" Enter
done
tmux set-option -t "$session" pane-border-status top
tmux select-pane -t "$session:localization.0" -T Camera
tmux select-pane -t "$session:localization.1" -T sqrtVINS
tmux select-pane -t "$session:localization.2" -T Foxglove
tmux select-pane -t "$session:localization.0"
exec tmux attach-session -t "$session"
