#!/usr/bin/env bash
# Jetson only. Exactly eight panes; host build/transfer never calls this script.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ -f $here/deployment.env ]]; then source "$here/deployment.env"; fi
mode=${1:-shadow}
[[ $mode == shadow || $mode == hardware ]] || { echo 'Usage: launch_system_tmux.sh shadow|hardware' >&2; exit 2; }
: "${DEPLOYMENT_CONFIG:?Set DEPLOYMENT_CONFIG to the directory containing mission.yaml, rig.json and bridge.yaml}"
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
container=${CONTAINER:-flight-stack-mission-$mode}
session=${SESSION:-flight-stack-$mode}
command -v tmux >/dev/null
# Reattach without launching another set of sensor/command publishers.
if tmux has-session -t "$session" 2>/dev/null; then
  tmux set-option -t "$session" mouse on
  exec tmux attach-session -t "$session"
fi
# Match the takeoff launcher: no competing camera, UART or flight publishers.
for other in flight-stack-takeoff-hold flight-stack-mission-hardware flight-stack-mission-shadow flight-stack-localization-runtime flight-stack-runtime; do
  if [[ $(docker inspect --format '{{.State.Running}}' "$other" 2>/dev/null || true) == true ]]; then
    echo "Stop $other before starting this mission; it shares camera/PX4 resources." >&2
    exit 2
  fi
done
source "$here/runtime_mounts.sh"
config=$(realpath "$DEPLOYMENT_CONFIG")
[[ -f $config/mission.yaml ]] || { echo 'Missing DEPLOYMENT_CONFIG/mission.yaml' >&2; exit 2; }
transport=${PX4_TRANSPORT:-udp4}; devices=()
case $transport in
  udp4) ;;
  serial)
    : "${PX4_DEVICE:?Set PX4_DEVICE for serial DDS}"
    [[ -c $PX4_DEVICE ]] || { echo 'PX4_DEVICE must be a serial character device' >&2; exit 2; }
    devices=(--device "$PX4_DEVICE:/dev/px4") ;;
  *) echo 'PX4_TRANSPORT must be udp4 or serial' >&2; exit 2 ;;
esac
dds=()
if [[ -n ${XRCE_REFS:-} ]]; then
  dds+=(-v "$(realpath "$XRCE_REFS"):/etc/flight-stack/agent_refs.xml:ro" -e XRCE_REFS=/etc/flight-stack/agent_refs.xml)
fi
if [[ -n ${DDS_PROFILE:-} ]]; then
  dds+=(-v "$(realpath "$DDS_PROFILE"):/etc/flight-stack/dds.xml:ro"
       -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml
       -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml)
fi
# A stopped container is not silently reused after configuration/image changes.
# No automatic restart of an executive that might have already flown.
if docker container inspect "$container" >/dev/null 2>&1; then
  echo "Container $container already exists. Stop its processes and remove it before a new mission." >&2
  exit 2
fi
docker run --rm --network none "${runtime_mounts[@]}" -v "$config:/config:ro" "$image" \
  python3 -c 'import sys; from larp_autonomy.runtime_config import load_config; load_config("/config/mission.yaml",sys.argv[1])' "$mode"
docker create --init --name "$container" "${runtime_mounts[@]}" --runtime nvidia --network host --ipc host \
  --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb "${devices[@]}" "${dds[@]}" \
  -v "$config:/config:ro" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e PX4_TRANSPORT="$transport" -e PX4_PORT="${PX4_PORT:-8888}" -e PX4_BAUD="${PX4_BAUD:-921600}" \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 "$image" sleep infinity >/dev/null
docker start "$container" >/dev/null
pane() {
  printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
    ros2 launch larp_autonomy mission_runtime.launch.py "component:=$1" \
    "mode:=$mode" mission_file:=/config/mission.yaml
}
parts=(drivers foxglove vins dds depth map super executive)
titles=(Drivers Foxglove sqrtVINS DDS LAS 'ROG map' SUPER 'Mission executive')
first=$(tmux new-session -d -P -F '#{pane_id}' -s "$session" -n system "bash --noprofile --norc -i")
tmux set-option -t "$session" mouse on
tmux set-option -t "$session" remain-on-exit off
tmux set-option -t "$session" pane-border-status top
tmux select-pane -t "$first" -T "${titles[0]}"
tmux send-keys -t "$first" -l "$(pane drivers)"
tmux send-keys -t "$first" Enter
for ((i=1; i<8; ++i)); do
  created=$(tmux split-window -d -P -F '#{pane_id}' -t "$session:system" "bash --noprofile --norc -i")
  tmux select-pane -t "$created" -T "${titles[i]}"
  tmux send-keys -t "$created" -l "$(pane "${parts[i]}")"
  tmux send-keys -t "$created" Enter
  tmux select-layout -t "$session:system" tiled >/dev/null
done
tmux select-pane -t "$first"
exec tmux attach-session -t "$session"
