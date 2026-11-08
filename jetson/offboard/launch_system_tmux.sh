#!/usr/bin/env bash
# Jetson only. Eight mission panes plus a manual recorder; host build/transfer never calls this script.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ -f $here/deployment.env ]]; then source "$here/deployment.env"; fi
mode=${1:-shadow}
if (($#)); then shift; fi
planner=super
check=false
usage() { echo 'Usage: launch_system_tmux.sh shadow|hardware [--planner super|ego] [--check]'; }
[[ $mode == shadow || $mode == hardware ]] || { usage >&2; exit 2; }
while (($#)); do
  case $1 in
    --planner) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; planner=$2; shift 2 ;;
    --check) check=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ $planner == super || $planner == ego ]] || { usage >&2; exit 2; }
: "${DEPLOYMENT_CONFIG:?Set DEPLOYMENT_CONFIG to the directory containing mission.yaml, rig.json and bridge.yaml}"
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
suffix=''
if [[ $planner == ego ]]; then suffix=-ego; fi
container=${CONTAINER:-flight-stack-mission$suffix-$mode}
session=${SESSION:-flight-stack$suffix-$mode}
if ! $check; then command -v tmux >/dev/null; fi
config=$(realpath "$DEPLOYMENT_CONFIG")
[[ -f $config/mission.yaml ]] || { echo 'Missing DEPLOYMENT_CONFIG/mission.yaml' >&2; exit 2; }
transport=${PX4_TRANSPORT:-udp4}; devices=()
case $transport in
  udp4) ;;
  serial)
    : "${PX4_DEVICE:?Set PX4_DEVICE for serial DDS}"
    if ! $check; then [[ -c $PX4_DEVICE ]] || { echo 'PX4_DEVICE must be a serial character device' >&2; exit 2; }; fi
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
# An explicit launch replaces known stack owners before runtime extraction.
# --check never stops a container or tmux session.
if ! $check; then source "$here/replace_stack.sh"; fi
source "$here/runtime_mounts.sh"
docker run --rm --network none "${runtime_mounts[@]}" -v "$config:/config:ro" "$image" \
  python3 -c 'import ctypes, sys; from pathlib import Path; from ament_index_python.packages import get_package_prefix; from larp_autonomy.runtime_config import load_config; load_config("/config/mission.yaml",sys.argv[1]); ctypes.CDLL(str(Path(get_package_prefix("larp_ego_planner"))/"lib/libego_optimizer.so")); assert (Path(get_package_prefix("larp_navigation"))/"lib/larp_navigation/depth_map_node").is_file()' "$mode"
if $check; then
  echo "Mission preflight passed: planner=$planner mode=$mode; no mission processes started."
  exit 0
fi
bag_dir=${ROSBAG_DIR:-$here/bags}
mkdir -p "$bag_dir"
bag_dir=$(realpath "$bag_dir")
docker create --init --name "$container" -v "$bag_dir:/bags" "${runtime_mounts[@]}" --runtime nvidia --network host --ipc host \
  --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb "${devices[@]}" "${dds[@]}" \
  -v "$config:/config:ro" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e PX4_TRANSPORT="$transport" -e PX4_PORT="${PX4_PORT:-8888}" -e PX4_BAUD="${PX4_BAUD:-921600}" \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 "$image" sleep infinity >/dev/null
docker start "$container" >/dev/null
pane() {
  if [[ $1 == rosbag ]]; then
    printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
      python3 /flight-stack/jetson/offboard/record_bag.py
    return
  fi
  printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
    ros2 launch larp_autonomy mission_runtime.launch.py "component:=$1" \
    "mode:=$mode" "planner:=$planner" mission_file:=/config/mission.yaml
}
parts=(drivers foxglove vins dds depth map planner executive rosbag)
titles=(Drivers Foxglove sqrtVINS DDS LAS 'ROG map' "${planner^^}" 'Mission executive' 'Record bag — press Enter')
first=$(tmux new-session -d -P -F '#{pane_id}' -s "$session" -n system "bash --noprofile --norc -i")
tmux set-option -t "$session" mouse on
tmux set-option -t "$session" remain-on-exit off
tmux set-option -t "$session" pane-border-status top
tmux select-pane -t "$first" -T "${titles[0]}"
tmux send-keys -t "$first" -l "$(pane drivers)"
tmux send-keys -t "$first" Enter
for ((i=1; i<${#parts[@]}; ++i)); do
  created=$(tmux split-window -d -P -F '#{pane_id}' -t "$session:system" "bash --noprofile --norc -i")
  tmux select-pane -t "$created" -T "${titles[i]}"
  tmux send-keys -t "$created" -l "$(pane "${parts[i]}")"
  if [[ ${parts[i]} != rosbag ]]; then tmux send-keys -t "$created" Enter; fi
  tmux select-layout -t "$session:system" tiled >/dev/null
done
tmux select-pane -t "$first"
exec tmux attach-session -t "$session"
