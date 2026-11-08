#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ -f $here/deployment.env ]]; then source "$here/deployment.env"; fi
check_only=false
case ${1:-} in
  --check) check_only=true ;;
  --help|-h)
    echo 'Usage: bash launch_fixed_course_tmux.sh [--check]'
    echo 'Six panes including manual recorder: pilot arms then selects Position; 1.0 m takeoff, fixed forward/right/tunnel/goal route, land. No depth, map or planner.'
    echo '--check validates setup without launching sensors or flight publishers.'
    exit 0 ;;
  '') ;;
  *) echo 'Unknown argument; use --help' >&2; exit 2 ;;
esac
mode=hardware
: "${DEPLOYMENT_CONFIG:?Set DEPLOYMENT_CONFIG or provide deployment.env next to this script}"
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
container=${CONTAINER:-flight-stack-fixed-course}
session=${SESSION:-flight-stack-fixed-course}
command -v tmux >/dev/null
if ! $check_only; then source "$here/replace_stack.sh"; fi
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
  dds+=(-v "$(realpath "$XRCE_REFS"):/etc/flight-stack/agent_refs.xml:ro"
        -e XRCE_REFS=/etc/flight-stack/agent_refs.xml)
fi
if [[ -n ${DDS_PROFILE:-} ]]; then
  dds+=(-v "$(realpath "$DDS_PROFILE"):/etc/flight-stack/dds.xml:ro"
       -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml
       -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml)
fi
docker run --rm --network none "${runtime_mounts[@]}" -v "$config:/config:ro" "$image" \
  python3 -c 'import sys; from larp_autonomy.runtime_config import load_config; load_config("/config/mission.yaml",sys.argv[1])' "$mode"
if $check_only; then
  docker run --rm --network none "${runtime_mounts[@]}" "$image" bash -ec '
    test -x /flight-stack/install/flight_executive/lib/flight_executive/flight_executive_node
    test -x /flight-stack/install/flight_trajectory/lib/flight_trajectory/course_trajectory_node
    test -f /flight-stack/install/larp_autonomy/share/larp_autonomy/launch/fixed_course.launch.py
    python3 -c "import flight_trajectory.fixed_course"
    command -v MicroXRCEAgent >/dev/null
    python3 -c "import flight_executive.flight_executive_node"
  '
  echo 'Configuration and executable checks passed. No sensors or flight publishers launched.'
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
  if [[ $1 == executive ]]; then
    printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
      ros2 launch larp_autonomy fixed_course.launch.py mission_file:=/config/mission.yaml
  else
    printf '%q ' docker exec -it "$container" /flight-stack/jetson/offboard/entrypoint.sh \
      ros2 launch larp_autonomy mission_runtime.launch.py "component:=$1" \
      "mode:=$mode" mission_file:=/config/mission.yaml
  fi
}
parts=(drivers foxglove vins dds executive rosbag)
titles=(Drivers Foxglove sqrtVINS DDS 'Fixed course and land' 'Record bag — press Enter')
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
