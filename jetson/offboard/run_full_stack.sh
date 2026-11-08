#!/usr/bin/env bash
# Foreground full-stack container. Ctrl+C stops the ROS launch and agent.
set -euo pipefail
if [[ ${1:-} == --help || $# == 0 ]]; then
  echo "Usage: $0 shadow|hardware [ROS launch name:=value ...]"
  echo 'Required: rig_file, boundary_config, goal_x/y/z. Hardware also needs bridge_params, alignment_verified:=true, mission_id.'
  echo 'Set DEPLOYMENT_CONFIG; configuration paths are under /config in the image.'
  echo 'Environment: ROS_DOMAIN_ID, PX4_TRANSPORT=udp4|serial, PX4_PORT=8888, PX4_DEVICE, PX4_BAUD=921600, BUILD_IMAGE, CONTAINER.'
  exit 0
fi
mode=$1; shift
[[ $mode == shadow || $mode == hardware ]] || { echo 'Select shadow or hardware' >&2; exit 2; }
# Prevent later CLI arguments from overriding the top-level mode or transport.
for arg in "$@"; do
  case $arg in mode:=*|agent_transport:=*|agent_device:=*|agent_port:=*|agent_baud:=*) echo "Set mode/transport through the documented interface: $arg" >&2; exit 2;; esac
done
: "${DEPLOYMENT_CONFIG:?Set DEPLOYMENT_CONFIG to the configuration directory}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
[[ $ROS_DOMAIN_ID =~ ^[0-9]+$ ]] && ((ROS_DOMAIN_ID <= 232)) || { echo 'Invalid ROS_DOMAIN_ID' >&2; exit 2; }
transport=${PX4_TRANSPORT:-udp4}
port=${PX4_PORT:-8888}; baud=${PX4_BAUD:-921600}
[[ $port =~ ^[0-9]+$ && $baud =~ ^[0-9]+$ ]] && ((port > 0 && port <= 65535 && baud > 0)) || { echo 'Invalid port/baud' >&2; exit 2; }
devices=()
case $transport in
  udp4) ;;
  serial)
    : "${PX4_DEVICE:?Set PX4_DEVICE to the host UART/USB serial device}"
    [[ -c $PX4_DEVICE ]] || { echo "Not a serial character device: $PX4_DEVICE" >&2; exit 2; }
    devices=(--device "$PX4_DEVICE:/dev/px4")
    ;;
  *) echo 'PX4_TRANSPORT must be udp4 or serial' >&2; exit 2;;
esac
dds=()
if [[ -n ${DDS_PROFILE:-} ]]; then
  profile=$(realpath "$DDS_PROFILE")
  [[ -f $profile ]] || { echo 'DDS_PROFILE must be a file' >&2; exit 2; }
  dds=(-v "$profile:/etc/flight-stack/dds.xml:ro"
       -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml
       -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/flight-stack/dds.xml)
fi
tty=(); if [[ -t 0 && -t 1 ]]; then tty=(-it); fi
exec docker run --rm --init "${tty[@]}" --name "${CONTAINER:-flight-stack-runtime}" \
  --runtime nvidia --network host --ipc host --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb "${devices[@]}" "${dds[@]}" \
  -v "$(realpath "$DEPLOYMENT_CONFIG"):/config:ro" -e ROS_DOMAIN_ID \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
  "${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}" \
  ros2 launch larp_autonomy full_stack.launch.py \
  static_cad_only:=false start_depth:=true start_visualization:=true \
  rig_file:=/config/rig.json boundary_config:=/config/boundary.yaml \
  "mode:=$mode" "agent_transport:=$transport" "agent_port:=$port" \
  "agent_device:=/dev/px4" "agent_baud:=$baud" "$@"
