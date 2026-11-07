#!/usr/bin/env bash
# One command per part of the native stack. Run on the drone (Jetson host); each
# runs in the foreground (Ctrl-C stops it), in container larp-depth (started if needed).
#
#   stack.sh camera [camera|neural|both|off]
#       OAK stereo + IMU, plus depth:
#         camera  (default) depth computed on the OAK, filtered on the Jetson GPU
#         neural  Fast-FoundationStereo (TensorRT) on the Jetson from the stereo pair
#         both    both side by side (camera depth stays the primary)
#         off     no depth
#       Depth for the planner is always /stereo/depth/image.
#   stack.sh vio [CALIBRATION_DIR]
#       sqrtVINS. Default calibration: CALIBRATION_DIR env, else the newest
#       generated one in the evidence folder. Keep the drone still while it starts.
#   stack.sh fox [--tracks]
#       Foxglove on ws://<drone-ip>:8766 with every stream (both cameras, filtered,
#       raw and neural depth, pose, path, IMU, feature clouds), throttled, newest only.
#       --tracks adds sqrtVINS's feature-track image (costs sqrtVINS a few Hz).
#       Layout: jetson/foxglove/larpcopter-autonomy-localization.json
#   stack.sh stop          stop everything (removes container larp-depth)
#
# Paths (override with env): DEPTH_WS ~/depth_ws (this branch, built with
# depth_ws.sh build), INTEGRATION ~/flight_integration_ws_20261107, NEURAL_WS
# ~/neural_depth_ws (larp_neural_depth build + models/), NEURAL_ENGINE
# /models/fast_foundationstereo_320x480_fp16.engine.
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
DEPTH_WS="${DEPTH_WS:-$HOME/depth_ws}"
INTEGRATION="${INTEGRATION:-$HOME/flight_integration_ws_20261107}"
NEURAL_WS="${NEURAL_WS:-$HOME/neural_depth_ws}"
IMAGE="${IMAGE:-larp-integration:20261107}"
DDS="${DDS_PROFILE:-$HOME/skybeta/host/dds/fastdds_localhost.xml}"
DOMAIN="${ROS_DOMAIN_ID:-$(grep -E '^ROS_DOMAIN_ID=' /etc/skybeta/drone.env 2>/dev/null | cut -d= -f2)}"
ENGINE="${NEURAL_ENGINE:-/models/fast_foundationstereo_320x480_fp16.engine}"
NAME=larp-depth
ENV='set +u; source /opt/ros/humble/setup.bash
[[ -f /root/skybeta_ws/install/local_setup.bash ]] && source /root/skybeta_ws/install/local_setup.bash
source /integration/install_clean/setup.bash
[[ -f /depth_ws/install/setup.bash ]] && source /depth_ws/install/setup.bash
[[ -f /neural_ws/install/setup.bash ]] && source /neural_ws/install/setup.bash
export LD_LIBRARY_PATH=/integration/sdk_install/lib:${LD_LIBRARY_PATH:-}'

ensure_container() {
  [[ $(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null) == true ]] && return
  mkdir -p "$NEURAL_WS/models"
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  docker run -d --name "$NAME" --init --network host --ipc host --runtime nvidia \
    --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
    -v "$INTEGRATION:/integration" -v "$DEPTH_WS:/depth_ws" \
    -v "$NEURAL_WS:/neural_ws" -v "$NEURAL_WS/models:/models" \
    -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
    -e ROS_DOMAIN_ID="$DOMAIN" -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
    --entrypoint sleep "$IMAGE" infinity >/dev/null
  echo "started container $NAME (ROS_DOMAIN_ID=$DOMAIN)"
}
# run a script in the container, foreground; Ctrl-C stops everything it started
in_container() { docker exec -it "$NAME" bash -c "$ENV
trap 'kill -INT -- \$(jobs -p) 2>/dev/null; sleep 2; kill -- \$(jobs -p) 2>/dev/null' INT TERM EXIT
$1"; }

case "${1:-}" in
  camera)
    mode="${2:-camera}"
    [[ $mode =~ ^(camera|neural|both|off)$ ]] || { echo "depth must be camera, neural, both or off"; exit 2; }
    ensure_container
    if [[ $mode == neural || $mode == both ]]; then
      docker exec "$NAME" test -f "$ENGINE" || { echo "neural engine missing: $ENGINE (build it: neural_depth_ws.sh build-engine)"; exit 1; }
      docker exec "$NAME" test -f /neural_ws/install/setup.bash \
        || { echo "larp_neural_depth not built in $NEURAL_WS, or $NAME was started without it: $0 stop, then retry"; exit 1; }
    fi
    depth_param=true; [[ $mode == off ]] && depth_param=false
    # The OAK's own depth stays on for 'neural' too: it is free and its camera_info
    # carries the stereo baseline the neural node uses.
    filter=""; neural=""
    [[ $mode == camera || $mode == both ]] && filter="ros2 run larp_perception depth_filter &"
    [[ $mode == neural ]] && neural="ros2 launch larp_neural_depth neural_depth.launch.py engine_path:=$ENGINE publish_as_primary:=true &"
    [[ $mode == both ]] && neural="ros2 launch larp_neural_depth neural_depth.launch.py engine_path:=$ENGINE publish_as_primary:=false &"
    in_container "mkdir -p /integration/evidence; run=\$(mktemp -d /integration/evidence/localization-XXXXXXXX)
echo \"camera (depth: $mode). Calibration export: \$run\"
ros2 run larp_hardware oak_native_node --ros-args -p connect_device:=true -p publish_images:=true \
  -p align_imu:=true -p imu_calibration_file:=\${IMU_CALIBRATION_FILE:-/integration/evidence/imu_calibration.json} \
  -p factory_calibration_out:=\$run/factory.json -p accel_hz:=500 -p gyro_hz:=400 \
  -p max_exposure_us:=5000 -p capture_seconds:=0.0 -p publish_depth:=$depth_param &
sleep 3
$filter
$neural
wait -n"
    ;;
  vio)
    ensure_container
    cal="${2:-${CALIBRATION_DIR:-}}"
    if [[ -z $cal ]]; then
      cal=$(ls -td "$INTEGRATION"/evidence/*/calibration 2>/dev/null | while read -r d; do
              [[ -f $d/estimator_config.yaml ]] && { echo "$d"; break; }; done)
      [[ -n $cal ]] || { echo "no generated calibration found; pass CALIBRATION_DIR"; exit 1; }
      cal="/integration${cal#"$INTEGRATION"}"
    fi
    echo "sqrtVINS with $cal (keep the drone still for a few seconds)"
    in_container "ros2 run ov_srvins run_subscribe_msckf --ros-args -r __ns:=/autonomy \
  -r /sim/processed/vio:=/autonomy/processed_vio -p config_path:=$cal/estimator_config.yaml \
  -p verbosity:=\${VIO_VERBOSITY:-INFO} & wait -n"
    ;;
  fox)
    ensure_container
    tracks=false; [[ ${2:-} == --tracks ]] && tracks=true
    echo "Foxglove: ws://$(ip -4 -br addr show tailscale0 2>/dev/null | awk '{print $3}' | cut -d/ -f1):8766"
    in_container "ros2 launch larp_perception perception.launch.py depth_filter:=false foxglove:=true \
  viz_right:=true viz_raw_depth:=true viz_tracks:=$tracks & wait -n"
    ;;
  stop) docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "not running" ;;
  *) sed -n '2,27p' "$0"; exit 2 ;;
esac
