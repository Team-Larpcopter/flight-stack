#!/usr/bin/env bash
# One command per part of the native stack, from this checkout (after bash jetson/build.sh).
# Each starts its part in the BACKGROUND and returns; running a part again restarts it.
#
#   jetson/stack.sh camera [las|off]
#       OAK stereo (1280x800 undistorted, for sqrtVINS) + IMU + the camera's rectified
#       800x500 pair, in container $CONTAINER (flight-stack-localization, created with
#       jetson/create_localization_container.sh if missing). With las (default) also
#       Lite Any Stereo depth on the GPU in container $LAS_CONTAINER (flight-stack-las,
#       same image and checkout, --runtime nvidia, same ROS domain / DDS profile)
#       -> /las_depth/* and the planner's /stereo/depth/image. LAS starts only once the
#       camera node is up.
#   jetson/stack.sh vio [CALIBRATION_DIR]   sqrtVINS (bundled OAK-D S2 calibration by default)
#   jetson/stack.sh fox [--tracks]          Foxglove on ws://<drone-ip>:8766 (read-only, throttled
#                                           /viz copies, newest only); --tracks adds sqrtVINS's
#                                           feature-track image (costs it a few Hz)
#   jetson/stack.sh status                  which parts run, last log line of each
#   jetson/stack.sh logs PART               follow a part's log (camera, las, vio, fox)
#   jetson/stack.sh stop [PART]             stop one part (camera also stops las), or all
#
# LAS runs on NVIDIA's public Isaac ROS Humble image for Jetson (ROS 2 Humble, CUDA,
# TensorRT 10.3 - the version the saved engine was built with - and PyTorch); it is the
# base of the skybeta images, so it is normally already on the drone (pulled if not).
# Logs: log/stack/<part>.log in this checkout.
# Env: CONTAINER, LAS_CONTAINER, BUILD_IMAGE (larp-flight-stack:humble), LAS_IMAGE,
# LAS_ENGINE (the saved engine in src/larp_liteanystereo/models), ROS_DOMAIN_ID / DDS_PROFILE
# (only used when the localization container is created).
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
container=${CONTAINER:-flight-stack-localization}
las_container=${LAS_CONTAINER:-flight-stack-las}
image=${BUILD_IMAGE:-larp-flight-stack:humble}
las_image=${LAS_IMAGE:-nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c}
las_engine=${LAS_ENGINE:-/flight-stack/src/larp_liteanystereo/models/las2_m_320x480_d192_fp16.engine}
logs="$root/log/stack"
ENV='set +u; source /opt/ros/humble/setup.bash; source /flight-stack/install/setup.bash
export LD_LIBRARY_PATH=/flight-stack/sdk_install/lib:${LD_LIBRARY_PATH:-}'

where() { [[ $1 == las ]] && echo "$las_container" || echo "$container"; }
is_running() { [[ $(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null) == true ]]; }
owned() {  # refuse containers that belong to another checkout
  local m; m=$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/flight-stack"}}{{.Source}}{{end}}{{end}}' "$1")
  [[ $m == "$root" ]] || { echo "Container $1 belongs to another checkout ($m); set CONTAINER / LAS_CONTAINER." >&2; exit 2; }
}
ensure_localization() {
  [[ -f $root/install/setup.bash ]] || { echo 'Run bash jetson/build.sh first' >&2; exit 2; }
  docker inspect "$container" >/dev/null 2>&1 || CONTAINER="$container" BUILD_IMAGE="$image" bash "$root/jetson/create_localization_container.sh" >/dev/null
  owned "$container"
  is_running "$container" || docker start "$container" >/dev/null
}
ensure_las() {
  ensure_localization
  if ! docker image inspect "$las_image" >/dev/null 2>&1; then
    echo "pulling $las_image (one time)..."
    docker pull "$las_image"
  fi
  if ! docker inspect "$las_container" >/dev/null 2>&1; then
    # Same ROS domain and DDS profile as the localization container.
    local env dds=()
    env=$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$container")
    local domain; domain=$(grep -m1 '^ROS_DOMAIN_ID=' <<<"$env" | cut -d= -f2)
    local profile; profile=$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/etc/flight-stack/fastdds.xml"}}{{.Source}}{{end}}{{end}}' "$container")
    [[ -n $profile ]] && dds=(-v "$profile:/etc/flight-stack/fastdds.xml:ro"
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/fastdds.xml -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/flight-stack/fastdds.xml)
    docker create --init --name "$las_container" --runtime nvidia --network host --ipc host \
      -v "$root:/flight-stack" -w /flight-stack -e ROS_DOMAIN_ID="${domain:-0}" \
      -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 "${dds[@]}" "$las_image" sleep infinity >/dev/null
    echo "created $las_container (GPU, ROS_DOMAIN_ID=${domain:-0})"
  fi
  owned "$las_container"
  is_running "$las_container" || docker start "$las_container" >/dev/null
}
running() {
  local c; c=$(where "$1")
  is_running "$c" && docker exec "$c" bash -c "p=\$(cat /tmp/stack-$1.pid 2>/dev/null) && kill -0 \$p 2>/dev/null" 2>/dev/null
}
stop_part() {  # INT the part's whole process group, then TERM/KILL what is left
  running "$1" || return 0
  docker exec "$(where "$1")" bash -c "g=\$(cat /tmp/stack-$1.pid); kill -INT -- -\$g 2>/dev/null
    for i in \$(seq 1 20); do kill -0 \$g 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- -\$g 2>/dev/null; sleep 1; kill -KILL -- -\$g 2>/dev/null; rm -f /tmp/stack-$1.pid" || true
  echo "stopped $1"
}
start_part() {  # part, command; own process group (pid file), output to its log
  stop_part "$1"
  mkdir -p "$logs"
  docker exec -d "$(where "$1")" setsid bash -c "echo \$\$ > /tmp/stack-$1.pid; exec > /flight-stack/log/stack/$1.log 2>&1
$ENV
$2"
}
show() { sleep "$2"; echo "--- $1 (log: $logs/$1.log)"; tail -n "${3:-3}" "$logs/$1.log" 2>/dev/null | cut -c1-200
  running "$1" || { echo "!! $1 is not running: see the log"; return 1; }; }

case "${1:-}" in
  camera)
    mode=${2:-las}
    [[ $mode =~ ^(las|off)$ ]] || { echo 'depth must be las or off' >&2; exit 2; }
    ensure_localization
    stop_part las 2>/dev/null || true
    rect=false; [[ $mode == las ]] && rect=true
    start_part camera "mkdir -p /flight-stack/evidence; run=\$(mktemp -d /flight-stack/evidence/localization-XXXXXXXX)
echo \"camera (depth: $mode). Calibration export: \$run\"
exec ros2 run larp_hardware oak_native_node --ros-args -p connect_device:=true -p publish_images:=true \
  -p align_imu:=true -p factory_calibration_out:=\$run/factory.json -p accel_hz:=500 -p gyro_hz:=400 \
  -p max_exposure_us:=5000 -p capture_seconds:=0.0 -p publish_rectified:=$rect"
    # LAS only on a running camera (a depth node on someone else's camera gets no rectified pair).
    for _ in $(seq 1 40); do
      running camera || { show camera 0 6; echo '!! camera node exited (camera owned by another process?)'; exit 1; }
      grep -q -E 'Rectified pair on the camera|\[oak_native\]' "$logs/camera.log" 2>/dev/null && break
      sleep 0.5
    done
    show camera 2 || exit 1
    if [[ $mode == las ]]; then
      ensure_las
      docker exec "$las_container" test -f "$las_engine" || { echo "engine missing: $las_engine" >&2; exit 1; }
      start_part las "exec ros2 launch larp_liteanystereo las_depth.launch.py engine_path:=$las_engine publish_as_primary:=true"
      show las 12
    fi
    ;;
  vio)
    ensure_localization
    cal=${2:-${CALIBRATION_DIR:-/flight-stack/src/larp_localization/config/oak_d_s2}}
    start_part vio "exec ros2 run ov_srvins run_subscribe_msckf --ros-args -r __ns:=/autonomy \
  -r /sim/processed/vio:=/autonomy/processed_vio -p config_path:=$cal/estimator_config.yaml \
  -p verbosity:=\${VIO_VERBOSITY:-INFO}"
    show vio 8
    ;;
  fox)
    ensure_localization
    tracks=false; [[ ${2:-} == --tracks ]] && tracks=true
    start_part fox "exec ros2 launch larp_perception perception.launch.py viz_right:=true viz_tracks:=$tracks"
    show fox 8 2
    echo "Foxglove: ws://<drone-ip>:8766, layout jetson/foxglove/larpcopter-autonomy-localization.json"
    ;;
  status)
    for part in camera las vio fox; do
      if running "$part"; then state=running; else state=stopped; fi
      printf '%-7s %-8s %s\n' "$part" "$state" "$(tail -n 1 "$logs/$part.log" 2>/dev/null | cut -c1-140)"
    done
    ;;
  logs) [[ -n ${2:-} ]] || { echo 'stack.sh logs camera|las|vio|fox' >&2; exit 2; }; tail -n 30 -f "$logs/$2.log" ;;
  stop)
    case ${2:-all} in
      all) for part in las camera vio fox; do stop_part "$part"; done ;;
      camera) stop_part las; stop_part camera ;;
      *) stop_part "$2" ;;
    esac
    ;;
  *) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 2 ;;
esac
