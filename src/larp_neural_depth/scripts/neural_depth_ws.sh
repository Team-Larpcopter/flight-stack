#!/usr/bin/env bash
# Build and run the standalone TensorRT neural stereo depth package (larp_neural_depth)
# on the Jetson WITHOUT touching ~/depth_ws, ~/flight_integration_ws_20261107, or the
# running camera/depth_filter container.
#
# Usage on the Jetson host:
#   ~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh build
#   ~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh build-engine [320] [480]
#   ~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh run [HEIGHT] [WIDTH]
#   ~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh bench [SECONDS]
#   ~/neural_depth_ws/src/larp_neural_depth/scripts/neural_depth_ws.sh stop
#
# Or from your laptop:
#   ./src/larp_neural_depth/scripts/neural_depth_ws.sh sync [dronehacks@skybeta-15]
set -euo pipefail

NEURAL_WS="${NEURAL_WS:-$HOME/neural_depth_ws}"
MODELS_DIR="${MODELS_DIR:-$NEURAL_WS/models}"
INTEGRATION="${INTEGRATION:-$HOME/flight_integration_ws_20261107}"
IMAGE="${IMAGE:-larp-integration:20261107}"
DDS="${DDS_PROFILE:-$HOME/skybeta/host/dds/fastdds_localhost.xml}"
DOMAIN="${ROS_DOMAIN_ID:-$(grep -E '^ROS_DOMAIN_ID=' /etc/skybeta/drone.env 2>/dev/null | cut -d= -f2 || echo 0)}"
NAME="larp-neural-depth"

ENV_SETUP='source /opt/ros/humble/setup.bash; [[ -f /root/skybeta_ws/install/local_setup.bash ]] && source /root/skybeta_ws/install/local_setup.bash; [[ -f /integration/install_clean/setup.bash ]] && source /integration/install_clean/setup.bash; [[ -f /neural_ws/install/setup.bash ]] && source /neural_ws/install/setup.bash'

case "${1:-}" in
  sync)
    REMOTE="${2:-dronehacks@skybeta-15}"
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    REPO_ROOT="$(cd "$PKG_DIR/../.." && pwd)"
    SSH_OPTS=()
    if [[ -S /tmp/claude-1000/cm-jet ]]; then
      SSH_OPTS=(-o ControlPath=/tmp/claude-1000/cm-jet)
    fi
    echo "Syncing larp_neural_depth + Foxglove layout to $REMOTE:~/neural_depth_ws/ ..."
    ssh "${SSH_OPTS[@]}" "$REMOTE" "mkdir -p ~/neural_depth_ws/src ~/neural_depth_ws/models ~/neural_depth_ws/foxglove"
    rsync -avz --exclude='__pycache__' --exclude='*.pyc' -e "ssh ${SSH_OPTS[*]}" \
      "$PKG_DIR" "$REMOTE:~/neural_depth_ws/src/"
    if [[ -f "$REPO_ROOT/jetson/foxglove/larpcopter-neural-depth-comparison.json" ]]; then
      rsync -avz -e "ssh ${SSH_OPTS[*]}" \
        "$REPO_ROOT/jetson/foxglove/larpcopter-neural-depth-comparison.json" \
        "$REMOTE:~/neural_depth_ws/foxglove/"
    fi
    echo "Synced cleanly to $REMOTE:~/neural_depth_ws (existing ~/depth_ws and git branches untouched)."
    ;;
  build)
    mkdir -p "$NEURAL_WS/src" "$MODELS_DIR"
    docker run --rm --memory=4g \
      -v "$INTEGRATION:/integration:ro" \
      -v "$NEURAL_WS:/neural_ws" \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; cd /neural_ws; \
        colcon build --base-paths src --build-base build --install-base install \
          --packages-select larp_neural_depth --symlink-install"
    ;;
  build-engine)
    H="${2:-320}"
    W="${3:-480}"
    mkdir -p "$MODELS_DIR"
    docker run --rm --runtime nvidia \
      -v "$NEURAL_WS:/neural_ws" \
      -v "$MODELS_DIR:/models" \
      --entrypoint bash "$IMAGE" \
      /neural_ws/src/larp_neural_depth/scripts/build_trt_engine.sh "$H" "$W" /models
    ;;
  run)
    H="${2:-320}"
    W="${3:-480}"
    PRIMARY="${4:-false}"
    ENGINE="/models/fast_foundationstereo_${H}x${W}_fp16.engine"
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -it --rm --name "$NAME" --init --network host --ipc host --runtime nvidia \
      -v "$INTEGRATION:/integration:ro" \
      -v "$NEURAL_WS:/neural_ws" \
      -v "$MODELS_DIR:/models" \
      -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      -e NEURAL_STEREO_ENGINE="$ENGINE" \
      -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; \
        exec ros2 launch larp_neural_depth neural_depth.launch.py \
          engine_path:=$ENGINE target_height:=$H target_width:=$W publish_as_primary:=$PRIMARY"
    ;;
  start)
    H="${2:-320}"
    W="${3:-480}"
    PRIMARY="${4:-false}"
    ENGINE="/models/fast_foundationstereo_${H}x${W}_fp16.engine"
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" --init --network host --ipc host --runtime nvidia \
      -v "$INTEGRATION:/integration:ro" \
      -v "$NEURAL_WS:/neural_ws" \
      -v "$MODELS_DIR:/models" \
      -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      -e NEURAL_STEREO_ENGINE="$ENGINE" \
      -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; \
        exec ros2 launch larp_neural_depth neural_depth.launch.py \
          engine_path:=$ENGINE target_height:=$H target_width:=$W publish_as_primary:=$PRIMARY" >/dev/null
    echo "Started $NAME in background (publish_as_primary=$PRIMARY; logs: docker logs -f $NAME)"
    ;;
  bench)
    SECONDS_RUN="${2:-15}"
    docker exec -it larp-depth bash -c "set +u; source /opt/ros/humble/setup.bash; \
      python3 /depth_ws/src/larp_perception/tools/bench.py $SECONDS_RUN --topics \
        /native_oak/left/image_undistorted \
        /native_oak/depth/image_raw \
        /stereo/depth/image \
        /neural_depth/depth/image_raw \
        /viz/stereo/depth/image/compressed \
        /viz/neural_depth/depth/image/compressed"
    ;;
  stop)
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "Stopped $NAME" || echo "$NAME not running"
    ;;
  *)
    sed -n '2,15p' "$0"
    exit 2
    ;;
esac
