#!/usr/bin/env bash
# Build and run this branch's camera/depth/Foxglove code on the drone as an overlay
# on the shared integration workspace, without modifying it. Run on the Jetson HOST.
#
#   depth_ws.sh build            build larp_hardware, larp_perception, larp_autonomy
#                                into $DEPTH_WS/install (on top of $INTEGRATION/install_clean)
#   depth_ws.sh start            start container larp-depth (USB, DDS profile, ROS domain)
#   depth_ws.sh pane NAME [ARGS] run localization_pane.sh NAME in it (camera, vio,
#                                monitor, perception, bench), using the overlay
#   depth_ws.sh tool FILE [ARGS] run a larp_perception tool (e.g. depth_calibration.py)
#   depth_ws.sh shell            bash in the container with everything sourced
#   depth_ws.sh stop             remove the container
#
# DEPTH_WS (default ~/depth_ws) holds src/ and jetson/ of this branch;
# INTEGRATION (default ~/flight_integration_ws_20261107) is the shared workspace
# (install_clean, sdk_install, evidence/). The camera is exclusive: stop any other
# camera owner (larp-localization-ready camera pane, larp-camera, the skybeta stack) first.
set -euo pipefail
DEPTH_WS="${DEPTH_WS:-$HOME/depth_ws}"
INTEGRATION="${INTEGRATION:-$HOME/flight_integration_ws_20261107}"
IMAGE="${IMAGE:-larp-integration:20261107}"
DDS="${DDS_PROFILE:-$HOME/skybeta/host/dds/fastdds_localhost.xml}"
DOMAIN="${ROS_DOMAIN_ID:-$(grep -E '^ROS_DOMAIN_ID=' /etc/skybeta/drone.env 2>/dev/null | cut -d= -f2)}"
NAME=larp-depth
ENV_SETUP='source /opt/ros/humble/setup.bash; [[ -f /root/skybeta_ws/install/local_setup.bash ]] && source /root/skybeta_ws/install/local_setup.bash; source /integration/install_clean/setup.bash; [[ -f /depth_ws/install/setup.bash ]] && source /depth_ws/install/setup.bash; export LD_LIBRARY_PATH=/integration/sdk_install/lib:${LD_LIBRARY_PATH:-}'

case "${1:-}" in
  build)
    docker run --rm --memory=4g -v "$INTEGRATION:/integration:ro" -v "$DEPTH_WS:/depth_ws" \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; cd /depth_ws; \
        export CMAKE_PREFIX_PATH=/integration/sdk_install:\${CMAKE_PREFIX_PATH:-}; \
        MAKEFLAGS=-j2 nice -n 10 colcon build --base-paths src --build-base build --install-base install \
          --packages-select larp_hardware larp_perception larp_neural_depth larp_autonomy \
          --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    ;;
  start)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    mkdir -p "$HOME/neural_depth_ws/models"
    docker run -d --name "$NAME" --init --network host --ipc host --runtime nvidia \
      --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
      -v "$INTEGRATION:/integration" -v "$DEPTH_WS:/depth_ws" \
      -v "$HOME/neural_depth_ws/models:/models" \
      -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" -e WORKSPACE=/integration -e OVERLAY_PREFIX=/depth_ws/install \
      -e PERCEPTION_SRC=/depth_ws/src/larp_perception -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
      --entrypoint sleep "$IMAGE" infinity >/dev/null
    echo "started $NAME (ROS_DOMAIN_ID=$DOMAIN)"
    ;;
  pane) shift; docker exec -it "$NAME" bash /depth_ws/jetson/integration_image/localization_pane.sh "$@" ;;
  tool) shift; f="$1"; shift
        docker exec -it "$NAME" bash -c "set +u; $ENV_SETUP; python3 /depth_ws/src/larp_perception/tools/$f $*" ;;
  shell) docker exec -it "$NAME" bash -c "set +u; $ENV_SETUP; exec bash" ;;
  stop) docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "not running" ;;
  *) sed -n '2,19p' "$0"; exit 2 ;;
esac
