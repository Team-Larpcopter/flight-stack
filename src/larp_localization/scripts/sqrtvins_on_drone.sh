#!/usr/bin/env bash
# Run sqrtVINS on the drone next to the skybeta stack, in its own container.
# Run on the Jetson HOST. sqrtVINS + larp_localization must be built in
# ~/team_ws/sqrtvins_ws (see README); the camera topics come from the stack
# (larp_oak in the slam window).
#
#   sqrtvins_on_drone.sh start     start container larp-sqrtvins (undistort + sqrtVINS)
#   sqrtvins_on_drone.sh logs      follow its log
#   sqrtvins_on_drone.sh status    rates of /ov_srvins/odomimu + CPU of each process
#   sqrtvins_on_drone.sh stop      remove the container (the flight stack is untouched)
#   sqrtvins_on_drone.sh camera    camera node ALONE (container larp-camera), for when the
#                                  stack is not running. Stop it before starting the stack.
#   sqrtvins_on_drone.sh camerastop
#   sqrtvins_on_drone.sh bag DIR   record a test bag (container larp-bag) into
#                                  ~/team_ws/sqrtvins_ws/results/DIR/bag (mcap)
#   sqrtvins_on_drone.sh bagstop   stop the recording cleanly
#
# Images: larp_oak undistorts on the camera (/oak/*/image_undistorted). Set
# SQRTVINS_HOST_UNDISTORT=true to run the old host-side Python undistorter instead
# (only if larp_oak runs with vio_undistorted:=false). The config is always
# config/oak_d_s2/estimator_config.yaml, read from src so edits need no rebuild.
#
# It joins the stack's DDS domain with the same local-only profile, so nothing
# goes over Wi-Fi, and it never publishes to /fmu or /skybeta.
set -euo pipefail
NAME=larp-sqrtvins
TAG="$(tr -d '[:space:]' < "$HOME/skybeta/VERSION")"
IMAGE="${IMAGE:-skybeta-team:${TAG}}"
WS="$HOME/team_ws/sqrtvins_ws"
DDS="$HOME/skybeta/host/dds/fastdds_localhost.xml"
DOMAIN="$(grep -E '^ROS_DOMAIN_ID=' /etc/skybeta/drone.env | cut -d= -f2)"

case "${1:-}" in
  start)
    [[ -f "$WS/install/setup.bash" ]] || { echo "not built: see README (Building on the drone)"; exit 1; }
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    # Capped so it can never starve the flight stack (sqrtVINS is single-threaded by design).
    docker run -d --name "$NAME" --network host --ipc host --cpus=2 --memory=1536m \
      -v "$WS:/root/sqrtvins_ws" -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      --entrypoint bash "$IMAGE" -c \
      "set +u; source /opt/ros/humble/setup.bash; source /root/sqrtvins_ws/install/setup.bash; \
       exec stdbuf -o0 ros2 launch larp_localization sqrtvins.launch.py undistort:=${SQRTVINS_HOST_UNDISTORT:-false} config:=/root/sqrtvins_ws/src/larp_localization/config/oak_d_s2/estimator_config.yaml" >/dev/null
    echo "started $NAME (keep the drone still for a few seconds while it initialises)"
    ;;
  logs) docker logs -f --tail 50 "$NAME" ;;
  status)
    docker exec "$NAME" bash -c "set +u; source /opt/ros/humble/setup.bash; source /root/sqrtvins_ws/install/setup.bash; \
      timeout 8 ros2 topic hz /ov_srvins/odomimu --window 50 2>/dev/null | grep -m1 average || echo 'odomimu: no messages yet'; \
      timeout 8 ros2 topic hz /oak/left/image_undistorted --window 50 2>/dev/null | grep -m1 average || echo 'undistorted: no messages'"
    docker top "$NAME" -eo pid,pcpu,rss,args | awk 'NR==1 || /run_subscribe|undistort/' | cut -c1-110
    ;;
  stop) docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "not running" ;;
  camera)
    docker ps --format '{{.Names}}' | grep -qx skybeta-stack && { echo "the stack is running: it already runs the camera"; exit 1; }
    docker rm -f larp-camera >/dev/null 2>&1 || true
    docker run -d --name larp-camera --privileged --network host --ipc host \
      -v /dev/bus/usb:/dev/bus/usb -v "$HOME/team_ws:/root/team_ws" -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      --entrypoint bash "$IMAGE" -c \
      "set +u; source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/setup.bash; \
       source /root/team_ws/install/setup.bash; exec ros2 run larp_oak oak_node --ros-args -r __node:=oak" >/dev/null
    echo "started larp-camera (logs: docker logs -f larp-camera)"
    ;;
  camerastop) docker rm -f larp-camera >/dev/null 2>&1 && echo "stopped larp-camera" || echo "not running" ;;
  bag)
    DIR="${2:?usage: bag DIR}"
    docker rm -f larp-bag >/dev/null 2>&1 || true
    # Everything needed to replay sqrtVINS offline (and to see what cuVSLAM did).
    # Raw images are left out: 4 full-size streams would load the Jetson too much.
    TOPICS="/oak/left/image_undistorted /oak/right/image_undistorted \
      /oak/left/image_undistorted/camera_info /oak/right/image_undistorted/camera_info \
      /oak/left/camera_info /oak/right/camera_info /oak/imu/data /tf_static \
      /ov_srvins/odomimu /visual_slam/tracking/odometry"
    docker run -d --name larp-bag --network host --ipc host --cpus=1 --memory=768m \
      -v "$WS:/root/sqrtvins_ws" -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      --entrypoint bash "$IMAGE" -c \
      "set +u; source /opt/ros/humble/setup.bash; mkdir -p /root/sqrtvins_ws/results/$DIR; \
       exec ros2 bag record -s mcap -o /root/sqrtvins_ws/results/$DIR/bag $TOPICS" >/dev/null
    echo "recording bag -> results/$DIR/bag"
    ;;
  bagstop)
    docker kill -s INT larp-bag >/dev/null 2>&1 || { echo "no bag running"; exit 0; }
    for _ in $(seq 1 20); do docker ps -q -f name=larp-bag | grep -q . || break; sleep 0.5; done
    docker logs larp-bag 2>&1 | grep -i -E "error|warn" | tail -3 || true
    docker rm -f larp-bag >/dev/null 2>&1 || true
    ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
