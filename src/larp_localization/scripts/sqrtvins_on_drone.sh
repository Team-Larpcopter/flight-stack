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
    docker run -d --name "$NAME" --network host --ipc host \
      -v "$WS:/root/sqrtvins_ws" -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$DOMAIN" \
      --entrypoint bash "$IMAGE" -c \
      "set +u; source /opt/ros/humble/setup.bash; source /root/sqrtvins_ws/install/setup.bash; \
       exec stdbuf -o0 ros2 launch larp_localization sqrtvins.launch.py" >/dev/null
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
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
