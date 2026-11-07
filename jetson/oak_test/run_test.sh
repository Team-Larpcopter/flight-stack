#!/usr/bin/env bash
# Larpcopter larp_oak pipeline test, OUTSIDE the skybeta stack (run on the Jetson host).
#
#   (stop the skybeta stack first: sudo systemctl stop skybeta-stack)
#   jetson/oak_test/run_test.sh start      start the test container:
#                                          tmux "oaktest": vslam (larp_oak + cuVSLAM + depth
#                                          filter), fox (Foxglove :8766), viz (/viz relay)
#   jetson/oak_test/run_test.sh monitor S  log rates/CPU/GPU/vo_state for S seconds
#   jetson/oak_test/run_test.sh stop       remove the test container
#   (then: sudo systemctl start skybeta-stack, and power-cycle the FC)
#
# Logs: ~/team_ws/oak_test/{launch,fox,viz}.log and monitor_*.jsonl
# No PX4 bridge or flight interface runs in this test: camera, VIO and depth only.
set -euo pipefail
NAME=larp-oaktest
TAG="$(tr -d '[:space:]' < "$HOME/skybeta/VERSION")"
IMAGE="skybeta-team:${TAG}"
LOGS="$HOME/team_ws/oak_test"
DDS="$HOME/skybeta/host/dds/fastdds_localhost.xml"
FOX_PORT="${FOX_PORT:-8766}"
in_c() { docker exec "$NAME" bash -c "source skybeta_env.sh; $*"; }
start_viz() {
  docker exec "$NAME" tmux kill-window -t oaktest:viz 2>/dev/null || true
  docker exec "$NAME" tmux new-window -d -t oaktest -n viz \
    -e "VIZ_RAW=['/oak/left/image_raw','/oak/right/image_raw']" \
    -e "VIZ_DEPTH=['/oak/stereo/image_raw','/oak/stereo/filtered/image']" \
    'bash -c "source skybeta_env.sh; ros2 run larp_viz viz_relay --ros-args \
      -p raw_topics:=\"\$VIZ_RAW\" -p depth_topics:=\"\$VIZ_DEPTH\" \
      -p rate_hz:=5.0 -p scale:=0.5 -p jpeg_quality:=80 > /root/team_ws/oak_test/viz.log 2>&1"'
}

case "${1:-}" in
  start)
    mkdir -p "$LOGS"
    [[ "$(systemctl is-active skybeta-stack)" != active ]] \
      || { echo "stop the stack first: sudo systemctl stop skybeta-stack"; exit 1; }
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d -t --name "$NAME" --privileged --runtime nvidia --network host --ipc host \
      -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb -v "$HOME/team_ws:/root/team_ws" \
      -v "$DDS:/etc/skybeta/fastdds.xml:ro" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -e ROS_DOMAIN_ID="$(grep -E '^ROS_DOMAIN_ID=' /etc/skybeta/drone.env | cut -d= -f2)" \
      -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
      --entrypoint bash "$IMAGE" -c "sleep infinity" >/dev/null
    in_c "team_build.sh --packages-select larp_oak larp_viz 2>&1 | tail -2"
    : > "$LOGS/launch.log"
    docker exec "$NAME" tmux new-session -d -s oaktest -n vslam \
      "bash -c 'source skybeta_env.sh; ros2 launch larp_oak larp_vslam.launch.py 2>&1 | tee /root/team_ws/oak_test/launch.log'"
    docker exec "$NAME" tmux new-window -d -t oaktest -n fox -e "FOX_PORT=${FOX_PORT}" \
      -e "FOX_TOPICS=['^(/viz/.*|(?!.*/image).*)\$']" -e "FOX_NONE=['^\$']" \
      'bash -c "source skybeta_env.sh; ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        port:=\$FOX_PORT address:=0.0.0.0 capabilities:=\"[connectionGraph,assets]\" \
        send_buffer_limit:=131072 min_qos_depth:=1 max_qos_depth:=5 \
        topic_whitelist:=\"\$FOX_TOPICS\" client_topic_whitelist:=\"\$FOX_NONE\" \
        service_whitelist:=\"\$FOX_NONE\" param_whitelist:=\"\$FOX_NONE\" > /root/team_ws/oak_test/fox.log 2>&1"'
    start_viz
    echo "started $NAME on $IMAGE; Foxglove ws://<drone>:${FOX_PORT}"
    ;;
  viz) start_viz; echo "viz relay restarted" ;;
  monitor)
    in_c "python3 -u /root/team_ws/src/flight-stack/jetson/oak_test/monitor.py ${2:-300} 10 /root/team_ws/oak_test" \
      | tee "$LOGS/monitor_$(date +%H%M%S).jsonl"
    ;;
  stop)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    echo "removed $NAME. Now: sudo systemctl start skybeta-stack"
    ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
