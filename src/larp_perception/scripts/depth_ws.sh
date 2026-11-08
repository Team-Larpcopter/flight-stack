#!/usr/bin/env bash
# Build this branch on the drone as an overlay on the shared integration workspace,
# without modifying it. Run on the Jetson HOST.
#
#   git clone -b <branch> https://github.com/Team-Larpcopter/flight-stack.git ~/depth_ws
#   ~/depth_ws/src/larp_perception/scripts/depth_ws.sh build
#
#   depth_ws.sh build            build larp_hardware (camera node), larp_perception, larp_localization,
#                                larp_liteanystereo and larp_autonomy into
#                                $DEPTH_WS/install, on top of $INTEGRATION/install_clean
#                                (DepthAI SDK, ov_srvins, messages)
#   depth_ws.sh test             run the Python package tests in the build image
#   depth_ws.sh tool FILE [ARGS] run a larp_perception tool (e.g. bench.py 20) in container larp-depth
#
# Then run things with stack.sh (camera / vio / fox).
set -euo pipefail
DEPTH_WS="${DEPTH_WS:-$HOME/depth_ws}"
INTEGRATION="${INTEGRATION:-$HOME/flight_integration_ws_20261107}"
IMAGE="${IMAGE:-larp-integration:20261107}"
ENV_SETUP='source /opt/ros/humble/setup.bash; [[ -f /root/skybeta_ws/install/local_setup.bash ]] && source /root/skybeta_ws/install/local_setup.bash; source /integration/install_clean/setup.bash; [[ -f /depth_ws/install/setup.bash ]] && source /depth_ws/install/setup.bash; export LD_LIBRARY_PATH=/integration/sdk_install/lib:${LD_LIBRARY_PATH:-}'

case "${1:-}" in
  build)
    docker run --rm --memory=4g -v "$INTEGRATION:/integration:ro" -v "$DEPTH_WS:/depth_ws" \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; cd /depth_ws; \
        export CMAKE_PREFIX_PATH=/integration/sdk_install:\${CMAKE_PREFIX_PATH:-}; \
        MAKEFLAGS=-j2 nice -n 10 colcon build --base-paths src --build-base build --install-base install \
          --packages-select larp_hardware larp_perception larp_liteanystereo larp_localization larp_autonomy \
          --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    ;;
  test)
    docker run --rm --memory=2g -v "$INTEGRATION:/integration:ro" -v "$DEPTH_WS:/depth_ws" \
      --entrypoint bash "$IMAGE" -c "set +u; $ENV_SETUP; cd /depth_ws/src; \
        for p in larp_perception larp_liteanystereo larp_autonomy; do (cd \$p && PYTHONPATH=\$PWD:\${PYTHONPATH:-} python3 -m pytest -q test | tail -1); done"
    ;;
  tool) shift; f="$1"; shift
        docker exec -it larp-depth bash -c "set +u; $ENV_SETUP; python3 /depth_ws/src/larp_perception/tools/$f $*" ;;
  *) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 2 ;;
esac
