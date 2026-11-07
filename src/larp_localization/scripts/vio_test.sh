#!/usr/bin/env bash
# One-command VIO test on the drone: fresh sqrtVINS start, record sqrtVINS and
# cuVSLAM side by side, print drift and the aligned comparison.
#
#   vio_test.sh NAME SECONDS [--no-fox-reset] [--no-bag]
#     e.g.  vio_test.sh lift3 20
#
# It records a rosbag (undistorted stereo, IMU, both odometries, calibration) to
# results/<NAME>_<time>/bag so a run can be replayed offline; --no-bag skips it.
#
# By default it also resets Foxglove first (restarts the bridge + image relay via
# `larp fox`), so the viewer starts clean; Foxglove reconnects by itself.
#
# 1. restarts sqrtVINS so its track starts at 0 (keep the drone STILL until it
#    prints "GO"), 2. records SECONDS of /ov_srvins/odomimu and
# /visual_slam/tracking/odometry, 3. prints summary + comparison.
# For a loop test: after GO, carry the drone around the loop and put it back
# exactly where it started before the time runs out. The end-to-start distance
# is the drift. Results: ~/team_ws/sqrtvins_ws/results/<NAME>_<time>/
set -euo pipefail
NAME="${1:?usage: vio_test.sh NAME SECONDS}"
SECS="${2:-60}"
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
OUT="${NAME}_$(date +%H%M%S)"
FOX_RESET=1
BAG=1
for arg in "${@:3}"; do
  case "$arg" in
    --no-fox-reset) FOX_RESET=0 ;;
    --no-bag) BAG=0 ;;
    *) echo "unknown option: $arg"; exit 2 ;;
  esac
done
IN_C() { docker exec larp-sqrtvins bash -c "set +u; source /opt/ros/humble/setup.bash; source /root/sqrtvins_ws/install/setup.bash; $*"; }

if [[ $FOX_RESET == 1 ]] && command -v "$HOME/.local/bin/larp" >/dev/null; then
  "$HOME/.local/bin/larp" fox stop >/dev/null 2>&1 || true
  "$HOME/.local/bin/larp" fox start --images >/dev/null 2>&1 && echo "Foxglove reset (it reconnects by itself)"
fi
if [[ $BAG == 1 ]]; then  # start before sqrtVINS so the bag includes its initialisation
  "$HERE/sqrtvins_on_drone.sh" bag "$OUT"
  trap '"$HERE/sqrtvins_on_drone.sh" bagstop' EXIT
  sleep 2
fi
"$HERE/sqrtvins_on_drone.sh" start >/dev/null
echo "sqrtVINS restarting -- keep the drone STILL"
for i in $(seq 1 30); do
  sleep 2
  if IN_C "timeout 3 ros2 topic echo --once /ov_srvins/odomimu --field pose.pose.position.x 2>/dev/null | head -1" | grep -q '[0-9]'; then
    break
  fi
  [[ $i == 30 ]] && { echo "sqrtVINS did not initialise in 60 s:"; docker logs --tail 20 larp-sqrtvins; exit 1; }
done
sleep 3
echo "GO: recording ${SECS}s (move now; end exactly where you started)"
IN_C "python3 -u /root/sqrtvins_ws/src/larp_localization/tools/stationary_test.py $SECS /root/sqrtvins_ws/results/$OUT" \
  | grep -E '"(rate_hz|max_gap_ms|drift_m_end|drift_m_max|yaw_drift_deg)"|odom|recording' || true
echo "--- sqrtVINS vs cuVSLAM (aligned)"
IN_C "python3 /root/sqrtvins_ws/src/larp_localization/tools/compare_vio.py \
  /root/sqrtvins_ws/results/$OUT/ov_srvins_odomimu.csv \
  /root/sqrtvins_ws/results/$OUT/visual_slam_tracking_odometry.csv \
  --names sqrtVINS cuVSLAM --out /root/sqrtvins_ws/results/$OUT/compare.json"
# results dir is root-owned (written from the container), so save the log from inside it
docker logs larp-sqrtvins 2>&1 | docker exec -i larp-sqrtvins tee "/root/sqrtvins_ws/results/$OUT/sqrtvins.log" >/dev/null || true
if [[ $BAG == 1 ]]; then
  "$HERE/sqrtvins_on_drone.sh" bagstop; trap - EXIT
  docker run --rm -v "$HOME/team_ws/sqrtvins_ws:/root/sqrtvins_ws" --entrypoint bash \
    "skybeta-team:$(tr -d '[:space:]' < "$HOME/skybeta/VERSION")" -c \
    "source /opt/ros/humble/setup.bash; ros2 bag info /root/sqrtvins_ws/results/$OUT/bag 2>/dev/null" \
    | grep -E "Duration|Messages|Bag size|image_undistorted |imu/data" | sed 's/^/  bag: /' || true
fi
echo "results: ~/team_ws/sqrtvins_ws/results/$OUT/"
