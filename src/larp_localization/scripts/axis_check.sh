#!/usr/bin/env bash
# Check the IMU->camera axes (flips) against real motion, on the drone.
#
#   axis_check.sh [SECONDS]      (default 30)
#
# Needs the camera node running (/oak/left/image_undistorted, /oak/imu/data).
# Records a bag, then measures the IMU->camera rotation from the motion
# (tools/check_imu_camera_rotation.py: camera rotation from the images vs
# integrated gyro) and compares it with kalibr_imucam_chain.yaml.
# Motion: after GO, slowly rotate the drone back and forth about EACH axis in
# turn (roll, pitch, yaw), ~30-60 deg each way, facing a textured scene.
set -euo pipefail
SECS="${1:-30}"
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
OUT="axis_$(date +%H%M%S)"
IMAGE="skybeta-team:$(tr -d '[:space:]' < "$HOME/skybeta/VERSION")"
"$HERE/sqrtvins_on_drone.sh" bag "$OUT"
trap '"$HERE/sqrtvins_on_drone.sh" bagstop' EXIT
sleep 3
echo "GO: rotate about each axis for ${SECS}s (roll, pitch, yaw; ~30-60 deg each way, slowly)"
sleep "$SECS"
"$HERE/sqrtvins_on_drone.sh" bagstop; trap - EXIT
echo "analysing..."
docker run --rm --cpus=1 --memory=1500m -v "$HOME/team_ws/sqrtvins_ws:/root/sqrtvins_ws:ro" --entrypoint bash "$IMAGE" -c \
  "source /opt/ros/humble/setup.bash; python3 /root/sqrtvins_ws/src/larp_localization/tools/check_imu_camera_rotation.py \
   /root/sqrtvins_ws/results/$OUT/bag --step 2 \
   --chain /root/sqrtvins_ws/src/larp_localization/config/oak_d_s2/kalibr_imucam_chain.yaml"
echo "bag: ~/team_ws/sqrtvins_ws/results/$OUT/bag"
