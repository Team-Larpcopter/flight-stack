#!/usr/bin/env bash
# Run one foreground process per tmux pane; no automatic timeout or PX4 output.
set -eo pipefail
workspace=${WORKSPACE:-/flight-stack}
source /opt/ros/humble/setup.bash
source "${INSTALL_PREFIX:-$workspace/install}/setup.bash"
# Optional overlay (e.g. a branch built separately on top of the main install).
if [[ -n ${OVERLAY_PREFIX:-} ]]; then source "$OVERLAY_PREFIX/setup.bash"; fi
export LD_LIBRARY_PATH="${SDK_PREFIX:-$workspace/sdk_install}/lib:${LD_LIBRARY_PATH:-}"
case ${1:-} in
camera)
  mkdir -p "$workspace/evidence"
  run=$(mktemp -d "$workspace/evidence/localization-XXXXXXXX")
  echo "Camera runs continuously. Calibration export: $run"
  exec ros2 run larp_hardware oak_native_node --ros-args \
    -p connect_device:=true -p publish_images:=true -p align_imu:=true \
    -p factory_calibration_out:="$run/factory.json" \
    -p accel_hz:=500 -p gyro_hz:=400 -p max_exposure_us:=5000 -p capture_seconds:=0.0
  ;;
vio)
  calibration=${CALIBRATION_DIR:-$workspace/src/larp_localization/config/oak_d_s2}
  exec ros2 run ov_srvins run_subscribe_msckf --ros-args \
    -r __ns:=/autonomy -r /sim/processed/vio:=/autonomy/processed_vio \
    -p config_path:="$calibration/estimator_config.yaml" \
    -p verbosity:="${VIO_VERBOSITY:-INFO}"
  ;;
monitor)
  exec ros2 topic hz /autonomy/poseimu
  ;;
fox)
  # read-only Foxglove bridge on 8766 + throttled /viz relay
  exec ros2 launch larp_perception perception.launch.py "${@:2}"
  ;;
*) echo 'Usage: localization_pane.sh camera|vio|monitor|fox'; exit 2;;
esac
