#!/usr/bin/env bash
set -euo pipefail
case ${1:?Select camera, vins or foxglove} in
  camera)
    calibration_dir=$(mktemp -d /tmp/flight-stack-camera.XXXXXXXX)
    exec ros2 run larp_hardware oak_native_node --ros-args \
      -p "factory_calibration_out:=$calibration_dir/factory.json" \
      -p connect_device:=true -p publish_images:=true -p publish_rectified:=false \
      -p align_imu:=true -p camera_fps:=30.0 -p accel_hz:=500 -p gyro_hz:=400 \
      -p max_exposure_us:=5000 -p capture_seconds:=0.0 ;;
  vins)
    config=$(ros2 pkg prefix --share larp_localization)/config/oak_d_s2/estimator_config.yaml
    exec ros2 run ov_srvins run_subscribe_msckf --ros-args -r __ns:=/autonomy \
      -p "config_path:=$config" -p verbosity:=INFO -r /sim/processed/vio:=/autonomy/processed_vio ;;
  foxglove)
    exec ros2 launch larp_autonomy visualization_runtime.launch.py ;;
  *) echo 'Select camera, vins or foxglove' >&2; exit 2 ;;
esac
