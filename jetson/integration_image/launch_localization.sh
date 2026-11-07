#!/usr/bin/env bash
# Bounded native stereo/IMU + sqrtVINS bench test. No PX4 device or publisher.
set -eo pipefail
if [[ ${1:-} != --inside ]]; then
  workspace=$(realpath "${1:?Usage: ROS_DOMAIN_ID=N bash launch_localization.sh WORKSPACE [SECONDS]}")
  duration=${2:-300}
  [[ $duration =~ ^[0-9]+$ && $duration -ge 10 && $duration -le 1800 ]] || { echo 'Duration must be 10..1800 seconds'; exit 2; }
  : "${ROS_DOMAIN_ID:?Set ROS_DOMAIN_ID to the vehicle ROS domain}"
  for owner in larp-camera larp-sqrtvins skybeta-stack; do
    if [[ $(docker inspect -f '{{.State.Running}}' "$owner" 2>/dev/null || true) == true ]]; then
      echo "Camera/VIO owner $owner is running; stop it before this test."; exit 2
    fi
  done
  script=$(realpath "$0")
  dds_profile=${DDS_PROFILE:-$HOME/skybeta/host/dds/fastdds_localhost.xml}
  [[ -r $dds_profile ]] || { echo "DDS profile missing: $dds_profile"; exit 2; }
  exec docker run --rm --init --name larp-localization-bench --network host --ipc host \
    --device-cgroup-rule='c 189:* rmw' \
    -v /dev/bus/usb:/dev/bus/usb -v "$workspace:/integration" \
    -v "$script:/bench-launch.sh:ro" -w /integration \
    -v "$dds_profile:/etc/skybeta/fastdds.xml:ro" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
    -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
    -e ROS_DOMAIN_ID -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
    "${BENCH_IMAGE:-larp-integration:20261107}" bash /bench-launch.sh --inside "$duration"
fi
seconds=${2:-300}
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
source /integration/install_clean/setup.bash
export LD_LIBRARY_PATH="/integration/sdk_install/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/integration/src/larp_autonomy:${PYTHONPATH:-}"
run=$(mktemp -d /integration/evidence/localization-XXXXXXXX)
chmod a+rx "$run"
echo "Logs and fresh calibration: $run"
native=/integration/install_clean/larp_hardware/lib/larp_hardware/oak_native_node
vio=/integration/install_clean/ov_srvins/lib/ov_srvins/run_subscribe_msckf
correction=/integration/evidence/imu_calibration.json
[[ -x $native && -x $vio && -f $correction ]] || { echo 'Missing binaries or measured IMU correction'; exit 2; }
cleanup() {
  trap - EXIT INT TERM
  [[ -z ${tail_pid:-} ]] || kill "$tail_pid" 2>/dev/null || true
  for pid in ${probe_pid:-} ${vio_pid:-} ${native_pid:-}; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in ${probe_pid:-} ${vio_pid:-} ${native_pid:-}; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
  chmod -R a+rX "$run"
  echo "Test stopped. Evidence: $run"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid "$native" --ros-args -p connect_device:=true -p publish_images:=true \
  -p align_imu:=true -p imu_calibration_file:="$correction" \
  -p factory_calibration_out:="$run/factory.json" -p capture_csv:="$run/raw.csv" \
  -p capture_seconds:="$((seconds+45)).0" -p accel_hz:=500 -p gyro_hz:=400 \
  -p max_exposure_us:=5000 > "$run/camera.log" 2>&1 &
native_pid=$!
for ((i=0;i<40;i++)); do
  kill -0 "$native_pid" 2>/dev/null || { cat "$run/camera.log"; exit 1; }
  [[ -s $run/raw.csv.left.json && -s $run/raw.csv.right.json ]] && break
  sleep 1
done
python3 -m larp_autonomy.native_calibration \
  --extrinsics "$run/factory.json.extrinsics.json" \
  --left "$run/raw.csv.left.json" --right "$run/raw.csv.right.json" \
  --imu-calibration "$correction" --out "$run/calibration" \
  --template-dir /integration/src/larp_localization/config/oak_d_s2 --bench-only
setsid "$vio" --ros-args -r __ns:=/autonomy \
  -r /sim/processed/vio:=/autonomy/processed_vio \
  -p config_path:="$run/calibration/estimator_config.yaml" \
  > "$run/vio.log" 2>&1 &
vio_pid=$!
setsid python3 -m larp_autonomy.bench_probe --seconds "$((seconds-2))" --warmup 0   --out "$run/summary.json" > "$run/probe.log" 2>&1 & probe_pid=$!
tail -n +1 -F "$run/camera.log" "$run/vio.log" & tail_pid=$!
echo 'Localization running: /autonomy/pathimu, /autonomy/odomimu, /autonomy/trackhist'
end=$((SECONDS+seconds))
while ((SECONDS<end)); do
  kill -0 "$native_pid" 2>/dev/null && kill -0 "$vio_pid" 2>/dev/null || { echo 'A pipeline process exited; ending test early'; exit 1; }
  sleep 1
done
