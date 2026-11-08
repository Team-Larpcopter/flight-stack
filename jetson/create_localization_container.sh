#!/usr/bin/env bash
# Create (do not start) a container using only this flight-stack checkout.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
[[ $ROS_DOMAIN_ID =~ ^[0-9]+$ ]] && ((ROS_DOMAIN_ID <= 232)) || { echo 'Invalid ROS_DOMAIN_ID' >&2; exit 2; }
container=${CONTAINER:-flight-stack-localization}
image=${BUILD_IMAGE:-larp-flight-stack:humble}
[[ -f $root/install/setup.bash ]] || { echo 'Run bash jetson/build.sh first' >&2; exit 2; }
if docker inspect "$container" >/dev/null 2>&1; then
  echo "Container $container already exists; use a different CONTAINER or remove it explicitly." >&2
  exit 2
fi
dds=()
if [[ -n ${DDS_PROFILE:-} ]]; then
  profile=$(realpath "$DDS_PROFILE")
  [[ -f $profile && -r $profile ]] || { echo "Missing DDS profile: $profile" >&2; exit 2; }
  dds=(-v "$profile:/etc/flight-stack/fastdds.xml:ro"
       -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/flight-stack/fastdds.xml
       -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/flight-stack/fastdds.xml)
fi
docker create --init --name "$container" --network host --ipc host \
  --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
  -v "$root:/flight-stack" -w /flight-stack \
  -e ROS_DOMAIN_ID -e WORKSPACE=/flight-stack -e INSTALL_PREFIX=/flight-stack/install \
  -e SDK_PREFIX=/flight-stack/sdk_install \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 "${dds[@]}" \
  "$image" sleep infinity
echo 'Created without starting sensors. Launch with: bash jetson/launch_localization_tmux.sh'
