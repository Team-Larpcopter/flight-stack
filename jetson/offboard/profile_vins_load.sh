#!/usr/bin/env bash
# Stationary computation profiling only: no DDS agent, PX4 bridge or hardware executive.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
seconds=${1:-90}
[[ $seconds =~ ^[0-9]+$ ]] && ((seconds>=10 && seconds<=600)) || exit 2
[[ ! -f $here/deployment.env ]] || source "$here/deployment.env"
image=${RUNTIME_IMAGE:-larp-flight-stack:arm64-runtime}
source "$here/runtime_mounts.sh"
if [[ -n ${PROFILE_PATCH_DIR:-} ]]; then
  for package in ov_srvins larp_autonomy flight_executive larp_route_follower; do
    runtime_mounts+=(-v "$(realpath "$PROFILE_PATCH_DIR/runtime/flight-stack/install/$package"):/flight-stack/install/$package:ro")
  done
fi
output=${PROFILE_OUTPUT:-$here/profiling/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$output"
cp "$here/profile_vins_topics.py" "$output/"
if [[ -f $here/planner-bench/closed_loop_test ]]; then
  cp "$here/planner-bench/closed_loop_test" "$output/"
fi
cp -a "${DEPLOYMENT_CONFIG:-$here/config}" "$output/config"
cp -a "$here/runtime/flight-stack/install/larp_localization/share/larp_localization/config/oak_d_s2" "$output/estimator"
# Copy all calibration relatives; edit only the isolated profiling configuration.
python3 - "$output" <<'PYCONFIG'
from pathlib import Path
import sys, os
p=Path(sys.argv[1]); cfg=p/'estimator/estimator_config.yaml'
s=cfg.read_text().replace('record_timing_information: false','record_timing_information: true')
s=s.replace('"/tmp/traj_timing.txt"','"/profile/vins_timing.csv"')
if os.environ.get('PROFILE_DOWNSAMPLE') == '1':
    s=s.replace('downsample_cameras: false','downsample_cameras: true')
cfg.write_text(s)
with (p/'config/mission.yaml').open('a') as f:
    f.write('\nestimator_config: /profile/estimator/estimator_config.yaml\n')
PYCONFIG
container=flight-stack-vins-profile
cleanup() { docker stop -t 3 "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM
# Private network + IPC and a separate domain prevent interaction with the aircraft
# and with an operator's replay/visualization container. No serial device is mapped.
docker run --rm --init --name "$container" --runtime nvidia --network none \
  --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
  "${runtime_mounts[@]}" -v "$(realpath "$output"):/profile" \
  -v "$(realpath "$output/config"):/config:ro" \
  -e ROS_DOMAIN_ID=81 -e ROS_LOCALHOST_ONLY=1 \
  "$image" bash -c '
    children=()
    for part in drivers foxglove vins depth map planner executive; do
      ros2 launch larp_route_follower mission_runtime.launch.py component:=$part mode:=shadow preplan_only:=true mission_file:=/config/mission.yaml > /profile/$part.log 2>&1 &
      children+=("$!")
    done
    if [[ -x /profile/closed_loop_test ]]; then
      (sleep 20
       timeout 20 /profile/closed_loop_test /flight-stack/install/larp_navigation/share/larp_navigation/config/navigation_fine.yaml /flight-stack/install/larp_navigation/share/larp_navigation/config/course_cad_prior_fine.txt /flight-stack/install/larp_route_follower/share/larp_route_follower/config/course_master_route.xyz) > /profile/course_benchmark.log 2>&1 &
    fi
    python3 /profile/profile_vins_topics.py "$1" &
    probe=$!
    wait -n "${children[@]}" "$probe"
    result=$?
    if kill -0 "$probe" 2>/dev/null; then
      echo "A mission component exited before profiling completed" >&2
      exit 1
    fi
    exit "$result"
  ' profile "$seconds" >"$output/container.log" 2>&1 &
pid=$!
while kill -0 "$pid" 2>/dev/null; do
  docker stats --no-stream --format '{{json .}}' "$container" >>"$output/resources.jsonl" 2>/dev/null || true
  sleep 2
done
wait "$pid"
echo "Profile saved: $output"
