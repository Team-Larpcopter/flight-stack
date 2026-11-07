#!/usr/bin/env bash
# Runs INSIDE a team-image container with the stack STOPPED. Starts the driver
# with our team config at 800x500, with and without subpixel, and reports rates
# plus depth granularity on live frames (depth_stats.py).
#   granularity.sh [SETTLE_S] [MEASURE_S]   -> /root/team_ws/depth_granularity.jsonl
set -uo pipefail
# shellcheck disable=SC1091  # provided by the organisers' image
source skybeta_env.sh
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
base="$here/../team_image/oakd_vslam.yaml"
out=/root/team_ws/depth_granularity.jsonl
settle="${1:-12}"
measure="${2:-8}"
: > "$out"
for v in "800_int|{}" "800_subpixel|{'i_subpixel': True}"; do
  IFS='|' read -r name overrides <<< "$v"
  cfg="/tmp/gran_${name}.yaml"
  python3 - "$base" "$cfg" "$overrides" <<'EOF'
import ast, sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
d["/oak"]["ros__parameters"]["stereo"].update(ast.literal_eval(sys.argv[3]))
yaml.safe_dump(d, open(sys.argv[2], "w"))
EOF
  ros2 launch depthai_ros_driver_v3 driver.launch.py name:=oak params_file:="$cfg" \
    > "/tmp/gran_${name}.log" 2>&1 &
  pid=$!
  sleep "$settle"
  rates="$(python3 "$here/rates.py" 8 /oak/left/camera_info /oak/stereo/camera_info 2>/dev/null | tail -1)"
  stats="$(python3 "$here/depth_stats.py" "$measure" 2>/dev/null | tail -1)"
  printf '{"variant":"%s","rates":%s,"depth":%s}\n' "$name" "${rates:-{\}}" "${stats:-{\}}" \
    | tee -a "$out"
  kill -INT "$pid" 2>/dev/null; sleep 4; kill -9 "$pid" 2>/dev/null
  pkill -9 -f component_container 2>/dev/null; sleep 3
done
echo "done -> $out"
