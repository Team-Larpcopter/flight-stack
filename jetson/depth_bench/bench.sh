#!/usr/bin/env bash
# Runs INSIDE a container from the team image, with the skybeta stack STOPPED
# (only one process can open the OAK). For each variant of our camera config,
# start the depthai driver alone, let it settle, measure every stream from
# header stamps, stop it. No cuVSLAM, no PX4: camera throughput only.
#
#   bench.sh [SETTLE_S] [MEASURE_S]      results -> /root/team_ws/depth_bench_results.jsonl
set -uo pipefail
# shellcheck disable=SC1091  # provided by the organisers' image
source skybeta_env.sh
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
base="$here/../team_image/oakd_vslam.yaml"
out=/root/team_ws/depth_bench_results.jsonl
settle="${1:-12}"
measure="${2:-12}"
: > "$out"

# name | stereo/left/right WxH | stereo overrides (python dict literal)
variants=(
  "1024_lr|1024x640|{}"
  "1024_nolr|1024x640|{'i_lr_check': False}"
  "960_lr|960x600|{}"
  "960_nolr|960x600|{'i_lr_check': False}"
  "800_lr|800x500|{}"
  "640_lr|640x400|{}"
  "1280_nolr|1280x800|{'i_lr_check': False}"
  "1280_lr|1280x800|{}"
  "1024_lr_robotics|1024x640|{'i_depth_preset': 'ROBOTICS'}"
  "1024_lr_highdetail|1024x640|{'i_depth_preset': 'HIGH_DETAIL'}"
  "1024_lr_nodepth|1024x640|{'i_publish_topic': False}"
)

for v in "${variants[@]}"; do
  IFS='|' read -r name size overrides <<< "$v"
  w="${size%x*}"; h="${size#*x}"
  cfg="/tmp/bench_${name}.yaml"
  python3 - "$base" "$cfg" "$w" "$h" "$overrides" <<'EOF'
import ast, sys, yaml
base, cfg, w, h, over = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
d = yaml.safe_load(open(base))
p = d["/oak"]["ros__parameters"]
for k in ("left", "right", "stereo"):
    p[k]["i_width"], p[k]["i_height"] = w, h
p["stereo"].update(ast.literal_eval(over))
yaml.safe_dump(d, open(cfg, "w"))
EOF
  echo "=== $name ($size $overrides)"
  ros2 launch depthai_ros_driver_v3 driver.launch.py name:=oak params_file:="$cfg" \
    > "/tmp/bench_${name}.log" 2>&1 &
  pid=$!
  sleep "$settle"
  result="$(python3 "$here/rates.py" "$measure" \
    /oak/left/camera_info /oak/right/camera_info /oak/stereo/camera_info 2>/dev/null \
    | tail -1)"
  [[ -n "$result" ]] || result='{}'
  load="$(awk '{print $1}' /proc/loadavg)"
  errs="$(grep -c -i -E 'error|exception|fail' "/tmp/bench_${name}.log" || true)"
  printf '{"variant":"%s","size":"%s","overrides":"%s","load":%s,"log_errors":%s,"rates":%s}\n' \
    "$name" "$size" "$overrides" "$load" "$errs" "$result" | tee -a "$out"
  kill -INT "$pid" 2>/dev/null; sleep 4; kill -9 "$pid" 2>/dev/null
  pkill -9 -f component_container 2>/dev/null; sleep 3
done
echo "done -> $out"
