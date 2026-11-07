#!/usr/bin/env bash
# Host launcher: native camera left, sqrtVINS right. No timeout or PX4 output.
set -euo pipefail

usage() {
  echo "Usage: $0 [CONTAINER_CALIBRATION_DIR]"
  echo 'Defaults to the bundled OAK-D S2 configuration. Optional: CALIBRATION_DIR, CONTAINER, SESSION, VIO_VERBOSITY.'
}
if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
if (($#>1)); then usage >&2; exit 2; fi
container=${CONTAINER:-larp-localization-ready}
session=${SESSION:-localization}
[[ $session =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid tmux session name' >&2; exit 2; }
command -v docker >/dev/null
command -v tmux >/dev/null

attach() {
  if [[ -n ${TMUX:-} ]]; then
    tmux switch-client -t "=$session"
  else
    tmux attach-session -t "=$session"
  fi
}
# Re-running the launcher must not start duplicate camera/estimator processes.
if tmux has-session -t "=$session" 2>/dev/null; then
  echo "Attaching to existing tmux session: $session"
  attach
  exit 0
fi
calibration=${1:-${CALIBRATION_DIR:-${WORKSPACE:-/integration}/src/larp_localization/config/oak_d_s2}}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
pane_script="$script_dir/integration_image/localization_pane.sh"
[[ -r $pane_script ]] || { echo "Missing pane helper: $pane_script" >&2; exit 2; }
if ! running=$(docker inspect --format '{{.State.Running}}' "$container" 2>/dev/null); then
  echo "Container '$container' does not exist; create the configured localization container first." >&2
  exit 2
fi
if [[ $running != true ]]; then docker start "$container" >/dev/null; fi
# Calibration is a path inside the container, not on the host.
docker exec -e "CALIBRATION_DIR=$calibration" "$container" bash -c '
  for f in estimator_config.yaml kalibr_imucam_chain.yaml kalibr_imu_chain.yaml; do
    test -r "$CALIBRATION_DIR/$f" || { echo "Missing calibration: $CALIBRATION_DIR/$f" >&2; exit 2; }
  done
'
remote_helper="/tmp/flight-stack-localization-${session}.sh"
docker cp "$pane_script" "$container:$remote_helper" >/dev/null
common=(docker exec -it -e "CALIBRATION_DIR=$calibration" -e "VIO_VERBOSITY=${VIO_VERBOSITY:-INFO}")
for name in WORKSPACE INSTALL_PREFIX SDK_PREFIX; do
  if [[ -n ${!name:-} ]]; then common+=(-e "$name=${!name}"); fi
done
printf -v camera_cmd '%q ' "${common[@]}" "$container" bash "$remote_helper" camera
printf -v vio_cmd '%q ' "${common[@]}" "$container" bash "$remote_helper" vio
left=$(tmux new-session -d -s "$session" -n localization -P -F '#{pane_id}')
# Keep errors visible if either process exits.
tmux set-option -w -t "$left" remain-on-exit on
right=$(tmux split-window -h -t "$left" -P -F '#{pane_id}')
tmux select-layout -t "$left" even-horizontal >/dev/null
tmux select-pane -t "$left" -T Camera
tmux select-pane -t "$right" -T sqrtVINS
tmux set-option -w -t "$left" pane-border-status top
tmux set-option -w -t "$left" pane-border-format ' #{pane_title} '
tmux respawn-pane -k -t "$left" "$camera_cmd"
tmux respawn-pane -k -t "$right" "$vio_cmd"
tmux select-pane -t "$right"
attach
