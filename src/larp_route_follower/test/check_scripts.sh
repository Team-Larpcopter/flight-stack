#!/usr/bin/env bash
# Usage: check_scripts.sh <scripts dir> <jetson/offboard/launch_system_tmux.sh>
# The follower launchers parse, run the same panes as the original launcher and
# wrap the right mode. Nothing is started.
set -euo pipefail
dir=$1 original=$2
for f in "$dir"/*.sh; do bash -n "$f"; done
follower=$dir/launch_follower_system_tmux.sh
[[ $(grep '^parts=' "$follower") == $(grep '^parts=' "$original") ]] || { echo 'pane list differs from launch_system_tmux.sh' >&2; exit 1; }
grep -q 'ros2 launch larp_route_follower mission_runtime.launch.py' "$follower"
! grep -q 'ros2 launch larp_autonomy mission_runtime.launch.py' "$follower"
grep -q 'launch_follower_system_tmux.sh" shadow' "$dir/launch_follower_full_system_tmux.sh"
grep -q 'launch_follower_system_tmux.sh" hardware' "$dir/launch_follower_mission_tmux.sh"
# Missing deployment helpers must stop the launcher before anything starts.
out=$(DEPLOY_DIR=$(mktemp -d) bash "$follower" shadow --check 2>&1) && { echo 'launcher ran without helpers' >&2; exit 1; }
grep -q 'Missing' <<<"$out"
echo 'launch scripts ok'
