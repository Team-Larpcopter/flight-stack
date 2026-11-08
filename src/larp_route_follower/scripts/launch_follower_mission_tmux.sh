#!/usr/bin/env bash
# Real PX4 route-follower mission. Pilot arming and Position-mode start gates remain active.
# Follower counterpart of jetson/offboard/launch_mission_tmux.sh.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_follower_system_tmux.sh" hardware "$@"
