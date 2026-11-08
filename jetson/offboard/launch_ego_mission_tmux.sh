#!/usr/bin/env bash
# Real PX4 EGO mission. Pilot arming and Position-mode start gates remain active.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_system_tmux.sh" hardware "$@" --planner ego
