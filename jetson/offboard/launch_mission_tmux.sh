#!/usr/bin/env bash
# Real PX4: pilot arms, then switches to Position; software requests Offboard; single goal followed by land.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_system_tmux.sh" hardware "$@"
