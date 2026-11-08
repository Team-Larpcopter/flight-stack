#!/usr/bin/env bash
# Full EGO sensing/planning stack with execution disabled.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_system_tmux.sh" shadow "$@" --planner ego
