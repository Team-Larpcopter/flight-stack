#!/usr/bin/env bash
# Full sensor/mapping/planning system with shadow trajectory output.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_system_tmux.sh" shadow "$@"
