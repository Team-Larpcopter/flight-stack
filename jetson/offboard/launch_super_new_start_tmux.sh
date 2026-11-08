#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export MISSION_CONFIG_OVERRIDE="$here/config_new_start"
exec bash "$here/launch_system_tmux.sh" hardware "$@"
