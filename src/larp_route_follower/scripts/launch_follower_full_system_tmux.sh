#!/usr/bin/env bash
# Full sensing/mapping/route-follower planning stack with execution disabled (shadow).
# Follower counterpart of jetson/offboard/launch_full_system_tmux.sh.
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/launch_follower_system_tmux.sh" shadow "$@"
