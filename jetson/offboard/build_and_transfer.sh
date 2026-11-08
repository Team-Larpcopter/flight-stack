#!/usr/bin/env bash
# Build and copy only. Never launches the stack on the target.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${SSH_HOST:?Set SSH_HOST to user@jetson}"
bash "$here/build_arm64.sh"
bash "$here/deploy.sh"
