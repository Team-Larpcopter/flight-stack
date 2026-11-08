#!/usr/bin/env bash
# Apply a small, prebuilt application patch. Never starts/stops a flight process.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
patch=${1:-$here/mission-integration-patch.tar.gz}
[[ -f $patch && -d $here/runtime/flight-stack/install && -f $here/runtime/.extracted ]] || {
  echo 'Place this script and the patch beside an existing extracted runtime.' >&2
  exit 2
}
runtime=$(realpath "$here/runtime/flight-stack")
for id in $(docker ps -q); do
  if docker inspect --format '{{range .Mounts}}{{println .Source}}{{end}}' "$id" | grep -Fqx "$runtime"; then
    echo 'Stop the stack container before applying this patch; no processes were stopped.' >&2
    exit 2
  fi
done
# Extract completely before touching the installed files, including on a broken transfer.
staging=$(mktemp -d "$here/.patch.XXXXXXXX")
trap 'rm -rf "$staging"' EXIT
tar -xzf "$patch" -C "$staging" --no-same-owner
cp -a "$staging/." "$here/"
touch "$here/runtime/.extracted"
echo 'Patch installed. No image rebuild, engine rebuild, or mission launch performed.'
