#!/usr/bin/env bash
# One-time, idempotent setup of the Jetson HOST for Team Larpcopter.
# Needs internet (join the venue Wi-Fi first). Does NOT touch tailscaled, the
# skybeta services, drone.env identity, or PX4.
#
#   ~/team_ws/src/flight-stack/jetson/install.sh
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
log() { printf '\n[install] %s\n' "$*"; }

curl -fsS --max-time 8 https://github.com >/dev/null || { echo "no internet -- connect Wi-Fi first"; exit 1; }

log "apt: tmux, htop, jq"
sudo apt-get update -qq
sudo apt-get install -y -qq tmux htop jq

log "larp helper -> ~/.local/bin/larp"
mkdir -p "$HOME/.local/bin"
ln -sf "$here/larp" "$HOME/.local/bin/larp"
grep -q 'HOME/.local/bin' "$HOME/.bashrc" || echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"

log "tmux config"
[[ -f "$HOME/.tmux.conf" ]] || cp "$here/tmux.conf" "$HOME/.tmux.conf"

log "Claude Code (native installer)"
if ! command -v claude >/dev/null && [[ ! -x "$HOME/.local/bin/claude" ]]; then
  curl -fsSL https://claude.ai/install.sh | bash
fi

log "Node 22 (official nodejs.org build, SHA-256 verified) -> ~/.local/node"
if [[ ! -x "$HOME/.local/node/bin/node" ]]; then
  tmp="$(mktemp -d)"
  base="https://nodejs.org/dist/latest-v22.x"
  curl -fsSL -o "$tmp/SHASUMS256.txt" "$base/SHASUMS256.txt"
  tarball="$(grep -o 'node-v22[^ ]*-linux-arm64\.tar\.xz' "$tmp/SHASUMS256.txt" | head -1)"
  curl -fsSL -o "$tmp/$tarball" "$base/$tarball"
  (cd "$tmp" && grep " ${tarball}\$" SHASUMS256.txt | sha256sum -c -)
  mkdir -p "$HOME/.local/node"
  tar -xJf "$tmp/$tarball" -C "$HOME/.local/node" --strip-components=1
  rm -rf "$tmp"
fi
for bin in node npm npx; do ln -sf "$HOME/.local/node/bin/$bin" "$HOME/.local/bin/$bin"; done
export PATH="$HOME/.local/node/bin:$HOME/.local/bin:$PATH"

log "Codex CLI"
command -v codex >/dev/null || npm install -g @openai/codex
ln -sf "$HOME/.local/node/bin/codex" "$HOME/.local/bin/codex"

log "done. Next, as yourself (interactive logins, never committed):"
echo "  claude          # log in with your Anthropic account"
echo "  codex           # sign in with ChatGPT, or: export OPENAI_API_KEY=..."
echo "  larp status; larp fox start"
