#!/usr/bin/env bash
# Sourced by explicit launch commands; leave unrelated containers alone.
for old in "$container" flight-stack-takeoff-hold flight-stack-triangle flight-stack-fixed-course flight-stack-mission-hardware flight-stack-mission-shadow flight-stack-mission-ego-hardware flight-stack-mission-ego-shadow flight-stack-mission-follower-hardware flight-stack-mission-follower-shadow flight-stack-localization-runtime flight-stack-runtime larp-localization-ready; do
  if docker container inspect "$old" >/dev/null 2>&1; then
    echo "Replacing existing stack container: $old"
    if [[ $(docker inspect --format '{{.State.Running}}' "$old") == true ]]; then
      # Give an active recorder its normal Ctrl+C finalization before shutdown.
      docker exec "$old" pkill -INT -f '[r]os2 bag record' >/dev/null 2>&1 || true
      docker stop --time 3 "$old" >/dev/null
    fi
    docker rm "$old" >/dev/null
  fi
done
for old in "$session" flight-stack-takeoff-hold flight-stack-triangle flight-stack-fixed-course flight-stack-hardware flight-stack-shadow flight-stack-ego-hardware flight-stack-ego-shadow flight-stack-follower-hardware flight-stack-follower-shadow flight-stack-runtime; do
  tmux kill-session -t "$old" 2>/dev/null || true
done
