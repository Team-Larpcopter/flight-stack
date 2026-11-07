# Jetson tools (skybeta drone)

Host-side helpers for our DroneHacks drone (`skybeta-15`). On the drone, ROS 2
runs only inside the organisers' container `skybeta-stack` (tmux session
`skybeta`: agent, slam, bridge, competitor). `~/team_ws` on the Jetson is
mounted at `/root/team_ws` inside it. This repo is checked out at
`~/team_ws/src/flight-stack`.

Nothing here touches the organisers' doctor, status services, `drone.env`
identity, PX4 parameters, or `/fmu/in/*`.

## Files

| File | What |
| --- | --- |
| `larp` | Helper CLI, linked as `~/.local/bin/larp`. Run `larp help`. |
| `install.sh` | One-time host setup: tmux, htop, jq, `larp`, Claude Code, Node 22, Codex CLI. Needs internet. |
| `tmux.conf` | Copied to `~/.tmux.conf` if none exists. `Ctrl-b Ctrl-b` sends the prefix to the stack's nested tmux. |

## `larp` commands

| Command | Runs |
| --- | --- |
| `larp status` | hostname, drone.env IDs, Wi-Fi IP, containers, VIO state, `/skybeta/state` |
| `larp shell` | `docker exec -it skybeta-stack bash` (ROS sourced) |
| `larp ros CMD` | one ROS command inside the stack |
| `larp attach` | the stack's tmux (`Ctrl-b d` to leave) |
| `larp build` | `team_build.sh` (colcon build of `~/team_ws`) inside the stack |
| `larp fox [start\|stop\|status]` | read-only Foxglove bridge on port 8765 (no client publish, services or params, rule 5.5) |
| `larp bag` | organisers' `host/run.sh bag` |
| `larp doctor` | organisers' `host/doctor.sh` |
| `larp restart` | restart the stack. **Power-cycle the flight controller afterwards** (skybeta bug #15) |
| `larp work` | host tmux session `larp` for dev work |
| `larp portal` | press ACCEPT on the Stanford Visitor guest portal when HTTPS stops working |

Foxglove: `ws://<tailscale-or-wifi-ip>:8765`. It stops whenever the stack
restarts; run `larp fox start` again.

## Setup on a fresh drone

```bash
cd ~/team_ws/src && git clone https://github.com/Team-Larpcopter/flight-stack.git
~/team_ws/src/flight-stack/jetson/install.sh
claude    # log in yourself
codex     # log in yourself
```

Never commit credentials, auth keys, or Wi-Fi passwords.
