# Larpcopter flight-stack guidance

## Project contract

- This repository (`flight-stack` / local checkout `flight_stack_ws`) is the
  first-party ROS 2 application workspace for Larpcopter flight software.
- Develop against ROS 2 Humble. Prefer the project’s supported containerized
  Humble environment over installing the stack ad hoc on the host.
- Treat autonomous flight, arming logic, payloads, and real-vehicle operation as
  out of scope unless the task explicitly expands the safety contract.

## Work in the supported environment

- Build from this workspace root with `colcon`. Keep `build/`, `install/`, and
  `log/` inside the workspace; never run `colcon` from a parent directory aimed
  at this tree’s packages by accident.
- Prefer bounded, headless checks when a display is unnecessary. State clearly
  when display, hardware, network, or a long build blocks verification.
- Canonical package build/test commands are documented in
  [docs/workspace.md](docs/workspace.md).

## Repository map

- `src/`: first-party ROS 2 packages (for example `flight_executive`).
- `docs/`: contributor and assistant guidance for this repo.
- `.github/`: PR template and GitHub metadata.
- `build/`, `install/`, `log/`: generated colcon output — do not commit.

## Change rules

- Put reusable application code in a ROS package under `src/`; declare runtime
  and test dependencies in that package’s manifest.
- Keep network addresses, peer lists, device mappings, and display values
  configurable. Never commit credentials, Tailscale auth keys, or host
  identities.
- Name assistant-authored branches `{assistant}/{feature}` (example:
  `codex/mission-sm`); human branches use `{name}/{feature}`.
- Keep this file as the shared assistant guidance. `CLAUDE.md` imports it so
  Codex and Claude Code share one source.
- Follow [CONTRIBUTING.md](CONTRIBUTING.md): focused PRs, identify heavily
  generated changes, self-review them, rebase-and-merge.

## Validation

Choose the smallest relevant set, then report exactly what ran:

- Documentation-only: `git diff --check` and verify changed links and commands.
- ROS package changes: from this workspace root, source ROS Humble (and any
  needed overlay), `colcon build` for the touched packages, then package tests
  or a short `ros2 run` smoke.

## Code review rules

- Flag any path that can arm, command, or connect to a real vehicle without an
  explicit scope and safety review.
- Flag committed credentials, runtime artifacts, or build outputs.
- Flag changes that assume an unsupported host ROS install without documenting
  why.
