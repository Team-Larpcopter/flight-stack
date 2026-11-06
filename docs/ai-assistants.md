# AI assistant setup

Durable project instructions for coding assistants live in the repository-root
[`AGENTS.md`](../AGENTS.md). It records commands, architecture boundaries, safety
constraints, and verification expectations for work in this repository.

Codex discovers `AGENTS.md` automatically. Claude Code reads the root
[`CLAUDE.md`](../CLAUDE.md), which imports `AGENTS.md` so both tools get the same
instructions without duplicated copies. Keep `AGENTS.md` as the canonical shared
source and reserve `CLAUDE.md` for the import plus any genuinely Claude-specific
exception.

Official references:

- [Codex custom instructions with AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md)
- [Claude Code project memory and AGENTS.md imports](https://code.claude.com/docs/en/memory#agents-md)

## Instruction placement

Both tools support nested instruction files below the repository root. Add one
only when a real subtree needs different build, safety, or review rules. Prefer
package-local notes under `src/<package>/` when a package’s workflow diverges
from the root.

Keep root instructions concise and operational. Human-facing procedures belong
in `docs/` and `CONTRIBUTING.md`, linked from `AGENTS.md` rather than copied in
full. Start a new assistant session after changing instructions so the new
context loads.

## What assistants need to know

- This repo is a ROS 2 Humble workspace of first-party application packages.
- Build and smoke from the workspace root with `colcon` / `ros2` (see
  [workspace.md](workspace.md)).
- Safety-sensitive real-vehicle work stays out of scope unless explicitly
  expanded.
- PR and branch conventions are in [CONTRIBUTING.md](../CONTRIBUTING.md).

## Branch naming

Assistant-authored branches use `{assistant}/{feature}`, such as
`codex/flight-executive-node`. Do not add an intermediate `feature/` segment.
Humans use `{name}/{feature}` as in the contribution guide.

## Maintaining the setup

Update `AGENTS.md` when canonical commands, source boundaries, or validation
expectations change. Keep `CLAUDE.md` pointed at the shared file, and put
reusable procedures in focused docs rather than growing the root instructions
indefinitely.
