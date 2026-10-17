# Black box and live telemetry system — agent instructions

## Before you work here

**Run your agent from the workspace root (`penn-air/`), not from inside this folder.** Started
here it never loads the workspace `CLAUDE.md`, and it will invent conventions that already exist.

**If you have not already done both of these this session**, do them now:

1. `uv run standards/scripts/tree.py blackbox` — the file tree, so you stop guessing at paths.
2. Read every `.md` at the top level of `standards/`, one Read call per file in a single
   message. Skip `standards/onboarding/`. `standards/CLAUDE.md` does not load on its own, so
   this is the only thing that puts the conventions in front of you.

**Then say which ones you read**, before your first substantive action — and if you skipped any,
which and why. A progress report, not a question; carry on working after it.

Keep both in context; if a compaction drops them, do them again. The workspace `CLAUDE.md` says
why this is worth the tokens. If you are running inside this folder rather than the workspace
root, the paths above are `../standards/`.

## Working on the black box and live telemetry system

Read [README.md](README.md) for the system's purpose and current code. Treat its goals as the
intended scope, not a claim that every capability is implemented. Read the source to establish
what works; do not invent sensors, logged fields, or sampling rates.

Follow the club-wide rules in
[`standards/CLAUDE.md`](../standards/CLAUDE.md) and the standards read above.

- Read the relevant source before changing it. Check every transmitter and receiver involved
  when changing the radio packet format; field order, byte representation, and packet length
  must agree at both ends. Locate the actual source files rather than assuming folder names.
- Confirm the target board, wiring, and library requirements before changing pin assignments,
  serial interfaces, or hardware-specific behavior. Do not infer missing hardware details.
- Report what you verified and how. Distinguish code inspection, compilation, and hardware
  testing; if a check could not be run, say so.

Keep these instructions here; `AGENTS.md` points to this file so agents share one source.
