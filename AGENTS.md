# Agent guide

ROS 2 Jazzy gaze-guided object selection for a robot arm. Start with `README.md`
and `docs/pi_runbook.md` (Pi access, run steps, status). Credentials are not in
the repo; ask the user for the Pi IP and password.

## Orchestrator vs subagents

The orchestrator is the main session. Subagents do most of the work (writing,
editing, building, deploying, testing, searching). The orchestrator only plans,
briefs, reviews results and manages the subagents, using as few tokens as it
can. The goal is to keep the orchestrator's context small.

### Standing subagents (T3 `delegate_task`, provider `opencode`)

| Name | Model | Use for |
|---|---|---|
| DeepSeek | `opencode/deepseek-v4.1-flash-free` | default worker: code and doc edits, Pi deploy and checks, builds, tests |
| Nemotron | `opencode/nemotron-3.5-super-vl-free` | review of a finished change, reading images or screenshots, second opinion |

Pick the model with `target:{providerInstanceId:"opencode", model:"..."}`. Both
are free; run independent jobs in parallel (at most 3).

**Do it yourself** when the task is:
- a read or edit of one or two known files, or a single small command;
- a short answer you already know from the runbook or this repo;
- a step whose result decides your next step (a failing check, an SSH probe).

**Delegate to a subagent** (use the cheapest model that can do it, e.g. Haiku,
at most 3 at once) when the task is:
- a search across many files where only the conclusion matters;
- slow or noisy work: builds, installs, long logs, dependency checks;
- a bulk mechanical change (formatting, renames, doc edits) across many files;
- an independent job that can run while you continue other work.

Rules for delegating:
- Give the subagent a complete brief: goal, exact paths and commands, and the
  form of the answer. It starts with no context.
- Ask for a short result (verdict plus the few lines that matter), never raw
  logs.
- Do not poll. Subagents report when finished; do other work or stop.
- Never delegate a decision, a destructive action (delete, force-push) or
  anything that needs the user's approval. Never pass credentials in a brief
  unless the subagent must connect to the Pi.
- Do not spawn a subagent for work that takes fewer tool calls than writing
  its brief.

Keep orchestrator output lean: read only the lines you need (`offset`/`limit`,
`grep`), bound every command with a timeout, pipe long output through `tail`,
and do not re-read files you just edited.

## Repo conventions

- Format and lint: `python -m ruff format src scripts training` then
  `python -m ruff check src scripts training` (config in `pyproject.toml`).
- No test suite; verify by running the affected node or script.
- `colcon build --symlink-install`: rebuild only for `.msg` or entry-point
  changes. Copy Python/yaml edits to the Pi with `pscp`.
- Only one `scripts/stream_webcam.py` may run at a time.
- Push to `dev` only when asked; never force-push or touch `main` unasked.
