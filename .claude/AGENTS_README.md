# Agent system for the ArmPi Ultra project

A small hierarchy of agents to speed up research and coding. Two halves:

## 1. Research brain — the `research-plan` workflow (max rigor)
`.claude/workflows/research-plan.js`

A head agent decomposes a question → one researcher per sub-question (web + docs) →
an adversarial skeptic tries to refute each finding → the head synthesizes a validated,
cited, actionable plan.

**Run it** (ask Claude, since workflows are billed and token-heavy — see note below):
> "Run the research-plan workflow on: <your question>"

Claude invokes it with your question as `args.question`. Watch live progress with `/workflows`.
Re-run anytime with a different question — it's reusable.

> 💡 Cost: a full run spawns ~10 agents and uses your Max-plan usage faster than a normal chat.
> Use it for substantial questions; use the `robotics-researcher` agent below for quick one-offs.

## 2. Execution hands — the interactive session + helper subagents
SSH-ing into the Pi and running code stays with the **main interactive Claude session**, which asks
before touching hardware. Do NOT push hardware actions into the background workflow — isolated agents
run blind/in parallel and can't hold a live SSH session (unsafe for a real robot arm).

Helper subagents for everyday sessions (`.claude/agents/`):
- **robotics-researcher** — quick single-topic web research with citations.
- **ros2-coder** — drafts/reviews ROS 2 Python for the arm (never runs it).

Invoke them by asking, e.g. "use the ros2-coder agent to write a wave.py for joint 1".

> Subagents are auto-discovered when you launch Claude Code **from this project folder**
> (`cd "Documents/Summer Research"` then `claude`).
