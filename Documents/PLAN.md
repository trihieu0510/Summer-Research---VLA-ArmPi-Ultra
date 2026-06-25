# PLAN.md — schedule to mid-August

Target: finish the voice-controlled manipulation project by **~Aug 15, 2026**.
Baseline as of **2026-06-25**: agent (gestures + parametric + sequential moves +
conversation) and voice OUTPUT (piper TTS) working over the network. Calibration
parked; voice INPUT and manipulation not started.

Availability: ~25–30 hrs/week, weekdays, with lab access. Time is not the binding
constraint — **execution risk (calibration + grasp reliability) is.** The plan front-loads
the unknowns and protects buffer at the end.

---

## Schedule

| Week | Dates | Focus | Exit criteria (done = ...) |
|---|---|---|---|
| **1** | Jun 25 – Jul 1 | **Hand-eye calibration first** (`tool.sh`, lab). Voice input (faster-whisper → `/voice_words`) in parallel/remote. | Gripper lands on the block reliably (≤1 cm). Speaking a command moves the arm. |
| **2** | Jul 2 – Jul 8 | Refactor `object_sorting` → callable `pick(color)` / `place(location)`; wire into the agent as steps. | Agent can run "pick the red block" end-to-end via a skill, not the stock demo. |
| **3** | Jul 9 – Jul 15 | `hand_to_human()`, conversation polish, error recovery (graceful "I couldn't reach that"). | Agent handles a failed grasp without crashing; hands an object to a person. |
| **4** | Jul 16 – Jul 22 | Full integration: mic → STT → reason → act → TTS. Tune latency. | One hands-free spoken command runs the whole loop and speaks back. |
| **5** | Jul 23 – Jul 29 | Edge cases, 1–2 more destinations/objects, robustness. **(Buffer week if earlier slipped.)** | Works across a few phrasings, block positions, lighting. |
| **6** | Jul 30 – Aug 5 | **Evaluation** — success rates across phrasings, lighting, positions, distances. | A results table with real numbers (N trials per condition). |
| **7** | Aug 6 – Aug 12 | Demo video + writeup. | Recorded demo + written report. |
| **+** | Aug 13 – Aug 15 | **Slack — protect it.** | — |

---

## Milestones (the things that actually de-risk the project)

1. **M1 — Reliable color pick (Week 1).** The single highest-leverage, highest-uncertainty
   task. Nothing in the manipulation half starts cleanly until this works. Do it on day 1 of
   the next lab session, before anything else.
2. **M2 — `pick`/`place` as agent skills (Week 2).** Turns the demo into something the LLM
   controls.
3. **M3 — Hands-free voice loop (Week 4).** The headline capability.
4. **M4 — Evaluation done (Week 6).** This is what makes it a *research project* vs a demo.

---

## Top risks & mitigations

- **Calibration fights back.** Mitigation: front-load it (Week 1, day 1). If it's still
  drifting after the `tool.sh` run, that's a hardware/setup conversation to have *early*, not
  in August.
- **Grasp reliability eats weeks 5–6.** Manipulation is the classic time-sink. Mitigation:
  Week 5 is deliberately a buffer; keep object set and destinations small until the core is
  solid.
- **Evaluation + writeup get under-budgeted** (they're not "building", so easy to shortchange).
  Mitigation: treat Weeks 6–7 as real work; start the eval harness in Week 5.

---

## Fallback (if manipulation slips badly)

You still have a strong, honest deliverable: a voice/chat LLM agent that converses and
performs expressive multi-step arm control, plus a documented vision→IK→grasp chain that
"works but needs calibration." De-scope manipulation, keep the voice agent + evaluation of
*that*, and it's a legitimate result — not a failure.

## Parallelization rule

Anything that doesn't need the robot — voice input, the `pick`/`place` refactor (write it
against existing code), prompt work, eval scaffolding — do it remotely *between* lab
sessions. Spend lab hours only on what physically needs the hardware.
