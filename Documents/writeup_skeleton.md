# Writeup Skeleton — Voice-Controlled Robotic Manipulation on a CPU-Only Platform

> Draft skeleton prepared 2026-07-22. Sections marked **[DATA]** wait on the
> Week-6 eval numbers (`eval_analysis.py` output drops straight in); sections
> marked **[DRAFT]** can be written NOW from what's already built and
> verified. Everything else is prompts/pointers so no session detail is lost.

## Abstract **[DATA]**
One paragraph: task, platform constraint (Raspberry Pi 5, no GPU), modular
pipeline choice, headline success rate with CI, end-to-end latency.

## 1. Introduction **[DRAFT]**
- Goal: *speak a command → robot perceives → reasons → acts → answers.*
- Why interesting on THIS hardware: the binding limit is 4 CPU cores +
  thermals, not RAM. Everything on-device except LLM reasoning.
- Design thesis: a modular pipeline (STT → LLM → vision → planar map → IK →
  servos → TTS) beats an end-to-end VLA at this scale — debuggable,
  swappable stages, each verifiable in isolation.

## 2. System overview **[DRAFT]**
- Hardware: ArmPi Ultra (6 servo, STM32), Aurora 930 RGB-D (arm-mounted —
  eye-in-hand), WonderEcho USB audio, Pi 5/16 GB in a ROS 2 Humble container.
- Figure: pipeline block diagram with topics as arrows (`/voice_words`,
  `/robot_speech`, `/tts_busy`, `/servo_controller`,
  `/kinematics/set_pose_target`).
- Stage table: component / runs where / latency (STT ~1–2 s base.en int8;
  LLM 1–3 s deepseek-v4-flash thinking-disabled; pick ~20 s; TTS piper).
- The agent schema: `{"steps": [...], "say": "..."}` — gestures, parametric
  moves, pick(color, place), describe. Rolling 4-turn memory.

## 3. The planar calibration system (the technical core) **[DRAFT]**
This is the paper's main contribution — the story writes itself:
1. **Problem:** vendor hand-eye was factory-default; recalibration left
   position-dependent error (1–7 cm); offset tuning provably non-convergent
   (six iterations, Jun 23 log).
2. **Insight:** the task is planar → skip 3D hand-eye entirely; fit pixel→XY
   directly, *with the robot as its own ground truth* (arm places the block
   at known XY, then looks).
3. **Method:** homography via normalized DLT (perspective is real: 328 px vs
   153 px for the same 12 cm near/far — an affine fit leaves 14 mm error, the
   homography ~3–4 mm); bounded leave-worst-out outlier rejection (≤1/3
   dropped, 12 mm threshold) — and WHY unbounded rejection failed live
   (locally-perfect map that extrapolated fiction).
4. **Guards:** trusted-zone from surviving points only (+2.5 cm), droop
   compensation (+5 mm r>0.19, +10 mm r>0.22), wrist alignment via mod-90
   circular mean with 13° deadband, grasp verify (same-spot=miss,
   moved=knocked, gripper blindspot 130 px).
5. **Sector scan:** eye-in-hand means base-only rotation preserves camera
   geometry → one calibration, 3 sectors, ~110° arc (~3× workspace).
6. Map quality achieved: 9/12 kept, mean 4.2 mm, worst 7.9 mm.

## 4. Voice loop **[DRAFT]**
- STT: arecord → webrtcvad segmentation → faster-whisper base.en int8;
  hallucination filter (the "You"/"Thank you" list, with the live log story).
- Half-duplex: `/tts_busy` + 0.7 s echo tail — the robot must not obey itself.
- LLM: DeepSeek v4-flash, thinking disabled (latency: seconds → ~1–3 s);
  graceful fallback if the param is rejected.
- Safety choice: open-mic is opt-in (shared-room policy).

## 5. Evaluation **[DATA]**
- Protocol: `Documents/lab_protocols.md` P5 — positions × colors × rounds
  matrix (phrasings rotated across trials), operator-verified verdicts,
  resumable sessions.
- Headline: overall success + Wilson 95% CI (paste `eval_analysis.py --md`).
- Breakdowns: by position (near vs far vs rotated), color, phrasing;
  robot-self-report vs operator disagreement rate (= verify-layer quality).
- Latency table: per-stage medians from `~/pick_trials.jsonl` stages field.
- Failure taxonomy: missed / knocked / not_seen / out_of_zone / ik_refused
  with counts and one-line causes.

## 6. Limitations & lessons **[DRAFT]**
- Planar-only grasps; fixed grasp pitch (80°); three known colors; single
  block per color; no depth used in the final pick (RGB-D bought, D unused —
  honest and interesting).
- Servo 3 gear damage: hardware fragility as a real constraint on iteration.
- Remote-development methodology (GitHub→curl loop, camera streaming,
  robot-as-ground-truth calibration) — arguably a secondary contribution:
  the entire pipeline past Week 1 was built without touching the robot.
- Whisper hallucinations, ICS networking fragility, the `.stop_ros.sh` nuke:
  systems-integration reality on vendor stacks.

## 7. Future work
End-to-end VLA comparison on stronger compute; depth-informed grasping for
non-planar objects; place destinations from vision ("put it in the red bowl");
multi-object scenes and language disambiguation ("the one on the left").

## Appendix
- A: planar map YAML format + refit-on-load behavior.
- B: full trial logs (JSONL schema table: source/phrasing/llm_s/sector/
  pixel/cal_xy/target_xy/wrist_delta/stages/outcome).
- C: prompt text of the agent.

---

# Demo video shot list (Week 7 — ~2 min target)

Shoot in 4K-ish framing where possible; capture ROS logs simultaneously —
overlay `LLM replied in 1.4s` style lines for credibility.

1. **Cold open (10 s):** spoken "grab the red block" → full pick → robot
   answers "Got the red block!" No cuts, one take, real audio.
2. **Pipeline explainer (20 s):** diagram voiceover: mic → whisper → LLM →
   camera → homography → IK → servos → voice.
3. **Chained command (15 s):** "go home, then turn right, then nod" —
   shows the step schema, not just single actions.
4. **Conversation (10 s):** "what can you do?" → spoken answer (no motion —
   shows it's an agent, not a keyword matcher).
5. **Perception (15 s):** "what do you see?" with blocks + a cup on the mat;
   cut to `/tmp/describe_debug.jpg` annotated view.
6. **Sector scan (15 s):** block placed far left → base rotates, finds,
   grasps. Caption: "one calibration, 3× the workspace."
7. **Rotated block (10 s):** block at 40° → wrist visibly aligns → grasp.
8. **Verification honesty (15 s):** a deliberate miss (block at zone edge) →
   robot says "I missed the red block." Trust comes from admitting failure.
9. **Place (10 s):** "put the blue block in the box" → pick, carry, drop.
10. **Numbers card (10 s):** headline success rate + CI, median latency,
    hardware cost. End card: repo link.

B-roll worth grabbing while in the lab: close-up of the gripper closing;
`/tmp/pick_debug.jpg` frames; the calibration dance (arm placing its own
block); terminal with the JSON steps streaming.
