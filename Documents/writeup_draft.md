# Voice-Controlled Robotic Manipulation on a CPU-Only Platform

> **Working draft, 2026-07-22.** All [DRAFT] sections from `writeup_skeleton.md`
> written out in full; slots marked **[DATA: …]** await the Week-6 evaluation
> numbers and drop in directly from `eval_analysis.py --md` output. Every
> number in this draft is live-verified on the hardware (sources: session
> handoffs + `Daily Update.txt`).

## Abstract

**[DATA: headline success rate + CI, end-to-end latency — one paragraph,
written last.]** Skeleton: We present a voice-controlled manipulation system
on a $300-class 6-DOF arm driven entirely by a Raspberry Pi 5 with no GPU.
A spoken command ("grab the red block") is transcribed on-device, interpreted
by a cloud LLM into a structured action plan, and executed through a
vision-guided grasp pipeline whose hand-eye layer we replaced with a
self-calibrating planar homography — the robot generates its own
ground-truth correspondences. The system achieves **[DATA]**% grasp success
(95% Wilson CI **[DATA]**) across **[DATA]** operator-verified trials, with
a median command-to-motion latency of **[DATA]** s.

## 1. Introduction

The goal of this project is a robot that closes the full loop between
natural language and physical action: *speak a command → the robot
perceives the scene → reasons about the request → acts → answers in a
natural voice.*

What makes this interesting is the platform it runs on. The Hiwonder ArmPi
Ultra is a hobby-class 6-servo arm controlled by a Raspberry Pi 5 — 16 GB of
RAM but **no GPU**, so the binding constraint is four CPU cores and their
thermal envelope. Nothing about the pipeline is allowed to assume
accelerator hardware: speech recognition runs as an int8-quantized Whisper
model on the CPU, perception is classical computer vision plus (optionally)
a nano-scale object detector, and only the language-reasoning step leaves
the device, as a single short LLM API call per command.

Our design thesis is that at this scale a **modular pipeline** beats an
end-to-end vision-language-action model:

    speech → STT → LLM planner → vision → planar map → IK → servos → TTS

Each stage is independently observable, testable, and swappable. This
mattered more than we anticipated: over seven weeks, every stage failed at
least once (vendor calibration, USB camera lifecycle, LLM API semantics,
microphone self-echo, a stripped servo gear), and in every case the modular
seam let us isolate and fix the failing layer without touching the others.
The project's main technical contribution (§3) exists precisely because the
calibration layer could be ripped out and replaced wholesale.

A secondary, more unusual constraint shaped the work: after Week 1 the
system was developed **almost entirely remotely** — laptop → remote desktop
→ lab PC → LAN → Pi — including camera-in-the-loop calibration. The
methodology that made this possible (§6.3) is, we argue, a minor
contribution in its own right.

## 2. System overview

### 2.1 Hardware

| Component | Detail |
|---|---|
| Arm | Hiwonder ArmPi Ultra, 6 bus servos (raw units 0–1000, 0.24°/unit), STM32 servo controller |
| Compute | Raspberry Pi 5, 16 GB RAM, no GPU; ROS 2 Humble in a Docker container |
| Camera | Aurora 930 RGB-D, **arm-mounted** (eye-in-hand), RGB at ~10 Hz |
| Audio | WonderEcho Pro USB device — single card serves both microphone and speaker |

The camera being arm-mounted rather than fixed is the single most
consequential hardware fact in the system: it forces all detection to
happen from one repeatable "view pose" (§3), and it enables the sector-scan
workspace extension (§3.5).

### 2.2 Pipeline

```
   mic ──arecord──► [stt_node] ──/voice_words──► [arm_agent] ◄──API──► DeepSeek LLM
                        ▲                            │ steps
                   /tts_busy                         ├─► gestures / parametric moves ──► /servo_controller
                   (half-duplex                      ├─► pick(color, place) ──► camera + /kinematics/set_pose_target
                    mute)                            └─► describe ──► camera (+ YOLOv8n)
                        │                            │ say
   speaker ◄──aplay── [tts_node] ◄──/robot_speech────┘
```

A typed console (`arm_console`) publishes to the same `/voice_words` topic
the speech node uses, so the text and voice front-ends are interchangeable
by construction — every capability works identically typed or spoken.

### 2.3 The agent

Each user turn is sent (with a rolling four-turn history) to the LLM, which
must return a single JSON object:

```json
{"steps": [ ...ordered actions... ], "say": "<spoken reply>"}
```

Steps are named gestures (`home`, `nod`, …), parametric servo moves
(absolute targets or relative deltas resolved against tracked servo state,
clamped to ±300 units per step), or skills: `pick(color, place)` and
`describe`. Because steps are an ordered list, compound commands — "go
home, then turn the base right about 100, then raise motor 4 by 300" —
execute as sequences without any special-casing. An empty `steps` list is a
pure conversational turn, which is what makes the system an agent rather
than a keyword-to-motion classifier.

Two LLM-integration details proved load-bearing. First, the provider's
model rename silently switched us onto a reasoning-by-default variant,
adding seconds of hidden chain-of-thought latency to every command; we now
explicitly request `thinking: disabled` and log per-turn latency, with a
graceful retry if an endpoint rejects the parameter. Second, all parsing is
defensive (fence-stripping, first-JSON-object extraction, schema
tolerance), because a robot that crashes on a malformed reply is a robot
that stops mid-conversation.

### 2.4 Stage latencies

| Stage | Where | Typical latency |
|---|---|---|
| STT (base.en, int8, beam 1) | Pi CPU | ~1–2 s per utterance |
| LLM (deepseek-v4-flash, thinking disabled) | cloud | ~1–3 s |
| Pick (scan → detect → grasp → verify) | Pi + arm | ~20 s (**[DATA: median from trial logs]**) |
| TTS (piper, en_US-amy-medium) | Pi CPU | ~1 s to first audio |

## 3. The planar calibration system

This section is the technical core. It exists because the vendor's
hand-eye calibration failed us three different ways, and the replacement
turned out to be simpler, more accurate, and more instructive than the
thing it replaced.

### 3.1 The problem

Grasping from camera detections requires a pixel→robot-frame transform. The
vendor stack ships a 3D hand-eye matrix and a GUI calibration tool. We
found, in order: (a) the shipped matrix was still at factory defaults — the
calibration GUI deadlocks silently before opening a window if the SDK
isn't running, so it had likely never completed on this unit; (b) after two
successful GUI calibrations, grasp error remained **1–7 cm and
position-dependent**; (c) tuning the vendor's post-hoc YAML offsets did not
converge — six documented iterations produced non-linear, coupled responses
(X improved while Y worsened, then reversed). Post-hoc offsets fight the
wrong layer when the underlying transform is bad.

### 3.2 The insight

The task is planar: blocks lie on a flat mat, and the grasp approach is
always vertical at a fixed pitch. A full 3D hand-eye transform is
unnecessary — what's needed is a map from image pixels (u, v) to mat-plane
robot coordinates (x, y), valid from one fixed camera pose.

The second insight is about ground truth: **the robot is its own measuring
device.** Calibration (`planar_calib`) has the arm *place* a block at a
commanded robot-frame (x, y) via IK, retreat to the view pose, detect the
block's pixel centroid, then re-grasp it and carry it to the next grid
point. Each cycle yields one (pixel, XY) correspondence in which IK-frame
biases cancel exactly where it matters — the same IK that will execute
future grasps defined the training targets. No calibration board, no
external measurement, and (critically for this project) no human in the
lab: the whole procedure runs remotely in ~15 minutes.

### 3.3 Fit: homography, not affine

The camera views the mat obliquely, so pixel→world has genuine perspective:
in our data the same 12 cm of mat spans **328 px near vs 153 px far**. A
constant-scale affine fit left 14 mm mean residual; a homography (estimated
with the normalized DLT) fits the same points to ~3–4 mm. On a 3 cm block
with ~±1 cm of grasp tolerance, that difference is the difference between
working and not.

### 3.4 Robustness: bounded rejection and honest zones

Individual correspondences go bad for physical reasons — the most common is
the block rolling as the gripper releases it. Naive leave-worst-out outlier
rejection handled this but produced a worse failure: run to convergence, it
once discarded 4 of 9 points — all in the far half of the grid — leaving a
map that was 2 mm-perfect in the near rows and pure extrapolated fiction
beyond them (it mapped a far-row detection to an impossible x = 0.251 m).

Two rules fixed this class of failure:

1. **Bounded rejection:** drop at most ⅓ of the points (threshold 12 mm),
   then live with the residuals. *A map honest to ~10 mm everywhere grasps
   a 3 cm block; a locally perfect extrapolating map does not.*
2. **The trusted zone is defined by surviving points only** (their bounding
   box + 2.5 cm). Dropped outliers must not extend the region the map
   claims to cover. Targets mapping outside the zone are refused aloud
   ("The red block is outside the zone I can reach") rather than attempted.

The stored map keeps the raw correspondences, and **refits on every load**
— map-format upgrades (affine-era maps became homographies) and rejection
improvements propagate to old calibrations automatically.

### 3.5 Sector scan: one calibration, three times the workspace

Eye-in-hand geometry gives an unusual gift: rotating **only the base servo**
moves the camera to a new patch of table with *identical* camera-to-mat
geometry. The same homography therefore applies in the rotated sector, and
the resulting target only needs rotating back about the base axis by the
known sector angle. `run_pick` scans three sectors (0, ±150 servo units ≈
±36°), turning one 15-minute calibration into ~110° of reachable arc —
roughly 3× the workspace. **[DATA: sector-sign live verification (protocol
P1) and per-sector success rates.]**

### 3.6 Grasp refinements (each one bought by a live failure)

- **Droop compensation.** The arm sags measurably at reach; at far grid
  points the gripper dug into the mat (bending a gripper screw on Jul 10).
  Approach heights get +5 mm beyond r = 0.19 m and +10 mm beyond 0.22 m.
- **Wrist alignment with a mod-90 circular mean.** A square block's image
  orientation is estimated per frame by `minAreaRect`, but is only defined
  modulo 90°; a plain median flips sign near ±45° (+44° and −44° are 2°
  apart, not 88°). We average on the 4×-angle circle — the correct mean for
  mod-90 quantities — then rotate the wrist only when misalignment exceeds
  ~13° (the wide jaws forgive 15–20° on a square block, and every rotation
  risks shifting the grasp point).
- **Detection hygiene.** Median of 5 frames; search restricted to the
  calibrated mat region (an unrestricted detector once locked onto a larger
  red object in the background); minimum blob area to reject noise.
- **Verification.** After lifting, the arm returns to the view pose and
  looks again. A same-color blob within 40 px of the original detection
  means a clean miss; a blob elsewhere on the mat means the block was
  knocked away; the bottom 130 px of the frame is excluded as the gripper's
  own blind spot (a held block legitimately appears there). The robot
  reports each case distinctly — "I missed it" and "I knocked it away" are
  different failures with different fixes, and the disagreement rate
  between this self-report and operator ground truth is itself measured by
  the evaluation harness. **[DATA: verify-layer disagreement rate.]**

### 3.7 Calibration quality achieved

The current production map kept 9 of 12 points with **mean residual
4.2 mm, worst 7.9 mm**, plus live-tuned constant trims (persisted in the
map, not the code). Informal prime-zone (x ≤ 0.19 m) success was ~80%
before formal trials. **[DATA: formal rates from §5.]**

## 4. The voice loop

### 4.1 Ears

`stt_node` streams raw PCM from the USB microphone (an `arecord`
subprocess — no audio libraries), gates it with WebRTC VAD into utterances
(30 ms frames; speech starts on a majority-voiced window with 300 ms of
pre-roll so first words aren't clipped; ~800 ms of silence ends the
utterance; sub-0.4 s fragments are discarded as clicks), and transcribes
each utterance with faster-whisper `base.en` (int8, 4 threads, beam 1) on
the CPU. Transcripts publish to `/voice_words` — the same topic the typed
console uses, so speech is literally typing.

Whisper hallucinates fixed phrases on breath noise — our live logs showed a
stream of bare "You" and "Thank you" utterances reaching the LLM. A small
stop-list of known hallucinations filters them; an optional wake word is
supported but off by default.

Voice input is deliberately **opt-in per session**: an open microphone in a
shared lab means anyone's conversation can command a moving robot arm.

### 4.2 Half-duplex

The robot must not obey itself. `tts_node` publishes `/tts_busy = true`
around every playback, and the STT node discards audio while busy plus a
0.7 s echo tail afterwards. This is the standard speakerphone problem in
miniature; the tail constant is a tunable parameter pending the combined
live test. **[DATA: talk+listen protocol P2 result.]**

### 4.3 Voice

`tts_node` speaks every `/robot_speech` message through the USB speaker,
preferring piper (neural, natural — voice `en_US-amy-medium`) with espeak-ng
as a fallback; utterances are serialized on a worker queue so overlapping
replies can't garble. A detail we're fond of: the STT installer ends with a
**silent self-test** — piper synthesizes "grab the red block" to a WAV file
and faster-whisper transcribes it. The robot's voice tests the robot's ears
with no human speech and no sound in the room, which is exactly what a
shared lab requires.

## 5. Evaluation

**[DATA: this entire section is generated — run protocol P5, then paste
`eval_analysis.py --md` output.]** Design (already implemented): a guided
trial runner walks the operator through a matrix of 10 positions (6 flat,
4 rotated 30–45°) × 3 colors × 4 phrasings × 4 rounds ≈ 120 trials, sending
each phrasing through the complete pipeline including the LLM. The
operator's verdict is ground truth; the robot's per-stage self-log
(detection pixel, mapped XY, sector, wrist delta, timings, outcome) is
cross-referenced for failure analysis. Success rates are reported with
Wilson 95% intervals, which remain valid at per-cell sample sizes.

## 6. Limitations and lessons

### 6.1 System limitations (honest list)

- Grasping is planar-only, at a fixed pitch (80°), of known-color blocks on
  a mat whose position must not drift (it is taped down; mat creep
  invalidated two calibrations). One block per color per scene.
- The depth channel of the RGB-D camera goes unused in the final grasp
  path: the planar map replaced it. We bought depth and shipped geometry —
  an honest reflection of what the task actually required.
- The reasoning step requires cloud connectivity; the local-LLM fallback
  (Ollama on the Pi) works but is slow on four cores.
- Left/right in scene descriptions is currently image-frame and pending a
  one-minute live check of whether it matches the operator's frame.

### 6.2 Hardware fragility as a schedule constraint

A July collision with the mat stripped gear teeth in servo 3 (it buzzes and
freewheels at specific angles). The failure is real but partial — the
system kept its ~80% informal rate — and the swap is deliberately deferred.
Lesson: on hobby-class hardware, *mechanical* degradation is a first-class
project risk, and residual grasp offsets should be re-trimmed after any
gear-train repair.

### 6.3 The remote methodology (a secondary contribution)

From Week 2 onward the robot was programmed without being touched:

- **Code path:** remote-desktop clipboards mangle code (indentation damage,
  truncated lines — half a day lost proving it). All code flows
  laptop → GitHub → `curl` installer on the Pi; the installer cache-busts
  the CDN and rebuilds the ROS package in one command.
- **Eyes:** an MJPEG-over-HTTP node streams the robot's camera to any
  browser; every detection also saves an annotated debug image
  (ROI, detection, orientation) inspectable over SSH.
- **Ground truth without hands:** the robot-places-the-block calibration
  (§3.2) is what made remote calibration possible at all — the only
  physical intervention a full calibration needs is a human nearby when a
  block rolls (and the tool's retry flow assumes even that may be
  unavailable).
- **Lab-time discipline:** anything requiring physical presence is written
  up in advance as a self-contained protocol with expected outcomes and
  decision tables (`lab_protocols.md`), so lab minutes are spent executing,
  not thinking.

### 6.4 Integration lessons (each one cost us a day or a part)

1. Vendor calibration stacks can fail silently *and* plausibly — verify the
   transform is not factory-default before trusting any of its outputs.
2. Post-hoc offset tuning cannot rescue a bad transform; six iterations of
   evidence say so.
3. LLM providers change model semantics under stable-looking names; log
   per-call latency so a silent regression is visible the day it happens.
4. On-device STT hallucinates deterministically; filter known phantoms
   before they reach an actuator-controlling LLM.
5. USB device lifecycles (camera) need explicit ownership rules — our chat
   launcher deliberately does *not* kill the camera on exit, because the
   kill/restart cycle raced USB release and jammed the device.
6. Blanket process-killers (`pkill`-style "stop everything with ros in the
   name") will eventually kill your own tooling; kill surgically.

## 7. Future work

End-to-end VLA comparison on stronger compute (the same task suite gives a
direct baseline); depth-informed grasping for non-planar objects;
vision-selected place destinations ("put it in the red bowl") replacing the
current named-coordinate table; multi-object scenes with linguistic
disambiguation ("the one on the left" — the describe skill already extracts
the needed spatial predicates); and porting the LLM step to a small local
model as CPU inference of 3B-class models matures.

## Appendix pointers

- **A. Map format:** `planar_map.yaml` — homography, raw correspondences
  (kept for refit-on-load), view pose, heights, trims, destinations.
- **B. Trial log schema:** one JSONL line per attempt — `source`,
  `phrasing`, `llm_s`, `sector`, `pixel`, `cal_xy`, `target_xy`,
  `wrist_delta`, `stages{}`, `outcome`, `duration_s`.
- **C. Agent prompt:** full system prompt text (in `arm_agent.py`).
