# Lab Protocols — pending live tests (prepared 2026-07-22, remotely)

Each protocol is self-contained: preconditions, exact commands, expected
result, and what to do on each outcome. Order them however lab time allows —
they are independent, EXCEPT that the eval session (P5) is best run last,
after P1 confirms the sector sign.

Common setup for all of them (one terminal):

```bash
BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
curl -fsSL "$BASE/install_pi.sh?v=$(date +%s)" | bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/src/armpi_voice/chat.sh        # SDK + camera + agent + TTS
```

---

## P1 — Sector-scan sign test (~5 min)

**What it settles:** whether `base_rot_sign` is +1 or −1. The sector scan is
built and shipped but the rotation-back sign was never verified on hardware.

1. Put the red block roughly **35° to the LEFT** of straight ahead (as seen
   from BEHIND the arm), inside the calibrated radius band (r ≈ 0.15–0.19 m).
2. In the chat: `grab the red block`.
3. Watch the log line `Found after rotating base +150 units.` then the
   mapped target in `-> robot (x, y)`.

| Result | Meaning | Action |
|---|---|---|
| Grasps the block | sign is correct | done — repeat once on the RIGHT to confirm |
| Reaches MIRRORED (right when block is left) | sign is flipped | test with `ros2 run armpi_voice planar_pick --ros-args -p color:=red -p base_rot_sign:=-1`, and if that grasps, bake `base_rot_sign=-1` into the default in `planar_pick.py` + `run_pick()` |
| "outside the zone" refusal | mapped XY landed outside guard — check `/tmp/pick_debug.jpg` first | if detection is clean, the sign is likely flipped (mirrored point fails the y-guard); try `base_rot_sign:=-1` |

4. Repeat on the RIGHT side (~35° right). Both sides must grasp.
5. Log the confirmed sign in `Daily Update.txt`.

## P2 — Talk+listen combined test (~5 min)

**What it settles:** the half-duplex loop (`/tts_busy` mute + 0.7 s echo
tail) — wired but never live-verified with both TTS and STT on.

1. `ARMPI_STT=1 ARMPI_MIC=plughw:2,0 bash ~/ros2_ws/src/armpi_voice/chat.sh`
   (TTS on — do NOT set ARMPI_NO_TTS).
2. Say: **"nod"**. Robot should nod and speak a reply.
3. THE TEST: watch `/tmp/armpi_stt.log` while the robot speaks. Its own
   reply must NOT appear as a transcription (muted + tail).
4. Immediately after the robot finishes speaking, say **"open your gripper"**
   — the first word must not be clipped by the echo tail.
5. Chain: "grab the red block" spoken → full pick → spoken outcome →
   speak another command. Three turns without a self-transcription = PASS.

Failure modes: robot transcribes itself → raise `echo_tail_s` (0.7 → 1.2);
first words clipped → lower it. It's a stt_node param, tunable live.

## P3 — "What do you see?" first light (~5 min)

**What it settles:** the new describe skill (built 2026-07-22, never run on
hardware).

1. Put 2–3 colored blocks on the mat, plus one everyday object (cup, phone).
2. In the chat: `what do you see?`
3. Expected: arm goes to view pose, ~2–5 s pause, then a natural sentence
   ("On the mat I can see a red block on the left and a blue block in the
   middle. Around me I also spot a cup.").
4. Check `/tmp/describe_debug.jpg` (copy to `~/ros2_ws/` to view): circles on
   blocks, yellow boxes on YOLO objects.
5. **Verify left/right wording matches reality** — it's image-frame; if
   mirrored versus the operator's view, flip the wording in
   `scene_describe.py::_thirds` (left↔right).
6. If it says blocks-only and the log warns `YOLO unavailable`: check where
   the weights live (`find ~/software -name "*.pt"`) and pass
   `-p yolo_model:=<path>` to arm_agent (or let ultralytics download
   `yolov8n.pt` once, with internet up). Blocks-only is still a pass for the
   HSV half.

## P4 — Measure in the real destinations (~5 min)

**What it settles:** real coordinates for "put it in the box".

1. Place the box/bowl where it will live for the demo.
2. Find its robot-frame XY: run one pick with place and watch where the arm
   releases: `put the red block in the box` (currently uses the default drop
   spot 0.13, −0.12).
3. Nudge coordinates until the release lands in the box:
   `ros2 run armpi_voice planar_pick --ros-args -p color:=red -p place:=box -p place_x:=0.14 -p place_y:=-0.14`
   (explicit place_x/place_y override the name while tuning).
4. Persist the result in the map — add to `~/planar_map.yaml` (top level):

   ```yaml
   destinations:
     box: [0.14, -0.14]
   ```

   Names in the map override the code table; no rebuild needed.

## P5 — Formal evaluation session (the Week-6 numbers)

**What it produces:** operator-verified success rates with Wilson CIs — the
headline numbers for the writeup. ~120 trials at roughly 1 min/trial; split
across multiple sessions freely — the runner RESUMES (skips logged trials).

1. Stack up (chat.sh), then in another terminal:
   `ros2 run armpi_voice eval_runner`
   (Smaller bite: `-p rounds:=1` = 30 trials. After P1 confirms the sector
   sign, add sector positions with `-p include_sector:=true`.)
2. Follow the prompts: place the block where it says, press Enter, confirm
   the verdict (Enter accepts the robot's own claim; override if the robot
   is wrong — e.g. claims success with an empty gripper).
3. Trials land in `~/eval_trials.jsonl`; the robot's own per-stage log in
   `~/pick_trials.jsonl`. Copy both off the Pi at the end of every session.
4. Analyze (Pi or laptop):
   `python3 eval_analysis.py ~/eval_trials.jsonl ~/pick_trials.jsonl --md report.md`

Protocol discipline for honest numbers: don't re-roll failed trials (that's
what `r = redo` is NOT for — redo only on operator mistakes, e.g. wrong
placement); place blocks within ~1 cm of the stated point; keep the mat
taped; note anything odd in the `o` verdict's note field.
