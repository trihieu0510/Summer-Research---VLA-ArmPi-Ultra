# Session Handoff — July 21, 2026 (supersedes 2026-06-25)

> **Read this first, fully, before touching anything.** It contains every live-verified
> constant, every gotcha, and the exact current state. Operational commands:
> `Documents/COMMANDS.md`. Journal: `Daily Update.txt`. Schedule: `Documents/PLAN.md`.

---

## 1. Project + where we are

Voice-controlled manipulation on a **Hiwonder ArmPi Ultra** (6-DOF, Raspberry Pi 5/16GB
**no GPU**, STM32 servo board, arm-mounted Aurora 930 RGB-D camera, WonderEcho USB
mic+speaker). Pipeline: `speech/text → LLM (DeepSeek) → vision → planar map → IK →
servos → spoken reply`. Deadline ~Aug 15.

**Status vs PLAN.md (as of end of Week 4):** Weeks 1–4 functionally complete.
- Working end-to-end, live-verified: **type OR SPEAK "grab the red block" → robot
  scans, finds, grasps (~80% in prime zone), verifies, answers.**
- Remaining: formal 10-pick tally (started, interrupted), live talk+listen combined
  test, evaluation harness + trials (Wk 5–6), demo video + writeup (Wk 7).
- **Hardware yellow flag: servo 3 has stripped gear teeth** (see §7). Spare ordered?
  → CHECK. Swap is the first lab task when the part arrives.

## 2. The remote workflow (unchanged, critical)

Laptop → UltraViewer → PC → LAN (Windows ICS, Pi = `192.168.137.x`) → Pi →
ROS 2 Humble. Edit on the laptop repo (`trihieu0510/Summer-Research---VLA-ArmPi-Ultra`),
push to GitHub, pull on the Pi:

```bash
BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
curl -fsSL "$BASE/install_pi.sh?v=$(date +%s)" | bash
source ~/ros2_ws/install/setup.bash
```

- **NEVER paste code through the remote-desktop clipboard** — it mangles indentation
  and truncates lines. Always the curl route. `?v=$(date +%s)` busts the CDN cache.
- `BASE` is per-terminal — a bare `$BASE` in a fresh shell = "bad/illegal format".
- ICS internet drops sometimes ("No route to host"): on the PC, ncpa.cpl → WiFi
  adapter → Sharing → untick, re-tick, choose the LAN adapter.

## 3. What runs where (current architecture)

All in `armpi_voice` on the Pi (`~/ros2_ws/src/armpi_voice/`), synced from the repo:

| File | Role |
|---|---|
| `arm_agent.py` | LLM agent. Steps: gestures, parametric moves, **pick** (`{"action":"pick","color":"red","place":false}`). Pick skill arms at startup if `~/planar_map.yaml` exists. DeepSeek `deepseek-v4-flash` **with thinking DISABLED** (see §7). Logs `LLM replied in X.Xs`. |
| `planar_common.py` | THE core: detection (HSV + minAreaRect), homography fit (bounded outlier rejection), sector rotation, wrist alignment, droop compensation, `run_pick()` shared by CLI + agent. |
| `planar_calib.py` | Interactive calibration: tune phase (live height/jaw), arm places block at grid points, fits map → `~/planar_map.yaml`. |
| `planar_pick.py` | CLI pick for testing; all knobs as ROS params. |
| `stt_node.py` | Ears: arecord → webrtcvad → faster-whisper (base.en int8) → `/voice_words`. Mutes on `/tts_busy` + 0.7s tail. Noise filter drops whisper hallucinations ("You", "Thank you"). |
| `tts_node.py` | Voice: piper (natural) or espeak. Publishes `/tts_busy` Bool around playback. |
| `arm_console.py` | Typed chat REPL (publishes `/voice_words` — same seam as STT). |
| `chat.sh` | ONE command runs everything (see §5). |
| repo root: `install_pi.sh`, `install_piper.sh`, `install_stt.sh`, `camera_stream.py` | sync / TTS setup / STT setup+silent self-test / MJPEG feed at `http://192.168.137.x:8080`. |

## 4. LIVE-VERIFIED hardware facts (do not re-derive, do not trust old docs over these)

- Servo units 0–1000, centre 500, **0.24°/unit**.
- **Servo 6 = base**: operator view left=800, right=200 (mirrored on purpose).
- **Servo 1 = gripper: 100 = WIDE OPEN (verified live; higher = narrower), 540 = closed
  on a block.** Old "open=200" docs are WRONG — 200 wedges the block.
- **Servo 3 = damaged** (stripped teeth; buzzes at specific angles, freewheels there
  when powered off). Servo 2 = wrist roll (used for jaw alignment; map for 2–5 still
  not fully verified).
- Grasp pitch = **80** (mined from Hiwonder's own `pick_and_place.pick(position, 80,
  yaw, 540, ...)`), pitch_range [55, 120].
- View pose (servo 1..6): **[500, 500, 208, 995, 753, 500]** — camera sees the mat.
  Stored in the map; ALL detection happens from it (camera is ARM-MOUNTED).
- Audio: card 2 = "USB PnP Audio Device" = BOTH speaker (`aplay`) and mic (`arecord`)
  → `plughw:2,0` for both.
- Reach: calibrated grid x 0.13–0.22 m, y ±0.09 m; **beyond r≈0.22 physically
  unreachable** (IK refuses). Sector scan (±150 units ≈ ±36° base) sweeps that band
  across ~110° of arc. Droop compensation: +5mm for r>0.19, +10mm r>0.22.
- Camera topic `/depth_cam/rgb/image_raw`, ~10 Hz, **sensor_data QoS required**.
- IK: service `/kinematics/set_pose_target` (SetRobotPose: position, pitch,
  pitch_range, resolution, duration → success, pulse[5] for servos **[6,5,4,3,2]**).
  Solve-only; publish pulses yourself on `/servo_controller` (servo_controller_msgs,
  position_unit='pulse'). IK TIMEOUT = the SDK isn't running, not a solver failure.

## 5. Runbook

```bash
# Everything, one command (serial cleanup + camera-if-needed + SDK + agent + TTS + chat):
bash ~/ros2_ws/src/armpi_voice/chat.sh
# Quiet room:            ARMPI_NO_TTS=1 bash .../chat.sh
# With ears (opt-in!):   ARMPI_STT=1 ARMPI_MIC=plughw:2,0 bash .../chat.sh
# chat.sh does NOT kill the camera on exit (by design) but DOES kill the SDK.

# Background services manually (needed for planar_calib / planar_pick standalone):
export need_compile=False CAMERA_TYPE=aurora
nohup ros2 launch peripherals depth_camera.launch.py > /tmp/cam.log 2>&1 &
nohup ros2 launch sdk armpi_ultra.launch.py > /tmp/sdk.log 2>&1 &

# Calibrate (~15 min; only when geometry changed — mat is now TAPED, so rarely):
ros2 run armpi_voice planar_calib --ros-args -p grid_x:="[0.13, 0.16, 0.19, 0.22]" -p grid_y:="[-0.09, 0.0, 0.09]"

# Test picks with all knobs:
ros2 run armpi_voice planar_pick --ros-args -p color:=red -p wrist_sign:=0 -p trim_x:=0.01 -p scan:=false
```

Logs: `/tmp/armpi_agent.log`, `/tmp/armpi_stt.log`, `/tmp/armpi_tts.log`,
`/tmp/armpi_sdk.log`, `/tmp/cam.log`. Every camera look saves
**`/tmp/pick_debug.jpg`** (green box = search ROI, red circle = detection, blue line
= estimated block orientation) — copy into `~/ros2_ws/` to view in VS Code.

## 6. The planar system (our replacement for Hiwonder's broken hand-eye)

History in one line: the vendor hand-eye matrix was factory-default forever (their GUI
deadlocks silently pre-window if SDK isn't up); two GUI solves still left
position-dependent error; we replaced the whole layer.

- `planar_calib`: arm carries a block to known robot XY grid points, releases,
  detects from the view pose, re-grasps → pixel↔XY pairs with the robot as its own
  ground truth. Interactive: tune phase first (d/u height steps, number = jaw pulse),
  r = retry a rolled point.
- Map = `~/planar_map.yaml`: homography + raw points + heights (z_hover/z_place/
  pitch/grips/**trim_x/trim_y**) + view_pose. **load_map REFITS from raw points at
  every load** (map upgrades travel automatically) with bounded outlier rejection
  (drop ≤1/3, thresh 12mm). Trusted zone = bbox of SURVIVING points + 2.5cm.
- `run_pick` order: sector scan (0, +150, −150 base units) → detect (median of 5,
  angle via mod-90 CIRCULAR mean) → homography → zone guard (calibration frame) →
  rotate to true frame (sector) → trim → wrist alignment (deadband: skip <13°;
  `wrist_sign` flips, 0 disables) → droop-compensated approach → grasp → verify
  (same-spot = missed FIRST, then knocked-away; gripper blindspot = bottom 130px).
- **Current map quality: 9/12 kept, mean 4.2mm, worst 7.9mm.** Trim values: check
  `heights:` in `~/planar_map.yaml` (operator tuned them live on Jul 21 — verify
  they were persisted!).
- **Sector scan (`scan:=true` default) is BUILT BUT the base_rot_sign was NOT yet
  live-verified** — first sector test pending; if side picks mirror, use
  `base_rot_sign:=-1` and bake it in.

## 7. Known issues & their fixes

1. **SERVO 3: stripped gears (Jul 21).** Buzzes/stalls at specific angles; freewheels
   there by hand when off. From the Jul 10 mat crash. → Order/replace (read model off
   its label; set new servo ID=3 via `~/software/servo_tool/servo_tool.sh` GUI on VNC;
   new servos ship ID=1). Until swapped: short sessions, no forcing; backlash may
   explain residual grasp offsets — RE-TRIM after the swap.
2. Camera dies with `open device failed 0x21003` = USB held by a zombie →
   `pkill -f aurora930; pkill -f depth_camera`, relaunch; last resort unplug/replug.
   "I can't see a block" almost always = camera down, not vision.
3. `deepseek-chat` name RETIRED Jul 24 — we already use `deepseek-v4-flash` **with
   `thinking: {type: disabled}`** (v4 thinks by default = seconds of latency; agent
   falls back gracefully if the param is rejected). Watch `LLM replied in X.Xs` (~1–3s
   is right).
4. Whisper hallucinates "You"/"Thank you" on breaths → stt_node drops them
   (NOISE_TRANSCRIPTS). STT stays **opt-in** (open mic in a shared room = anyone
   commands the robot).
5. `~/.stop_ros.sh` is a kill-9 nuke on anything matching "ros" — chat.sh has its own
   surgical version. Never run the nuke mid-session.
6. Servos hold torque when powered; to fix hardware, POWER OFF (support the arm).
7. The mat MUST stay taped down — mat creep invalidated two calibrations before.

## 8. Immediate next steps (ordered)

1. **Swap servo 3** when the spare arrives → re-run one quick calibration (geometry
   may shift) → RE-TRIM.
2. **Formal 10-pick tally** in prime zone (x≤0.19), 6 straight + 4 rotated → this
   number formally closes M1. Was ~80% informally on Jul 21.
3. **Sector-scan live sign test** (block ~35° left, then right).
4. **Talk+listen combined test** (TTS on + STT on; half-duplex is wired, unverified).
5. **Eval harness** (remote-buildable): per-trial JSONL logging in arm_agent; then
   Wk 6 trials (~130 across positions/phrasings/colors, Wilson CIs).
6. Optional/queued: YOLO "what do you see?" (`ultralytics` already at `~/software/`),
   place destinations, demo video (Wk 7), writeup.

## 9. Session log quick-index (for archaeology)

- Jun 25: agent+chat+TTS built (previous handoff).
- Jul 6–10: hand-eye GUI saga → planar pipeline built → first verified picks →
  agent pick wiring ("grab the red block" from chat). Gripper screw incident + fix.
- Jul 13: STT built + live spoken commands worked same day; noise filter; ICS fix.
- Jul 21: thinking-mode latency fix; camera lifecycle fix; wrist yaw alignment
  (circular mean + deadband); constant trim knob; **sector scan** (~3× workspace);
  servo 3 diagnosed. Informal pick rate ~80% prime zone.
