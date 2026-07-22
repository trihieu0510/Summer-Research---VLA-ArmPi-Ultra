# armpi_voice

Voice-controlled manipulation for the **Hiwonder ArmPi Ultra** (Raspberry
Pi 5, no GPU, ROS 2 Humble in Docker).

Pipeline: **speech/typed text → LLM agent → gestures / moves / vision-guided
pick → IK → servos → spoken reply.**

> The authoritative operational docs are `Documents/session_handoff_*.md`
> (latest state + verified constants), `Documents/COMMANDS.md` (cheat sheet)
> and `Documents/lab_protocols.md` (pending live tests). This README is the
> package overview.

## Run it (the normal way)

```bash
bash ~/ros2_ws/src/armpi_voice/chat.sh          # SDK + camera + agent + TTS + chat
# quiet room:          ARMPI_NO_TTS=1 bash .../chat.sh
# with ears (opt-in):  ARMPI_STT=1 ARMPI_MIC=plughw:2,0 bash .../chat.sh
```

Type (or say) things like: `grab the red block`, `put the blue block in the
box`, `what do you see?`, `go home, then turn the base right about 100`.

## What's in the package

| Module | Role |
|---|---|
| `arm_agent.py` | The brain: LLM agent (DeepSeek `deepseek-v4-flash`, thinking disabled) returning `{"steps": [...], "say": "..."}`. Skills: gestures, parametric moves, `pick(color, place)`, `describe`. |
| `planar_common.py` | The core pick machinery: HSV detection, homography map, sector scan, wrist alignment, droop compensation, `run_pick()`. |
| `planar_calib.py` | Interactive calibration — the arm generates its own pixel↔XY ground truth → `~/planar_map.yaml`. |
| `planar_pick.py` | CLI pick with every knob as a ROS param (testing). |
| `stt_node.py` | Ears: arecord → webrtcvad → faster-whisper → `/voice_words`. |
| `tts_node.py` | Voice: piper (natural) or espeak; publishes `/tts_busy` for half-duplex. |
| `arm_console.py` | Typed chat REPL (same `/voice_words` seam as STT). |
| `eval_runner.py` | Guided, resumable evaluation sessions → `~/eval_trials.jsonl`. |
| `eval_analysis.py` | Wilson-CI success-rate report (stdlib-only; runs on the laptop too). |
| `scene_describe.py` | "What do you see?" — HSV blocks + optional YOLOv8n. |
| `trial_log.py` | One JSONL line per pick attempt (never breaks a pick). |
| `voice_arm_control.py` | **Legacy** first prototype (single-action classifier). Superseded by `arm_agent`; kept for history. Its old servo values predate live verification — trust the table below, not that file. |

## Live-verified servo facts (do not trust older docs)

- Units 0–1000, centre 500, 0.24°/unit.
- **Servo 1 = gripper: 100 = WIDE OPEN, 540 = closed on a block.**
  (Old "open=200" docs are wrong — 200 wedges the block.)
- **Servo 6 = base:** operator view left = 800, right = 200.
- Grasp pitch 80, range [55, 120]. View pose: `[500, 500, 208, 995, 753, 500]`.

## Deploy (never paste code over remote desktop — it mangles)

```bash
BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
curl -fsSL "$BASE/install_pi.sh?v=$(date +%s)" | bash
source ~/ros2_ws/install/setup.bash
```

API key: one line in `~/.armpi_key` (never committed). TTS/STT setup:
`install_piper.sh` / `install_stt.sh`.

## Test without hardware attached to your voice

```bash
ros2 topic pub --once /voice_words std_msgs/msg/String "{data: 'grab the red block'}"
```

Logs land in `/tmp/armpi_*.log`; every camera look saves
`/tmp/pick_debug.jpg`.
