# Session Handoff — June 25, 2026 (end of day)

> Read this first next session. It's the full picture: what the project is, how the
> machines connect, what's built, what works, and what's next. Operational commands
> live in `Documents/COMMANDS.md`; the running journal is `Daily Update.txt`.

---

## 1. The project in one paragraph

Voice-controlled robotic manipulation on a **Hiwonder ArmPi Ultra** (6-DOF arm,
**Raspberry Pi 5 / 16 GB, no GPU**, STM32 servo controller, Aurora RGB-D camera,
WonderEcho USB mic, USB speaker). Goal: **speak/type a command → the robot perceives,
reasons, acts, and talks back.** Architecture is a **modular pipeline** (not end-to-end):
`speech/text → LLM reasoning → (vision + depth + calibration) → IK → servos → voice`.
The LLM is **DeepSeek cloud** (`deepseek-chat`). 7-week plan runs to mid-August.

## 2. How the machines connect (critical)

You work **fully remotely** right now:
**you → remote desktop → a PC → LAN cable → Raspberry Pi 5**.
- The Pi runs **ROS 2 Humble inside a Docker container** (Hiwonder's image). The host Pi
  OS has **no ros2** — every `ros2` command runs in the container.
- Internet reaches the Pi via **Windows ICS on the PC** (Pi gets `192.168.137.x`). ICS
  drops easily when the PC/network changes — symptom is API calls timing out at DNS.
- You edit on the Pi via **VS Code Remote-SSH** (workspace opened at `~/ros2_ws`).

### The remote-work gotcha that ate half the day
The **remote-desktop clipboard mangles pasted code** — it adds leading indentation
(breaks Python) and truncates long lines. **Do not paste code or long commands.**
Instead, edits are pushed to GitHub from the laptop and **pulled onto the Pi via
`curl`** (the repo is public). `install_pi.sh` does this in one short command. See COMMANDS.md.

## 3. What's built and working (as of today)

### Remote live video — `camera_stream.py`
MJPEG-over-HTTP server: subscribes to the camera RGB topic, serves frames to a browser.
View at `http://192.168.137.<pi>:8080` from the PC. Lets you watch the arm remotely
(laggy — good for confirming a gesture, not judging fast motion).

### The conversational + motion agent — `armpi_voice/arm_agent.py`
An LLM **agent** (not just a classifier). Each turn it returns
`{"steps": [...], "say": "..."}`:
- **`say`** → always published on `/robot_speech` (spoken by TTS) and shown in the chat.
- **`steps`** → an *ordered* list executed one-by-one. Each step is either:
  - a named gesture: `{"action":"home"}` (home/left/right/open/close/nod), or
  - a parametric move: `{"action":"move","moves":[{"servo":N,"target":T}|{"servo":N,"delta":D}]}`.
- Tracks each servo's position so **relative deltas** ("up a bit", "a bit more") and
  **chained steps** resolve against the running state.
- Keeps a short rolling conversation memory.
- Works: `"go home, then turn the base right about 100, then raise motor 4 by 300"` runs
  as a sequence; `"turn halfway right"`, `"nod and open the gripper"`, plain chat — all work.

### The one-terminal chat — `arm_console.py` + `chat.sh`
`bash chat.sh` starts SDK (servo controller) + `arm_agent` + `tts_node` in the
background, then drops you into a turn-based chat prompt (`arm_console`). Quitting tears
everything down. This is the normal way to drive the robot now.

### Voice output (Wk 4) — `armpi_voice/tts_node.py`
Subscribes `/robot_speech`, speaks each reply. Two engines: **espeak-ng** (offline,
robotic) and **piper** (neural, natural — installed via `install_piper.sh`, voice
`en_US-amy-medium` at `~/piper/`). Speaker = **USB PnP audio = ALSA card 2 =
`plughw:2,0`** (cards 0/1 are HDMI). `chat.sh` auto-prefers piper, routes to card 2.
**End state: type → arm moves → robot speaks naturally.**

## 4. What is NOT done / parked

- **Color pick-and-place calibration** — the vision→IK→grasp chain runs end-to-end on the
  stock `object_sorting` demo, but the gripper lands off the block (hand-eye drift). Remote
  offset-tuning got it from ~2 blocks off to grazing, but can't finish: near-misses nudge
  the block, and the error is partly position-dependent. **Real fix = hand-eye calibration
  tool `~/software/calibration/tool.sh` (GUI → needs VNC + physically placing a calibration
  board) — a LAB job.** Working baseline left in `calibration.yaml`:
  `kinematics.offset = [-0.005, -0.065, 0.005]`, `scale = [1.0, 1.07, 1.0]`.
- **Voice INPUT (STT)** — not started. The agent already listens on `/voice_words`; a
  faster-whisper node publishing transcripts there closes the loop (the chat console and a
  mic are interchangeable front-ends).
- **Manipulation skills** — `pick(color)`/`place(location)` not yet refactored out of
  `object_sorting.py`; gated on calibration.

## 5. Servo facts

Raw units **0–1000, 500 = centre**. Hard mechanical limits — test new ranges small.
- **servo 6 = base rotation.** Operator's view: **right = lower (~200), left = higher
  (~800)** — defined this way after it ran mirrored. Relative "right" = negative delta.
- **servo 1 = gripper:** open ~200, closed ~500.
- **servos 2–5 = arm joints** (shoulder/elbow/wrist) — exact map + "up/down" direction
  **unverified**. Convention in the prompt: up = +delta, down = −delta; if a joint goes the
  wrong way, tell the agent "other way" (and we should verify + bake in real directions).

## 6. Key file paths

Laptop repo (this Windows folder, pushed to GitHub `trihieu0510/Summer-Research---VLA-ArmPi-Ultra`):
- `armpi_voice/armpi_voice/arm_agent.py` — the agent (steps schema)
- `armpi_voice/armpi_voice/arm_console.py` — the chat REPL
- `armpi_voice/armpi_voice/tts_node.py` — voice output
- `armpi_voice/chat.sh` — one-command launcher
- `install_pi.sh` / `install_piper.sh` — pull-from-GitHub installers (avoid clipboard)
- `camera_stream.py` — remote MJPEG feed
- `Documents/COMMANDS.md` — the operational cheat sheet

On the Pi:
- `~/ros2_ws/src/armpi_voice/...` — the package (separate from the repo; sync via install_pi.sh)
- `~/ros2_ws/camera_stream.py`, `~/piper/` (piper bin + voice), `~/.armpi_key` (DeepSeek key)
- `~/ros2_ws/src/app/...` — the pick-and-place demo + `config/calibration.yaml`

## 7. Gotchas worth remembering

- **Never paste code into the Pi terminal/editor** — clipboard corrupts it. Push to GitHub,
  pull with `curl` (`install_pi.sh`). Cache-bust the raw URL (`?v=$(date +%s)`) — the CDN
  caches a few minutes.
- **`~/.stop_ros.sh` is a nuke** (`ps|grep ros|kill -9`) — it kills anything with "ros" in
  its command line, including scripts under `~/ros2_ws/` and the camera. `chat.sh` now does a
  surgical version that spares the camera + itself.
- **DeepSeek key** is read from `~/.armpi_key` (never commit it). It was exposed in chat once
  — rotate it.
- **Camera feed is laggy** over remote desktop; trust the agent's `Published servo positions`
  log over the video.
- **Background logs** when using chat.sh: `/tmp/armpi_sdk.log`, `/tmp/armpi_agent.log`,
  `/tmp/armpi_tts.log`.

## 8. Recommended next steps

1. **Remote, high-value:** add **speech-to-text** (faster-whisper) publishing to
   `/voice_words` → full hands-free voice loop (mic → reason → move → speak).
2. **Remote:** verify servo 2–5 "up/down" directions and bake correct conventions into the
   agent prompt.
3. **In lab:** run `tool.sh` hand-eye calibration, finish reliable color pick, then refactor
   into `pick(color)`/`place(location)` and let the agent call them.
