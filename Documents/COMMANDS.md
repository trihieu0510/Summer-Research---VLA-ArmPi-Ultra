# COMMANDS.md — ArmPi Ultra operational cheat sheet

Quick reference for running the robot. All commands run **on the Pi, inside the ROS 2
container** (VS Code Remote-SSH → terminal). Pi is at `192.168.137.x` on the LAN.

---

## 0. One-time setup (per fresh Pi / after reflash)

```bash
# DeepSeek API key (never committed; read by the agent):
echo "sk-your-deepseek-key" > ~/.armpi_key

# Natural voice (piper + en_US-amy voice + a test wav):
BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
curl -fsSL "$BASE/install_piper.sh?v=$(date +%s)" | bash

# (espeak fallback, if you ever want it)
sudo apt-get install -y espeak-ng

# Voice INPUT (faster-whisper STT + silent self-test):
curl -fsSL "$BASE/install_stt.sh?v=$(date +%s)" | bash
```

---

## 1. The normal way to drive the robot — ONE terminal

```bash
bash ~/ros2_ws/src/armpi_voice/chat.sh
```
Starts SDK + agent + TTS in the background, drops you into a chat prompt. Type commands,
the arm moves and speaks. `quit` (or Ctrl+D) exits and cleans everything up.

**Example commands to type:**
```
turn left
turn halfway to the right
nod and open the gripper
move motor 3 up a bit
go home, then turn the base right about 100, then raise motor 4 by 300
grab the red block
pick up the blue block and put it away
what can you do?
```
The pick commands need `~/planar_map.yaml` (run `planar_calib` once — sec 5b) and
the depth camera; chat.sh now auto-starts the camera if it isn't running.

**Hands-free (speak instead of type):** `ARMPI_STT=1 ARMPI_MIC=plughw:1,0 bash chat.sh`
(mic device from `arecord -l`; needs install_stt.sh once). STT is OPT-IN because an
open mic in a shared room lets anyone's conversation command the robot. The robot
mutes its ears while speaking (/tts_busy) so it never obeys its own voice. Typing
in the console keeps working alongside — both feed /voice_words.

---

## 2. Live video feed (watch the robot remotely)

Two terminals (the camera is independent of the chat). **Start the camera AFTER chat.sh**
if both are running (chat.sh's serial cleanup spares the camera, but only one already up).

```bash
# Terminal A — camera driver:
export need_compile=False
export CAMERA_TYPE=aurora
ros2 launch peripherals depth_camera.launch.py

# Terminal B — the MJPEG stream:
python3 ~/ros2_ws/camera_stream.py --topic /depth_cam/rgb/image_raw --port 8080
```
Then open **`http://192.168.137.<pi-ip>:8080`** in the PC's browser. Find the Pi IP with
`hostname -I`.

---

## 3. Syncing code from GitHub (NEVER paste code — clipboard corrupts it)

```bash
BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
curl -fsSL "$BASE/install_pi.sh?v=$(date +%s)" | bash
```
Downloads the latest `armpi_voice` files from GitHub and rebuilds. The `?v=$(date +%s)`
busts the GitHub CDN cache (it caches files for a few minutes). After it runs:
```bash
source ~/ros2_ws/install/setup.bash
```

---

## 4. Manual / multi-terminal (for debugging)

```bash
# Terminal 1 — free the serial port, start the arm hardware:
~/.stop_ros.sh
ros2 launch sdk armpi_ultra.launch.py

# Terminal 2 — the agent (export the key first):
export LLM_API_KEY="$(cat ~/.armpi_key)"
ros2 run armpi_voice arm_agent --ros-args -p base_url:=https://api.deepseek.com -p model:=deepseek-v4-flash

# Terminal 3 — send a command without the chat console:
ros2 topic pub --once /voice_words std_msgs/msg/String "{data: 'turn left'}"

# Terminal 4 — watch the spoken replies:
ros2 topic echo /robot_speech
```

---

## 5. TTS (voice output) quick tests

```bash
# espeak straight to the USB speaker (card 2):
espeak-ng --stdout "test" | aplay -D plughw:2,0

# piper (natural) to a wav you can download + listen on the PC:
echo "hello from the robot" | ~/piper/piper --model ~/piper/en_US-amy-medium.onnx --output_file ~/ros2_ws/piper_test.wav

# run the TTS node alone (piper, card 2):
ros2 run armpi_voice tts_node --ros-args -p engine:=piper \
  -p piper_bin:=$HOME/piper/piper -p piper_model:=$HOME/piper/en_US-amy-medium.onnx \
  -p alsa_device:=plughw:2,0
```
List audio devices: `aplay -l` (speaker = card 2 "USB PnP Audio Device").
Hear audio remotely: write a WAV, copy into `~/ros2_ws/`, download in VS Code, play on PC.

---

## 5b. PLANAR calibration + pick (our own tools — replaces vendor hand-eye)

The vendor hand-eye tool gave position-dependent error twice (2026-07-08), so
grasping now uses a planar pixel→XY affine map fitted by our own tool. Camera
is ARM-MOUNTED: detection always happens from one fixed view pose.

```bash
# Prereq: camera + SDK running in the background (survives terminal reuse):
export need_compile=False CAMERA_TYPE=aurora
nohup ros2 launch peripherals depth_camera.launch.py > /tmp/cam.log 2>&1 &
nohup ros2 launch sdk armpi_ultra.launch.py > /tmp/sdk.log 2>&1 &

# 1. Calibrate (~15 min, interactive; robot places a block at known XY points):
ros2 run armpi_voice planar_calib                      # red block by default
#   -> fits affine, prints residuals (mm), saves ~/planar_map.yaml

# 2. Pick:
ros2 run armpi_voice planar_pick                       # detect red, grasp, hold
ros2 run armpi_voice planar_pick --ros-args -p color:=blue -p place_after:=true
```
Tunables (pass as -p name:=value): z_place (grip height; tuned live in the tune
phase), z_hover (0.10), pitch (80.0), grid_x/grid_y (the calibration grid),
grip_close (540), gripper_open (100 = wide open; higher = narrower).

## 6. Pick-and-place demo (vendor's; superseded by 5b for grasping)

```bash
# Terminal A — camera:
~/.stop_ros.sh
ros2 launch peripherals depth_camera.launch.py
# Terminal B — SDK:
ros2 launch sdk armpi_ultra.launch.py
# Terminal C — the sorting node (wait for "init finish"):
ros2 run app object_sorting --ros-args -p start:=true -p display:=false -p broadcast:=false
```
Calibration offsets live-reload from `~/ros2_ws/src/app/config/calibration.yaml` every grasp
(no restart). Working baseline: `kinematics.offset = [-0.005, -0.065, 0.005]`.

---

## 7. ROS 2 introspection (debugging)

```bash
ros2 node list                         # running nodes
ros2 topic list                        # all topics
ros2 topic echo <topic>                # stream a topic
ros2 topic echo /robot_speech          # the agent's spoken replies
ros2 topic hz <topic>                  # publish rate
ros2 pkg executables armpi_voice       # arm_agent / arm_console / tts_node / voice_arm_control
```

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Chat says `(no response)` | Agent not answering. `cat /tmp/armpi_agent.log` — usually key (`~/.armpi_key`) or network (ICS/DNS). |
| API call times out at DNS | ICS not sharing internet. Re-arm sharing on the PC's internet adapter → the `192.168.137.x` LAN adapter. |
| `chat.sh` prints `Killed` | Old bug (stop_ros nuking the script). Re-pull with `install_pi.sh`. |
| Pasted file won't build (IndentationError) | Clipboard mangled it. Don't paste — pull via `install_pi.sh`. |
| Arm doesn't move but chat replies | SDK not up. `cat /tmp/armpi_sdk.log`; check `ros2 node list | grep controller`. |
| No voice | No TTS engine / wrong device. Run `install_piper.sh`; speaker is `plughw:2,0`. |
| Camera feed died | `chat.sh`/stop_ros killed it. Restart camera (section 2) — after chat.sh. |
| Joint moves wrong direction | servo 2–5 up/down unverified; tell the agent "other way". |

Background logs (when using chat.sh): `/tmp/armpi_sdk.log`, `/tmp/armpi_agent.log`,
`/tmp/armpi_tts.log`.
