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
what can you do?
```

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
ros2 run armpi_voice arm_agent --ros-args -p base_url:=https://api.deepseek.com -p model:=deepseek-chat

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

## 6. Pick-and-place demo (vision → grasp; needs lab calibration to land)

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
