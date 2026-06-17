# armpi_voice

Voice-commanded control for the **Hiwonder ArmPi Ultra**.

Pipeline: `/voice_words` (speech-to-text transcript) → **LLM** (intent → JSON) → servo motion
on `/ros_robot_controller/bus_servo/set_position`.

Runs **on the Pi**, inside the Hiwonder ROS 2 Humble Docker container. The LLM can run
**off-board on a dev laptop** (recommended during development) or **on the Pi** itself.

## Supported actions
`home`, `left`, `right`, `open`, `close`, `nod` (anything else → `unknown`, ignored).
Edit the `ACTIONS` table at the top of `armpi_voice/voice_arm_control.py` to add or tune gestures.

## LLM backend (local by default — no API key, no cost)

The node talks to any OpenAI-compatible endpoint. Default is a local **Ollama** server.
Use a **3B-class or larger** model for reliable JSON — `qwen:0.5b` is too weak.

### Recommended: model on the dev laptop, Pi calls it over the LAN
The Pi is CPU-only (no GPU); offloading the LLM to the laptop is faster and keeps the
Pi's cores free. The two are already wired together via LAN cable + Windows ICS
(laptop = `192.168.137.1`, Pi = `192.168.137.x`).

On the **laptop** (Windows), one time:
```powershell
# Make Ollama listen on the LAN, not just localhost, then RESTART the Ollama app:
setx OLLAMA_HOST "0.0.0.0:11434"
ollama pull qwen3:8b
# Open the firewall port (admin PowerShell):
New-NetFirewallRule -DisplayName "Ollama LAN" -Direction Inbound -LocalPort 11434 -Protocol TCP -Action Allow
```
> ⚠️ If a **VPN** is active, enable "allow local/LAN access" or disconnect it — VPNs
> commonly block the `192.168.137.x` subnet, which stops the Pi from reaching the laptop.

Verify the Pi can reach it (from an SSH session on the Pi):
```bash
curl http://192.168.137.1:11434/v1/models      # should list qwen3:8b
```

### Alternative: model on the Pi (standalone/offline, slower)
`ollama pull qwen2.5:3b` on the Pi and leave `base_url` at its `localhost` default.

### Alternative: cloud (best reasoning, costs money, needs network)
```bash
export LLM_API_KEY="sk-..."     # never hardcode — this repo is pushed to GitHub
# then launch with -p base_url:=https://api.deepseek.com -p model:=deepseek-chat
```

## Deploy & build (in the container, on the Pi)
```bash
pip install openai                                   # OpenAI-compatible client
# copy the package into the workspace (scp from the laptop, or git pull), then:
cd ~/ros2_ws
colcon build --packages-select armpi_voice --symlink-install
source install/setup.bash
```

## Run
```bash
# Terminal 1 — free the serial port, then start the arm hardware
~/.stop_ros.sh
ros2 launch sdk armpi_ultra.launch.py

# Terminal 2 — the voice node, pointed at the laptop's Ollama
cd ~/ros2_ws && source install/setup.bash
ros2 run armpi_voice voice_arm_control --ros-args \
  -p base_url:=http://192.168.137.1:11434/v1 -p model:=qwen3:8b
```

## Test without a microphone
Publish a fake transcript directly onto the voice topic:
```bash
ros2 topic pub --once /voice_words std_msgs/msg/String "{data: 'turn left'}"
```
Expected: the node logs `Heard: "turn left"` → `Decided action: left` and the base servo rotates.

## Configurable parameters
Override at launch with `--ros-args -p <name>:=<value>`:

| Parameter | Default | Purpose |
|---|---|---|
| `model` | `qwen2.5:3b` | LLM model name (use `qwen3:8b` when pointing at the laptop) |
| `base_url` | `http://localhost:11434/v1` | LLM endpoint; set to `http://192.168.137.1:11434/v1` for the laptop |
| `voice_topic` | `/voice_words` | input transcript topic |
| `servo_topic` | `/ros_robot_controller/bus_servo/set_position` | output servo topic |
| `request_timeout` | `30.0` | LLM request timeout (seconds) |

## ⚠️ Verify on the hardware before trusting the motions
These were carried over from the original prototype and are **not yet confirmed**:
- **Servo IDs** — assumed `1 = gripper`, `6 = base rotation`.
- **Pose values** — the position numbers in `ACTIONS` are starting guesses; tune them with the arm clear.
- **Topic / message type** — confirm with `ros2 topic list` / `ros2 interface show` under your launch file
  (the old `main.py` used a different topic, `/servo_controller`).
