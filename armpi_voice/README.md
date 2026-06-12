# armpi_voice

Voice-commanded control for the **Hiwonder ArmPi Ultra**.

Pipeline: `/voice_words` (speech-to-text transcript) → **DeepSeek LLM** (intent → JSON) → servo motion
on `/ros_robot_controller/bus_servo/set_position`.

Runs **on the Pi**, inside the Hiwonder ROS 2 Humble Docker container.

## Supported actions
`home`, `left`, `right`, `open`, `close`, `nod` (anything else → `unknown`, ignored).
Edit the `ACTIONS` table at the top of `armpi_voice/voice_arm_control.py` to add or tune gestures.

## One-time setup (in the container)
```bash
pip install openai                                   # DeepSeek uses the OpenAI client
cp -r <your-repo>/armpi_voice ~/ros2_ws/src/         # put the package in the workspace
cd ~/ros2_ws
colcon build --packages-select armpi_voice --symlink-install
```

Set your DeepSeek key (add to `~/.bashrc` to persist):
```bash
export DEEPSEEK_API_KEY="sk-..."
```

## Run
```bash
# Terminal 1 — free the serial port, then start the arm hardware
~/.stop_ros.sh
ros2 launch sdk armpi_ultra.launch.py

# Terminal 2
cd ~/ros2_ws && source install/setup.bash
ros2 run armpi_voice voice_arm_control
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
| `model` | `deepseek-chat` | LLM model name |
| `base_url` | `https://api.deepseek.com` | LLM API endpoint (OpenAI-compatible) |
| `voice_topic` | `/voice_words` | input transcript topic |
| `servo_topic` | `/ros_robot_controller/bus_servo/set_position` | output servo topic |
| `request_timeout` | `15.0` | LLM request timeout (seconds) |

Example:
```bash
ros2 run armpi_voice voice_arm_control --ros-args -p request_timeout:=20.0
```

## ⚠️ Verify on the hardware before trusting the motions
These were carried over from the original prototype and are **not yet confirmed**:
- **Servo IDs** — assumed `1 = gripper`, `6 = base rotation`.
- **Pose values** — the position numbers in `ACTIONS` are starting guesses; tune them with the arm clear.
- **Topic / message type** — confirm with `ros2 topic list` / `ros2 interface show` under your launch file
  (the old `main.py` used a different topic, `/servo_controller`).
