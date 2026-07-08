#!/usr/bin/env bash
# chat.sh — ONE-terminal chat for the ArmPi Ultra.
#
# Starts the arm hardware (sdk) and the LLM agent in the BACKGROUND, then drops
# you into an interactive chat console in the foreground. Type a command, see the
# robot's reply. Quitting the console ("quit" / Ctrl+D) tears the background down.
#
# Run from inside the ROS 2 container, after building armpi_voice:
#     bash ~/ros2_ws/src/armpi_voice/chat.sh
#
# API KEY: put your DeepSeek key (one line) in ~/.armpi_key — kept out of git.
#     echo "sk-your-key-here" > ~/.armpi_key
# Background logs (for debugging): /tmp/armpi_sdk.log and /tmp/armpi_agent.log

# Note: deliberately NOT using `set -u` — sourcing ROS 2 setup scripts trips
# over unset variables and would abort the whole script.

# --- API key, from ~/.armpi_key (never committed) ---
if [ -f "$HOME/.armpi_key" ]; then
    export LLM_API_KEY="$(cat "$HOME/.armpi_key")"
fi
if [ -z "${LLM_API_KEY:-}" ]; then
    echo "WARNING: no LLM_API_KEY found. Put your key in ~/.armpi_key:"
    echo "    echo \"sk-your-key\" > ~/.armpi_key"
    echo "Continuing, but LLM calls will fail until it's set."
fi

# --- source ROS 2 + the workspace (harmless if already sourced) ---
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# --- tear down background services on exit ---
cleanup() {
    echo
    echo "Shutting down background services..."
    [ -n "${TTS_PID:-}" ]   && kill "$TTS_PID" 2>/dev/null
    [ -n "${AGENT_PID:-}" ] && kill "$AGENT_PID" 2>/dev/null
    [ -n "${SDK_PID:-}" ]   && kill -INT "$SDK_PID" 2>/dev/null
    wait 2>/dev/null
    echo "bye."
}
trap cleanup EXIT INT TERM

echo "Freeing the STM32 serial port (stopping the autostart)..."
# ~/.stop_ros.sh does `ps aux | grep ros | ... kill -9` — it nukes EVERY process
# whose command line contains "ros", which (a) kills THIS script, since its path
# contains "ros2_ws", and (b) kills the camera driver + camera_stream too. So we
# do a SURGICAL version: kill the ROS autostart but SPARE the camera and ourselves.
CAMERA_KEEP='ascamera|deptrum|aurora|depth_camera|camera_stream'
for pid in $(ps -eo pid=,args= | grep -i '[r]os' | grep -viE "$CAMERA_KEEP" | awk '{print $1}'); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill -9 "$pid" 2>/dev/null
done
sleep 1   # let the serial port actually release before we grab it

echo "Starting arm hardware (sdk armpi_ultra.launch.py)..."
ros2 launch sdk armpi_ultra.launch.py >/tmp/armpi_sdk.log 2>&1 &
SDK_PID=$!

echo "Starting LLM agent (DeepSeek)..."
# NOTE: "deepseek-chat" is retired 2026-07-24; deepseek-v4-flash is its successor.
ros2 run armpi_voice arm_agent --ros-args \
    -p base_url:=https://api.deepseek.com -p model:=deepseek-v4-flash \
    >/tmp/armpi_agent.log 2>&1 &
AGENT_PID=$!

# Voice output (TTS) — prefer piper (natural) if installed, else espeak (robotic),
# else skip. A missing engine never breaks the chat. Route to the USB speaker
# (card 2) by default; override with ARMPI_TTS_DEVICE.
# Set ARMPI_NO_TTS=1 to start with voice OFF (quiet — e.g. shared room / lab).
DEV="${ARMPI_TTS_DEVICE:-plughw:2,0}"
PIPER_BIN="$HOME/piper/piper"
PIPER_MODEL="$HOME/piper/en_US-amy-medium.onnx"
if [ -n "${ARMPI_NO_TTS:-}" ]; then
    echo "(ARMPI_NO_TTS set — voice output OFF.)"
elif [ -x "$PIPER_BIN" ] && [ -f "$PIPER_MODEL" ]; then
    echo "Starting voice output (piper — natural voice)..."
    ros2 run armpi_voice tts_node --ros-args \
        -p engine:=piper -p piper_bin:="$PIPER_BIN" -p piper_model:="$PIPER_MODEL" \
        -p alsa_device:="$DEV" >/tmp/armpi_tts.log 2>&1 &
    TTS_PID=$!
elif command -v espeak-ng >/dev/null 2>&1; then
    echo "Starting voice output (espeak)..."
    ros2 run armpi_voice tts_node --ros-args \
        -p alsa_device:="$DEV" >/tmp/armpi_tts.log 2>&1 &
    TTS_PID=$!
else
    echo "(No TTS engine found — skipping voice. Install espeak-ng or run install_piper.sh.)"
fi

echo "Waiting ~8s for hardware + agent to come up..."
sleep 8

echo
# Foreground: the interactive chat box. When it exits, the EXIT trap cleans up.
ros2 run armpi_voice arm_console
