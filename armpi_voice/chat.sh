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

set -u

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
    [ -n "${AGENT_PID:-}" ] && kill "$AGENT_PID" 2>/dev/null
    [ -n "${SDK_PID:-}" ]   && kill -INT "$SDK_PID" 2>/dev/null
    wait 2>/dev/null
    echo "bye."
}
trap cleanup EXIT INT TERM

echo "Freeing the STM32 serial port..."
[ -x "$HOME/.stop_ros.sh" ] && "$HOME/.stop_ros.sh"

echo "Starting arm hardware (sdk armpi_ultra.launch.py)..."
ros2 launch sdk armpi_ultra.launch.py >/tmp/armpi_sdk.log 2>&1 &
SDK_PID=$!

echo "Starting LLM agent (DeepSeek)..."
ros2 run armpi_voice arm_agent --ros-args \
    -p base_url:=https://api.deepseek.com -p model:=deepseek-chat \
    >/tmp/armpi_agent.log 2>&1 &
AGENT_PID=$!

echo "Waiting ~8s for hardware + agent to come up..."
sleep 8

echo
# Foreground: the interactive chat box. When it exits, the EXIT trap cleans up.
ros2 run armpi_voice arm_console
