#!/usr/bin/env bash
# install_pi.sh — pull the latest armpi_voice files from GitHub and rebuild.
#
# Avoids copy-paste-through-remote-desktop, which mangles indentation and
# truncates long lines. Run on the Pi with ONE short command:
#
#     curl -fsSL $BASE/install_pi.sh | bash
#
# (where BASE is the raw-GitHub master URL; it's hardcoded below too, so the
#  pipe form works even if BASE isn't set).

set -e

BASE=https://raw.githubusercontent.com/trihieu0510/Summer-Research---VLA-ArmPi-Ultra/master
PKG="$HOME/ros2_ws/src/armpi_voice"

echo "Downloading armpi_voice files from GitHub..."
# Cache-bust the raw.githubusercontent CDN (it caches files for a few minutes).
V="?v=$(date +%s)"
curl -fsSL "$BASE/armpi_voice/setup.py$V"                  -o "$PKG/setup.py"
curl -fsSL "$BASE/armpi_voice/chat.sh$V"                   -o "$PKG/chat.sh"
curl -fsSL "$BASE/armpi_voice/armpi_voice/arm_console.py$V" -o "$PKG/armpi_voice/arm_console.py"
curl -fsSL "$BASE/armpi_voice/armpi_voice/arm_agent.py$V"   -o "$PKG/armpi_voice/arm_agent.py"
curl -fsSL "$BASE/armpi_voice/armpi_voice/tts_node.py$V"    -o "$PKG/armpi_voice/tts_node.py"

echo "Building armpi_voice..."
cd "$HOME/ros2_ws"
colcon build --packages-select armpi_voice

echo
echo "=== Done. Executables: ==="
. "$HOME/ros2_ws/install/setup.bash"
ros2 pkg executables armpi_voice
echo
echo "Next: source ~/ros2_ws/install/setup.bash"
echo "      bash ~/ros2_ws/src/armpi_voice/chat.sh"
