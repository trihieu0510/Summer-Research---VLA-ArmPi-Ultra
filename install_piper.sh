#!/usr/bin/env bash
# install_piper.sh — install piper (natural neural TTS) + an English voice on the Pi.
#
# Self-contained aarch64 binary (no pip / onnxruntime hassle) + a voice model.
# Run with one short command:
#     curl -fsSL "$BASE/install_piper.sh?v=$(date +%s)" | bash
#
# Afterwards chat.sh auto-detects ~/piper and uses the natural voice.

set -e

DEST="$HOME/piper"
PIPER_URL="https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_aarch64.tar.gz"
VOICE_BASE="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium"
VOICE="en_US-amy-medium"

echo "Downloading piper (aarch64)..."
curl -fsSL "$PIPER_URL" -o /tmp/piper.tgz
tar -xzf /tmp/piper.tgz -C "$HOME"        # creates ~/piper/piper + bundled libs
rm -f /tmp/piper.tgz

echo "Downloading voice model ($VOICE, ~60 MB)..."
curl -fsSL "$VOICE_BASE/$VOICE.onnx"      -o "$DEST/$VOICE.onnx"
curl -fsSL "$VOICE_BASE/$VOICE.onnx.json" -o "$DEST/$VOICE.onnx.json"

echo "Testing synthesis -> ~/ros2_ws/piper_test.wav ..."
echo "Hello, I am the robot arm, and now I have a natural voice." \
    | "$DEST/piper" --model "$DEST/$VOICE.onnx" --output_file "$HOME/ros2_ws/piper_test.wav"

echo
echo "=== Done ==="
echo "piper binary: $DEST/piper"
echo "voice model:  $DEST/$VOICE.onnx"
echo "Download ~/ros2_ws/piper_test.wav and listen — it should sound natural."
echo "chat.sh will now auto-use piper."
