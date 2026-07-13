#!/usr/bin/env bash
# install_stt.sh — set up voice INPUT (faster-whisper) on the Pi, then run a
# fully SILENT self-test: piper synthesizes "grab the red block" to a WAV and
# faster-whisper transcribes that file. The robot's voice tests the robot's
# ears — no human speech, no sound from the speaker.
#
# Run on the Pi (inside the container), with internet up:
#     curl -fsSL $BASE/install_stt.sh | bash

set -e

echo "=== 1/3 Installing python deps (faster-whisper + VAD) ==="
# webrtcvad-wheels, NOT webrtcvad: the latter is a 2017 sdist needing a compiler.
python3 -m pip install --user faster-whisper webrtcvad-wheels numpy

echo
echo "=== 2/3 Pre-downloading the base.en model (so first use is instant) ==="
python3 - <<'EOF'
from faster_whisper import WhisperModel
print("downloading/loading base.en (int8)...")
WhisperModel('base.en', device='cpu', compute_type='int8')
print("model ready.")
EOF

echo
echo "=== 3/3 SILENT self-test: piper speaks a WAV, whisper transcribes it ==="
PIPER_BIN="$HOME/piper/piper"
PIPER_MODEL="$HOME/piper/en_US-amy-medium.onnx"
if [ -x "$PIPER_BIN" ] && [ -f "$PIPER_MODEL" ]; then
    echo "grab the red block" | "$PIPER_BIN" --model "$PIPER_MODEL" \
        --output_file /tmp/stt_selftest.wav >/dev/null 2>&1
    python3 - <<'EOF'
from faster_whisper import WhisperModel
model = WhisperModel('base.en', device='cpu', compute_type='int8', cpu_threads=4)
import time
t0 = time.time()
segments, _ = model.transcribe('/tmp/stt_selftest.wav', language='en', beam_size=1)
text = ' '.join(s.text.strip() for s in segments).strip()
print(f'transcribed in {time.time()-t0:.1f}s: "{text}"')
ok = 'red block' in text.lower()
print('SELF-TEST', 'PASSED' if ok else f'FAILED (expected "grab the red block")')
raise SystemExit(0 if ok else 1)
EOF
else
    echo "(piper not found at ~/piper — skipping the loopback test."
    echo " The node will still work; test by speaking once you can.)"
fi

echo
echo "=== Done. Start the ears with: ==="
echo "  ros2 run armpi_voice stt_node --ros-args -p mic_device:=<from arecord -l>"
echo "or let chat.sh start it: ARMPI_STT=1 bash chat.sh"
