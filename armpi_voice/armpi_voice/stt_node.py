#!/usr/bin/env python3
"""
stt_node.py
===========
The robot's EARS: microphone -> faster-whisper -> /voice_words.

Captures raw PCM from the USB mic via an `arecord` subprocess (no extra audio
libraries), gates it with WebRTC voice-activity detection into utterances
(speech starts -> keep recording -> ~0.8s of silence ends it), transcribes the
utterance with faster-whisper on the CPU, and publishes the text to
/voice_words — the exact topic arm_agent already listens on. The chat console
and this node are interchangeable front-ends: speaking a command IS typing it.

Half-duplex rule: while tts_node reports it is speaking (/tts_busy True) the
mic is muted (plus a short echo tail), or the robot would transcribe — and
obey — its own voice.

Deps (install once with install_stt.sh):
    pip install faster-whisper webrtcvad-wheels numpy

Run standalone (agent + tts running elsewhere):
    ros2 run armpi_voice stt_node --ros-args -p mic_device:=plughw:1,0
Find the mic device with `arecord -l` (WonderEcho Pro = "USB PnP Audio").

Runs ON THE PI, inside the ROS 2 container (needs /dev/snd passthrough).
"""

import collections
import subprocess
import threading
import time

# pyrefly: ignore [missing-import]
import numpy as np
# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import Bool, String

RATE = 16000                      # faster-whisper's native rate
FRAME_MS = 30                     # webrtcvad accepts 10/20/30 ms frames
FRAME_BYTES = RATE * 2 * FRAME_MS // 1000   # S16_LE mono -> 960 bytes


class STTNode(Node):
    """Streams the mic, segments utterances by voice activity, transcribes."""

    def __init__(self) -> None:
        super().__init__('stt_node')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('busy_topic', '/tts_busy')
        self.declare_parameter('mic_device', 'default')   # arecord -D value; see `arecord -l`
        self.declare_parameter('model', 'base.en')        # tiny.en if the Pi struggles
        self.declare_parameter('vad_aggressiveness', 2)   # 0..3 (3 = strictest)
        self.declare_parameter('silence_ms', 800)         # this much quiet ends an utterance
        self.declare_parameter('max_utterance_s', 10.0)
        self.declare_parameter('min_utterance_s', 0.4)    # shorter = noise, dropped
        self.declare_parameter('echo_tail_s', 0.7)        # stay muted after TTS stops
        # Optional wake word (e.g. 'robot'): only utterances containing it get
        # published (with the wake word stripped). Empty = every utterance goes.
        self.declare_parameter('wake_word', '')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.mic_device = p('mic_device')
        self.model_name = p('model')
        self.vad_level = int(p('vad_aggressiveness'))
        self.silence_frames = max(1, int(p('silence_ms')) // FRAME_MS)
        self.max_frames = int(float(p('max_utterance_s')) * 1000 / FRAME_MS)
        self.min_frames = int(float(p('min_utterance_s')) * 1000 / FRAME_MS)
        self.echo_tail = float(p('echo_tail_s'))
        self.wake_word = str(p('wake_word')).strip().lower()

        self.pub = self.create_publisher(String, p('voice_topic'), 10)
        self.create_subscription(Bool, p('busy_topic'), self._on_tts_busy, 10)
        self._tts_busy = False
        self._muted_until = 0.0

        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._run, name='stt-worker', daemon=True)
        self._worker.start()

    # --- half-duplex mute -----------------------------------------------------------
    def _on_tts_busy(self, msg: Bool) -> None:
        if msg.data:
            self._tts_busy = True
        else:
            self._tts_busy = False
            self._muted_until = time.monotonic() + self.echo_tail

    def _muted(self) -> bool:
        return self._tts_busy or time.monotonic() < self._muted_until

    # --- capture + transcribe loop --------------------------------------------------
    def _run(self) -> None:
        # Heavy imports on the worker so node startup stays instant.
        try:
            # pyrefly: ignore [missing-import]
            import webrtcvad
            # pyrefly: ignore [missing-import]
            from faster_whisper import WhisperModel
        except ImportError as exc:
            self.get_logger().error(
                f'STT deps missing ({exc}) — run install_stt.sh, then restart.')
            return

        self.get_logger().info(f'Loading whisper "{self.model_name}" (first run downloads it)...')
        model = WhisperModel(self.model_name, device='cpu',
                             compute_type='int8', cpu_threads=4)
        vad = webrtcvad.Vad(self.vad_level)
        self.get_logger().info(
            f'Listening on mic "{self.mic_device}" — speak, I transcribe to /voice_words.')

        while not self._shutdown.is_set():
            try:
                self._capture_session(model, vad)
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().error(f'Capture session died: {exc}; retrying in 3s.')
                time.sleep(3.0)

    def _capture_session(self, model, vad) -> None:
        """One arecord lifetime. Returns/raises when the recorder dies."""
        rec = subprocess.Popen(
            ['arecord', '-q', '-D', self.mic_device, '-f', 'S16_LE',
             '-r', str(RATE), '-c', '1', '-t', 'raw'],
            stdout=subprocess.PIPE,
        )
        # Keep ~300ms of pre-speech audio so the first word isn't clipped.
        preroll = collections.deque(maxlen=10)
        utterance = []
        voiced_recent = collections.deque(maxlen=5)
        silence_run = 0

        try:
            while not self._shutdown.is_set():
                frame = rec.stdout.read(FRAME_BYTES)
                if len(frame) < FRAME_BYTES:
                    raise RuntimeError('arecord stream ended (mic unplugged or device busy)')

                if self._muted():
                    preroll.clear()
                    utterance = []
                    voiced_recent.clear()
                    continue

                is_voiced = vad.is_speech(frame, RATE)
                voiced_recent.append(is_voiced)

                if not utterance:
                    preroll.append(frame)
                    # Start when a clear majority of recent frames are speech.
                    if sum(voiced_recent) >= 3:
                        utterance = list(preroll)
                        silence_run = 0
                else:
                    utterance.append(frame)
                    silence_run = 0 if is_voiced else silence_run + 1
                    if silence_run >= self.silence_frames or len(utterance) >= self.max_frames:
                        self._finish_utterance(model, utterance)
                        utterance = []
                        preroll.clear()
                        voiced_recent.clear()
        finally:
            rec.kill()

    def _finish_utterance(self, model, frames) -> None:
        if len(frames) < self.min_frames:
            return                                        # a click, not a command
        audio = np.frombuffer(b''.join(frames), dtype=np.int16).astype(np.float32) / 32768.0
        t0 = time.monotonic()
        segments, _info = model.transcribe(audio, language='en', beam_size=1,
                                           vad_filter=False)
        text = ' '.join(s.text.strip() for s in segments).strip()
        latency = time.monotonic() - t0
        if not text or len(text) < 3:
            return
        if self.wake_word:
            low = text.lower()
            if self.wake_word not in low:
                self.get_logger().info(f'(no wake word) heard: "{text}"')
                return
            text = text[low.index(self.wake_word) + len(self.wake_word):].lstrip(' ,.!?') or text
        self.get_logger().info(f'Heard ({latency:.1f}s): "{text}"')
        self.pub.publish(String(data=text))

    def destroy_node(self) -> None:
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = STTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
