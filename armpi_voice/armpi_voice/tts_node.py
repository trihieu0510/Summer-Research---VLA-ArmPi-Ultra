#!/usr/bin/env python3
"""
tts_node.py
===========
Speak the agent's replies out loud through the robot's speaker.

Subscribes to /robot_speech (std_msgs/String) — the same topic arm_agent
publishes its "say" replies on — and synthesizes each one to audio. This is the
voice-output half of the pipeline; it needs no change to the agent.

Engines (param `engine`):
  * "espeak"  (default) — espeak-ng, fully offline, always works, robotic voice.
  * "piper"             — neural TTS, natural voice; needs `piper` installed and
                          a voice model (.onnx) at `piper_model`.

Runs ON THE PI, inside the container — which must have audio passthrough
(/dev/snd). Verify first with `aplay -l` and `espeak-ng "test"`.

Run standalone:
    ros2 run armpi_voice tts_node
    ros2 run armpi_voice tts_node --ros-args -p engine:=piper -p piper_model:=/path/voice.onnx
"""

import queue
import subprocess
import threading

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String


class TTSNode(Node):
    """Subscribes to /robot_speech and speaks each message through the speaker."""

    def __init__(self) -> None:
        super().__init__('tts_node')
        self.declare_parameter('speech_topic', '/robot_speech')
        self.declare_parameter('engine', 'espeak')        # 'espeak' | 'piper'
        self.declare_parameter('voice', 'en+f3')          # espeak voice name
        self.declare_parameter('rate', 160)               # espeak words/minute
        self.declare_parameter('piper_model', '')         # .onnx path for piper
        # ALSA output device, e.g. 'plughw:2,0' for the USB speaker (card 2).
        # Empty = system default (often HDMI, which won't reach the speaker).
        self.declare_parameter('alsa_device', '')

        self.speech_topic = self.get_parameter('speech_topic').value
        self.engine = self.get_parameter('engine').value
        self.voice = self.get_parameter('voice').value
        self.rate = int(self.get_parameter('rate').value)
        self.piper_model = self.get_parameter('piper_model').value
        self.alsa_device = self.get_parameter('alsa_device').value

        # Serialize utterances on a worker thread so playback never blocks the
        # executor and replies don't overlap into garbled audio.
        self._q: "queue.Queue[str]" = queue.Queue()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._loop, name='tts-worker', daemon=True)
        self._worker.start()

        self.create_subscription(String, self.speech_topic, self._on_speech, 10)
        self.get_logger().info(
            f'TTS ready (engine={self.engine}); speaking text from "{self.speech_topic}".'
        )

    def _on_speech(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self._q.put(text)

    def _loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                text = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._speak(text)
            except FileNotFoundError:
                self.get_logger().error(
                    f'TTS engine "{self.engine}" not found — install it '
                    '(e.g. `apt-get install espeak-ng`) or check the speaker.'
                )
            except Exception as exc:
                self.get_logger().error(f'TTS failed for "{text}": {exc}')
            finally:
                self._q.task_done()

    def _aplay_cmd(self):
        cmd = ['aplay', '-q']
        if self.alsa_device:
            cmd += ['-D', self.alsa_device]   # route to a specific card, e.g. plughw:2,0
        return cmd + ['-']

    def _speak(self, text: str) -> None:
        if self.engine == 'piper':
            if not self.piper_model:
                self.get_logger().warn('piper engine selected but piper_model not set.')
                return
            # piper synthesizes a WAV to stdout; pipe it straight to aplay.
            piper = subprocess.Popen(
                ['piper', '--model', self.piper_model, '--output_file', '-'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            )
            aplay = subprocess.Popen(self._aplay_cmd(), stdin=piper.stdout)
            piper.stdin.write(text.encode())
            piper.stdin.close()
            aplay.wait()
        elif self.alsa_device:
            # espeak-ng -> WAV on stdout -> aplay on the chosen device.
            espeak = subprocess.Popen(
                ['espeak-ng', '-s', str(self.rate), '-v', self.voice, '--stdout', text],
                stdout=subprocess.PIPE,
            )
            aplay = subprocess.Popen(self._aplay_cmd(), stdin=espeak.stdout)
            espeak.stdout.close()
            aplay.wait()
        else:
            # Default ALSA device.
            subprocess.run(
                ['espeak-ng', '-s', str(self.rate), '-v', self.voice, text],
                check=False,
            )

    def destroy_node(self) -> None:
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TTSNode()
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
