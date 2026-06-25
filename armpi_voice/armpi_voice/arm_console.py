#!/usr/bin/env python3
"""
arm_console.py
==============
Interactive chat console for the ArmPi Ultra agent — one terminal, type-and-reply.

Instead of running `ros2 topic pub ... /voice_words ...` for every command and
watching a separate terminal for the answer, run this: type a line, the robot
acts and replies right below it, like a chat box.

How it fits in:
    you type  ──► publishes to /voice_words ──► [arm_agent] does the work
    reply     ◄── prints from /robot_speech ◄── [arm_agent] publishes its "say"

It's a thin CLIENT — the LLM/agent (`arm_agent`) and the arm hardware (SDK) still
run in the background. This node holds no robot logic; it only sends text and
prints replies. Run it standalone:

    ros2 run armpi_voice arm_console

Type a command ("turn left", "what can you do?", ...). Type "quit" to exit.
"""

import queue
import threading

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String


class ArmConsole(Node):
    """Publishes typed commands to the agent and prints its spoken replies."""

    def __init__(self) -> None:
        super().__init__('arm_console')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('speech_topic', '/robot_speech')
        # Wait at least as long as the agent's LLM request timeout (default 30s).
        self.declare_parameter('reply_timeout', 35.0)

        self.voice_topic = self.get_parameter('voice_topic').value
        self.speech_topic = self.get_parameter('speech_topic').value
        self.reply_timeout = float(self.get_parameter('reply_timeout').value)

        self.pub = self.create_publisher(String, self.voice_topic, 10)
        self.create_subscription(String, self.speech_topic, self._on_reply, 10)
        self._replies: "queue.Queue[str]" = queue.Queue()

    def _on_reply(self, msg: String) -> None:
        self._replies.put(msg.data)

    def ask(self, text: str):
        """Send one command and wait (turn-based) for the agent's reply."""
        # Drop any stale reply that arrived late from a previous turn, so we don't
        # mismatch this command with the wrong answer.
        while not self._replies.empty():
            try:
                self._replies.get_nowait()
            except queue.Empty:
                break
        self.pub.publish(String(data=text))
        try:
            return self._replies.get(timeout=self.reply_timeout)
        except queue.Empty:
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmConsole()

    # Spin the executor on a background thread so reply callbacks fire while the
    # main thread is blocked on input().
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('ArmPi Ultra chat — type a command (e.g. "turn left"), or "quit" to exit.')
    print(f'  sending to {node.voice_topic}, listening on {node.speech_topic}\n')
    try:
        while rclpy.ok():
            try:
                text = input('you> ').strip()
            except EOFError:
                break
            if not text:
                continue
            if text.lower() in ('quit', 'exit', 'q'):
                break
            reply = node.ask(text)
            if reply is None:
                print('robot> (no response — is arm_agent running?)\n')
            else:
                print(f'robot> {reply}\n')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        print('\nbye.')


if __name__ == '__main__':
    main()
