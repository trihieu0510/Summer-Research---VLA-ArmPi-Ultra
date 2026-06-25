#!/usr/bin/env python3
"""
arm_agent.py
============
Conversational agent node for the Hiwonder ArmPi Ultra (6-DOF arm).

This supersedes ``voice_arm_control.py``: instead of classifying a transcript into
exactly one gesture (else "unknown"), the LLM acts as an *agent* that decides each
turn whether to PERFORM AN ACTION, just TALK, or both — and keeps a short rolling
memory so a back-and-forth chat stays coherent.

Data flow:
    /voice_words (std_msgs/String)        speech-to-text transcript (or typed test input)
        -> LLM (OpenAI-compatible)        -> {"action": <gesture|none>, "say": <reply>}
        -> /ros_robot_controller/bus_servo/set_position   (ServosPosition)  [if action]
        -> /robot_speech (std_msgs/String)                 the spoken reply  [always]

`/robot_speech` is the seam for the upcoming TTS stage — a text-to-speech node can
subscribe to it without this node changing. For now the reply is also just logged.

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.

LLM backend (any OpenAI-compatible endpoint):
    Local Ollama (default, no key):
        ros2 run armpi_voice arm_agent
    Model on the dev laptop over the LAN:
        ros2 run armpi_voice arm_agent --ros-args \
            -p base_url:=http://192.168.137.1:11434/v1 -p model:=qwen3:8b
    DeepSeek cloud (best chat quality; set a key, never hardcode):
        export LLM_API_KEY="sk-..."
        ros2 run armpi_voice arm_agent --ros-args \
            -p base_url:=https://api.deepseek.com -p model:=deepseek-chat

Design notes:
    * The blocking LLM call runs in a worker thread so it never stalls the ROS executor.
    * A queue serialises turns so the arm executes one motion at a time.
    * Conversation memory is a bounded rolling window (last N exchanges) to keep prompts
      small on the CPU-bound Pi while preserving context.
"""

import json
import os
import queue
import re
import threading
import time
from collections import deque

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String
# pyrefly: ignore [missing-import]
from std_srvs.srv import Trigger
# pyrefly: ignore [missing-import]
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
# pyrefly: ignore [missing-import]
from openai import OpenAI


# --- Servo / motion configuration -------------------------------------------------
# Servo positions are raw units in [0, 1000] where 500 is centre. Each servo has hard
# mechanical limits; commanding outside them can damage the hardware.
# Servo IDs (verify against the SDK URDF / servo config): 1 = gripper, 6 = base rotation.
SERVO_MIN = 0
SERVO_MAX = 1000

HOME_POSE = [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500)]

# Each action maps to a sequence of motion steps: (duration_seconds, [(servo_id, position), ...]).
# Multi-step actions (e.g. "nod") run their steps in order, waiting out each duration.
ACTIONS = {
    # "left"/"right" are from the OPERATOR's view (facing the arm / camera view),
    # which mirrors the robot's own frame — hence left=800, right=200 on base servo 6.
    "home":  [(1.5, HOME_POSE)],
    "left":  [(1.0, [(6, 800)])],
    "right": [(1.0, [(6, 200)])],
    "open":  [(0.5, [(1, 200)])],
    "close": [(0.5, [(1, 500)])],
    "nod":   [(0.5, [(4, 300)]), (0.5, [(4, 500)])],   # tilt down, then return
}

# Short human-readable descriptions, surfaced to the LLM so it can map intent to a gesture.
ACTION_DESCRIPTIONS = {
    "home":  "return to the neutral/rest pose",
    "left":  "rotate the whole arm to the left",
    "right": "rotate the whole arm to the right",
    "open":  "open the gripper (release an object)",
    "close": "close the gripper (grasp an object)",
    "nod":   "nod the wrist, e.g. as a greeting or a yes",
}


class ArmAgent(Node):
    """An LLM agent that either drives an ArmPi Ultra gesture or holds a conversation."""

    def __init__(self) -> None:
        super().__init__('arm_agent')

        # --- Parameters (override at launch without editing code) ---
        self.declare_parameter('model', 'qwen2.5:3b')
        self.declare_parameter('base_url', 'http://localhost:11434/v1')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('servo_topic', '/ros_robot_controller/bus_servo/set_position')
        self.declare_parameter('speech_topic', '/robot_speech')
        self.declare_parameter('request_timeout', 30.0)
        # How many past exchanges (user+assistant pairs) to keep as conversational memory.
        self.declare_parameter('history_turns', 4)

        self.model_name = self.get_parameter('model').value
        base_url = self.get_parameter('base_url').value
        voice_topic = self.get_parameter('voice_topic').value
        servo_topic = self.get_parameter('servo_topic').value
        speech_topic = self.get_parameter('speech_topic').value
        self.request_timeout = float(self.get_parameter('request_timeout').value)
        history_turns = int(self.get_parameter('history_turns').value)

        # --- LLM client (any OpenAI-compatible endpoint: Ollama, DeepSeek, OpenAI) ---
        api_key = (
            os.environ.get('LLM_API_KEY')
            or os.environ.get('DEEPSEEK_API_KEY')
            or os.environ.get('OPENAI_API_KEY')
            or 'ollama'  # dummy placeholder for keyless local servers
        )
        self.llm_client = OpenAI(
            base_url=base_url,
            api_key=api_key,
            timeout=self.request_timeout,
        )
        self.system_prompt = self._build_system_prompt()
        self.get_logger().info(f'LLM reasoning via "{self.model_name}" at {base_url}')

        # Rolling conversation memory: a deque of {"role", "content"} dicts, capped at
        # 2 messages per turn (user + assistant). maxlen bounds prompt growth on the Pi.
        self._history: "deque[dict]" = deque(maxlen=max(0, history_turns) * 2)

        # --- ROS interfaces ---
        self.servo_pub = self.create_publisher(ServosPosition, servo_topic, 1)
        self.speech_pub = self.create_publisher(String, speech_topic, 10)

        # Block until the hardware controller is ready, so the first command isn't dropped.
        self.init_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        while not self.init_client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                raise RuntimeError('Interrupted while waiting for the robot controller service.')
            self.get_logger().info(
                'Waiting for robot hardware controller (/ros_robot_controller/init_finish)...'
            )

        # --- Background worker so the LLM round-trip never blocks the ROS executor ---
        self._turn_queue: "queue.Queue[str]" = queue.Queue()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, name='agent-worker', daemon=True)
        self._worker.start()

        self.voice_sub = self.create_subscription(String, voice_topic, self._on_voice, 10)
        self.get_logger().info(
            f'Ready. Listening on "{voice_topic}"; speaking on "{speech_topic}".'
        )

    # --- Prompt -------------------------------------------------------------------
    def _build_system_prompt(self) -> str:
        action_lines = "\n".join(
            f'    - "{name}": {desc}' for name, desc in ACTION_DESCRIPTIONS.items()
        )
        return (
            "You are the mind of a friendly robotic arm (Hiwonder ArmPi Ultra). "
            "Each turn you either PERFORM a physical action, just TALK, or both.\n\n"
            "Physical actions you can perform:\n"
            f"{action_lines}\n"
            '    - "none": do not move (use this when the user is just chatting).\n\n'
            "Rules:\n"
            "- If the user asks you to move/gesture/grab/wave, choose the single best matching "
            'action and add a brief spoken confirmation.\n'
            "- If the user is asking a question, greeting you, or making small talk, set "
            '"action" to "none" and reply conversationally.\n'
            "- Keep \"say\" short and natural (1-2 sentences) — it will be spoken aloud.\n"
            "- Only use an action from the list above; never invent new ones.\n\n"
            'Respond with ONLY a JSON object of the form '
            '{"action": "<action or none>", "say": "<your spoken reply>"}.'
        )

    # --- ROS callback (runs on the executor thread; must stay fast) ---------------
    def _on_voice(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'Heard: "{text}"')
        self._turn_queue.put(text)

    # --- Worker thread ------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                text = self._turn_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                action, say = self._reason(text)
                self.get_logger().info(f'Decided -> action={action!r}, say={say!r}')
                if say:
                    self._speak(say)
                if action in ACTIONS:
                    self.execute_action(action)
            except Exception as exc:  # network error, malformed response, etc.
                self.get_logger().error(f'Failed to handle turn "{text}": {exc}')
            finally:
                self._turn_queue.task_done()

    # --- LLM reasoning ------------------------------------------------------------
    def _reason(self, text: str):
        """Send the turn (with rolling history) to the LLM; return (action, say)."""
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(self._history)
        messages.append({"role": "user", "content": text})

        response = self.llm_client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            temperature=0.3,                       # a little warmth for chat, still grounded
            response_format={"type": "json_object"},
        )
        content = (response.choices[0].message.content or "").strip()
        self.get_logger().info(f'LLM response: {content}')
        action, say = self._parse_reply(content)

        # Record this exchange so the next turn has context. Store the raw JSON as the
        # assistant message so the model sees its own prior structured decisions.
        self._history.append({"role": "user", "content": text})
        self._history.append({"role": "assistant", "content": content})
        return action, say

    @staticmethod
    def _parse_reply(content: str):
        """Extract (action, say) from the LLM reply.

        Tolerant of markdown ```json fences, leading prose, and reasoning models
        (e.g. qwen3) that emit a <think>...</think> block before the JSON.
        """
        content = re.sub(r"<think>.*?</think>", "", content, flags=re.DOTALL)
        match = re.search(r"\{.*\}", content, flags=re.DOTALL)
        if not match:
            return "none", content.strip() or ""
        try:
            reply = json.loads(match.group(0))
        except json.JSONDecodeError:
            return "none", ""
        action = reply.get("action", "none")
        if action not in ACTIONS:
            action = "none"
        say = str(reply.get("say", "")).strip()
        return action, say

    # --- Speech output ------------------------------------------------------------
    def _speak(self, text: str) -> None:
        """Publish the reply for any TTS subscriber, and log it prominently."""
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f'🗣  {text}')

    # --- Motion -------------------------------------------------------------------
    def execute_action(self, action: str) -> None:
        steps = ACTIONS.get(action)
        if steps is None:
            self.get_logger().info(f'No motion mapped for action "{action}"; ignoring.')
            return
        self.get_logger().info(f'Executing: {action}')
        for duration, positions in steps:
            self.set_servo_position(duration, positions)
            time.sleep(duration)   # let the motion finish before the next step

    def set_servo_position(self, duration: float, positions) -> None:
        """Publish one servo command. `positions` is an iterable of (servo_id, position)."""
        positions = list(positions)
        msg = ServosPosition()
        msg.duration = float(duration)
        servo_msgs = []
        for servo_id, position in positions:
            clamped = max(SERVO_MIN, min(SERVO_MAX, int(position)))
            if clamped != int(position):
                self.get_logger().warn(
                    f'Servo {servo_id} position {position} out of range; clamped to {clamped}.'
                )
            servo = ServoPosition()
            servo.id = int(servo_id)
            servo.position = clamped
            servo_msgs.append(servo)
        msg.position = servo_msgs
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Published servo positions: {positions} (duration {duration}s)')

    # --- Shutdown -----------------------------------------------------------------
    def destroy_node(self) -> None:
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArmAgent()
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
