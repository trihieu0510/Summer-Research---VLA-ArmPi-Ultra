#!/usr/bin/env python3
"""
voice_arm_control.py
====================
Voice-commanded control node for the Hiwonder ArmPi Ultra (6-DOF arm).

Data flow:
    /voice_words (std_msgs/String)        speech-to-text transcript (WonderEcho)
        -> DeepSeek LLM                   natural language -> structured JSON intent
        -> /ros_robot_controller/bus_servo/set_position (ServosPosition)

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.

Setup:
    export DEEPSEEK_API_KEY="sk-..."      # required; never hardcode (repo is pushed to GitHub)

Design notes:
    * The blocking LLM network call runs in a dedicated worker thread so it never stalls
      the ROS executor.
    * A queue serialises commands so the arm executes one motion at a time (no overlapping
      or competing servo commands).
"""

import json
import os
import queue
import threading
import time

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
    "home":  [(1.5, HOME_POSE)],
    "left":  [(1.0, [(6, 200)])],
    "right": [(1.0, [(6, 800)])],
    "open":  [(0.5, [(1, 200)])],
    "close": [(0.5, [(1, 500)])],
    "nod":   [(0.5, [(4, 300)]), (0.5, [(4, 500)])],   # tilt down, then return
}


class VoiceArmController(Node):
    """Bridges spoken commands to ArmPi Ultra servo motions via an LLM intent parser."""

    def __init__(self) -> None:
        super().__init__('voice_arm_controller')

        # --- Parameters (override at launch without editing code) ---
        self.declare_parameter('model', 'deepseek-chat')
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('servo_topic', '/ros_robot_controller/bus_servo/set_position')
        self.declare_parameter('request_timeout', 15.0)

        self.model_name = self.get_parameter('model').value
        base_url = self.get_parameter('base_url').value
        voice_topic = self.get_parameter('voice_topic').value
        servo_topic = self.get_parameter('servo_topic').value
        self.request_timeout = float(self.get_parameter('request_timeout').value)

        # --- LLM client (DeepSeek is OpenAI-API compatible) ---
        api_key = os.environ.get('DEEPSEEK_API_KEY')
        if not api_key:
            raise RuntimeError(
                "DEEPSEEK_API_KEY environment variable is not set. "
                'Run: export DEEPSEEK_API_KEY="sk-..." (add it to ~/.bashrc to persist).'
            )
        self.llm_client = OpenAI(
            base_url=base_url,
            api_key=api_key,
            timeout=self.request_timeout,
        )
        self.system_prompt = self._build_system_prompt()
        self.get_logger().info(f'LLM reasoning via "{self.model_name}" at {base_url}')

        # --- ROS interfaces ---
        self.pub = self.create_publisher(ServosPosition, servo_topic, 1)

        # Block until the hardware controller is ready, so the first command isn't dropped.
        self.init_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        while not self.init_client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                raise RuntimeError('Interrupted while waiting for the robot controller service.')
            self.get_logger().info(
                'Waiting for robot hardware controller (/ros_robot_controller/init_finish)...'
            )

        # --- Background worker so the LLM round-trip never blocks the ROS executor ---
        self._command_queue: "queue.Queue[str]" = queue.Queue()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, name='llm-worker', daemon=True)
        self._worker.start()

        self.voice_sub = self.create_subscription(String, voice_topic, self._on_voice, 10)
        self.get_logger().info(f'Ready. Listening for voice commands on "{voice_topic}".')

    # --- Prompt -------------------------------------------------------------------
    def _build_system_prompt(self) -> str:
        action_names = "\n".join(f'    - "{name}"' for name in ACTIONS)
        return (
            "You are the reasoning module of a robotic arm (Hiwonder ArmPi Ultra).\n"
            "Convert the user's natural-language command into exactly ONE supported action.\n\n"
            "Supported actions:\n"
            f"{action_names}\n"
            '    - "unknown": the command does not map to any action above.\n\n'
            "Action meanings:\n"
            "    home  - return to the neutral/rest pose\n"
            "    left  - rotate the whole arm to the left\n"
            "    right - rotate the whole arm to the right\n"
            "    open  - open the gripper (release an object)\n"
            "    close - close the gripper (grasp an object)\n"
            "    nod   - nod the wrist as a greeting\n\n"
            'Respond with a single JSON object and nothing else, e.g. {"action": "left"}.'
        )

    # --- ROS callback (runs on the executor thread; must stay fast) ---------------
    def _on_voice(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'Heard: "{text}"')
        self._command_queue.put(text)

    # --- Worker thread ------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                text = self._command_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                action = self._reason(text)
                self.get_logger().info(f'Decided action: {action}')
                self.execute_action(action)
            except Exception as exc:  # network error, malformed response, etc.
                self.get_logger().error(f'Failed to handle command "{text}": {exc}')
            finally:
                self._command_queue.task_done()

    # --- LLM intent parsing -------------------------------------------------------
    def _reason(self, text: str) -> str:
        """Ask the LLM to classify the transcript into a supported action name."""
        response = self.llm_client.chat.completions.create(
            model=self.model_name,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": text},
            ],
            temperature=0.0,                       # deterministic, consistent JSON
            response_format={"type": "json_object"},
        )
        content = (response.choices[0].message.content or "").strip()
        self.get_logger().info(f'LLM response: {content}')
        return self._parse_action(content)

    @staticmethod
    def _parse_action(content: str) -> str:
        """Extract the action name from the LLM reply, tolerating stray markdown fences."""
        if content.startswith("```"):
            content = content.strip("`")
            if content.startswith("json"):
                content = content[4:]
            content = content.strip()
        try:
            intent = json.loads(content)
        except json.JSONDecodeError:
            return "unknown"
        action = intent.get("action", "unknown")
        return action if action in ACTIONS else "unknown"

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
        self.pub.publish(msg)
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
        node = VoiceArmController()
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
