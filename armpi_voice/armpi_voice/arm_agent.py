#!/usr/bin/env python3
"""
arm_agent.py
============
Conversational + motion agent for the Hiwonder ArmPi Ultra (6-DOF arm).

Each turn the LLM decides to either PERFORM AN ACTION, just TALK, or both, and
keeps a short rolling memory so a back-and-forth chat stays coherent.

Two kinds of actions:
  * Named gestures   — home / left / right / open / close / nod (fixed poses).
  * Parametric moves — "action": "move" with a list of per-servo targets/deltas,
    so commands like "turn halfway right" or "move motor 3 up a bit" work.

Data flow:
    /voice_words (std_msgs/String)        text command (typed console or STT)
        -> LLM (OpenAI-compatible)        -> {"action", "moves", "say"}
        -> /ros_robot_controller/bus_servo/set_position   (ServosPosition)
        -> /robot_speech (std_msgs/String)                 the spoken reply

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.
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
SERVO_MIN = 0
SERVO_MAX = 1000
MOVE_DURATION = 1.0           # seconds per parametric move
MAX_DELTA = 300               # clamp a single relative nudge so the arm can't lurch

HOME_POSE = [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500)]

# Named gestures: each maps to motion steps (duration_seconds, [(servo_id, position), ...]).
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


class ArmAgent(Node):
    """An LLM agent that drives ArmPi Ultra gestures/moves or holds a conversation."""

    def __init__(self) -> None:
        super().__init__('arm_agent')

        # --- Parameters (override at launch without editing code) ---
        self.declare_parameter('model', 'qwen2.5:3b')
        self.declare_parameter('base_url', 'http://localhost:11434/v1')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('servo_topic', '/ros_robot_controller/bus_servo/set_position')
        self.declare_parameter('speech_topic', '/robot_speech')
        self.declare_parameter('request_timeout', 30.0)
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
        self.llm_client = OpenAI(base_url=base_url, api_key=api_key, timeout=self.request_timeout)
        self.system_prompt = self._build_system_prompt()
        self.get_logger().info(f'LLM reasoning via "{self.model_name}" at {base_url}')

        # Track each servo's last-known position so RELATIVE moves ("up a bit") work.
        # Starts at home (all 500); updated whenever we publish a command.
        self.current = {sid: pos for sid, pos in HOME_POSE}

        self._history: "deque[dict]" = deque(maxlen=max(0, history_turns) * 2)

        # --- ROS interfaces ---
        self.servo_pub = self.create_publisher(ServosPosition, servo_topic, 1)
        self.speech_pub = self.create_publisher(String, speech_topic, 10)

        self.init_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        while not self.init_client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                raise RuntimeError('Interrupted while waiting for the robot controller service.')
            self.get_logger().info(
                'Waiting for robot hardware controller (/ros_robot_controller/init_finish)...'
            )

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
        return (
            "You are the mind of a friendly 6-servo robotic arm (Hiwonder ArmPi Ultra). "
            "Each turn you either PERFORM an action, just TALK, or both.\n\n"
            "SERVOS: ids 1-6, raw units 0-1000, 500 = centre. Known mapping:\n"
            "  - servo 6 = base rotation. From the operator's view: right is LOWER "
            "(~200), left is HIGHER (~800), centre 500.\n"
            "  - servo 1 = gripper: open ~200, closed ~500.\n"
            "  - servos 2-5 = arm joints (shoulder/elbow/wrist), exact mapping unverified.\n\n"
            "NAMED GESTURES (use for simple whole-arm commands):\n"
            '  "home", "left", "right", "open", "close", "nod".\n\n'
            "PARAMETRIC MOVES (use action \"move\" for partial/relative/per-servo commands). "
            "Provide a \"moves\" list; each item is either absolute or relative:\n"
            '  - absolute: {"servo": <1-6>, "target": <0-1000>}\n'
            '  - relative: {"servo": <1-6>, "delta": <signed amount>}\n'
            "Guidance: 'halfway to the right' on the base = target ~350 (between centre 500 "
            "and right 200). 'a little' ~ 80-120 units. 'up/raise' = positive delta, "
            "'down/lower' = negative delta (~120). Keep any single move modest (<=300 units). "
            "You may list several servos at once.\n\n"
            "OTHERWISE just chat: set \"action\" to \"none\" and reply in \"say\".\n\n"
            "Always respond with ONLY a JSON object: "
            '{"action": "<home|left|right|open|close|nod|move|none>", '
            '"moves": [ ... only when action is \"move\" ... ], '
            '"say": "<short spoken reply, 1-2 sentences>"}.\n'
            "Examples:\n"
            '  "turn halfway to the right" -> {"action":"move","moves":[{"servo":6,"target":350}],"say":"Turning halfway right."}\n'
            '  "move motor 3 up a bit" -> {"action":"move","moves":[{"servo":3,"delta":120}],"say":"Raising joint 3."}\n'
            '  "nudge the base left a little" -> {"action":"move","moves":[{"servo":6,"delta":120}],"say":"Nudging left."}\n'
            '  "open your gripper" -> {"action":"open","say":"Opening up."}\n'
            '  "what can you do?" -> {"action":"none","say":"I can turn, nod, grip, and move each joint to a position."}'
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
                action, moves, say = self._reason(text)
                self.get_logger().info(f'Decided -> action={action!r}, moves={moves}, say={say!r}')
                if say:
                    self._speak(say)
                if action == 'move':
                    self.execute_moves(moves)
                elif action in ACTIONS:
                    self.execute_action(action)
            except Exception as exc:  # network error, malformed response, etc.
                self.get_logger().error(f'Failed to handle turn "{text}": {exc}')
            finally:
                self._turn_queue.task_done()

    # --- LLM reasoning ------------------------------------------------------------
    def _reason(self, text: str):
        """Send the turn (with rolling history) to the LLM; return (action, moves, say)."""
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(self._history)
        messages.append({"role": "user", "content": text})

        response = self.llm_client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            temperature=0.3,
            response_format={"type": "json_object"},
        )
        content = (response.choices[0].message.content or "").strip()
        self.get_logger().info(f'LLM response: {content}')
        action, moves, say = self._parse_reply(content)

        self._history.append({"role": "user", "content": text})
        self._history.append({"role": "assistant", "content": content})
        return action, moves, say

    @staticmethod
    def _parse_reply(content: str):
        """Extract (action, moves, say) from the LLM reply. Tolerant of fences / <think>."""
        content = re.sub(r"<think>.*?</think>", "", content, flags=re.DOTALL)
        match = re.search(r"\{.*\}", content, flags=re.DOTALL)
        if not match:
            return "none", [], content.strip() or ""
        try:
            reply = json.loads(match.group(0))
        except json.JSONDecodeError:
            return "none", [], ""
        action = reply.get("action", "none")
        moves = reply.get("moves", []) or []
        if not isinstance(moves, list):
            moves = []
        say = str(reply.get("say", "")).strip()
        if action != "move" and action not in ACTIONS:
            action = "none"
        return action, moves, say

    # --- Speech output ------------------------------------------------------------
    def _speak(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f'🗣  {text}')

    # --- Motion -------------------------------------------------------------------
    def execute_moves(self, moves) -> None:
        """Execute parametric per-servo moves (absolute target or relative delta)."""
        positions = []
        for m in moves:
            try:
                servo = int(m.get("servo"))
            except (TypeError, ValueError):
                continue
            if servo < 1 or servo > 6:
                self.get_logger().warn(f'Ignoring move for invalid servo id {servo}.')
                continue
            if m.get("target") is not None:
                target = int(m["target"])
            elif m.get("delta") is not None:
                delta = max(-MAX_DELTA, min(MAX_DELTA, int(m["delta"])))
                target = self.current.get(servo, 500) + delta
            else:
                continue
            positions.append((servo, target))
        if not positions:
            self.get_logger().info('No valid servo moves to execute.')
            return
        self.get_logger().info(f'Executing move: {positions}')
        self.set_servo_position(MOVE_DURATION, positions)
        time.sleep(MOVE_DURATION)

    def execute_action(self, action: str) -> None:
        steps = ACTIONS.get(action)
        if steps is None:
            self.get_logger().info(f'No motion mapped for action "{action}"; ignoring.')
            return
        self.get_logger().info(f'Executing gesture: {action}')
        for duration, positions in steps:
            self.set_servo_position(duration, positions)
            time.sleep(duration)

    def set_servo_position(self, duration: float, positions) -> None:
        """Publish one servo command and update tracked state. `positions`: iterable of (id, pos)."""
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
            self.current[int(servo_id)] = clamped   # remember for relative moves
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
