#!/usr/bin/env python3
"""
arm_agent.py
============
Conversational + motion agent for the Hiwonder ArmPi Ultra (6-DOF arm).

Each turn the LLM returns an ordered list of STEPS to perform (gestures and/or
per-servo moves), plus a spoken reply. Steps run sequentially, so you can chain
commands like "go home, then turn the base right 100, then raise motor 4 by 300".
It keeps a short rolling memory so a back-and-forth chat stays coherent.

Step kinds:
  * Named gesture   — {"action": "home"} (home/left/right/open/close/nod).
  * Parametric move — {"action": "move", "moves": [{"servo": N, "target"|"delta": V}, ...]}.

Data flow:
    /voice_words (std_msgs/String)        text command (typed console or STT)
        -> LLM (OpenAI-compatible)        -> {"steps": [...], "say": "..."}
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
MOVE_DURATION = 1.0           # seconds per parametric move step
MAX_DELTA = 300               # clamp a single relative nudge so the arm can't lurch

HOME_POSE = [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500)]

# Named gestures: each maps to motion steps (duration_seconds, [(servo_id, position), ...]).
ACTIONS = {
    # "left"/"right" are from the OPERATOR's view (facing the arm / camera view),
    # which mirrors the robot's own frame — hence left=800, right=200 on base servo 6.
    "home":  [(1.5, HOME_POSE)],
    "left":  [(1.0, [(6, 800)])],
    "right": [(1.0, [(6, 200)])],
    "open":  [(0.5, [(1, 100)])],   # 100 = wide open (verified live 2026-07-10)
    "close": [(0.5, [(1, 500)])],
    "nod":   [(0.5, [(4, 300)]), (0.5, [(4, 500)])],   # tilt down, then return
}


class ArmAgent(Node):
    """An LLM agent that runs ArmPi Ultra gesture/move sequences or holds a conversation."""

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
        # Pick skill (needs a planar_map.yaml from planar_calib + the camera):
        self.declare_parameter('camera_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('map_path', '~/planar_map.yaml')

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

        # Track each servo's last-known position so RELATIVE moves ("up a bit") work,
        # and so chained steps resolve their deltas against the running state.
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

        # --- Planar pick skill (optional — armed only if a calibration map exists) ---
        self.map_path = os.path.expanduser(self.get_parameter('map_path').value)
        self._camera_topic = self.get_parameter('camera_topic').value
        self._planar = None                      # (planar_common, ArmIO) once armed
        if os.path.exists(self.map_path):
            self._init_planar()
        else:
            self.get_logger().info(
                f'No planar map at {self.map_path} — pick skill dormant until calibrated.')

        self.voice_sub = self.create_subscription(String, voice_topic, self._on_voice, 10)
        self.get_logger().info(
            f'Ready. Listening on "{voice_topic}"; speaking on "{speech_topic}".'
        )

    def _init_planar(self) -> None:
        """Arm the pick skill: camera subscription + IK client via planar_common."""
        try:
            from . import planar_common as pc
            self._planar = (pc, pc.ArmIO(self, self._camera_topic))
            self.get_logger().info(f'Pick skill armed (map: {self.map_path}).')
        except Exception as exc:                  # missing cv_bridge, bad map, ...
            self._planar = None
            self.get_logger().warn(f'Pick skill unavailable: {exc}')

    # --- Prompt -------------------------------------------------------------------
    def _build_system_prompt(self) -> str:
        return (
            "You are the mind of a friendly 6-servo robotic arm (Hiwonder ArmPi Ultra). "
            "Each turn you return an ordered list of movement STEPS to perform (possibly "
            "empty for pure conversation) and a short spoken reply.\n\n"
            "SERVOS: ids 1-6, raw units 0-1000, 500 = centre. Known mapping:\n"
            "  - servo 6 = base rotation. Operator's view: RIGHT is LOWER (toward 200), "
            "LEFT is HIGHER (toward 800). So a relative 'right' = NEGATIVE delta, "
            "'left' = POSITIVE delta.\n"
            "  - servo 1 = gripper: wide open ~100, closed ~540 (higher = tighter).\n"
            "  - servos 2-5 = arm joints (shoulder/elbow/wrist); 'up/raise' = POSITIVE "
            "delta, 'down/lower' = NEGATIVE delta (~120 for 'a bit'). Exact mapping unverified.\n\n"
            "EACH STEP is one of:\n"
            '  - a named gesture: {"action": "home"}  (home/left/right/open/close/nod)\n'
            '  - a parametric move: {"action": "move", "moves": [ '
            '{"servo": <1-6>, "target": <0-1000>}  OR  {"servo": <1-6>, "delta": <signed>} ]}\n'
            '  - a pick skill: {"action": "pick", "color": "red"} — visually find the colored '
            'block on the mat and grab it (colors: red, green, blue). Add "place": true to '
            'also carry it to the drop spot and release. Use this whenever the user asks to '
            'grab/pick/get a block.\n'
            "Steps run in order, one after another, so chain them for sequences. Servos listed "
            "together inside ONE move happen simultaneously. Keep any single delta <=300.\n\n"
            "Respond with ONLY a JSON object: "
            '{"steps": [ ...ordered steps... ], "say": "<reply, 1-2 sentences>"}.\n'
            "Examples:\n"
            '  "go home, then turn the base right about 100, then raise motor 4 by 300" -> '
            '{"steps":[{"action":"home"},{"action":"move","moves":[{"servo":6,"delta":-100}]},'
            '{"action":"move","moves":[{"servo":4,"delta":300}]}],"say":"Homing, then base right 100, then joint 4 up 300."}\n'
            '  "turn halfway to the right" -> '
            '{"steps":[{"action":"move","moves":[{"servo":6,"target":350}]}],"say":"Turning halfway right."}\n'
            '  "nod and open your gripper" -> '
            '{"steps":[{"action":"nod"},{"action":"open"}],"say":"Nodding, then opening up."}\n'
            '  "grab the red block" -> '
            '{"steps":[{"action":"pick","color":"red"}],"say":"Looking for the red block!"}\n'
            '  "pick up the blue block and put it away" -> '
            '{"steps":[{"action":"pick","color":"blue","place":true}],"say":"Blue block, coming up."}\n'
            '  "what can you do?" -> '
            '{"steps":[],"say":"I can turn, nod, move each joint, chain sequences, and pick up '
            'colored blocks from the mat."}'
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
                steps, say = self._reason(text)
                self.get_logger().info(f'Decided -> steps={steps}, say={say!r}')
                if say:
                    self._speak(say)
                self.execute_steps(steps)
            except Exception as exc:  # network error, malformed response, etc.
                self.get_logger().error(f'Failed to handle turn "{text}": {exc}')
            finally:
                self._turn_queue.task_done()

    # --- LLM reasoning ------------------------------------------------------------
    def _reason(self, text: str):
        """Send the turn (with rolling history) to the LLM; return (steps, say)."""
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
        steps, say = self._parse_reply(content)

        self._history.append({"role": "user", "content": text})
        self._history.append({"role": "assistant", "content": content})
        return steps, say

    @staticmethod
    def _parse_reply(content: str):
        """Extract (steps, say) from the LLM reply. Tolerant of fences / <think> and of
        the older single-action form (wrapped into a one-element step list)."""
        content = re.sub(r"<think>.*?</think>", "", content, flags=re.DOTALL)
        match = re.search(r"\{.*\}", content, flags=re.DOTALL)
        if not match:
            return [], content.strip() or ""
        try:
            reply = json.loads(match.group(0))
        except json.JSONDecodeError:
            return [], ""

        say = str(reply.get("say", "")).strip()

        raw_steps = reply.get("steps")
        if not isinstance(raw_steps, list):
            # Backward compatibility: accept a single top-level {"action", "moves"}.
            if reply.get("action"):
                raw_steps = [{"action": reply.get("action"), "moves": reply.get("moves", [])}]
            else:
                raw_steps = []

        steps = []
        for s in raw_steps:
            if not isinstance(s, dict):
                continue
            action = s.get("action", "none")
            if action == "move":
                moves = s.get("moves", []) or []
                if isinstance(moves, list) and moves:
                    steps.append({"action": "move", "moves": moves})
            elif action == "pick":
                steps.append({"action": "pick",
                              "color": str(s.get("color", "red")).lower(),
                              "place": bool(s.get("place", False))})
            elif action in ACTIONS:
                steps.append({"action": action})
        return steps, say

    # --- Speech output ------------------------------------------------------------
    def _speak(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f'🗣  {text}')

    # --- Motion -------------------------------------------------------------------
    def execute_steps(self, steps) -> None:
        """Run an ordered list of gesture/move/pick steps, one after another."""
        for step in steps:
            if step["action"] == "move":
                self.execute_moves(step["moves"])
            elif step["action"] == "pick":
                self.execute_pick(step.get("color", "red"), step.get("place", False))
            else:
                self.execute_action(step["action"])

    def execute_pick(self, color: str, place: bool) -> None:
        """Run the planar pick skill (blocking, ~20s). Narrates its own outcome."""
        if self._planar is None and os.path.exists(self.map_path):
            self._init_planar()   # map may have appeared since startup
        if self._planar is None:
            self._speak("I haven't been calibrated for picking yet — run planar_calib first.")
            return
        pc, io = self._planar
        if color not in pc.COLOR_RANGES:
            self._speak(f"I only know red, green and blue blocks, not {color}.")
            return
        self.get_logger().info(f'Executing pick: color={color}, place={place}')
        try:
            pc.run_pick(self, io, color, self.map_path, self._speak, place_after=place)
        except FileNotFoundError:
            self._speak("My calibration map is missing — run planar_calib first.")
        except Exception as exc:                  # noqa: BLE001
            self.get_logger().error(f'Pick failed: {exc}')
            self._speak('Something went wrong while I was picking.')
        # The pick drove servos 2-6 via IK, so our tracked positions are stale;
        # snap the expectation back to the view pose the skill ends in.
        self.current.update({sid: pos for sid, pos in HOME_POSE})

    def execute_moves(self, moves) -> None:
        """Execute one parametric move (absolute target or relative delta per servo)."""
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
