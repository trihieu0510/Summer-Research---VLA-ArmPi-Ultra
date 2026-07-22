#!/usr/bin/env python3
"""
eval_runner.py
==============
Guided evaluation sessions: walks the operator through the Week-6 trial
matrix (positions × colors × phrasings) so a lab session is just "place the
block where I say, press Enter, confirm what happened".

Each trial:
    1. prints WHERE to place WHICH block (and whether rotated),
    2. publishes the phrasing to /voice_words — the FULL pipeline runs
       (LLM included), exactly as if the command were spoken,
    3. collects the robot's replies from /robot_speech until one looks like
       an outcome ("Got the...", "I missed..."), suggests a verdict,
    4. the OPERATOR confirms or overrides — the operator's verdict is the
       ground truth eval_analysis.py reports on (the robot also self-logs to
       ~/pick_trials.jsonl via run_pick's TrialLog; the two files cross-check).

Sessions are RESUMABLE: already-logged trial ids are skipped on restart, so
an interrupted session (or a servo swap mid-week) costs nothing.

Run (chat.sh stack must be up — agent + SDK + camera; console not needed):
    ros2 run armpi_voice eval_runner
    ros2 run armpi_voice eval_runner --ros-args -p rounds:=2 -p colors:="[red]"

Analysis afterwards (works on the Pi or the laptop):
    python3 eval_analysis.py ~/eval_trials.jsonl ~/pick_trials.jsonl

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.
"""

import json
import os
import queue
import re
import threading
import time

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String

# The trial matrix. Positions cover the calibrated band (x 0.13-0.22,
# y +/-0.09); P* flat, R* rotated ~30-45 deg. Sector positions (block beyond
# +/-36 deg) stay OUT of the default matrix until base_rot_sign is
# live-verified — add them with -p include_sector:=true afterwards.
POSITIONS = [
    # label,  x,     y,     rotated
    ('P1', 0.13, -0.06, False),
    ('P2', 0.13,  0.06, False),
    ('P3', 0.16, -0.09, False),
    ('P4', 0.16,  0.09, False),
    ('P5', 0.19,  0.00, False),
    ('P6', 0.22,  0.00, False),   # far row — droop-compensated territory
    ('R1', 0.14,  0.00, True),
    ('R2', 0.16, -0.06, True),
    ('R3', 0.16,  0.06, True),
    ('R4', 0.19,  0.09, True),
]
SECTOR_POSITIONS = [
    ('SL', 0.16,  0.00, False),   # place ~35 deg LEFT of straight ahead
    ('SR', 0.16,  0.00, False),   # place ~35 deg RIGHT of straight ahead
]

PHRASINGS = [
    'grab the {color} block',
    'pick up the {color} block',
    'can you get the {color} block?',
    'please grab the {color} one',
]

# Map the robot's spoken outcome lines (from run_pick) to suggested verdicts.
CLAIM_PATTERNS = [
    (r'picked and placed', 'success'),
    (r'got the \w+ block', 'success'),
    (r'drop spot is unreachable', 'success'),   # pick itself succeeded
    (r'i missed', 'missed'),
    (r'knocked away', 'knocked'),
    (r"can't see", 'not_seen'),
    (r'outside the zone', 'out_of_zone'),
    (r"can't reach", 'ik_refused'),
    (r'went wrong', 'error'),
]

VERDICT_KEYS = {
    '': None,            # accept the suggestion
    'y': 'success',
    'm': 'missed',
    'k': 'knocked',
    'n': 'not_seen',
    'u': 'ik_refused',
    'z': 'out_of_zone',
    'o': 'other',
}


class EvalRunner(Node):
    def __init__(self) -> None:
        super().__init__('eval_runner')
        self.declare_parameter('voice_topic', '/voice_words')
        self.declare_parameter('speech_topic', '/robot_speech')
        self.declare_parameter('log_path', '~/eval_trials.jsonl')
        self.declare_parameter('colors', ['red', 'green', 'blue'])
        # rounds x positions x colors trials total (4 x 10 x 3 = 120 ~ the
        # ~130-trial Week-6 target). Phrasings rotate across trials.
        self.declare_parameter('rounds', 4)
        self.declare_parameter('include_sector', False)
        # A pick with a sector scan can take a while; be generous.
        self.declare_parameter('reply_timeout', 120.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.log_path = os.path.expanduser(p('log_path'))
        self.colors = list(p('colors'))
        self.rounds = int(p('rounds'))
        self.include_sector = bool(p('include_sector'))
        self.reply_timeout = float(p('reply_timeout'))

        self.pub = self.create_publisher(String, p('voice_topic'), 10)
        self.create_subscription(String, p('speech_topic'), self._on_reply, 10)
        self._replies: "queue.Queue[str]" = queue.Queue()

    def _on_reply(self, msg: String) -> None:
        self._replies.put(msg.data)

    # -- one command round-trip ------------------------------------------------------
    def send_and_wait(self, text):
        """Publish a command; return (all_replies, claim) where claim is the
        first verdict whose pattern matched a reply (None on timeout)."""
        while not self._replies.empty():          # drop stale chatter
            try:
                self._replies.get_nowait()
            except queue.Empty:
                break
        self.pub.publish(String(data=text))
        replies, deadline = [], time.monotonic() + self.reply_timeout
        while time.monotonic() < deadline:
            try:
                reply = self._replies.get(timeout=deadline - time.monotonic())
            except queue.Empty:
                break
            replies.append(reply)
            print(f'  robot> {reply}')
            for pattern, verdict in CLAIM_PATTERNS:
                if re.search(pattern, reply.lower()):
                    return replies, verdict
        return replies, None


def build_matrix(node):
    positions = list(POSITIONS) + (SECTOR_POSITIONS if node.include_sector else [])
    trials = []
    n = 0
    for rnd in range(1, node.rounds + 1):
        for label, x, y, rotated in positions:
            for color in node.colors:
                trials.append({
                    'trial_id': f'r{rnd}:{label}:{color}',
                    'round': rnd, 'label': label, 'x': x, 'y': y,
                    'rotated': rotated, 'color': color,
                    'phrasing': PHRASINGS[n % len(PHRASINGS)].format(color=color),
                })
                n += 1
    return trials


def done_ids(path):
    """Trial ids already logged — lets an interrupted session resume."""
    ids = set()
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    ids.add(json.loads(line)['trial_id'])
                except (json.JSONDecodeError, KeyError):
                    continue
    return ids


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return 'q'


def placement_text(t):
    side = 'centre' if abs(t['y']) < 0.01 else (
        f"{abs(t['y']) * 100:.0f}cm {'LEFT' if t['y'] > 0 else 'RIGHT'}")
    rot = 'ROTATED ~30-45 deg' if t['rotated'] else 'square to the arm'
    extra = ''
    if t['label'] == 'SL':
        extra = '  ** ~35 deg LEFT of straight ahead (sector test) **'
    if t['label'] == 'SR':
        extra = '  ** ~35 deg RIGHT of straight ahead (sector test) **'
    return (f"Place the {t['color'].upper()} block at {t['label']}: "
            f"x={t['x']:.2f}m out, {side}, {rot}.{extra}")


def run(node):
    trials = build_matrix(node)
    done = done_ids(node.log_path)
    todo = [t for t in trials if t['trial_id'] not in done]
    print(f'\n=== Eval session: {len(todo)} of {len(trials)} trials remaining '
          f'(log: {node.log_path}) ===')
    print('Directions are from BEHIND the arm (operator behind the base): '
          '+y = LEFT.\nKeys at the verdict prompt: Enter = accept suggestion, '
          'y/m/k/n/u/z/o = success/missed/knocked/not_seen/unreachable/'
          'out_of_zone/other, r = redo, s = skip, q = quit.\n')

    for t in todo:
        while True:                                       # redo loop
            print(f"--- {t['trial_id']}  ({len(done)} done) ---")
            print(placement_text(t))
            cmd = ask('Enter = send command | s = skip | q = quit: ')
            if cmd == 'q':
                return
            if cmd == 's':
                break
            print(f"  you>  {t['phrasing']}")
            t0 = time.time()
            replies, claim = node.send_and_wait(t['phrasing'])
            elapsed = round(time.time() - t0, 1)
            if not replies:
                print('  (no reply at all — is arm_agent running?)')
            suggestion = claim or 'other'
            reply = ask(f'Verdict [{suggestion}]? ')
            if reply == 'q':
                return
            if reply == 'r':
                continue
            if reply == 's':
                break
            verdict = VERDICT_KEYS.get(reply, None)
            if reply and verdict is None:
                verdict = 'other'
            verdict = verdict or suggestion
            note = ''
            if verdict == 'other':
                note = ask('  note (what actually happened): ')
            rec = dict(t, robot_claim=claim, verdict=verdict, note=note,
                       replies=replies, elapsed_s=elapsed,
                       ts=round(time.time(), 2))
            with open(node.log_path, 'a') as f:
                f.write(json.dumps(rec) + '\n')
            done.add(t['trial_id'])
            print(f'  logged: {verdict}\n')
            break

    print(f'\nSession complete — {len(done)} trials in {node.log_path}.')
    print('Analyze with:  python3 eval_analysis.py ~/eval_trials.jsonl')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvalRunner()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        run(node)
    except KeyboardInterrupt:
        print('\nInterrupted — session is resumable, nothing lost.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
