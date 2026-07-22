#!/usr/bin/env python3
"""
planar_pick.py
==============
Pick a colored block using the planar map fitted by planar_calib.

Flow: view pose -> detect the block's pixel centroid -> map to robot XY via
the affine from ~/planar_map.yaml -> IK hover -> descend -> close -> lift.
The camera is arm-mounted, so detection ALWAYS happens from the view pose
stored in the map (the pose the map was calibrated at).

Run (arm SDK + camera must be up):
    ros2 run armpi_voice planar_pick                       # red, then hold
    ros2 run armpi_voice planar_pick --ros-args -p color:=blue -p place_after:=true

`place_after:=true` drops the block at place_x/place_y after a successful lift
(a fixed, IK-reachable drop spot — the seam for "put it in the box" later).
"""

import threading
import time

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String

from . import planar_common as pc


class PlanarPick(Node):
    def __init__(self) -> None:
        super().__init__('planar_pick')
        self.declare_parameter('color', 'red')
        self.declare_parameter('camera_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('map_path', pc.MAP_PATH_DEFAULT)
        self.declare_parameter('place_after', False)
        self.declare_parameter('place_x', 0.13)
        self.declare_parameter('place_y', -0.12)
        # Named destination ("box") — overrides place_x/place_y and implies
        # place_after. Names come from the map's `destinations:` / DESTINATIONS.
        self.declare_parameter('place', '')
        # Eval harness: one JSONL line per attempt ('' disables).
        self.declare_parameter('trial_log', '~/pick_trials.jsonl')
        # Flip to -1 if the wrist rotates AWAY from the block's angle on test.
        self.declare_parameter('wrist_sign', 1)
        # Constant grasp trim in meters (tune live, then persist in the map's
        # heights: trim_x/trim_y). +x forward, +y left (viewed from BEHIND).
        self.declare_parameter('trim_x', 0.0)
        self.declare_parameter('trim_y', 0.0)
        # Scan neighboring base sectors (~±36°) when the block isn't in front.
        self.declare_parameter('scan', True)
        # Flip to -1 if sector picks land mirrored around the base axis.
        self.declare_parameter('base_rot_sign', 1)
        self.declare_parameter('speech_topic', '/robot_speech')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.color = p('color')
        self.map_path = p('map_path')
        self.place_after = bool(p('place_after'))
        self.place_x = float(p('place_x'))
        self.place_y = float(p('place_y'))
        self.place_name = str(p('place')).strip()
        self.trial_log = str(p('trial_log')).strip()
        self.wrist_sign = int(p('wrist_sign'))
        self.trim_x = float(p('trim_x'))
        self.trim_y = float(p('trim_y'))
        self.scan = bool(p('scan'))
        self.base_rot_sign = int(p('base_rot_sign'))

        self.speech_pub = self.create_publisher(String, p('speech_topic'), 10)
        self.io = pc.ArmIO(self, p('camera_topic'))

    def say(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f'🗣  {text}')


def pick(node) -> bool:
    # The whole behavior lives in planar_common.run_pick — shared verbatim
    # with arm_agent's chat "pick" step, so CLI and chat can't drift apart.
    trial = pc.TrialLog(node.trial_log or None, source='cli', color=node.color)
    return pc.run_pick(node, node.io, node.color, node.map_path, node.say,
                       place_after=node.place_after,
                       place_x=node.place_x, place_y=node.place_y,
                       wrist_sign=node.wrist_sign,
                       trim_x=node.trim_x, trim_y=node.trim_y,
                       scan=node.scan, base_rot_sign=node.base_rot_sign,
                       place_name=node.place_name, trial=trial)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarPick()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    time.sleep(1.0)   # let the camera subscription warm up
    ok = False
    try:
        ok = pick(node)
    except FileNotFoundError:
        node.get_logger().error(
            f'No planar map at {node.map_path} — run planar_calib first.')
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the executor BEFORE destroying the node — the reverse order
        # races the spin thread and aborts with "terminate called without
        # an active exception" on exit.
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
    print('PICK RESULT:', 'success' if ok else 'FAILED')


if __name__ == '__main__':
    main()
