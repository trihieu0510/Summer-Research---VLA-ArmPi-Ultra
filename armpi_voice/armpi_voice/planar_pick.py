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
        self.declare_parameter('speech_topic', '/robot_speech')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.color = p('color')
        self.map_path = p('map_path')
        self.place_after = bool(p('place_after'))
        self.place_x = float(p('place_x'))
        self.place_y = float(p('place_y'))

        self.speech_pub = self.create_publisher(String, p('speech_topic'), 10)
        self.io = pc.ArmIO(self, p('camera_topic'))

    def say(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f'🗣  {text}')


def pick(node) -> bool:
    # The whole behavior lives in planar_common.run_pick — shared verbatim
    # with arm_agent's chat "pick" step, so CLI and chat can't drift apart.
    return pc.run_pick(node, node.io, node.color, node.map_path, node.say,
                       place_after=node.place_after,
                       place_x=node.place_x, place_y=node.place_y)


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
