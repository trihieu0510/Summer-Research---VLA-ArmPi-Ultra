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
    io = node.io
    m = pc.load_map(node.map_path)
    affine = m['affine']
    h = m['heights']
    z_hover, z_place = h['z_hover'], h['z_place']
    pitch, pitch_range = h['pitch'], h['pitch_range']
    grip_close = h.get('grip_close', 540)
    grip_open = h.get('gripper_open', pc.GRIPPER_OPEN)

    io.go_view_pose(m['view_pose'])
    det = io.detect_median(node.color)
    if det is None:
        node.say(f"I can't see a {node.color} block.")
        return False

    x, y = pc.apply_affine(affine, det[0], det[1])
    node.get_logger().info(
        f'{node.color} block at pixel ({det[0]:.1f}, {det[1]:.1f}) -> robot ({x:.3f}, {y:.3f})')

    io.gripper(grip_open)
    if not (io.move_xyz(x, y, z_hover, pitch, pitch_range)
            and io.move_xyz(x, y, z_place, pitch, pitch_range, duration=1.0)):
        node.say("I can't reach that spot.")
        return False
    io.gripper(grip_close)
    io.move_xyz(x, y, z_hover, pitch, pitch_range, duration=1.0)

    # Verify: look again — if the blob is still at the pick spot, we missed.
    io.go_view_pose(m['view_pose'])
    still_there = io.detect_median(node.color, samples=3)
    if still_there is not None:
        du, dv = still_there[0] - det[0], still_there[1] - det[1]
        if (du * du + dv * dv) ** 0.5 < 40:          # px; same place = failed grasp
            node.say(f'I missed the {node.color} block.')
            return False

    if node.place_after:
        if (io.move_xyz(node.place_x, node.place_y, z_hover, pitch, pitch_range)
                and io.move_xyz(node.place_x, node.place_y, z_place, pitch, pitch_range,
                                duration=1.0)):
            io.gripper(grip_open)
            io.move_xyz(node.place_x, node.place_y, z_hover, pitch, pitch_range, duration=1.0)
            node.say(f'Picked and placed the {node.color} block.')
        else:
            node.say(f'Picked the {node.color} block, but the drop spot is unreachable.')
    else:
        node.say(f'Got the {node.color} block!')
    return True


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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('PICK RESULT:', 'success' if ok else 'FAILED')


if __name__ == '__main__':
    main()
