#!/usr/bin/env python3
"""
planar_calib.py
===============
Interactive planar calibration: the ARM generates its own ground truth.

For each point of a small XY grid the arm (holding a colored block):
    1. places the block at a KNOWN robot (x, y) via IK and releases it,
    2. retreats to the fixed view pose and detects the block's pixel centroid,
    3. re-grasps the block at the same commanded (x, y) and moves on.

That yields pixel->XY correspondences with the robot itself as the measuring
device, so IK-frame biases cancel where it matters. A least-squares affine map
is fitted and saved to ~/planar_map.yaml for planar_pick.

Run (arm SDK + camera must be up; see COMMANDS.md):
    ros2 run armpi_voice planar_calib
    ros2 run armpi_voice planar_calib --ros-args -p color:=green -p z_place:=0.05

You stay in the loop: the console prompts before anything moves, and any point
can be retried or skipped. Keep a hand near the block for the re-grasp steps.
"""

import threading

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node

from . import planar_common as pc


class PlanarCalib(Node):
    def __init__(self) -> None:
        super().__init__('planar_calib')
        self.declare_parameter('color', 'red')
        self.declare_parameter('camera_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('map_path', pc.MAP_PATH_DEFAULT)
        # Grid of robot-frame targets (meters). 3x3 = 9 correspondences.
        self.declare_parameter('grid_x', [0.13, 0.17, 0.21])
        self.declare_parameter('grid_y', [-0.06, 0.0, 0.06])
        # Heights + approach. z_place is only a STARTING GUESS — the tune phase
        # at the beginning of every run steps it to the real height live.
        # pitch=80 and grip_close=540 come from Hiwonder's own working grasp
        # call: pick_and_place.pick(position, 80, yaw, 540, ...).
        self.declare_parameter('z_hover', 0.10)
        self.declare_parameter('z_place', 0.03)
        self.declare_parameter('pitch', 80.0)
        self.declare_parameter('pitch_range', [55.0, 120.0])
        self.declare_parameter('grip_close', 540)
        self.declare_parameter('gripper_open', 200)
        # View pose (servo pulses 1..6) — MUST match planar_pick's.
        self.declare_parameter('view_pose', [500, 500, 208, 995, 753, 500])

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.color = p('color')
        self.map_path = p('map_path')
        self.grid_x = list(p('grid_x'))
        self.grid_y = list(p('grid_y'))
        self.z_hover = float(p('z_hover'))
        self.z_place = float(p('z_place'))
        self.pitch = float(p('pitch'))
        self.pitch_range = list(p('pitch_range'))
        self.grip_close = int(p('grip_close'))
        self.gripper_open = int(p('gripper_open'))
        self.view_pose = {i + 1: int(v) for i, v in enumerate(p('view_pose'))}

        self.io = pc.ArmIO(self, p('camera_topic'))


def ask(prompt):
    """Console gate. Returns the (lowercased, stripped) reply."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return 'abort'


def place_block(node, x, y):
    io = node.io
    ok = (io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range)
          and io.move_xyz(x, y, node.z_place, node.pitch, node.pitch_range, duration=1.0))
    if not ok:
        return False
    io.gripper(node.gripper_open)                   # release
    io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range, duration=1.0)
    return True


def regrasp_block(node, x, y):
    io = node.io
    ok = (io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range)
          and io.move_xyz(x, y, node.z_place, node.pitch, node.pitch_range, duration=1.0))
    if not ok:
        return False
    io.gripper(node.grip_close)
    io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range, duration=1.0)
    return True


def tune_grip(node, x, y):
    """Interactive height + jaw tune at one spot. Ends with the block GRIPPED.

    Returns the tuned z_place, or None on abort. This exists because the mat's
    height in the IK frame is not knowable remotely — the first run proved that
    guessed constants drop the block mid-air and close the jaws on its top.
    """
    io = node.io
    z = max(node.z_place, 0.05)   # start clearly above, only ever step down
    io.gripper(node.gripper_open)
    if not io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range):
        print('Cannot reach the tune point — adjust grid parameters.')
        return None
    if ask(f'\nTUNE PHASE: put the block on the mat DIRECTLY UNDER the jaws '
           f'(x={x:.3f}, y={y:.3f}), then press Enter (or abort): ') == 'abort':
        return None
    print('Tip: first find the WIDEST jaw opening — type 100, watch the jaws, '
          'then try 700; whichever direction opens wider, keep that value. The '
          'jaws must clear the block with room to spare or the descent wedges it.')

    while True:
        if not io.move_xyz(x, y, z, node.pitch, node.pitch_range, duration=0.8):
            print(f'z={z:.3f} unreachable, stepping back up.')
            z += 0.005
            continue
        reply = ask(f'z={z * 1000:.0f}mm | d = down 5mm | u = up 5mm | '
                    f'<number> = jaw pulse (lower = wider, now {node.gripper_open}) | '
                    'Enter = jaws straddle the block, GRAB IT | abort: ')
        if reply == 'abort':
            return None
        if reply == 'd':
            z = max(0.0, z - 0.005)
        elif reply == 'u':
            z += 0.005
        elif reply.isdigit():
            node.gripper_open = int(reply)
            io.gripper(node.gripper_open)
        elif reply == '':
            io.gripper(node.grip_close)
            io.move_xyz(x, y, node.z_hover, node.pitch, node.pitch_range, duration=1.0)
            if ask('Lifted — is the block IN the jaws? Enter = yes, n = retry: ') in ('', 'y', 'yes'):
                print(f'Tuned: z_place={z:.3f}, jaws open={node.gripper_open}, '
                      f'close={node.grip_close}')
                return z
            io.gripper(node.gripper_open)
            if ask('Re-place the block under the jaws, press Enter (or abort): ') == 'abort':
                return None


def run(node) -> None:
    io = node.io
    grid = [(x, y) for x in node.grid_x for y in node.grid_y]
    print(f'\n=== Planar calibration: {len(grid)} points, color "{node.color}" ===')
    print('Arm SDK + camera must already be running (COMMANDS.md sec 2/4).')

    # -- feasibility pass: drop unreachable points before touching anything --
    reachable = [pt for pt in grid
                 if io.ik_solve(pt[0], pt[1], node.z_place, node.pitch, node.pitch_range)]
    if len(reachable) < 4:
        print(f'Only {len(reachable)}/{len(grid)} grid points are IK-reachable — '
              'adjust grid_x/grid_y/z_place parameters first.')
        return
    if len(reachable) < len(grid):
        print(f'Note: {len(grid) - len(reachable)} unreachable point(s) skipped.')

    # -- tune heights/jaws at the centre point; ends with the block gripped --
    z = tune_grip(node, *reachable[len(reachable) // 2])
    if z is None:
        return
    node.z_place = z

    pixels, xys, audit = [], [], []
    for i, (x, y) in enumerate(reachable, 1):
        while True:
            print(f'\n--- Point {i}/{len(reachable)}: place at x={x:.3f}, y={y:.3f} ---')
            if not place_block(node, x, y):
                print('IK/motion failed; skipping point.')
                break
            io.go_view_pose(node.view_pose)
            det = io.detect_median(node.color)
            if det is None:
                choice = ask('Block NOT detected from view pose. retry / skip / abort? ')
            else:
                print(f'Detected at pixel ({det[0]:.1f}, {det[1]:.1f})')
                choice = ask('Accept? Enter = yes, r = retry, s = skip, abort: ')
                if choice in ('', 'y', 'yes'):
                    pixels.append(det)
                    xys.append((x, y))
                    audit.append({'pixel': [det[0], det[1]], 'xy': [x, y]})
                    choice = 'accepted'
            if choice == 'abort':
                return
            # Re-grasp so the arm carries the block to the next point.
            if not regrasp_block(node, x, y):
                print('Re-grasp motion failed.')
            # If the jaws missed, let the user open/close them to reseat the
            # block by hand — a closed empty gripper was a dead end before.
            while True:
                reply = ask('Block back IN the gripper? Enter = yes | o = open jaws | '
                            'c = close jaws | abort: ')
                if reply == 'abort':
                    return
                if reply == 'o':
                    io.gripper(node.gripper_open)
                    continue
                if reply == 'c':
                    io.gripper(node.grip_close)
                    continue
                break
            if choice in ('accepted', 's', 'skip'):
                break

    if len(pixels) < 3:
        print(f'\nOnly {len(pixels)} usable points — not enough to fit. Nothing saved.')
        return

    spread_x = max(p[0] for p in xys) - min(p[0] for p in xys)
    spread_y = max(p[1] for p in xys) - min(p[1] for p in xys)
    if spread_x < 0.05 or spread_y < 0.05:
        print(f'\nWARNING: accepted points span only {spread_x * 1000:.0f} x '
              f'{spread_y * 1000:.0f} mm — the fit will extrapolate badly outside '
              'that area. Strongly consider re-running with more spread-out points.')

    affine, residuals = pc.fit_affine(pixels, xys)
    print('\n=== Fit results ===')
    for (x, y), r in zip(xys, residuals):
        print(f'  ({x:+.3f}, {y:+.3f})  residual {r * 1000:.1f} mm')
    print(f'  worst {residuals.max() * 1000:.1f} mm, mean {residuals.mean() * 1000:.1f} mm')

    heights = {'z_hover': node.z_hover, 'z_place': node.z_place,
               'pitch': node.pitch, 'pitch_range': node.pitch_range,
               'grip_close': node.grip_close, 'gripper_open': node.gripper_open}
    pc.save_map(node.map_path, affine, node.view_pose, heights, audit)
    print(f'\nSaved planar map -> {node.map_path}')
    print('Test it with:  ros2 run armpi_voice planar_pick')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarCalib()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        run(node)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
