#!/usr/bin/env python3
"""
planar_common.py
================
Shared helpers for the PLANAR calibration approach (planar_calib / planar_pick).

Why this exists: the vendor hand-eye calibration produced position-dependent
grasp error twice. The pick task is planar (blocks on a flat mat), so instead
of a full 3D hand-eye transform we fit a direct affine map

        image pixel (u, v)  ->  robot XY on the mat plane

from a handful of measured correspondences. The camera is ARM-MOUNTED
(eye-in-hand), so the map is only valid from ONE fixed "view pose": both
calibration capture and every later pick MUST detect from that same pose.

The fitted map is stored in ~/planar_map.yaml together with the view pose and
the working heights, so planar_pick needs no other configuration.

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.
"""

import importlib
import os
import time

import numpy as np
import yaml

# pyrefly: ignore [missing-import]
import cv2
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
# pyrefly: ignore [missing-import]
from rclpy.qos import qos_profile_sensor_data
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from servo_controller_msgs.msg import ServoPosition, ServosPosition

MAP_PATH_DEFAULT = os.path.expanduser('~/planar_map.yaml')

# The pose the camera detects from. Defaults to the pose Hiwonder's own
# calibration GUI drives to on "回中" (verified live 2026-07-08: camera sees the
# whole mat). Servo ids 1..6.
VIEW_POSE_DEFAULT = {1: 500, 2: 500, 3: 208, 4: 995, 5: 753, 6: 500}

GRIPPER_ID = 1
GRIPPER_OPEN = 200

# HSV ranges (OpenCV H in 0..180). Red wraps around 0 so it needs two bands.
COLOR_RANGES = {
    'red':   [((0, 100, 70), (10, 255, 255)), ((170, 100, 70), (180, 255, 255))],
    'green': [((45, 80, 60), (85, 255, 255))],
    'blue':  [((95, 80, 60), (130, 255, 255))],
}
MIN_BLOB_AREA = 300   # px; below this a "detection" is probably noise


def import_set_robot_pose():
    """Find the SetRobotPose service type across Hiwonder image variants.

    The service is served at /kinematics/set_pose_target (verified live), but
    the msg package name differs between image builds — probe the candidates
    and fail with an actionable message.
    """
    candidates = ['kinematics_msgs.srv', 'servo_controller_msgs.srv',
                  'ros_robot_controller_msgs.srv', 'kinematics.srv']
    for mod_name in candidates:
        try:
            mod = importlib.import_module(mod_name)
        except ImportError:
            continue
        if hasattr(mod, 'SetRobotPose'):
            return getattr(mod, 'SetRobotPose')
    raise ImportError(
        'Could not find SetRobotPose. Run on the Pi:\n'
        "  grep -rn 'import.*SetRobotPose' ~/software/hand2cam_tf_matrix_software/calibration/main.py\n"
        'and report the import line.'
    )


# --- Detection ---------------------------------------------------------------------

def detect_block(bgr, color):
    """Return (u, v, area) of the largest blob of `color`, or None."""
    ranges = COLOR_RANGES.get(color)
    if ranges is None:
        raise ValueError(f'Unknown color "{color}" (have: {list(COLOR_RANGES)})')
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in ranges:
        band = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = band if mask is None else cv2.bitwise_or(mask, band)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    biggest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(biggest)
    if area < MIN_BLOB_AREA:
        return None
    m = cv2.moments(biggest)
    return (m['m10'] / m['m00'], m['m01'] / m['m00'], area)


# --- The planar map ----------------------------------------------------------------

def fit_affine(pixels, xys):
    """Least-squares affine fit pixel->XY. Returns (2x3 matrix, per-point residuals [m])."""
    pixels = np.asarray(pixels, dtype=float)
    xys = np.asarray(xys, dtype=float)
    if len(pixels) < 3:
        raise ValueError('Need at least 3 point pairs for an affine fit.')
    design = np.hstack([pixels, np.ones((len(pixels), 1))])       # [u v 1]
    coeffs, *_ = np.linalg.lstsq(design, xys, rcond=None)          # (3x2)
    affine = coeffs.T                                              # (2x3)
    predicted = design @ coeffs
    residuals = np.linalg.norm(predicted - xys, axis=1)
    return affine, residuals


def apply_affine(affine, u, v):
    x = affine[0][0] * u + affine[0][1] * v + affine[0][2]
    y = affine[1][0] * u + affine[1][1] * v + affine[1][2]
    return float(x), float(y)


def save_map(path, affine, view_pose, heights, points):
    data = {
        'affine': np.asarray(affine).tolist(),
        'view_pose': {int(k): int(v) for k, v in view_pose.items()},
        'heights': heights,          # {'z_hover': ..., 'z_place': ..., 'pitch': ...}
        'points': points,            # the raw correspondences, for auditing
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def load_map(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    data['view_pose'] = {int(k): int(v) for k, v in data['view_pose'].items()}
    return data


# --- Arm + camera I/O ---------------------------------------------------------------

class ArmIO:
    """Motion + vision primitives shared by planar_calib and planar_pick.

    Owns nothing rclpy-global: pass in an already-constructed Node. A background
    executor must be spinning the node (both tools spin in a daemon thread).
    """

    def __init__(self, node, camera_topic, ik_timeout=6.0):
        self.node = node
        self.ik_timeout = ik_timeout
        self.bridge = CvBridge()
        self._frame = None
        self._frame_stamp = 0.0

        self.servo_pub = node.create_publisher(ServosPosition, '/servo_controller', 1)
        SetRobotPose = import_set_robot_pose()
        self.ik_client = node.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self._SetRobotPose = SetRobotPose
        node.create_subscription(Image, camera_topic, self._on_image, qos_profile_sensor_data)

    # -- camera --
    def _on_image(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._frame_stamp = time.monotonic()
        except Exception as exc:                      # noqa: BLE001
            self.node.get_logger().warn(f'Image conversion failed: {exc}')

    def fresh_frame(self, max_age=1.0, timeout=5.0):
        """Return the newest frame, waiting for one younger than max_age."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._frame is not None and time.monotonic() - self._frame_stamp < max_age:
                return self._frame.copy()
            time.sleep(0.05)
        return None

    def detect_median(self, color, samples=5):
        """Median centroid over several frames — rejects single-frame flicker."""
        hits = []
        for _ in range(samples):
            frame = self.fresh_frame()
            if frame is None:
                continue
            det = detect_block(frame, color)
            if det is not None:
                hits.append(det[:2])
            time.sleep(0.15)
        if len(hits) < max(2, samples // 2):
            return None
        arr = np.array(hits)
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))

    # -- servos --
    def set_servos(self, duration, positions):
        """positions: iterable of (servo_id, pulse). Blocks for `duration`."""
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'pulse'
        for servo_id, pulse in positions:
            p = ServoPosition()
            p.id = int(servo_id)
            p.position = float(max(0, min(1000, pulse)))
            msg.position.append(p)
        self.servo_pub.publish(msg)
        time.sleep(duration + 0.2)

    def go_view_pose(self, view_pose, duration=1.5, settle=1.0):
        self.set_servos(duration, sorted(view_pose.items()))
        time.sleep(settle)   # let the arm stop swaying before trusting pixels

    def gripper(self, pulse, duration=0.6):
        self.set_servos(duration, [(GRIPPER_ID, pulse)])

    # -- IK --
    def ik_solve(self, x, y, z, pitch, pitch_range, resolution=1.0, duration=1.5):
        """Ask /kinematics/set_pose_target for pulses; return list or None.

        Response.pulse holds 5 pulses for servos [6, 5, 4, 3, 2] in that order
        (verified against Hiwonder's calibration GUI, 2026-07-08).
        """
        req = self._SetRobotPose.Request()
        req.position = [float(x), float(y), float(z)]
        req.pitch = float(pitch)
        req.pitch_range = [float(pitch_range[0]), float(pitch_range[1])]
        req.resolution = float(resolution)
        req.duration = float(duration)
        future = self.ik_client.call_async(req)
        deadline = time.monotonic() + self.ik_timeout
        while not future.done():
            if time.monotonic() > deadline:
                self.node.get_logger().error('IK service call timed out.')
                return None
            time.sleep(0.05)
        resp = future.result()
        if resp is None or not resp.success:
            return None
        return list(resp.pulse)

    def move_xyz(self, x, y, z, pitch, pitch_range, duration=1.5):
        """IK + execute. Returns True on success. Gripper is left untouched."""
        pulses = self.ik_solve(x, y, z, pitch, pitch_range, duration=duration)
        if pulses is None or len(pulses) != 5:
            self.node.get_logger().warn(f'IK failed for ({x:.3f}, {y:.3f}, {z:.3f})')
            return False
        pairs = list(zip((6, 5, 4, 3, 2), pulses))
        self.set_servos(duration, pairs)
        return True
