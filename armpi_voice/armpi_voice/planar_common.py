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
GRIPPER_OPEN = 100   # verified live 2026-07-10: 100 = wide open, higher = narrower

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

def detect_block(bgr, color, roi=None):
    """Return (u, v, area) of the largest blob of `color`, or None.

    roi=(u0, v0, u1, v1) restricts the search to that pixel window — used to
    keep detection ON THE MAT. Without it, any bigger red object in the
    background wins (seen live 2026-07-10: v=122 hit, mat starts at v~180).
    """
    ranges = COLOR_RANGES.get(color)
    if ranges is None:
        raise ValueError(f'Unknown color "{color}" (have: {list(COLOR_RANGES)})')
    off_u = off_v = 0
    if roi is not None:
        h, w = bgr.shape[:2]
        u0 = max(0, min(w - 1, int(roi[0])))
        v0 = max(0, min(h - 1, int(roi[1])))
        u1 = max(u0 + 1, min(w, int(roi[2])))
        v1 = max(v0 + 1, min(h, int(roi[3])))
        bgr = bgr[v0:v1, u0:u1]
        off_u, off_v = u0, v0
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
    # minAreaRect: center is steadier than the moment centroid on partial
    # masks, and the angle lets the wrist align the jaws to a rotated block.
    (cx, cy), _size, angle = cv2.minAreaRect(biggest)
    angle = angle % 90.0
    if angle > 45.0:
        angle -= 90.0           # square block: orientation is mod 90, in [-45, 45)
    return (cx + off_u, cy + off_v, area, angle)


def roi_from_points(points, margin=60):
    """Pixel bounding box of the calibration points (+margin) = the mat area."""
    if not points:
        return None
    us = [p['pixel'][0] for p in points]
    vs = [p['pixel'][1] for p in points]
    return (min(us) - margin, min(vs) - margin, max(us) + margin, max(vs) + margin)


def hover_z(x, z_hover):
    """Far targets can't reach the standard hover height (IK refused
    (0.25, y, 0.10) live) — approach them lower."""
    return z_hover - 0.04 if x > 0.21 else z_hover


def wrist_delta_units(H, u, v, angle_img_deg, x, y, sign=1):
    """Wrist-roll offset (servo units) to align the jaws with a rotated block.

    The block's image angle is mapped through the homography to a world angle,
    then compared to the arm's radial direction at the target (IK aligns the
    jaws radially by default). 0.24 deg/unit; result clamped to +/-45 deg.
    `sign` flips direction if the wrist turns the wrong way on hardware.
    """
    import math
    du = math.cos(math.radians(angle_img_deg))
    dv = math.sin(math.radians(angle_img_deg))
    x0, y0 = apply_planar(H, u, v)
    x1, y1 = apply_planar(H, u + 20 * du, v + 20 * dv)
    yaw_world = math.degrees(math.atan2(y1 - y0, x1 - x0))
    radial = math.degrees(math.atan2(y, x))
    delta = (yaw_world - radial) % 90.0
    if delta > 45.0:
        delta -= 90.0
    return int(round(sign * delta / 0.24))


# --- The planar map ----------------------------------------------------------------
# The camera views the mat at an angle, so pixel->world has real PERSPECTIVE:
# on 2026-07-08 data the same 12cm of mat spanned 328px near vs 153px far. An
# affine (constant-scale) fit left 14mm mean error; a homography fits ~3mm.

def _fit_homography(pixels, xys):
    """Normalized DLT homography pixel->XY. Returns 3x3 H."""
    pixels = np.asarray(pixels, dtype=float)
    xys = np.asarray(xys, dtype=float)
    n = len(pixels)

    def norm_transform(p):
        mean = p.mean(0)
        scale = np.sqrt(2) / np.mean(np.linalg.norm(p - mean, axis=1))
        return np.array([[scale, 0, -scale * mean[0]],
                         [0, scale, -scale * mean[1]],
                         [0, 0, 1]])

    Tp, Tx = norm_transform(pixels), norm_transform(xys)
    ph = (Tp @ np.hstack([pixels, np.ones((n, 1))]).T).T
    xh = (Tx @ np.hstack([xys, np.ones((n, 1))]).T).T
    rows = []
    for (u, v, _), (x, y, _) in zip(ph, xh):
        rows.append([0, 0, 0, -u, -v, -1, y * u, y * v, y])
        rows.append([u, v, 1, 0, 0, 0, -x * u, -x * v, -x])
    _, _, vt = np.linalg.svd(np.array(rows))
    return np.linalg.inv(Tx) @ vt[-1].reshape(3, 3) @ Tp


def _residuals(H, pixels, xys):
    pixels = np.asarray(pixels, dtype=float)
    xys = np.asarray(xys, dtype=float)
    q = (np.asarray(H) @ np.hstack([pixels, np.ones((len(pixels), 1))]).T).T
    return np.linalg.norm(q[:, :2] / q[:, 2:3] - xys, axis=1)


def fit_planar(pixels, xys, drop_thresh=0.012, min_keep=5):
    """Homography fit with BOUNDED leave-worst-out outlier rejection.

    Rejection exists because a rolled block poisons the map — but unbounded
    rejection overfits: live it dropped 4/9 points (all in the far half),
    leaving a map that was 2mm in the near rows and fictional far (v=187
    mapped to an impossible x=0.251). Rule: drop at most 1/3 of the points,
    then live with the residuals. A map honest to ~10mm everywhere grasps a
    3cm block; a locally-perfect extrapolating map does not.
    Returns (H, kept_indices, kept_residuals).
    """
    kept = list(range(len(pixels)))
    if len(kept) < 4:
        raise ValueError('Need at least 4 point pairs for a homography fit.')
    max_drops = min(len(kept) // 3, len(kept) - min_keep)
    drops = 0
    while True:
        H = _fit_homography([pixels[i] for i in kept], [xys[i] for i in kept])
        res = _residuals(H, [pixels[i] for i in kept], [xys[i] for i in kept])
        if res.max() <= drop_thresh or drops >= max_drops:
            return H, kept, res
        kept.pop(int(res.argmax()))
        drops += 1


def apply_planar(H, u, v):
    H = np.asarray(H)
    q = H @ np.array([u, v, 1.0])
    return float(q[0] / q[2]), float(q[1] / q[2])


def far_z(x, z):
    """Height compensation at reach — the arm droops as it extends (dug into
    the mat at far grid points, bending a gripper screw on 2026-07-10).
    Graded, not a single step: +5mm beyond x=0.19, +10mm beyond x=0.22
    (Hiwonder's own demo uses +10mm past 0.22)."""
    if x > 0.22:
        return z + 0.010
    if x > 0.19:
        return z + 0.005
    return z


def save_map(path, H, view_pose, heights, points):
    data = {
        'homography': np.asarray(H).tolist(),
        'view_pose': {int(k): int(v) for k, v in view_pose.items()},
        'heights': heights,          # {'z_hover': ..., 'z_place': ..., 'pitch': ...}
        'points': points,            # the raw correspondences, for auditing/refits
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def load_map(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    data['view_pose'] = {int(k): int(v) for k, v in data['view_pose'].items()}
    # Refit from the stored raw points when possible: older maps (affine-era)
    # upgrade to a homography automatically, and outliers get re-rejected.
    points = data.get('points') or []
    if len(points) >= 4:
        pixels = [p['pixel'] for p in points]
        xys = [p['xy'] for p in points]
        H, kept, res = fit_planar(pixels, xys)
        data['H'] = H
        # The map is only trustworthy where SURVIVING points support it —
        # dropped outliers must not extend the trusted zone (live failure:
        # far-row points got rejected, yet far targets passed the guard and
        # the arm overshot "in front of the cube").
        data['kept_points'] = [points[i] for i in kept]
        data['fit_info'] = (f'{len(kept)}/{len(points)} points kept, '
                            f'worst {res.max() * 1000:.1f}mm')
    elif 'homography' in data:
        data['H'] = np.asarray(data['homography'])
        data['kept_points'] = []
        data['fit_info'] = 'stored homography (no raw points)'
    else:
        raise ValueError(f'{path} has neither enough points nor a homography.')
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
        self.frame_height = None

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
            self.frame_height = self._frame.shape[0]
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

    def detect_median(self, color, samples=5, roi=None):
        """Median centroid+angle over several frames — rejects flicker.

        Returns (u, v, angle_deg) or None; angle is the block's image-frame
        orientation in [-45, 45).
        """
        hits = []
        for _ in range(samples):
            frame = self.fresh_frame()
            if frame is None:
                continue
            det = detect_block(frame, color, roi=roi)
            if det is not None:
                hits.append((det[0], det[1], det[3]))
            time.sleep(0.15)
        if len(hits) < max(2, samples // 2):
            return None
        arr = np.array(hits)
        return (float(np.median(arr[:, 0])), float(np.median(arr[:, 1])),
                float(np.median(arr[:, 2])))

    def save_debug(self, path, det=None, roi=None):
        """Write the last frame with the ROI box + detection dot for eyeballing."""
        if self._frame is None:
            return
        img = self._frame.copy()
        if roi is not None:
            h, w = img.shape[:2]
            cv2.rectangle(img, (max(0, int(roi[0])), max(0, int(roi[1]))),
                          (min(w - 1, int(roi[2])), min(h - 1, int(roi[3]))),
                          (0, 255, 0), 2)
        if det is not None:
            cv2.circle(img, (int(det[0]), int(det[1])), 12, (0, 0, 255), 3)
        cv2.imwrite(path, img)

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

    def move_xyz(self, x, y, z, pitch, pitch_range, duration=1.5, wrist_delta=None):
        """IK + execute. Returns True on success. Gripper is left untouched.

        wrist_delta (servo units) is added to the IK's servo-2 (wrist roll)
        solution so the jaws track a rotated block through the approach.
        """
        pulses = self.ik_solve(x, y, z, pitch, pitch_range, duration=duration)
        if pulses is None or len(pulses) != 5:
            self.node.get_logger().warn(f'IK failed for ({x:.3f}, {y:.3f}, {z:.3f})')
            return False
        pairs = list(zip((6, 5, 4, 3, 2), pulses))
        if wrist_delta:
            pairs = [(sid, p + wrist_delta if sid == 2 else p) for sid, p in pairs]
        self.set_servos(duration, pairs)
        return True


# --- The pick behavior (shared by planar_pick CLI and arm_agent chat skill) --------

def run_pick(node, io, color, map_path, say, place_after=False,
             place_x=0.13, place_y=-0.12, wrist_sign=1):
    """Detect the colored block, map to robot XY, grasp, verify. True on success.

    `say` is a callback(str) — publishes to /robot_speech in both callers, so
    the robot narrates outcomes identically from the CLI and from the chat.
    Raises FileNotFoundError if the map is missing (caller explains).
    """
    m = load_map(map_path)
    node.get_logger().info(f'Planar map fit: {m["fit_info"]}')
    h = m['heights']
    z_hover, z_place = h['z_hover'], h['z_place']
    pitch, pitch_range = h['pitch'], h['pitch_range']
    grip_close = h.get('grip_close', 540)
    grip_open = h.get('gripper_open', GRIPPER_OPEN)
    roi = roi_from_points(m.get('points'))

    io.go_view_pose(m['view_pose'])
    det = io.detect_median(color, roi=roi)
    io.save_debug('/tmp/pick_debug.jpg', det, roi)   # eyeball what it saw
    if det is None:
        say(f"I can't see a {color} block on the mat.")
        return False

    x, y = apply_planar(m['H'], det[0], det[1])
    node.get_logger().info(
        f'{color} block at pixel ({det[0]:.1f}, {det[1]:.1f}) -> robot ({x:.3f}, {y:.3f})')

    # Targets outside the SUPPORTED area (surviving fit points only) are
    # extrapolation — physically out of reach or a bad detection. Refuse.
    support = m.get('kept_points') or m.get('points', [])
    cal_x = [pt['xy'][0] for pt in support]
    cal_y = [pt['xy'][1] for pt in support]
    # 2.5cm margin: a good map degrades fast beyond its last supported row
    # (4cm of grace produced "too in front of the cube" overshoots live).
    if cal_x and not (min(cal_x) - 0.025 <= x <= max(cal_x) + 0.025
                      and min(cal_y) - 0.025 <= y <= max(cal_y) + 0.025):
        node.get_logger().error(
            f'Mapped target ({x:.3f}, {y:.3f}) outside calibrated area '
            f'x[{min(cal_x):.3f},{max(cal_x):.3f}] y[{min(cal_y):.3f},{max(cal_y):.3f}].')
        say(f'The {color} block is outside the zone I can reach.')
        return False

    # Align the jaws with the block's orientation — an axis-aligned grip on a
    # rotated block catches a corner/edge instead of the faces (seen live).
    wrist = wrist_delta_units(m['H'], det[0], det[1], det[2], x, y, sign=wrist_sign)
    node.get_logger().info(f'block angle {det[2]:.0f}° in image -> wrist delta {wrist} units')

    z_pl = far_z(x, z_place)
    z_hv = hover_z(x, z_hover)
    io.gripper(grip_open)
    if not (io.move_xyz(x, y, z_hv, pitch, pitch_range, wrist_delta=wrist)
            and io.move_xyz(x, y, z_pl, pitch, pitch_range, duration=1.0, wrist_delta=wrist)):
        node.get_logger().error(
            f'IK refused ({x:.3f}, {y:.3f}) at z_hover={z_hv} / z_place={z_pl}, '
            f'pitch={pitch} range={pitch_range}')
        say("I can't reach that spot.")
        return False
    io.gripper(grip_close)
    io.move_xyz(x, y, z_hv, pitch, pitch_range, duration=1.0, wrist_delta=wrist)

    # Verify: any blob still ON THE MAT means the grasp failed — same spot is
    # a clean miss, elsewhere means we knocked it. A held block only ever
    # appears in the bottom strip of the frame, where the gripper is.
    io.go_view_pose(m['view_pose'])
    still_there = io.detect_median(color, samples=3, roi=roi)
    if still_there is not None:
        du, dv = still_there[0] - det[0], still_there[1] - det[1]
        if (du * du + dv * dv) ** 0.5 < 40:
            say(f'I missed the {color} block.')
            return False
        frame_h = io.frame_height or 400
        if still_there[1] <= frame_h - 130:
            say(f'I knocked away the {color} block.')
            return False

    if place_after:
        place_hv = hover_z(place_x, z_hover)
        if (io.move_xyz(place_x, place_y, place_hv, pitch, pitch_range)
                and io.move_xyz(place_x, place_y, far_z(place_x, z_place),
                                pitch, pitch_range, duration=1.0)):
            io.gripper(grip_open)
            io.move_xyz(place_x, place_y, place_hv, pitch, pitch_range, duration=1.0)
            say(f'Picked and placed the {color} block.')
        else:
            say(f'Picked the {color} block, but the drop spot is unreachable.')
    else:
        say(f'Got the {color} block!')
    return True
