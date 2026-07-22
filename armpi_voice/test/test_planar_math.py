#!/usr/bin/env python3
"""
test_planar_math.py
===================
Laptop-runnable tests for the PURE MATH in planar_common: homography fit +
bounded outlier rejection, sector rotation, wrist-delta geometry, droop/hover
height rules, and destination resolution. No robot, no ROS, no camera.

The ROS/vision imports at the top of planar_common are stubbed out when the
real packages are missing (i.e. on the Windows laptop) — every function under
test here uses only numpy + math.

Run from the repo root:
    venv/Scripts/python.exe armpi_voice/test/test_planar_math.py
(or via pytest if available). Prints PASS/FAIL per test; exit code 0 = all
pass.
"""

import importlib
import math
import sys
import types


def _stub_missing_modules():
    """Insert dummy modules for imports that only exist on the Pi."""
    def ensure(name, attrs=()):
        try:
            importlib.import_module(name)
            return
        except ImportError:
            pass
        parts = name.split('.')
        for i in range(1, len(parts) + 1):
            mod_name = '.'.join(parts[:i])
            if mod_name not in sys.modules:
                sys.modules[mod_name] = types.ModuleType(mod_name)
        mod = sys.modules[name]
        for attr in attrs:
            setattr(mod, attr, type(attr, (), {}))

    ensure('cv2', ())
    ensure('cv_bridge', ('CvBridge',))
    ensure('rclpy.qos', ())
    sys.modules['rclpy.qos'].qos_profile_sensor_data = object()
    ensure('sensor_msgs.msg', ('Image',))
    ensure('servo_controller_msgs.msg', ('ServoPosition', 'ServosPosition'))
    ensure('yaml', ())


_stub_missing_modules()

import os  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from armpi_voice import planar_common as pc  # noqa: E402

import numpy as np  # noqa: E402

FAILURES = []


def check(name, cond, detail=''):
    status = 'PASS' if cond else 'FAIL'
    print(f'  [{status}] {name}' + (f'  ({detail})' if detail and not cond else ''))
    if not cond:
        FAILURES.append(name)


# --- helpers -----------------------------------------------------------------------

def synthetic_correspondences(h_true, n_side=4):
    """Project a pixel grid through a known homography -> (pixels, xys)."""
    pixels, xys = [], []
    for u in np.linspace(100, 540, n_side):
        for v in np.linspace(120, 360, n_side - 1):
            q = h_true @ np.array([u, v, 1.0])
            pixels.append((float(u), float(v)))
            xys.append((float(q[0] / q[2]), float(q[1] / q[2])))
    return pixels, xys


# A mild camera-like perspective: x grows with v, y shrinks with u, slight
# projective term (far rows compressed — the 328px-vs-153px effect).
H_TRUE = np.array([
    [0.0,     4.0e-4, 0.10],
    [-4.5e-4, 0.0,    0.145],
    [0.0,     -8.0e-4, 1.30],
])


def test_homography_fit():
    print('homography fit:')
    pixels, xys = synthetic_correspondences(H_TRUE)
    H, kept, res = pc.fit_planar(pixels, xys)
    check('noise-free fit keeps all points', len(kept) == len(pixels))
    check('noise-free residuals ~0', float(res.max()) < 1e-9,
          f'max={res.max():.2e}')
    u, v = 321.0, 240.0
    q = H_TRUE @ np.array([u, v, 1.0])
    x, y = pc.apply_planar(H, u, v)
    check('apply_planar matches ground truth',
          abs(x - q[0] / q[2]) < 1e-9 and abs(y - q[1] / q[2]) < 1e-9)


def test_outlier_rejection_bounded():
    print('bounded outlier rejection:')
    pixels, xys = synthetic_correspondences(H_TRUE)   # 12 points
    bad = list(xys)
    bad[5] = (bad[5][0] + 0.03, bad[5][1] - 0.03)     # one rolled block: 4cm off
    H, kept, res = pc.fit_planar(pixels, bad)
    check('single outlier dropped', 5 not in kept and len(kept) == 11)
    check('clean points refit tightly', float(res.max()) < 1e-6)

    worse = list(xys)                                  # poison HALF the points
    for i in range(0, 12, 2):
        worse[i] = (worse[i][0] + 0.05, worse[i][1])
    H, kept, res = pc.fit_planar(pixels, worse)
    check('rejection bounded to <=1/3 dropped', len(kept) >= 8,
          f'kept {len(kept)}/12')


def test_sector_rotation_convention():
    print('sector rotation (the base_rot_sign geometry):')
    # rotate_xy is counter-clockwise viewed from above; +y = LEFT from
    # behind the arm. Sector +150 pulses moves servo 6 toward 800 = LEFT,
    # so with base_rot_sign=+1 a block seen straight-ahead in that sector
    # must land at positive y (left) after rotation.
    rot_deg = pc.SCAN_SECTORS[1] * pc.DEG_PER_UNIT * 1   # +150 -> +36 deg
    check('+150 pulses is +36 degrees', abs(rot_deg - 36.0) < 1e-9)
    x, y = pc.rotate_xy(0.16, 0.0, rot_deg)
    check('left sector maps to +y (left)', y > 0.09 and 0.12 < x < 0.14,
          f'({x:.4f}, {y:.4f})')
    x2, y2 = pc.rotate_xy(0.16, 0.0, -rot_deg)
    check('right sector mirrors to -y', abs(y2 + y) < 1e-12 and abs(x2 - x) < 1e-12)
    # base_rot_sign=-1 must be an exact mirror — the live P1 test decides
    # which sign is real; the math must make them symmetric alternatives.
    xm, ym = pc.rotate_xy(0.16, 0.0, pc.SCAN_SECTORS[1] * pc.DEG_PER_UNIT * -1)
    check('base_rot_sign=-1 mirrors the target', abs(ym + y) < 1e-12)
    # Radius is preserved — rotation cannot push a target out of reach.
    check('rotation preserves radius',
          abs(math.hypot(x, y) - 0.16) < 1e-12)


def test_height_rules():
    print('droop/hover height rules:')
    check('far_z inside prime zone unchanged', pc.far_z(0.15, 0.03) == 0.03)
    check('far_z +5mm past 0.19', abs(pc.far_z(0.20, 0.03) - 0.035) < 1e-12)
    check('far_z +10mm past 0.22', abs(pc.far_z(0.23, 0.03) - 0.040) < 1e-12)
    check('hover_z drops 4cm past 0.21', abs(pc.hover_z(0.22, 0.10) - 0.06) < 1e-12)
    check('hover_z normal inside', pc.hover_z(0.18, 0.10) == 0.10)


def test_wrist_delta():
    print('wrist delta geometry:')
    # Identity-scale map: pixel (u, v) -> meters (u, v) * 1e-3. Radial
    # direction at (0.2, 0) is 0 deg, so image angle == world yaw here.
    H = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1.0]])
    d0 = pc.wrist_delta_units(H, 200, 0, 0.0, 0.2, 0.0)
    check('aligned block needs no wrist', d0 == 0)
    d20 = pc.wrist_delta_units(H, 200, 0, 20.0, 0.2, 0.0)
    check('20 deg block -> ~83 units', abs(d20 - round(20 / 0.24)) <= 1,
          f'got {d20}')
    check('sign=-1 flips exactly',
          pc.wrist_delta_units(H, 200, 0, 20.0, 0.2, 0.0, sign=-1) == -d20)
    d_fold = pc.wrist_delta_units(H, 200, 0, 70.0, 0.2, 0.0)
    check('mod-90 fold: 70 deg == -20 deg', abs(d_fold + d20) <= 1,
          f'got {d_fold}')
    check('sector extra_rot folds away at 90',
          pc.wrist_delta_units(H, 200, 0, 20.0, 0.2, 0.0, extra_rot_deg=90.0) == d20)


def test_resolve_destination():
    print('destination resolution:')
    check('builtin drop resolves', pc.resolve_destination({}, 'drop') == (0.13, -0.12))
    check('case/space insensitive', pc.resolve_destination({}, '  BOX ') is not None)
    check('unknown name -> None', pc.resolve_destination({}, 'shelf') is None)
    m = {'destinations': {'box': [0.14, -0.14], 'Bin': [0.15, 0.10]}}
    check('map overrides builtin', pc.resolve_destination(m, 'box') == (0.14, -0.14))
    check('map adds new names', pc.resolve_destination(m, 'bin') == (0.15, 0.10))
    bad = {'destinations': {'box': 5, 'short': [0.1], 'ok': [0.16, 0.02],
                            'words': ['a', 'b']}}
    try:
        check('malformed entries skipped, good one kept',
              pc.resolve_destination(bad, 'ok') == (0.16, 0.02))
        check('malformed name falls back to builtin table',
              pc.resolve_destination(bad, 'box') == (0.13, -0.12))
    except Exception as exc:                          # noqa: BLE001
        check('malformed destinations must not raise', False, str(exc))


def main():
    for test in (test_homography_fit, test_outlier_rejection_bounded,
                 test_sector_rotation_convention, test_height_rules,
                 test_wrist_delta, test_resolve_destination):
        test()
    print()
    if FAILURES:
        print(f'{len(FAILURES)} FAILURE(S): {FAILURES}')
        sys.exit(1)
    print('ALL PLANAR MATH TESTS PASSED')


if __name__ == '__main__':
    main()
