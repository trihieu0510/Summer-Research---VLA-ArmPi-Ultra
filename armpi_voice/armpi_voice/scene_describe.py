#!/usr/bin/env python3
"""
scene_describe.py
=================
The "what do you see?" skill: name the colored blocks on the mat (HSV — always
available, reuses planar_common's proven detector) plus, when ultralytics is
installed, general objects via YOLOv8.

YOLO is OPTIONAL and lazy-loaded on first use: if the import or the model load
fails, the skill silently degrades to blocks-only — a missing pip package must
never break the chat. On the CPU-only Pi run yolov8n at imgsz=320 (~1–2 s per
frame — fine for a conversational look, do NOT put it in a control loop).

Left/right wording is IMAGE-frame ("on the left" = left half of the camera
image). Whether that matches the operator's left is a 1-minute live check —
see Documents/lab_protocols.md.

Runs ON THE PI, inside the Hiwonder ROS 2 Humble Docker container.
"""

import os

# pyrefly: ignore [missing-import]
import cv2

_YOLO = None          # loaded once per process
_YOLO_FAILED = False  # remember a failed load; don't retry every question

# Annotation colors (BGR) for the debug image.
_BLOCK_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


def _load_yolo(model_path, log_warn=None):
    """Return a cached YOLO model, or None if ultralytics/model unavailable."""
    global _YOLO, _YOLO_FAILED
    if _YOLO is not None or _YOLO_FAILED:
        return _YOLO
    try:
        # pyrefly: ignore [missing-import]
        from ultralytics import YOLO
        path = os.path.expanduser(model_path) if model_path else 'yolov8n.pt'
        if model_path and not os.path.exists(path):
            path = 'yolov8n.pt'      # let ultralytics fetch/cache the default
        _YOLO = YOLO(path)
    except Exception as exc:                                # noqa: BLE001
        _YOLO_FAILED = True
        if log_warn:
            log_warn(f'YOLO unavailable ({exc}) — describing blocks only.')
    return _YOLO


def _thirds(u, width):
    if u < width / 3:
        return 'on the left'
    if u < 2 * width / 3:
        return 'in the middle'
    return 'on the right'


def _plural(n, name):
    words = {2: 'two', 3: 'three', 4: 'four', 5: 'five'}
    if n == 1:
        article = 'an' if name[0] in 'aeiou' else 'a'
        return f'{article} {name}'
    return f'{words.get(n, str(n))} {name}s'


def describe_frame(bgr, roi=None, yolo_model='', conf=0.4, log_warn=None):
    """Look at one frame; return (sentence, annotated_bgr).

    Blocks are searched inside `roi` (the calibrated mat area) so background
    reds can't win — same rule as picking. YOLO sees the WHOLE frame: general
    objects beyond the mat are exactly what it adds over the HSV detector.
    """
    from . import planar_common as pc

    annotated = bgr.copy()
    h, w = bgr.shape[:2]

    # -- colored blocks (mat only) --
    block_phrases = []
    for color in pc.COLOR_RANGES:
        det = pc.detect_block(bgr, color, roi=roi)
        if det is None:
            continue
        u, v = int(det[0]), int(det[1])
        block_phrases.append(f'a {color} block {_thirds(u, w)}')
        cv2.circle(annotated, (u, v), 12, _BLOCK_BGR.get(color, (255, 255, 255)), 3)
        cv2.putText(annotated, color, (u + 15, v), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, _BLOCK_BGR.get(color, (255, 255, 255)), 2)

    # -- general objects (whole frame, optional) --
    object_counts = {}
    model = _load_yolo(yolo_model, log_warn)
    if model is not None:
        try:
            results = model.predict(bgr, imgsz=320, conf=conf, verbose=False)
            for box in results[0].boxes:
                name = results[0].names[int(box.cls[0])]
                object_counts[name] = object_counts.get(name, 0) + 1
                x0, y0, x1, y1 = (int(v) for v in box.xyxy[0])
                cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 255), 2)
                cv2.putText(annotated, f'{name} {float(box.conf[0]):.2f}',
                            (x0, max(15, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)
        except Exception as exc:                            # noqa: BLE001
            if log_warn:
                log_warn(f'YOLO inference failed ({exc}) — blocks only.')

    # -- one natural sentence --
    parts = []
    if block_phrases:
        parts.append('On the mat I can see ' + _join(block_phrases) + '.')
    else:
        parts.append("I don't see any colored blocks on the mat right now.")
    if object_counts:
        names = [_plural(n, name) for name, n in
                 sorted(object_counts.items(), key=lambda kv: -kv[1])]
        parts.append('Around me I also spot ' + _join(names) + '.')
    return ' '.join(parts), annotated


def _join(items):
    """['a', 'b', 'c'] -> 'a, b and c' (natural-speech list)."""
    if len(items) == 1:
        return items[0]
    return ', '.join(items[:-1]) + ' and ' + items[-1]
