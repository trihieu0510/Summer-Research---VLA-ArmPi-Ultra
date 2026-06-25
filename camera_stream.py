#!/usr/bin/env python3
# pyrefly: ignore  # noqa  -- ROS packages only exist on the Pi, not on the laptop
"""
camera_stream.py — live MJPEG feed of the ArmPi Ultra camera for remote work.

Runs ON THE PI (inside the ROS 2 container). Subscribes to a camera image
topic, JPEG-encodes each frame, and serves it as an MJPEG stream over HTTP so
you can watch the robot's view from a browser while working remotely.

Uses only what's already in the container: rclpy, cv_bridge, cv2. No pip installs.

------------------------------------------------------------------------------
USAGE (on the Pi, in a sourced ROS 2 terminal — camera must be publishing):

    # 1. Bring the camera up first (separate terminal):
    ~/.stop_ros.sh
    ros2 launch peripherals depth_camera.launch.py

    # 2. Find the real RGB topic if unsure:
    ros2 topic list | grep -iE 'rgb|image'

    # 3. Start the stream (default topic shown; override with --topic):
    python3 camera_stream.py --topic /ascamera/camera_publisher/rgb0/image --port 8080

------------------------------------------------------------------------------
VIEWING REMOTELY (from your laptop / remote PC):

  Forward the Pi's port over your existing SSH connection, e.g.:

    ssh -L 8080:localhost:8080 <user>@192.168.137.<pi>

  Then open  http://localhost:8080  in any browser.
  (If you're already inside the PC that's LAN-connected to the Pi, you can also
   just browse to http://192.168.137.<pi>:8080 directly.)

Press Ctrl+C in the Pi terminal to stop.
"""

import argparse
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2  # pyrefly: ignore [missing-import]
import rclpy  # pyrefly: ignore [missing-import]
from rclpy.node import Node  # pyrefly: ignore [missing-import]
from rclpy.qos import qos_profile_sensor_data  # pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image  # pyrefly: ignore [missing-import]
from cv_bridge import CvBridge  # pyrefly: ignore [missing-import]


class FrameStore:
    """Thread-safe holder for the latest JPEG-encoded frame."""

    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None  # bytes of the most recent encoded frame
        self._count = 0

    def set(self, jpeg_bytes):
        with self._lock:
            self._jpeg = jpeg_bytes
            self._count += 1

    def get(self):
        with self._lock:
            return self._jpeg

    @property
    def count(self):
        with self._lock:
            return self._count


class CameraStreamNode(Node):
    """Subscribes to a camera image topic and pushes encoded frames to a store."""

    def __init__(self, topic, store, jpeg_quality):
        super().__init__("camera_stream")
        self.store = store
        self.bridge = CvBridge()
        self.jpeg_quality = jpeg_quality
        # Camera topics publish with SENSOR_DATA (BEST_EFFORT) QoS — using the
        # default reliable profile here would silently receive nothing.
        self.sub = self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {topic}; serving MJPEG.")

    def _on_image(self, msg):
        try:
            # Many depth-cam RGB topics are 'rgb8'; convert to BGR for cv2/JPEG.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # encoding mismatch, etc.
            self.get_logger().warn(f"cv_bridge convert failed: {exc}")
            return
        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if ok:
            self.store.set(buf.tobytes())


def make_handler(store):
    boundary = "frame"

    class MJPEGHandler(BaseHTTPRequestHandler):
        def log_message(self, *_args):  # silence per-request console spam
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._serve_index()
            elif self.path == "/stream":
                self._serve_stream()
            else:
                self.send_error(404)

        def _serve_index(self):
            body = (
                b"<!doctype html><html><head><title>ArmPi Ultra live feed</title>"
                b"<style>body{background:#111;margin:0;display:flex;"
                b"justify-content:center;align-items:center;height:100vh}"
                b"img{max-width:100%;max-height:100vh}</style></head>"
                b"<body><img src='/stream'></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_stream(self):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={boundary}",
            )
            self.end_headers()
            try:
                while True:
                    jpeg = store.get()
                    if jpeg is None:
                        # No frame yet — wait briefly rather than busy-spin.
                        threading.Event().wait(0.05)
                        continue
                    self.wfile.write(f"--{boundary}\r\n".encode())
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    threading.Event().wait(0.03)  # ~30 fps cap on the wire
            except (BrokenPipeError, ConnectionResetError):
                pass  # client closed the tab

    return MJPEGHandler


def main():
    parser = argparse.ArgumentParser(description="Live MJPEG feed of the ArmPi camera.")
    parser.add_argument(
        "--topic",
        default="/ascamera/camera_publisher/rgb0/image",
        help="ROS 2 image topic to stream (default: Aurora RGB).",
    )
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080).")
    parser.add_argument(
        "--quality", type=int, default=80, help="JPEG quality 1-100 (default 80)."
    )
    # Strip ROS args (e.g. when launched via ros2 run) before argparse sees them.
    args, _ = parser.parse_known_args()

    rclpy.init()
    store = FrameStore()
    node = CameraStreamNode(args.topic, store, args.quality)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(store))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(
        f"MJPEG server on http://0.0.0.0:{args.port}  "
        f"(forward with: ssh -L {args.port}:localhost:{args.port} <user>@<pi-ip>)"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
