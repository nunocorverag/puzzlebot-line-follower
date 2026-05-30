#!/usr/bin/env python3
"""Teleop + recorder with preview.

Controls:
  W  forward       S  back
  A  left          D  right
  space  stop
  R  toggle auto-save (saves every second)
  F  manual save (one frame)
  Q  quit
"""
import os
import sys
import threading

if not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":0"
_ld = os.environ.get("LD_LIBRARY_PATH", "")
_clean = ":".join(p for p in _ld.split(":") if "/ros" not in p and "/humble" not in p)
if _clean:
    os.environ["LD_LIBRARY_PATH"] = _clean
import time
import tty
import termios
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

sys.path.insert(0, str(Path(__file__).resolve().parent))
from line_vision_calibrator import (
    build_gstreamer_pipeline,
    load_camera_params,
    load_illumination_gain,
    apply_illumination_gain,
)

REPO_DIR = Path(__file__).resolve().parents[1]


class TeleopRecorder(Node):

    def __init__(self):
        super().__init__("teleop_recorder")

        # Calibration
        self.camera_matrix, self.dist_coeffs = load_camera_params(
            REPO_DIR / "config" / "camera_params.npz"
        )
        self.illumination_gain = load_illumination_gain(
            REPO_DIR / "config" / "illumination_flatfield.npz"
        )

        # Direct GStreamer camera
        self.cap = cv2.VideoCapture(
            build_gstreamer_pipeline(1280, 720, 30), cv2.CAP_GSTREAMER
        )
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            raise RuntimeError("Camera failed")

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Save directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = Path.home() / "teleop_data" / timestamp / "screenshots"
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # State
        self.current_key = None
        self.running = True
        self.auto_save = False
        self.save_count = 0
        self.last_save_time = 0.0
        self.linear_speed = 0.10
        self.angular_speed = 0.7

        # Preview window
        cv2.namedWindow("Teleop Recorder", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Teleop Recorder", 640, 480)

        # Keyboard thread
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        # 30Hz timer
        self.create_timer(1.0 / 30.0, self._loop)

        print(f"\nSaving to: {self.save_dir}")
        print("W/S=fwd/back  A/D=left/right  space=stop")
        print("R=auto-save toggle  F=save frame  Q=quit\n")

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running:
                ch = sys.stdin.read(1).lower()
                if ch == "q":
                    self.running = False
                    break
                self.current_key = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _save_frame(self, frame):
        path = self.save_dir / f"img_{self.save_count:06d}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.save_count += 1
        print(f"[save] {path.name}  (total: {self.save_count})", flush=True)

    def _loop(self):
        if not self.running:
            self._shutdown()
            return

        ok, frame = self.cap.read()
        if not ok:
            return

        # Scale to 640x480 to match calibration
        frame = cv2.resize(frame, (640, 480))

        # Apply calibration
        if self.camera_matrix is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        frame = apply_illumination_gain(frame, self.illumination_gain)

        # Save
        now = time.time()
        key = self.current_key
        self.current_key = None

        if key == "r":
            self.auto_save = not self.auto_save
            print(f"[AUTO-SAVE {'ON' if self.auto_save else 'OFF'}]", flush=True)

        if key == "f":
            self._save_frame(frame)

        if self.auto_save and (now - self.last_save_time) >= 1.0:
            self._save_frame(frame)
            self.last_save_time = now

        # cmd_vel
        cmd = Twist()
        if key == "w":
            cmd.linear.x = self.linear_speed
        elif key == "s":
            cmd.linear.x = -self.linear_speed
        elif key == "a":
            cmd.angular.z = self.angular_speed
        elif key == "d":
            cmd.angular.z = -self.angular_speed
        self.cmd_pub.publish(cmd)

        # Preview
        mode = "AUTO-SAVE ON" if self.auto_save else "manual"
        color = (0, 255, 0) if self.auto_save else (0, 200, 255)
        preview = frame.copy()
        cv2.putText(preview, f"{mode} | saved:{self.save_count} | R=auto F=frame Q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(preview, f"{mode} | saved:{self.save_count} | R=auto F=frame Q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1)
        if self.auto_save:
            cv2.rectangle(preview, (0, 0), (preview.shape[1]-1, preview.shape[0]-1), (0, 200, 0), 4)
        cv2.imshow("Teleop Recorder", preview)
        cv2.waitKey(1)

    def _shutdown(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        self.cap.release()
        cv2.destroyAllWindows()
        print(f"\n[done] {self.save_count} images in {self.save_dir}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TeleopRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()


if __name__ == "__main__":
    main()
