#!/usr/bin/env python3
"""Record camera frames with full calibration applied, ready for YOLO training.

Reads from the ROS topic /video_source/raw (published by camera_jetson.launch.py).

Applies in order:
  1. Camera undistortion  (config/camera_params.npz)
  2. Illumination flat-field correction  (config/illumination_flatfield.npz)

Saved images have NO overlays — clean pixels for YOLO training.

Usage:
  python3 tools/recorder.py --headless --interval 0.5
  python3 tools/recorder.py --headless --interval 0.5 --duration 60
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from line_vision_calibrator import load_camera_params, load_illumination_gain, apply_illumination_gain


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    dtype = np.uint8
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}.get(msg.encoding, 3)
    frame = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ("rgba8", "bgra8"):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR if msg.encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    return frame.copy()

REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CAMERA_PARAMS     = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_ILLUMINATION_PARAMS = REPO_DIR / "config" / "illumination_flatfield.npz"
DEFAULT_OUTPUT_DIR        = REPO_DIR / "dataset"


# ---------------------------------------------------------------------------
# Saving
# ---------------------------------------------------------------------------

def next_index(folder: Path) -> int:
    existing = list(folder.glob("frame_*.jpg"))
    if not existing:
        return 0
    indices = []
    for p in existing:
        try:
            indices.append(int(p.stem.split("_")[1]))
        except (IndexError, ValueError):
            pass
    return max(indices) + 1 if indices else 0


def save_frame(frame: np.ndarray, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    idx = next_index(output_dir)
    out_path = output_dir / f"frame_{idx:05d}.jpg"
    cv2.imwrite(str(out_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return out_path


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class RecorderNode(Node):
    def __init__(self, args: argparse.Namespace,
                 camera_matrix, dist_coeffs, illumination_gain):
        super().__init__("recorder_node")
        self.args = args
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.illumination_gain = illumination_gain
        self.undistort = camera_matrix is not None and dist_coeffs is not None
        self.save_count = 0
        self.frame_count = 0
        self.last_save_time = 0.0
        self.start_time = time.time()
        self.recording = False  # start paused — press Enter to begin

        self.sub = self.create_subscription(
            Image, "/video_source/raw", self.image_callback, 10)

        if not args.headless:
            cv2.namedWindow("Recorder", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Recorder", 640, 480)

        print(f"[info] Saving to: {args.output_dir}  interval={args.interval}s", flush=True)
        print("[info] Presiona Enter para INICIAR/PAUSAR grabacion. Ctrl+C para salir.", flush=True)
        print("[PAUSADO] Listo — presiona Enter cuando quieras grabar.", flush=True)

    def toggle_recording(self):
        self.recording = not self.recording
        status = "GRABANDO" if self.recording else "PAUSADO"
        print(f"[{status}] frames guardados hasta ahora: {self.save_count}", flush=True)

    def image_callback(self, msg: Image):
        if self.args.duration > 0 and (time.time() - self.start_time) >= self.args.duration:
            rclpy.shutdown()
            return

        frame = imgmsg_to_cv2(msg)

        if self.undistort:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        frame = apply_illumination_gain(frame, self.illumination_gain)
        self.frame_count += 1

        if not self.args.headless:
            preview = frame.copy()
            status = "GRABANDO" if self.recording else "PAUSADO"
            color = (0, 255, 0) if self.recording else (0, 100, 255)
            cv2.putText(preview, f"{status}  saved:{self.save_count}  Enter=toggle  Ctrl+C=salir",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(preview, f"{status}  saved:{self.save_count}  Enter=toggle  Ctrl+C=salir",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            cv2.imshow("Recorder", preview)
            cv2.waitKey(1)

        if not self.recording:
            return

        now = time.time()
        if (now - self.last_save_time) >= self.args.interval:
            path = save_frame(frame, self.args.output_dir)
            self.save_count += 1
            self.last_save_time = now
            print(f"[save] {path}  (total: {self.save_count})", flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-params",        type=Path, default=DEFAULT_CAMERA_PARAMS)
    parser.add_argument("--illumination-params",  type=Path, default=DEFAULT_ILLUMINATION_PARAMS)
    parser.add_argument("--no-undistort",         action="store_true")
    parser.add_argument("--no-illumination-correction", action="store_true")
    parser.add_argument("--output-dir",           type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--interval",             type=float, default=0.5)
    parser.add_argument("--duration",             type=float, default=0.0,
                        help="stop after N seconds (0 = run until Ctrl+C)")
    # kept for backwards compat, ignored (camera comes from ROS topic now)
    parser.add_argument("--gstreamer",   action="store_true", default=False)
    parser.add_argument("--headless",    action="store_true", default=False)
    parser.add_argument("--camera",      type=int, default=0)
    parser.add_argument("--width",       type=int, default=640)
    parser.add_argument("--height",      type=int, default=480)
    parser.add_argument("--fps",         type=int, default=30)
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    if args.no_undistort:
        camera_matrix = dist_coeffs = None

    illumination_gain = None
    if not args.no_illumination_correction:
        illumination_gain = load_illumination_gain(args.illumination_params)

    rclpy.init()
    node = RecorderNode(args, camera_matrix, dist_coeffs, illumination_gain)

    def stdin_listener():
        while rclpy.ok():
            try:
                input()  # blocks until Enter
                node.toggle_recording()
            except EOFError:
                break

    t = threading.Thread(target=stdin_listener, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[done] saved {node.save_count} frames to {args.output_dir}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
