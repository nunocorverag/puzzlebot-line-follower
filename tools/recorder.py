#!/usr/bin/env python3
"""Record camera frames with full calibration applied, ready for YOLO training.

Opens the CSI camera directly (GStreamer) — no separate camera node needed.

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

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.camera import (  # noqa: E402
    load_camera_params,
    load_illumination_gain,
    open_csi_capture,
    preprocess_frame,
)
from puzzlebot_ros.perception.stream import Preview  # noqa: E402
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
        # Start paused (press Enter to begin) unless --start-recording was given,
        # which is what the WASD+record flow uses since it runs detached (no tty).
        self.recording = bool(getattr(args, "start_recording", False))

        # downscale=True so nvvidconv delivers width x height (640x480) in HW,
        # matching camera_params + the H264 encoder. Without it the CSI streams
        # native 1280x720 and the encoder spams gst_buffer_resize_range errors.
        self.cap = open_csi_capture(width=args.width, height=args.height, fps=args.fps,
                                    downscale=True, log=self.get_logger().info)
        if self.cap is None:
            raise RuntimeError("camera unavailable")

        mode = "none" if args.headless else None
        self.preview = (Preview("Recorder", mode="none", log=self.get_logger().info)
                        if mode == "none"
                        else Preview.from_env("Recorder", fps=args.fps, log=self.get_logger().info))

        self.create_timer(1.0 / max(1, args.fps), self.tick)

        print(f"[info] Saving to: {args.output_dir}  interval={args.interval}s", flush=True)
        if self.recording:
            print("[RECORDING] auto-started. Ctrl+C to quit.", flush=True)
        else:
            print("[info] Press Enter to START/PAUSE recording. Ctrl+C to quit.", flush=True)
            print("[PAUSED] Ready - press Enter when you want to record.", flush=True)

    def toggle_recording(self):
        self.recording = not self.recording
        status = "RECORDING" if self.recording else "PAUSED"
        print(f"[{status}] frames saved so far: {self.save_count}", flush=True)

    def tick(self):
        if self.args.duration > 0 and (time.time() - self.start_time) >= self.args.duration:
            rclpy.shutdown()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        frame = preprocess_frame(
            frame,
            camera_matrix=self.camera_matrix if self.undistort else None,
            dist_coeffs=self.dist_coeffs if self.undistort else None,
            gain=self.illumination_gain,
        )
        self.frame_count += 1

        # Preview gets the status overlay; saved frames stay clean for training.
        preview = frame.copy()
        status = "RECORDING" if self.recording else "PAUSED"
        color = (0, 255, 0) if self.recording else (0, 100, 255)
        cv2.putText(preview, f"{status}  saved:{self.save_count}  Enter=toggle  Ctrl+C=quit",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(preview, f"{status}  saved:{self.save_count}  Enter=toggle  Ctrl+C=quit",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        self.preview.show(preview)

        if not self.recording:
            return

        now = time.time()
        if (now - self.last_save_time) >= self.args.interval:
            path = save_frame(frame, self.args.output_dir)
            self.save_count += 1
            self.last_save_time = now
            print(f"[save] {path}  (total: {self.save_count})", flush=True)

    def destroy_node(self):
        self.preview.close()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


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
    parser.add_argument("--start-recording",      action="store_true",
                        help="begin recording immediately (no Enter; for detached runs)")
    parser.add_argument("--duration",             type=float, default=0.0,
                        help="stop after N seconds (0 = run until Ctrl+C)")
    # kept for backwards compat, ignored (camera is opened directly via GStreamer)
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
