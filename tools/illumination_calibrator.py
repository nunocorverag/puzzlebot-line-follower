#!/usr/bin/env python3
"""Capture a flat-field illumination correction from a white wall/sheet.

This tool is safe to run on the Jetson. It only reads the camera and writes an
npz file with a smoothed per-channel gain map.

Keys:
  c        capture/save current flat-field
  q / ESC  quit
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from line_vision_calibrator import build_gstreamer_pipeline, load_camera_params


REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_OUTPUT = REPO_DIR / "config" / "illumination_flatfield.npz"


def open_capture(args: argparse.Namespace) -> cv2.VideoCapture:
    if args.gstreamer:
        cap = cv2.VideoCapture(
            build_gstreamer_pipeline(args.width, args.height, args.fps),
            cv2.CAP_GSTREAMER,
        )
        if cap.isOpened():
            return cap
        cap.release()
        print("[warn] GStreamer camera failed, trying camera index")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("could not open camera")
    return cap


def build_gain_map(frame: np.ndarray, blur: int) -> np.ndarray:
    blur = max(3, int(blur) | 1)
    reference = frame.astype(np.float32)
    reference = cv2.GaussianBlur(reference, (blur, blur), 0)
    channel_means = reference.reshape(-1, 3).mean(axis=0)
    gain = channel_means.reshape(1, 1, 3) / np.maximum(reference, 1.0)
    return np.clip(gain, 0.25, 4.0).astype(np.float32)


def apply_gain(frame: np.ndarray, gain: np.ndarray) -> np.ndarray:
    corrected = frame.astype(np.float32) * gain
    return np.clip(corrected, 0, 255).astype(np.uint8)


def draw_stats(frame: np.ndarray, corrected: np.ndarray) -> np.ndarray:
    preview = np.hstack([frame, corrected])
    raw_std = frame.reshape(-1, 3).std(axis=0)
    corrected_std = corrected.reshape(-1, 3).std(axis=0)
    lines = [
        "left: raw | right: corrected",
        f"raw std BGR: {raw_std[0]:.1f} {raw_std[1]:.1f} {raw_std[2]:.1f}",
        f"corr std BGR: {corrected_std[0]:.1f} {corrected_std[1]:.1f} {corrected_std[2]:.1f}",
        "keys: c capture/save | q quit",
    ]
    y = 26
    for line in lines:
        cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        y += 26
    return preview


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default=0)
    parser.add_argument("--gstreamer", action="store_true", default=False)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    parser.add_argument("--no-undistort", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--blur", type=int, default=151)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    undistort_enabled = not args.no_undistort and camera_matrix is not None and dist_coeffs is not None
    cap = open_capture(args)

    window = "Illumination Calibrator"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width * 2, args.height)

    last_gain = None
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[warn] frame read failed")
            break
        if undistort_enabled:
            frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gain = build_gain_map(frame, args.blur)
        corrected = apply_gain(frame, gain)
        last_gain = gain
        cv2.imshow(window, draw_stats(frame, corrected))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("c") and last_gain is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                str(args.output),
                gain=last_gain,
                width=frame.shape[1],
                height=frame.shape[0],
                blur=args.blur,
            )
            print(f"[save] {args.output}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
