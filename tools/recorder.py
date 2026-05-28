#!/usr/bin/env python3
"""Record camera frames with full calibration applied, ready for YOLO training.

Applies in order:
  1. Camera undistortion  (config/camera_params.npz)
  2. Illumination flat-field correction  (config/illumination_flatfield.npz)

The images SAVED to disk have NO overlays — clean pixels for training.
The PREVIEW window shows a minimal HUD so you can see what is being captured.

Keys:
  s        save current frame
  r        toggle auto-capture (saves every --interval seconds)
  l        type a label name in terminal (organises saves into subfolders)
  q / ESC  quit
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from line_vision_calibrator import (
    build_gstreamer_pipeline,
    load_camera_params,
    load_illumination_gain,
    apply_illumination_gain,
)

REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_ILLUMINATION_PARAMS = REPO_DIR / "config" / "illumination_flatfield.npz"
DEFAULT_OUTPUT_DIR = REPO_DIR / "dataset"


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def open_capture(args: argparse.Namespace) -> cv2.VideoCapture:
    if args.gstreamer:
        cap = cv2.VideoCapture(
            build_gstreamer_pipeline(args.width, args.height, args.fps),
            cv2.CAP_GSTREAMER,
        )
        if cap.isOpened():
            return cap
        cap.release()
        print("[warn] GStreamer failed, trying /dev/video0")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("could not open camera")
    return cap


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


def save_frame(frame: np.ndarray, output_dir: Path, label: str) -> Path:
    subfolder = output_dir / (label if label else "unlabeled")
    subfolder.mkdir(parents=True, exist_ok=True)
    idx = next_index(subfolder)
    out_path = subfolder / f"frame_{idx:05d}.jpg"
    cv2.imwrite(str(out_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return out_path


# ---------------------------------------------------------------------------
# Preview HUD (drawn only on preview, never on saved image)
# ---------------------------------------------------------------------------

def draw_hud(
    frame: np.ndarray,
    label: str,
    auto_record: bool,
    frame_count: int,
    save_count: int,
    next_save_in: float,
) -> np.ndarray:
    preview = frame.copy()
    h, w = preview.shape[:2]

    mode = f"AUTO  next:{next_save_in:.1f}s" if auto_record else "MANUAL"
    color_mode = (0, 255, 0) if auto_record else (0, 200, 255)

    lines = [
        (f"label: {label or 'unlabeled'}", (0, 255, 255)),
        (f"mode:  {mode}", color_mode),
        (f"frame: {frame_count}   saved: {save_count}", (200, 200, 200)),
        ("s=save  r=auto  l=label  q=quit", (120, 120, 120)),
    ]

    y = 26
    for text, color in lines:
        cv2.putText(preview, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(preview, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        y += 24

    if auto_record:
        cv2.rectangle(preview, (0, 0), (w - 1, h - 1), (0, 200, 0), 3)

    return preview


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default=0, type=int)
    parser.add_argument("--gstreamer", action="store_true", default=False)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    parser.add_argument("--illumination-params", type=Path, default=DEFAULT_ILLUMINATION_PARAMS)
    parser.add_argument("--no-undistort", action="store_true")
    parser.add_argument("--no-illumination-correction", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between auto-captures (default 0.5)")
    parser.add_argument("--label", type=str, default="",
                        help="initial label subfolder name")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    undistort = not args.no_undistort and camera_matrix is not None and dist_coeffs is not None
    if not undistort:
        print("[warn] undistortion disabled or params missing")

    illumination_gain = None
    if not args.no_illumination_correction:
        illumination_gain = load_illumination_gain(args.illumination_params)
    if illumination_gain is None:
        print("[warn] illumination correction disabled or params missing")

    cap = open_capture(args)

    window = "Recorder — calibrated"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    label = args.label
    auto_record = False
    frame_count = 0
    save_count = 0
    last_save_time = 0.0

    print(f"[info] saving to: {args.output_dir}")
    print(f"[info] undistort={undistort}  illumination={'on' if illumination_gain is not None else 'off'}")
    print("[info] press 'l' then Enter in this terminal to set a new label")

    while True:
        ok, raw = cap.read()
        if not ok:
            print("[warn] frame read failed")
            break

        frame = raw.copy()
        if undistort:
            frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        frame = apply_illumination_gain(frame, illumination_gain)
        frame_count += 1

        now = time.time()
        next_save_in = max(0.0, args.interval - (now - last_save_time))

        if auto_record and (now - last_save_time) >= args.interval:
            path = save_frame(frame, args.output_dir, label)
            save_count += 1
            last_save_time = now
            print(f"[auto] {path}  (total: {save_count})")

        preview = draw_hud(frame, label, auto_record, frame_count, save_count, next_save_in)
        cv2.imshow(window, preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("s"):
            path = save_frame(frame, args.output_dir, label)
            save_count += 1
            print(f"[save] {path}  (total: {save_count})")
        elif key == ord("r"):
            auto_record = not auto_record
            last_save_time = time.time()
            print(f"[info] auto-record {'ON' if auto_record else 'OFF'}  interval={args.interval}s")
        elif key == ord("l"):
            new_label = input("label> ").strip()
            if new_label:
                label = new_label
                print(f"[info] label set to: {label}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"[done] saved {save_count} frames to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
