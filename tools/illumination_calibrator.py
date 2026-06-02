#!/usr/bin/env python3
"""Auto-guided flat-field (illumination) calibration from a white surface.

Point the camera at a uniform white banner/sheet. The tool checks
each frame for good exposure, no specular hotspots and no gross shadows, and
when it has enough steady good frames it AVERAGES them (noise down), builds a
per-channel gain map and saves it. The gain removes the reddish color cast and
the vignetting so downstream masks are stable.

Run this AFTER the checkerboard calibration so frames are undistorted first.
Preview streams over the unified Preview (H264 by default).

  python3 tools/illumination_calibrator.py
  python3 tools/illumination_calibrator.py --frames 25 --no-undistort

By default it waits for Enter before collecting frames, so you can position the
white surface and remove your hand/shadow first. Use --auto-start for old behavior.
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.camera import load_camera_params, open_csi_capture  # noqa: E402
from puzzlebot_ros.perception.stream import Preview  # noqa: E402

DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_OUTPUT = REPO_DIR / "config" / "illumination_flatfield.npz"


def build_gain_map(frame: np.ndarray, blur: int) -> np.ndarray:
    blur = max(3, int(blur) | 1)
    reference = cv2.GaussianBlur(frame.astype(np.float32), (blur, blur), 0)
    channel_means = reference.reshape(-1, 3).mean(axis=0)
    gain = channel_means.reshape(1, 1, 3) / np.maximum(reference, 1.0)
    return np.clip(gain, 0.25, 4.0).astype(np.float32)


def apply_gain(frame: np.ndarray, gain: np.ndarray) -> np.ndarray:
    return np.clip(frame.astype(np.float32) * gain, 0, 255).astype(np.uint8)


def assess(frame: np.ndarray) -> tuple[bool, str]:
    """Quality gate for a single white-surface frame."""
    luma = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean = float(luma.mean())
    if mean < 70:
        return False, "too dark: move closer or add light"
    if mean > 240:
        return False, "too bright: lower the exposure"
    clipped = float((luma > 250).mean())
    if clipped > 0.005:
        return False, "specular highlight: change the angle"
    # Gross shadow: a heavily blurred region far darker than the mean.
    small = cv2.resize(luma, (32, 24)).astype(np.float32)
    if small.min() < 0.55 * small.mean():
        return False, "shadow detected: light it evenly"
    return True, "ok"


def residual_report(avg: np.ndarray, corrected: np.ndarray) -> list[str]:
    raw_std = avg.reshape(-1, 3).std(axis=0)
    cor_std = corrected.reshape(-1, 3).std(axis=0)
    raw_mean = avg.reshape(-1, 3).mean(axis=0)
    cor_mean = corrected.reshape(-1, 3).mean(axis=0)
    return [
        f"std BGR before: {raw_std[0]:.1f} {raw_std[1]:.1f} {raw_std[2]:.1f}",
        f"std BGR after : {cor_std[0]:.1f} {cor_std[1]:.1f} {cor_std[2]:.1f}",
        f"mean BGR before: {raw_mean[0]:.1f} {raw_mean[1]:.1f} {raw_mean[2]:.1f}",
        f"mean BGR after : {cor_mean[0]:.1f} {cor_mean[1]:.1f} {cor_mean[2]:.1f}",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    parser.add_argument("--no-undistort", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--frames", type=int, default=25, help="good frames to average")
    parser.add_argument("--blur", type=int, default=151)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--auto-start", action="store_true",
                        help="start collecting immediately (default waits for Enter)")
    parser.add_argument("--settle-seconds", type=float, default=3.0,
                        help="delay after pressing Enter, so hands/shadows leave the frame")
    # kept for backwards compat (camera is always direct GStreamer now)
    parser.add_argument("--gstreamer", action="store_true", default=False)
    parser.add_argument("--camera", default=0)
    args = parser.parse_args()

    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    undistort = not args.no_undistort and camera_matrix is not None and dist_coeffs is not None
    if not undistort:
        print("[warn] sin undistort (faltan camera_params o --no-undistort).")

    cap = open_csi_capture(width=args.width, height=args.height, fps=args.fps, downscale=True)
    if cap is None:
        print("[error] camera unavailable")
        return 1
    preview = Preview.from_env("Illumination Calibrator", fps=args.fps)

    buffer: list[np.ndarray] = []
    armed = bool(args.auto_start)
    start_at = 0.0
    if armed:
        print(f"[info] auto-start: collecting {args.frames} good frames of the white banner. Ctrl+C to abort.")
    else:
        print("[info] position the white banner so it fills the frame.")
        print("[info] press Enter to start; then wait "
              f"{args.settle_seconds:.1f}s and {args.frames} good frames are captured.")
        print("[info] Ctrl+C aborts without saving.")
    saved = False
    try:
        while len(buffer) < args.frames:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            if undistort:
                frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

            if not armed and sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                armed = True
                start_at = time.time() + max(0.0, args.settle_seconds)
                print(f"[info] start armed; remove hands/shadows ({args.settle_seconds:.1f}s)...")

            good, reason = assess(frame)
            if not armed:
                status, color = "ready? Enter to start | " + reason, (0, 255, 0) if good else (0, 200, 255)
            elif time.time() < start_at:
                remaining = max(0.0, start_at - time.time())
                status, color = f"starting in {remaining:.1f}s: remove hands/shadows", (0, 200, 255)
            elif good:
                buffer.append(frame.astype(np.float32))
                status, color = f"capturing {len(buffer)}/{args.frames}", (0, 255, 0)
            else:
                status, color = reason, (0, 0, 255)

            disp = frame.copy()
            parts = status.split(" | ", 1)
            for i, line in enumerate(parts):
                y = 26 + i * 25
                cv2.putText(disp, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4)
                cv2.putText(disp, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2)
            preview.show(disp)

        avg = np.clip(np.mean(buffer, axis=0), 0, 255).astype(np.uint8)
        gain = build_gain_map(avg, args.blur)
        corrected = apply_gain(avg, gain)

        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            str(args.output),
            gain=gain,
            width=avg.shape[1],
            height=avg.shape[0],
            blur=args.blur,
            frames_averaged=len(buffer),
        )
        print(f"\n[save] {args.output}")
        for line in residual_report(avg, corrected):
            print("   ", line)
        print("   (lower std afterwards and more even BGR means = reddish tint removed)")
        # Leave a side-by-side artifact for headless verification.
        cv2.imwrite(str(REPO_DIR / "config" / "illumination_preview.jpg"),
                    np.hstack([avg, corrected]))
        saved = True
    except KeyboardInterrupt:
        print("\n[abort] not saved")
    finally:
        cap.release()
        preview.close()
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())
