#!/usr/bin/env python3
"""Auto-guided flat-field (illumination) calibration from a white surface.

Point the camera at a uniform white banner/sheet ("la lona"). The tool checks
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
        return False, "muy oscuro: acerca o sube luz"
    if mean > 240:
        return False, "muy brillante: baja la exposicion"
    clipped = float((luma > 250).mean())
    if clipped > 0.005:
        return False, "brillo especular: cambia el angulo"
    # Gross shadow: a heavily blurred region far darker than the mean.
    small = cv2.resize(luma, (32, 24)).astype(np.float32)
    if small.min() < 0.55 * small.mean():
        return False, "sombra detectada: ilumina parejo"
    return True, "ok"


def residual_report(avg: np.ndarray, corrected: np.ndarray) -> list[str]:
    raw_std = avg.reshape(-1, 3).std(axis=0)
    cor_std = corrected.reshape(-1, 3).std(axis=0)
    raw_mean = avg.reshape(-1, 3).mean(axis=0)
    cor_mean = corrected.reshape(-1, 3).mean(axis=0)
    return [
        f"std BGR antes : {raw_std[0]:.1f} {raw_std[1]:.1f} {raw_std[2]:.1f}",
        f"std BGR despues: {cor_std[0]:.1f} {cor_std[1]:.1f} {cor_std[2]:.1f}",
        f"media BGR antes : {raw_mean[0]:.1f} {raw_mean[1]:.1f} {raw_mean[2]:.1f}",
        f"media BGR despues: {cor_mean[0]:.1f} {cor_mean[1]:.1f} {cor_mean[2]:.1f}",
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
        print(f"[info] auto-start: juntando {args.frames} frames buenos de la lona blanca. Ctrl+C para abortar.")
    else:
        print("[info] acomoda la lona blanca llenando el cuadro.")
        print("[info] presiona Enter para iniciar; luego espera "
              f"{args.settle_seconds:.1f}s y se capturan {args.frames} frames buenos.")
        print("[info] Ctrl+C aborta sin guardar.")
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
                print(f"[info] inicio armado; retira manos/sombras ({args.settle_seconds:.1f}s)...")

            good, reason = assess(frame)
            if not armed:
                status, color = "listo? Enter para iniciar | " + reason, (0, 255, 0) if good else (0, 200, 255)
            elif time.time() < start_at:
                remaining = max(0.0, start_at - time.time())
                status, color = f"inicia en {remaining:.1f}s: retira manos/sombras", (0, 200, 255)
            elif good:
                buffer.append(frame.astype(np.float32))
                status, color = f"capturando {len(buffer)}/{args.frames}", (0, 255, 0)
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
        print("   (std despues mas baja y medias BGR mas parejas = tinte rojizo removido)")
        # Leave a side-by-side artifact for headless verification.
        cv2.imwrite(str(REPO_DIR / "config" / "illumination_preview.jpg"),
                    np.hstack([avg, corrected]))
        saved = True
    except KeyboardInterrupt:
        print("\n[abort] sin guardar")
    finally:
        cap.release()
        preview.close()
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())
