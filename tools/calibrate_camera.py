#!/usr/bin/env python3
"""Compute camera intrinsics from captured checkerboard images.

Adapted from the TE3002B activity_2_07 calibrator, made robust and repo-aware:
  - detects only the expected inner-corner pattern (5x7 or its transpose),
  - refines corners with cornerSubPix,
  - runs cv2.calibrateCamera, then drops per-image reprojection outliers and
    recalibrates,
  - writes config/camera_params.npz with the keys the rest of the repo loads
    (camera_matrix, dist_coeffs) plus metadata,
  - saves undistorted_preview.jpg (no GUI, works headless over SSH).

  python3 tools/calibrate_camera.py
  python3 tools/calibrate_camera.py --pattern 5x7 --square-size-mm 25
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_IMAGES = REPO_DIR / "calibration_images"
DEFAULT_OUTPUT = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_PREVIEW = REPO_DIR / "config" / "undistorted_preview.jpg"



def parse_pattern(text: str) -> tuple[int, int]:
    cols, rows = (int(v) for v in text.lower().split("x"))
    return cols, rows


def load_image_paths(images_dir: Path) -> list[Path]:
    paths: list[Path] = []
    for pat in ("*.jpg", "*.jpeg", "*.png"):
        paths.extend(images_dir.glob(pat))
    return sorted(paths)


def make_object_points(pattern: tuple[int, int], square: float) -> np.ndarray:
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return objp


def detect(gray, candidates):
    """Return (pattern, refined_corners) for the first matching candidate."""
    for pattern in candidates:
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                return pattern, corners
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return pattern, corners
    return None, None


def per_image_errors(objpts, imgpts, rvecs, tvecs, K, dist) -> np.ndarray:
    errors = []
    for op, ip, rv, tv in zip(objpts, imgpts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        errors.append(cv2.norm(ip, proj, cv2.NORM_L2) / len(proj))
    return np.array(errors)


def calibrate(objpts, imgpts, image_size):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpts, imgpts, image_size, None, None)
    return rms, K, dist, rvecs, tvecs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images-dir", type=Path, default=DEFAULT_IMAGES)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--pattern", default="5x7")
    parser.add_argument("--square-size-mm", type=float, default=1.0,
                        help="square edge in mm (does not affect K; for metadata/scale)")
    args = parser.parse_args()

    primary = parse_pattern(args.pattern)
    candidates = [primary, (primary[1], primary[0])]

    paths = load_image_paths(args.images_dir)
    if not paths:
        print(f"[error] no images in {args.images_dir}")
        return 1
    print(f"[info] {len(paths)} images in {args.images_dir}")

    objpts: list[np.ndarray] = []
    imgpts: list[np.ndarray] = []
    used_paths: list[Path] = []
    image_size = None
    locked = None
    first_image = None

    for p in paths:
        img = cv2.imread(str(p))
        if img is None:
            print(f"  SKIP unreadable: {p.name}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cands = [locked] if locked else candidates
        pattern, corners = detect(gray, cands)
        if pattern is None:
            print(f"  SKIP no board: {p.name}")
            continue
        if locked is None:
            locked = pattern
            print(f"[info] locked pattern (inner corners): {pattern[0]}x{pattern[1]}")
        objpts.append(make_object_points(pattern, args.square_size_mm))
        imgpts.append(corners)
        used_paths.append(p)
        if first_image is None:
            first_image = img
        image_size = gray.shape[::-1]
        print(f"  OK {p.name}")

    if len(imgpts) < 5 or image_size is None:
        print(f"[error] need >=5 good detections, got {len(imgpts)}")
        return 1

    rms, K, dist, rvecs, tvecs = calibrate(objpts, imgpts, image_size)
    print(f"\n[pass 1] {len(imgpts)} images, RMS={rms:.4f}")

    # Drop per-image outliers (> 2x median error) and recalibrate once.
    errs = per_image_errors(objpts, imgpts, rvecs, tvecs, K, dist)
    thresh = max(2.0 * float(np.median(errs)), float(np.median(errs)) + 0.3)
    keep = errs <= thresh
    if keep.sum() >= 5 and keep.sum() < len(errs):
        dropped = [used_paths[i].name for i in range(len(errs)) if not keep[i]]
        objpts = [o for o, k in zip(objpts, keep) if k]
        imgpts = [c for c, k in zip(imgpts, keep) if k]
        rms, K, dist, rvecs, tvecs = calibrate(objpts, imgpts, image_size)
        print(f"[pass 2] dropped {len(dropped)} outlier(s) {dropped}; "
              f"{len(imgpts)} images, RMS={rms:.4f}")

    print("\n--- Results ---")
    print("Camera matrix K:\n", K)
    print("dist coeffs:", dist.ravel())
    print(f"RMS reprojection error: {rms:.4f} px  (640x480)")
    verdict = ("EXCELLENT" if rms < 0.5 else "OK" if rms < 1.0
               else "HIGH - recapture with more variety")
    print(f"Verdict: {verdict}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(args.output),
        camera_matrix=K,
        dist_coeffs=dist,
        rms_error=rms,
        image_size=np.array(image_size),
        pattern=np.array(locked),
        square_size_mm=args.square_size_mm,
        num_images=len(imgpts),
    )
    print(f"\n[save] {args.output}")

    if first_image is not None:
        undist = cv2.undistort(first_image, K, dist)
        side = np.hstack((first_image, undist))
        cv2.imwrite(str(DEFAULT_PREVIEW), side)
        print(f"[save] {DEFAULT_PREVIEW} (left: original | right: undistorted)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
