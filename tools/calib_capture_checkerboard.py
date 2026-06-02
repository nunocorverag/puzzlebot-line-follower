#!/usr/bin/env python3
"""Auto-guided checkerboard capture for camera (intrinsics) calibration.

Opens the CSI camera directly and streams an annotated preview over the unified
``Preview`` (H264 by default — watch it on the laptop). It detects the board
live and **auto-captures** only frames that are sharp, steady and add NEW pose
coverage (distance x perspective pose), telling you what is still missing.

Frames are saved CLEAN (no overlay) at the SAME resolution the runtime uses
(640x480), so the resulting intrinsics match line_follower / sign_detector.

  python3 tools/calib_capture_checkerboard.py
  python3 tools/calib_capture_checkerboard.py --pattern 5x7 --target 30

Then compute the calibration with tools/calibrate_camera.py.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.camera import open_csi_capture  # noqa: E402
from puzzlebot_ros.perception.stream import Preview  # noqa: E402

DEFAULT_OUTPUT = REPO_DIR / "calibration_images"

# INNER-corner pattern comes from --pattern (default 5x7). We only accept that
# board in its two orientations (5x7 and its 7x5 transpose) — never other sizes:
# a coarse live detector could otherwise lock onto a WRONG size and ruin the set.

# Variety we encourage (advisory; not a hard gate that can become unreachable).
POSE_BUCKETS = ("front", "yaw_left", "yaw_right", "pitch_up", "pitch_down", "roll")
MIN_POSES = 4
DIST_BUCKETS = ("near", "mid", "far")

SHARPNESS_MIN = 60.0          # variance of Laplacian on the board ROI
STABLE_PX = 1.5               # max mean centroid motion (px) to count as steady
STABLE_FRAMES = 3             # consecutive steady detect-cycles required
MIN_CAPTURE_GAP = 0.8         # seconds between captures
SPREAD_MIN_PX = 30.0          # a new pose must differ from previous saves by this
                              # much (centroid) OR add a new tilt/distance bucket,
                              # so we never bank near-duplicate frames.
DETECT_SCALE = 0.5            # detect on a half-res image (4x cheaper) for speed
DETECT_EVERY = 2              # detect every N frames; stream every frame (fluid)
ANGLE_MIN_DEG = 16.0          # target off-axis angle for useful perspective views
ROLL_MIN_DEG = 20.0           # in-plane rotation threshold


def parse_pattern(text: str) -> tuple[int, int]:
    cols, rows = (int(v) for v in text.lower().split("x"))
    return cols, rows


def detect_board(gray, pattern):
    """Fast LIVE detection (for gating/coverage only — NOT subpixel-precise).

    Runs the cheap classic detector with FAST_CHECK on a downscaled image so the
    preview stays fluid on the Jetson CPU. Precision does not matter here: the
    SAVED images are clean full-res and tools/calibrate_camera.py re-detects them
    with the accurate SB + cornerSubPix path. Returns corners in FULL-res coords.
    """
    small = cv2.resize(gray, None, fx=DETECT_SCALE, fy=DETECT_SCALE,
                       interpolation=cv2.INTER_AREA)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(small, pattern, flags)
    if not found:
        return None
    return (corners / DETECT_SCALE).astype(np.float32)


@dataclass
class BoardPose:
    centroid: np.ndarray
    dist: str
    buckets: set[str]
    area_frac: float
    pitch_deg: float
    yaw_deg: float
    roll_deg: float


def board_orientation_deg(_rvec: np.ndarray, corners: np.ndarray, pattern: tuple[int, int]) -> tuple[float, float, float]:
    """Return intuitive live guidance angles from the detected corner grid.

    For guidance, geometric image cues are more controllable than Euler angles
    from solvePnP with a guessed camera matrix. A fronto-parallel board has:
      - left and right sides with similar apparent length -> yaw near 0,
      - top and bottom sides with similar apparent length -> pitch near 0,
      - grid rows/columns close to horizontal/vertical -> roll2d near 0.

    These are not metric calibration angles; they are stable operator feedback.
    The actual calibration still uses full corner reprojection offline.
    """
    cols, rows = pattern
    grid = corners.reshape(rows, cols, 2).astype(np.float32)
    tl, tr = grid[0, 0], grid[0, -1]
    bl, br = grid[-1, 0], grid[-1, -1]

    top = float(np.linalg.norm(tr - tl))
    bottom = float(np.linalg.norm(br - bl))
    left = float(np.linalg.norm(bl - tl))
    right = float(np.linalg.norm(br - tr))

    # Positive yaw means the right side appears larger/closer; positive pitch
    # means the bottom appears larger/closer. The scale factor makes the value
    # read like a useful degree-ish cue without pretending to be exact metrology.
    yaw = 2.0 * np.degrees(np.arctan2(right - left, max(right + left, 1e-6)))
    pitch = 2.0 * np.degrees(np.arctan2(bottom - top, max(bottom + top, 1e-6)))

    row_vec = tr - tl
    col_vec = bl - tl
    row_angle = np.degrees(np.arctan2(row_vec[1], row_vec[0]))
    col_angle = np.degrees(np.arctan2(col_vec[1], col_vec[0])) - 90.0
    roll = float((row_angle + col_angle) / 2.0)
    while roll > 90:
        roll -= 180
    while roll < -90:
        roll += 180
    return float(pitch), float(yaw), roll


def board_metrics(corners, pattern, frame_shape):
    """Centroid, distance bucket and perspective pose buckets for a detection."""
    h, w = frame_shape[:2]
    pts = corners.reshape(-1, 2)
    centroid = np.array([pts[:, 0].mean(), pts[:, 1].mean()])

    area = cv2.contourArea(cv2.convexHull(pts.astype(np.float32)))
    frac = area / float(w * h)
    dist = "near" if frac > 0.30 else "mid" if frac > 0.12 else "far"

    # solvePnP is used only to verify a coherent planar pose; live angles come
    # from image geometry below because that is more intuitive to control.
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    guess_k = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], np.float32)
    buckets: set[str] = set()
    pitch = yaw = roll = 0.0
    ok, rvec, _ = cv2.solvePnP(objp, corners, guess_k, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok:
        pitch, yaw, roll = board_orientation_deg(rvec, corners, pattern)

    if abs(pitch) < 10 and abs(yaw) < 10:
        buckets.add("front")
    if yaw <= -ANGLE_MIN_DEG:
        buckets.add("yaw_left")
    elif yaw >= ANGLE_MIN_DEG:
        buckets.add("yaw_right")
    if pitch <= -ANGLE_MIN_DEG:
        buckets.add("pitch_up")
    elif pitch >= ANGLE_MIN_DEG:
        buckets.add("pitch_down")
    if abs(roll) >= ROLL_MIN_DEG:
        buckets.add("roll")
    if not buckets:
        buckets.add("transition")

    return BoardPose(centroid, dist, buckets, frac, pitch, yaw, roll)

def sharpness(gray, corners) -> float:
    pts = corners.reshape(-1, 2)
    x0, y0 = pts.min(axis=0).astype(int)
    x1, y1 = pts.max(axis=0).astype(int)
    x0, y0 = max(0, x0), max(0, y0)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return 0.0
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def next_index(folder: Path) -> int:
    existing = sorted(folder.glob("calib_*.jpg"))
    if not existing:
        return 0
    nums = [int(p.stem.split("_")[1]) for p in existing if p.stem.split("_")[1].isdigit()]
    return (max(nums) + 1) if nums else 0


def hint(dists, poses) -> str:
    """The single most useful thing to do next (advisory)."""
    if "far" not in dists:
        return "move the board farther away (small views cover the frame corners)"
    if "near" not in dists:
        return "move the board closer (so it fills more of the frame)"
    missing_pose = [p for p in POSE_BUCKETS if p not in poses]
    if missing_pose:
        return {
            "front": "front pitch/yaw between -10 and +10 deg",
            "yaw_left": f"front yaw <= -{ANGLE_MIN_DEG:.0f} deg (bring the left edge closer)",
            "yaw_right": f"front yaw >= +{ANGLE_MIN_DEG:.0f} deg (bring the right edge closer)",
            "pitch_up": f"front pitch <= -{ANGLE_MIN_DEG:.0f} deg (bring the top edge closer)",
            "pitch_down": f"front pitch >= +{ANGLE_MIN_DEG:.0f} deg (bring the bottom edge closer)",
            "roll": f"roll2d >= +/-{ROLL_MIN_DEG:.0f} deg (rotate the board in-plane)",
        }[missing_pose[0]]
    return "move it to another area of the frame; angles already covered"

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pattern", default="5x7", help="inner corners cols x rows (default 5x7)")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--target", type=int, default=30, help="target number of captures")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    primary = parse_pattern(args.pattern)
    # Accept ONLY this board, in either orientation. No other sizes — a coarse
    # live detector could otherwise lock onto a wrong size and ruin the set.
    candidates = [primary, (primary[1], primary[0])]
    locked_pattern = None

    cap = open_csi_capture(width=args.width, height=args.height, fps=args.fps, downscale=True)
    if cap is None:
        print("[error] camera unavailable")
        return 1
    preview = Preview.from_env("Checkerboard Capture", fps=args.fps)

    dists: set = set()
    poses: set = set()
    saved_centroids: list = []
    saved = next_index(args.output_dir)
    if saved:
        print(f"[info] {saved} existing images in {args.output_dir} (will append)")
    prev_centroid = None
    steady = 0
    last_capture = 0.0
    print(f"[info] target {args.target} captures, pattern {primary} (or its transpose). Ctrl+C to stop.")

    status = "looking for board..."
    color = (0, 200, 255)
    last_corners = None
    last_pose = None
    frame_idx = 0

    try:
        while saved < args.target:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            frame_idx += 1

            # Detect only every Nth frame (heavy on CPU); stream every frame so
            # the H264 preview stays fluid. Between detections we redraw the last
            # corners — the board moves slowly relative to the frame rate.
            if frame_idx % DETECT_EVERY == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners = None
                if locked_pattern is not None:
                    corners = detect_board(gray, locked_pattern)
                else:
                    for pat in candidates:
                        corners = detect_board(gray, pat)
                        if corners is not None:
                            locked_pattern = pat
                            print(f"[info] locked pattern (inner corners): {pat[0]}x{pat[1]}")
                            break
                last_corners = corners

                if corners is not None:
                    pose = board_metrics(corners, locked_pattern, frame.shape)
                    last_pose = pose
                    motion = (np.linalg.norm(pose.centroid - prev_centroid)
                              if prev_centroid is not None else 999)
                    prev_centroid = pose.centroid
                    steady = steady + 1 if motion < STABLE_PX else 0
                    sharp = sharpness(gray, corners)

                    # A pose is worth banking if it is far from every previous save
                    # OR it adds a new pose/distance bucket, never a near-duplicate.
                    far_from_saves = all(
                        np.linalg.norm(pose.centroid - c) > SPREAD_MIN_PX for c in saved_centroids
                    ) if saved_centroids else True
                    new_dist = pose.dist not in dists
                    new_pose = not pose.buckets.issubset(poses)
                    coverage_ready = (len(dists) >= len(DIST_BUCKETS) and
                                      len(poses) >= len(POSE_BUCKETS))
                    distinct = new_dist or new_pose or (coverage_ready and far_from_saves)
                    now = time.time()

                    if sharp < SHARPNESS_MIN:
                        status, color = "blurry: hold still for a moment", (0, 0, 255)
                    elif steady < STABLE_FRAMES:
                        status, color = "stabilizing... don't move", (0, 200, 255)
                    elif not distinct:
                        status, color = "need: " + hint(dists, poses), (0, 200, 255)
                    elif now - last_capture < MIN_CAPTURE_GAP:
                        status, color = "wait...", (0, 200, 255)
                    else:
                        out = args.output_dir / f"calib_{saved:03d}.jpg"
                        cv2.imwrite(str(out), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                        dists.add(pose.dist); poses.update(pose.buckets); saved_centroids.append(pose.centroid)
                        saved += 1
                        last_capture = now
                        pose_label = "+".join(sorted(pose.buckets))
                        status, color = f"CAPTURE #{saved} ({pose.dist},{pose_label})", (0, 255, 0)
                        print(f"[save] {out.name}  dist={pose.dist} pose={pose_label} "
                              f"front_pitch={pose.pitch_deg:.0f} front_yaw={pose.yaw_deg:.0f} roll2d={pose.roll_deg:.0f} "
                              f"sharp={sharp:.0f}  [{len(dists)}/3 dist, {len(poses)}/6 pose]")
                else:
                    prev_centroid = None
                    steady = 0
                    status, color = "looking for board (must be fully visible)...", (0, 200, 255)
                    last_pose = None

            if last_corners is not None and locked_pattern is not None:
                cv2.drawChessboardCorners(frame, locked_pattern, last_corners, True)
            bar = f"{saved}/{args.target}  dist {len(dists)}/3  pose {len(poses)}/6"
            cv2.putText(frame, bar, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(frame, bar, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(frame, status, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, status, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if last_corners is not None:
                tip = hint(dists, poses)
                cv2.putText(frame, tip, (8, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(frame, tip, (8, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            if last_pose is not None:
                angles = (f"front pitch {last_pose.pitch_deg:+.0f} deg  front yaw {last_pose.yaw_deg:+.0f} deg  "
                          f"roll2d {last_pose.roll_deg:+.0f} deg  area {last_pose.area_frac * 100:.0f}%")
                cv2.putText(frame, angles, (8, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
                cv2.putText(frame, angles, (8, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            preview.show(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        preview.close()

    print(f"\n[done] {saved} images in {args.output_dir}")
    print(f"       variety: {len(dists)}/3 distances, {len(poses)}/6 poses")
    enough = saved >= 12 and len(dists) >= 2 and len(poses) >= MIN_POSES
    print("       quality:", "OK to calibrate" if enough else "LOW variety - capture more")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
