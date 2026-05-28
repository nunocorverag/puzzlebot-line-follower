#!/usr/bin/env python3
"""Interactive calibration tool for Puzzlebot line/intersection vision.

This tool does not use ROS and never publishes /cmd_vel. It is meant for safe
perception tuning from a live Jetson CSI camera, a USB camera, or saved images.

Keys:
  q / ESC  quit
  s        save raw/processed/mask/overlay + metadata JSON
  u        toggle undistortion
  p        pause/resume live camera
  h        toggle compact state panel
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_ILLUMINATION_PARAMS = REPO_DIR / "config" / "illumination_flatfield.npz"
DEFAULT_OUTPUT_DIR = REPO_DIR / "debug_dataset"


@dataclass
class CalibrationParams:
    roi_y0_pct: int = 72
    roi_y1_pct: int = 88
    dash_min_area: int = 40
    near_dash_min_area: int = 700
    dynamic_dash_area: int = 1
    near_dash_y0_pct: int = 72
    dash_max_area: int = 2400
    rectangularity_pct: int = 25
    max_aspect_x10: int = 60
    min_dash_count: int = 5
    stable_frames_needed: int = 6
    ahead_x0_pct: int = 35
    ahead_x1_pct: int = 65
    side_y0_pct: int = 45
    side_y1_pct: int = 72
    left_x0_pct: int = 10
    left_x1_pct: int = 38
    right_x0_pct: int = 62
    right_x1_pct: int = 90
    option_min_dash_count: int = 2
    option_x0_pct: int = 4
    option_x1_pct: int = 96
    option_y0_pct: int = 35
    option_y1_pct: int = 68
    dynamic_option_roi: int = 1
    entry_y0_pct: int = 58
    entry_margin_pct: int = 10
    dynamic_option_height_pct: int = 22
    split_option_rois: int = 1
    option_gap_pct: int = 4
    straight_option_width_pct: int = 24
    show_state_panel: int = 1
    enable_ratio_fallback: int = 0
    ahead_ratio_pct: int = 6
    side_ratio_pct: int = 8


@dataclass
class DetectionResult:
    dashed_detected: bool
    options: list[str]
    stable_frames: int
    dashed_count: int
    left_dash: int
    center_dash: int
    right_dash: int
    ahead_ratio: float
    left_ratio: float
    right_ratio: float
    dashed_boxes: list[tuple[int, int, int, int]]
    entry_y_pct: float | None
    option_box_pct: tuple[int, int, int, int]
    option_roi_boxes: dict[str, tuple[int, int, int, int]]
    option_counts: dict[str, int]
    option_valid: dict[str, bool]
    state_name: str
    dash_min_area_range: tuple[int, int]


def build_gstreamer_pipeline(width: int = 640, height: int = 480, fps: int = 30) -> str:
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink max-buffers=1 drop=true"
    )


def load_camera_params(path: Path) -> tuple[np.ndarray | None, np.ndarray | None]:
    if not path.exists():
        print(f"[warn] camera params not found: {path}")
        return None, None
    data = np.load(str(path))
    print(f"[info] loaded camera params: {path}")
    return data["camera_matrix"], data["dist_coeffs"]


def load_illumination_gain(path: Path) -> np.ndarray | None:
    if not path.exists():
        print(f"[warn] illumination params not found: {path}")
        return None
    data = np.load(str(path))
    print(f"[info] loaded illumination params: {path}")
    return data["gain"].astype(np.float32)


def apply_illumination_gain(frame: np.ndarray, gain: np.ndarray | None) -> np.ndarray:
    if gain is None:
        return frame
    if gain.shape[:2] != frame.shape[:2]:
        gain = cv2.resize(gain, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
    corrected = frame.astype(np.float32) * gain
    return np.clip(corrected, 0, 255).astype(np.uint8)


def open_capture(args: argparse.Namespace) -> cv2.VideoCapture | None:
    if args.image:
        return None
    if args.video:
        cap = cv2.VideoCapture(str(args.video))
        return cap if cap.isOpened() else None
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
    return cap if cap.isOpened() else None


def black_mask(frame: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def clamp_box(mask: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> tuple[int, int, int, int]:
    h, w = mask.shape[:2]
    ix0 = max(0, min(w, int(x0)))
    ix1 = max(0, min(w, int(x1)))
    iy0 = max(0, min(h, int(y0)))
    iy1 = max(0, min(h, int(y1)))
    return ix0, ix1, iy0, iy1


def black_ratio(mask: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> float:
    ix0, ix1, iy0, iy1 = clamp_box(mask, x0, x1, y0, y1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    roi = mask[iy0:iy1, ix0:ix1]
    return float(cv2.countNonZero(roi)) / float(roi.size)



def build_option_roi_boxes(params: CalibrationParams, option_box: tuple[int, int, int, int]) -> dict[str, tuple[int, int, int, int]]:
    x0, x1, y0, y1 = option_box
    gap = max(0, params.option_gap_pct)
    straight_half = max(4, params.straight_option_width_pct // 2)
    straight_x0 = max(x0, 50 - straight_half)
    straight_x1 = min(x1, 50 + straight_half)
    left_x1 = min(straight_x0 - gap, 50 - gap)
    right_x0 = max(straight_x1 + gap, 50 + gap)
    boxes = {
        "left": (x0, max(x0 + 1, left_x1), y0, y1),
        "straight": (straight_x0, max(straight_x0 + 1, straight_x1), y0, y1),
        "right": (min(x1 - 1, right_x0), x1, y0, y1),
    }
    return boxes


def dashes_in_pct_box(
    dashed: list[tuple[float, float, int, int, float]],
    frame_w: int,
    frame_h: int,
    box_pct: tuple[int, int, int, int],
) -> list[tuple[float, float, int, int, float]]:
    x0, x1, y0, y1 = box_pct
    px0 = frame_w * x0 / 100.0
    px1 = frame_w * x1 / 100.0
    py0 = frame_h * y0 / 100.0
    py1 = frame_h * y1 / 100.0
    return [d for d in dashed if px0 <= d[0] <= px1 and py0 <= d[1] <= py1]


def aligned_option_pattern(points: list[tuple[float, float, int, int, float]], option: str, min_count: int) -> bool:
    if len(points) < min_count:
        return False
    if option == "straight":
        return True
    if len(points) < 2:
        return False
    pts = sorted(points, key=lambda d: d[0])
    xs = np.array([p[0] for p in pts], dtype=np.float32)
    ys = np.array([p[1] for p in pts], dtype=np.float32)
    if float(xs.max() - xs.min()) < 8.0:
        return False
    slope = float(np.polyfit(xs, ys, 1)[0])
    # In image coordinates, left-option zebra usually rises toward the center;
    # right-option zebra usually falls away from the center.
    if option == "left":
        return slope < -0.10
    if option == "right":
        return slope > 0.10
    return False

def analyze_intersection(frame: np.ndarray, params: CalibrationParams, stable_frames: int) -> DetectionResult:
    h, w = frame.shape[:2]
    mask = black_mask(frame)
    roi_y0 = int(h * params.roi_y0_pct / 100.0)
    roi_y1 = int(h * params.roi_y1_pct / 100.0)
    contours, _ = cv2.findContours(mask[roi_y0:roi_y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dashed: list[tuple[float, float, int, int, float]] = []
    boxes: list[tuple[int, int, int, int]] = []
    rectangularity_min = params.rectangularity_pct / 100.0
    max_aspect = max(1.0, params.max_aspect_x10 / 10.0)
    near_y0 = h * params.near_dash_y0_pct / 100.0

    def min_area_for_y(cy: float) -> float:
        if not params.dynamic_dash_area:
            return float(params.dash_min_area)
        if cy <= near_y0:
            return float(params.dash_min_area)
        denom = max(1.0, float(roi_y1) - near_y0)
        t = min(1.0, max(0.0, (cy - near_y0) / denom))
        return float(params.dash_min_area) + t * float(params.near_dash_min_area - params.dash_min_area)

    for c in contours:
        area = cv2.contourArea(c)
        x, y, bw, bh = cv2.boundingRect(c)
        y += roi_y0
        if bw < 5 or bh < 5:
            continue
        cx, cy = x + bw / 2.0, y + bh / 2.0
        if area < min_area_for_y(cy) or area > params.dash_max_area:
            continue
        rectangularity = area / float(bw * bh)
        if rectangularity < rectangularity_min:
            continue
        aspect = max(bw / float(bh), bh / float(bw))
        if aspect > max_aspect:
            continue
        dashed.append((cx, cy, bw, bh, area))
        boxes.append((x, y, bw, bh))

    center_x = w / 2.0
    entry_candidates = [d for d in dashed if d[1] >= h * params.entry_y0_pct / 100.0]
    entry_y_pct = None
    if entry_candidates:
        entry_y = float(np.median([d[1] for d in entry_candidates]))
        entry_y_pct = 100.0 * entry_y / h
    option_x0_pct = params.option_x0_pct
    option_x1_pct = params.option_x1_pct
    option_y0_pct = params.option_y0_pct
    option_y1_pct = params.option_y1_pct
    if params.dynamic_option_roi and entry_y_pct is not None:
        option_y1_pct = max(1, int(entry_y_pct - params.entry_margin_pct))
        option_y0_pct = max(0, option_y1_pct - params.dynamic_option_height_pct)
    option_box_pct = (option_x0_pct, option_x1_pct, option_y0_pct, option_y1_pct)
    option_x0 = w * option_x0_pct / 100.0
    option_x1 = w * option_x1_pct / 100.0
    option_y0 = h * option_y0_pct / 100.0
    option_y1 = h * option_y1_pct / 100.0
    option_dashed = [d for d in dashed if option_x0 <= d[0] <= option_x1 and option_y0 <= d[1] <= option_y1]
    option_roi_boxes = build_option_roi_boxes(params, option_box_pct)
    option_points = {
        name: dashes_in_pct_box(dashed, w, h, box)
        for name, box in option_roi_boxes.items()
    }
    option_counts = {name: len(points) for name, points in option_points.items()}
    option_valid = {
        name: aligned_option_pattern(points, name, params.option_min_dash_count)
        for name, points in option_points.items()
    }
    left_dash = option_points["left"]
    center_dash = option_points["straight"]
    right_dash = option_points["right"]

    ahead_ratio = black_ratio(
        mask,
        w * params.ahead_x0_pct / 100.0,
        w * params.ahead_x1_pct / 100.0,
        h * params.roi_y0_pct / 100.0,
        h * params.roi_y1_pct / 100.0,
    )
    left_ratio = black_ratio(
        mask,
        w * params.left_x0_pct / 100.0,
        w * params.left_x1_pct / 100.0,
        h * params.side_y0_pct / 100.0,
        h * params.side_y1_pct / 100.0,
    )
    right_ratio = black_ratio(
        mask,
        w * params.right_x0_pct / 100.0,
        w * params.right_x1_pct / 100.0,
        h * params.side_y0_pct / 100.0,
        h * params.side_y1_pct / 100.0,
    )

    raw_detected = len(dashed) >= params.min_dash_count or (
        option_counts["straight"] >= params.option_min_dash_count
        and (option_counts["left"] + option_counts["right"]) >= params.option_min_dash_count
    )
    stable_frames = stable_frames + 1 if raw_detected else 0
    dashed_detected = stable_frames >= params.stable_frames_needed

    state_name = "READ_OPTIONS" if dashed_detected else ("APPROACH_ENTRY" if raw_detected else "FOLLOW_LINE")
    options: list[str] = []
    if dashed_detected:
        for name in ("left", "straight", "right"):
            if option_valid[name]:
                options.append(name)

    if dashed_detected and params.enable_ratio_fallback:
        if "left" not in options and left_ratio > params.side_ratio_pct / 100.0:
            options.append("left")
        if "straight" not in options and ahead_ratio > params.ahead_ratio_pct / 100.0:
            options.append("straight")
        if "right" not in options and right_ratio > params.side_ratio_pct / 100.0:
            options.append("right")


    return DetectionResult(
        dashed_detected=dashed_detected,
        options=options,
        stable_frames=stable_frames,
        dashed_count=len(dashed),
        left_dash=option_counts["left"],
        center_dash=option_counts["straight"],
        right_dash=option_counts["right"],
        ahead_ratio=ahead_ratio,
        left_ratio=left_ratio,
        right_ratio=right_ratio,
        dashed_boxes=boxes,
        entry_y_pct=entry_y_pct,
        option_box_pct=option_box_pct,
        option_roi_boxes=option_roi_boxes,
        option_counts=option_counts,
        option_valid=option_valid,
        state_name=state_name,
        dash_min_area_range=(
            params.dash_min_area,
            params.near_dash_min_area if params.dynamic_dash_area else params.dash_min_area,
        ),
    )


TRACKBAR_BINDINGS = {
    "roi_y0_pct": ("roi_y0_pct", 0, 95),
    "roi_y1_pct": ("roi_y1_pct", 1, 100),
    "dash_min_area": ("dash_min_area", 1, 2000),
    "near_dash_min_area": ("near_dash_min_area", 1, 2000),
    "dynamic_dash_area": ("dynamic_dash_area", 0, 1),
    "near_dash_y0_pct": ("near_dash_y0_pct", 0, 100),
    "dash_max_area": ("dash_max_area", 2, 5000),
    "rect_pct": ("rect_pct", 0, 100),
    "rectangularity_pct": ("rect_pct", 0, 100),
    "max_aspect_x10": ("max_aspect_x10", 10, 120),
    "min_dash_count": ("min_dash_count", 1, 20),
    "stable_frames": ("stable_frames", 1, 20),
    "stable_frames_needed": ("stable_frames", 1, 20),
    "option_dash_count": ("option_dash_count", 1, 10),
    "option_min_dash_count": ("option_dash_count", 1, 10),
    "option_x0_pct": ("option_x0_pct", 0, 100),
    "option_x1_pct": ("option_x1_pct", 1, 100),
    "option_y0_pct": ("option_y0_pct", 0, 100),
    "option_y1_pct": ("option_y1_pct", 1, 100),
    "dynamic_option_roi": ("dynamic_option_roi", 0, 1),
    "entry_y0_pct": ("entry_y0_pct", 0, 100),
    "entry_margin_pct": ("entry_margin_pct", 0, 30),
    "dynamic_option_height_pct": ("dynamic_option_height_pct", 1, 80),
    "split_option_rois": ("split_option_rois", 0, 1),
    "option_gap_pct": ("option_gap_pct", 0, 20),
    "straight_option_width_pct": ("straight_option_width_pct", 6, 60),
    "show_state_panel": ("show_state_panel", 0, 1),
    "ratio_fallback": ("ratio_fallback", 0, 1),
    "enable_ratio_fallback": ("ratio_fallback", 0, 1),
    "ahead_ratio_pct": ("ahead_ratio_pct", 0, 30),
    "side_ratio_pct": ("side_ratio_pct", 0, 30),
    "side_y0_pct": ("side_y0_pct", 0, 100),
    "side_y1_pct": ("side_y1_pct", 1, 100),
}


def start_stdin_command_thread(command_queue: queue.Queue[str]) -> None:
    def worker() -> None:
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            command_queue.put(line.strip())

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()


def parse_param_command(command: str) -> tuple[str, str] | None:
    command = command.strip()
    if not command or command.startswith("#"):
        return None
    if command.startswith("set "):
        command = command[4:].strip()
    if "=" in command:
        name, value = command.split("=", 1)
    else:
        parts = command.split()
        if len(parts) != 2:
            return None
        name, value = parts
    return name.strip(), value.strip()


def apply_param_command(controls_window: str, command: str, state: dict[str, str]) -> bool:
    parsed = parse_param_command(command)
    if parsed is None:
        print(f"[cmd] ignored: {command}")
        return False
    name, raw_value = parsed
    if name == "label":
        label = raw_value.strip().replace(" ", "_")
        if not label:
            print("[cmd] ignored empty label")
            return False
        state["label"] = label
        print(f"[cmd] label={label}")
        return True

    binding = TRACKBAR_BINDINGS.get(name)
    if binding is None:
        known = ", ".join(["label"] + sorted(TRACKBAR_BINDINGS))
        print(f"[cmd] unknown parameter '{name}'. Known: {known}")
        return False
    try:
        value = int(float(raw_value))
    except ValueError:
        print(f"[cmd] invalid numeric value for {name}: {raw_value}")
        return False
    trackbar_name, min_value, max_value = binding
    clamped = max(min_value, min(max_value, value))
    cv2.setTrackbarPos(trackbar_name, controls_window, clamped)
    print(f"[cmd] {name}={clamped}")
    return True


def apply_command_file(controls_window: str, command_file: Path, last_mtime: float | None, state: dict[str, str]) -> float | None:
    if not command_file.exists():
        return last_mtime
    mtime = command_file.stat().st_mtime
    if last_mtime is not None and mtime <= last_mtime:
        return last_mtime
    for line in command_file.read_text().splitlines():
        apply_param_command(controls_window, line, state)
    return mtime

def create_trackbars(controls_window: str, params: CalibrationParams) -> None:
    def noop(_: int) -> None:
        return

    for name, value, max_value in [
        ("roi_y0_pct", params.roi_y0_pct, 95),
        ("roi_y1_pct", params.roi_y1_pct, 100),
        ("dash_min_area", params.dash_min_area, 2000),
        ("near_dash_min_area", params.near_dash_min_area, 2000),
        ("dynamic_dash_area", params.dynamic_dash_area, 1),
        ("near_dash_y0_pct", params.near_dash_y0_pct, 100),
        ("dash_max_area", params.dash_max_area, 5000),
        ("rect_pct", params.rectangularity_pct, 100),
        ("max_aspect_x10", params.max_aspect_x10, 120),
        ("min_dash_count", params.min_dash_count, 20),
        ("stable_frames", params.stable_frames_needed, 20),
        ("option_dash_count", params.option_min_dash_count, 10),
        ("option_x0_pct", params.option_x0_pct, 100),
        ("option_x1_pct", params.option_x1_pct, 100),
        ("option_y0_pct", params.option_y0_pct, 100),
        ("option_y1_pct", params.option_y1_pct, 100),
        ("dynamic_option_roi", params.dynamic_option_roi, 1),
        ("entry_y0_pct", params.entry_y0_pct, 100),
        ("entry_margin_pct", params.entry_margin_pct, 30),
        ("dynamic_option_height_pct", params.dynamic_option_height_pct, 80),
        ("split_option_rois", params.split_option_rois, 1),
        ("option_gap_pct", params.option_gap_pct, 20),
        ("straight_option_width_pct", params.straight_option_width_pct, 60),
        ("show_state_panel", params.show_state_panel, 1),
        ("ratio_fallback", params.enable_ratio_fallback, 1),
        ("ahead_ratio_pct", params.ahead_ratio_pct, 30),
        ("side_ratio_pct", params.side_ratio_pct, 30),
        ("side_y0_pct", params.side_y0_pct, 100),
        ("side_y1_pct", params.side_y1_pct, 100),
    ]:
        cv2.createTrackbar(name, controls_window, int(value), int(max_value), noop)


def read_trackbars(controls_window: str, params: CalibrationParams) -> CalibrationParams:
    updated = CalibrationParams(**asdict(params))
    updated.roi_y0_pct = cv2.getTrackbarPos("roi_y0_pct", controls_window)
    updated.roi_y1_pct = cv2.getTrackbarPos("roi_y1_pct", controls_window)
    updated.dash_min_area = max(1, cv2.getTrackbarPos("dash_min_area", controls_window))
    updated.near_dash_min_area = max(1, cv2.getTrackbarPos("near_dash_min_area", controls_window))
    updated.dynamic_dash_area = cv2.getTrackbarPos("dynamic_dash_area", controls_window)
    updated.near_dash_y0_pct = cv2.getTrackbarPos("near_dash_y0_pct", controls_window)
    updated.dash_max_area = max(updated.dash_min_area + 1, cv2.getTrackbarPos("dash_max_area", controls_window))
    updated.rectangularity_pct = cv2.getTrackbarPos("rect_pct", controls_window)
    updated.max_aspect_x10 = max(10, cv2.getTrackbarPos("max_aspect_x10", controls_window))
    updated.min_dash_count = max(1, cv2.getTrackbarPos("min_dash_count", controls_window))
    updated.stable_frames_needed = max(1, cv2.getTrackbarPos("stable_frames", controls_window))
    updated.option_min_dash_count = max(1, cv2.getTrackbarPos("option_dash_count", controls_window))
    updated.option_x0_pct = cv2.getTrackbarPos("option_x0_pct", controls_window)
    updated.option_x1_pct = cv2.getTrackbarPos("option_x1_pct", controls_window)
    updated.option_y0_pct = cv2.getTrackbarPos("option_y0_pct", controls_window)
    updated.option_y1_pct = cv2.getTrackbarPos("option_y1_pct", controls_window)
    updated.dynamic_option_roi = cv2.getTrackbarPos("dynamic_option_roi", controls_window)
    updated.entry_y0_pct = cv2.getTrackbarPos("entry_y0_pct", controls_window)
    updated.entry_margin_pct = cv2.getTrackbarPos("entry_margin_pct", controls_window)
    updated.dynamic_option_height_pct = max(1, cv2.getTrackbarPos("dynamic_option_height_pct", controls_window))
    updated.split_option_rois = cv2.getTrackbarPos("split_option_rois", controls_window)
    updated.option_gap_pct = cv2.getTrackbarPos("option_gap_pct", controls_window)
    updated.straight_option_width_pct = max(6, cv2.getTrackbarPos("straight_option_width_pct", controls_window))
    updated.show_state_panel = cv2.getTrackbarPos("show_state_panel", controls_window)
    updated.enable_ratio_fallback = cv2.getTrackbarPos("ratio_fallback", controls_window)
    updated.ahead_ratio_pct = cv2.getTrackbarPos("ahead_ratio_pct", controls_window)
    updated.side_ratio_pct = cv2.getTrackbarPos("side_ratio_pct", controls_window)
    updated.side_y0_pct = cv2.getTrackbarPos("side_y0_pct", controls_window)
    updated.side_y1_pct = cv2.getTrackbarPos("side_y1_pct", controls_window)
    updated.roi_y1_pct = max(updated.roi_y0_pct + 1, updated.roi_y1_pct)
    updated.side_y1_pct = max(updated.side_y0_pct + 1, updated.side_y1_pct)
    updated.option_x1_pct = max(updated.option_x0_pct + 1, updated.option_x1_pct)
    updated.option_y1_pct = max(updated.option_y0_pct + 1, updated.option_y1_pct)
    return updated


def draw_box_pct(frame: np.ndarray, x0_pct: int, x1_pct: int, y0_pct: int, y1_pct: int, color: tuple[int, int, int]) -> None:
    h, w = frame.shape[:2]
    p0 = (int(w * x0_pct / 100.0), int(h * y0_pct / 100.0))
    p1 = (int(w * x1_pct / 100.0), int(h * y1_pct / 100.0))
    cv2.rectangle(frame, p0, p1, color, 2)


def draw_overlay(frame: np.ndarray, result: DetectionResult, params: CalibrationParams, undistort_enabled: bool, label: str) -> np.ndarray:
    overlay = frame.copy()
    h, w = overlay.shape[:2]

    # Red: active entry/dash detection band. It stays low and does not define options.
    draw_box_pct(overlay, 0, 100, params.roi_y0_pct, params.roi_y1_pct, (0, 0, 255))
    if result.entry_y_pct is not None:
        entry_y = int(h * result.entry_y_pct / 100.0)
        cv2.line(overlay, (0, entry_y), (w, entry_y), (0, 128, 255), 2)

    if result.dashed_detected and params.split_option_rois:
        colors = {
            "left": (255, 0, 255),
            "straight": (255, 255, 0),
            "right": (255, 0, 255),
        }
        for name, box in result.option_roi_boxes.items():
            color = (0, 255, 0) if result.option_valid.get(name, False) else colors[name]
            draw_box_pct(overlay, *box, color)
    elif result.dashed_detected:
        draw_box_pct(overlay, *result.option_box_pct, (255, 0, 0))

    for x, y, bw, bh in result.dashed_boxes:
        cv2.rectangle(overlay, (x, y), (x + bw, y + bh), (0, 255, 255), 2)

    status = "INTERSECTION" if result.dashed_detected else "normal"
    options = ",".join(result.options) if result.options else "none"
    line = f"{result.state_name} | {status} | opt:{options} | dash:{result.dashed_count} | area:{result.dash_min_area_range[0]}->{result.dash_min_area_range[1]}"
    cv2.putText(overlay, line, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(overlay, line, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    cv2.line(overlay, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)
    return overlay


def draw_state_panel(result: DetectionResult, params: CalibrationParams, undistort_enabled: bool, label: str, paused: bool) -> np.ndarray:
    panel = np.zeros((360, 560, 3), dtype=np.uint8)
    rows = [
        ("STATE", result.state_name),
        ("label", label),
        ("paused", str(int(paused))),
        ("undistort", str(int(undistort_enabled))),
        ("stable", f"{result.stable_frames}/{params.stable_frames_needed}"),
        ("dash", str(result.dashed_count)),
        ("options", ",".join(result.options) if result.options else "none"),
        ("counts", f"L:{result.left_dash} S:{result.center_dash} R:{result.right_dash}"),
        ("valid", " ".join(f"{k}:{int(v)}" for k, v in result.option_valid.items())),
        ("entry_y", f"{result.entry_y_pct:.1f}" if result.entry_y_pct is not None else "none"),
        ("entry_roi", f"red y:{params.roi_y0_pct}-{params.roi_y1_pct}"),
        ("option_roi", f"{result.option_box_pct}"),
        ("area", f"min {result.dash_min_area_range[0]}->{result.dash_min_area_range[1]} near_y {params.near_dash_y0_pct}"),
        ("keys", "s save | p pause | h panel | q quit"),
        ("jog", "scripts/jog_forward_jetson.sh 0.04 1.5"),
    ]
    y = 28
    for key, value in rows:
        color = (0, 255, 0) if key == "STATE" and value == "READ_OPTIONS" else (0, 255, 255)
        cv2.putText(panel, f"{key}: {value}", (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        y += 22
    return panel

def save_sample(
    output_dir: Path,
    raw: np.ndarray,
    processed: np.ndarray,
    mask: np.ndarray,
    overlay: np.ndarray,
    params: CalibrationParams,
    result: DetectionResult,
    label: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = output_dir / f"{stamp}_{label}"
    cv2.imwrite(str(prefix.with_name(prefix.name + "_raw.jpg")), raw)
    cv2.imwrite(str(prefix.with_name(prefix.name + "_processed.jpg")), processed)
    cv2.imwrite(str(prefix.with_name(prefix.name + "_mask.png")), mask)
    cv2.imwrite(str(prefix.with_name(prefix.name + "_overlay.jpg")), overlay)
    metadata = {
        "label": label,
        "params": asdict(params),
        "result": {k: v for k, v in asdict(result).items() if k != "dashed_boxes"},
        "dashed_boxes": result.dashed_boxes,
    }
    prefix.with_name(prefix.name + "_meta.json").write_text(json.dumps(metadata, indent=2))
    print(f"[save] {prefix.name}_*.jpg/png/json")


def iter_images(paths: Iterable[Path]) -> list[np.ndarray]:
    frames = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"[warn] could not read image: {path}")
            continue
        frames.append(image)
    return frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", nargs="*", type=Path, help="Image file(s) for offline calibration")
    parser.add_argument("--video", type=Path, help="Video file for offline calibration")
    parser.add_argument("--camera", default=0, help="Camera index/path fallback")
    parser.add_argument("--gstreamer", action="store_true", default=False, help="Use Jetson CSI GStreamer pipeline")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    parser.add_argument("--illumination-params", type=Path, default=DEFAULT_ILLUMINATION_PARAMS)
    parser.add_argument("--no-illumination-correction", action="store_true")
    parser.add_argument("--no-undistort", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--label", default="sample", help="Label used when saving samples")
    parser.add_argument("--command-file", type=Path, default=DEFAULT_OUTPUT_DIR / "calibrator_commands.txt")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    undistort_enabled = not args.no_undistort and camera_matrix is not None and dist_coeffs is not None
    illumination_gain = None
    if not args.no_illumination_correction:
        illumination_gain = load_illumination_gain(args.illumination_params)

    frames = iter_images(args.image or [])
    cap = None if frames else open_capture(args)
    if not frames and cap is None:
        print("[error] no image/video/camera source available")
        return 1

    image_window = "Line Vision Calibrator"
    controls_window = "Controls"
    mask_window = "Mask"
    state_window = "State"
    cv2.namedWindow(image_window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(controls_window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(state_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(image_window, args.width, args.height)
    cv2.resizeWindow(mask_window, args.width, args.height)
    cv2.resizeWindow(controls_window, 760, 520)
    cv2.resizeWindow(state_window, 560, 360)
    params = CalibrationParams()
    create_trackbars(controls_window, params)
    command_queue: queue.Queue[str] = queue.Queue()
    start_stdin_command_thread(command_queue)
    command_file_mtime = None
    state = {"label": args.label}
    print("[cmd] type commands here, e.g. min_dash_count=6, label=true_intersection, or set roi_y0_pct 42")
    print(f"[cmd] also watching command file: {args.command_file}")

    paused = False
    frame_index = 0
    last_raw = frames[0].copy() if frames else None
    stable_frames = 0

    while True:
        if frames:
            raw = frames[frame_index].copy()
        elif not paused or last_raw is None:
            ok, raw = cap.read()
            if not ok:
                print("[warn] frame read failed")
                break
            last_raw = raw.copy()
        else:
            raw = last_raw.copy()

        while not command_queue.empty():
            apply_param_command(controls_window, command_queue.get_nowait(), state)
        command_file_mtime = apply_command_file(controls_window, args.command_file, command_file_mtime, state)
        params = read_trackbars(controls_window, params)
        processed = raw.copy()
        if undistort_enabled:
            processed = cv2.undistort(processed, camera_matrix, dist_coeffs)
        processed = apply_illumination_gain(processed, illumination_gain)

        result = analyze_intersection(processed, params, stable_frames)
        stable_frames = result.stable_frames
        mask = black_mask(processed)
        overlay = draw_overlay(processed, result, params, undistort_enabled, state["label"])
        cv2.imshow(image_window, overlay)
        cv2.imshow(mask_window, mask)
        if params.show_state_panel:
            cv2.imshow(state_window, draw_state_panel(result, params, undistort_enabled, state["label"], paused))

        key = cv2.waitKey(0 if frames else 1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            save_sample(args.output_dir, raw, processed, mask, overlay, params, result, state["label"])
        elif key == ord("u"):
            undistort_enabled = not undistort_enabled and camera_matrix is not None and dist_coeffs is not None
            print(f"[info] undistort={undistort_enabled}")
        elif key == ord("p"):
            paused = not paused
            print(f"[info] paused={paused}")
        elif key == ord("h"):
            cv2.setTrackbarPos("show_state_panel", controls_window, 0 if params.show_state_panel else 1)
        elif frames and key in (ord("n"), ord(" ")):
            frame_index = (frame_index + 1) % len(frames)
            stable_frames = 0
        elif frames and key == ord("b"):
            frame_index = (frame_index - 1) % len(frames)
            stable_frames = 0

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
