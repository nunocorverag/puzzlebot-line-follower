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
import os
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

# Single source of truth for the detection logic lives in the package so the
# runtime node and this calibrator never diverge again.
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.intersection import (  # noqa: E402
    IntersectionParams,
    IntersectionResult as DetectionResult,
    analyze_intersection,
    black_mask,
    save_intersection_params,
)
# Camera capture / preprocessing helpers also live in the package now. Re-export
# them so other tools that historically imported from this module keep working.
from puzzlebot_ros.perception.camera import (  # noqa: E402,F401
    apply_illumination_gain,
    build_gstreamer_pipeline,
    load_camera_params,
    load_illumination_gain,
    preprocess_frame,
)
from puzzlebot_ros.perception.stream import Preview  # noqa: E402


@dataclass
class CalibrationParams(IntersectionParams):
    """Detection params (inherited) plus calibrator UI-only fields."""

    show_state_panel: int = 1


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
    "option_scan_y0_pct": ("option_scan_y0_pct", 0, 95),
    "dynamic_option_roi": ("dynamic_option_roi", 0, 1),
    "entry_y0_pct": ("entry_y0_pct", 0, 100),
    "entry_margin_pct": ("entry_margin_pct", 0, 30),
    "dynamic_option_height_pct": ("dynamic_option_height_pct", 1, 80),
    "split_option_rois": ("split_option_rois", 0, 1),
    "option_gap_pct": ("option_gap_pct", 0, 20),
    "straight_option_width_pct": ("straight_option_width_pct", 6, 60),
    "option_roi_skew_pct": ("option_roi_skew_pct", 0, 30),
    "roi_skew": ("option_roi_skew_pct", 0, 30),
    "entry_line_tol_pct": ("entry_line_tol_pct", 0, 20),
    "entry_max_slope_x10": ("entry_max_slope_x10", 0, 30),
    "center_tol_pct": ("center_tol_pct", 0, 50),
    "merge_width_factor_x10": ("merge_width_factor_x10", 10, 50),
    "require_centered": ("require_centered", 0, 1),
    "show_state_panel": ("show_state_panel", 0, 1),
    "ratio_fallback": ("ratio_fallback", 0, 1),
    "enable_ratio_fallback": ("ratio_fallback", 0, 1),
    "ahead_ratio_pct": ("ahead_ratio_pct", 0, 30),
    "side_ratio_pct": ("side_ratio_pct", 0, 30),
    "side_y0_pct": ("side_y0_pct", 0, 100),
    "side_y1_pct": ("side_y1_pct", 1, 100),
}


PARAM_ALIASES = {
    "rect_pct": "rectangularity_pct",
    "stable_frames": "stable_frames_needed",
    "option_dash_count": "option_min_dash_count",
    "ratio_fallback": "enable_ratio_fallback",
    "roi_skew": "option_roi_skew_pct",
}


def set_param_direct(params: CalibrationParams, name: str, value: int) -> CalibrationParams:
    field_name = PARAM_ALIASES.get(name, name)
    if not hasattr(params, field_name):
        return params
    updated = CalibrationParams(**asdict(params))
    setattr(updated, field_name, int(value))
    return updated


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


def apply_param_command(
    controls_window: str | None,
    command: str,
    state: dict[str, str],
    params: CalibrationParams | None = None,
) -> CalibrationParams | None:
    parsed = parse_param_command(command)
    if parsed is None:
        print(f"[cmd] ignored: {command}")
        return params
    name, raw_value = parsed
    if name in ("s", "save"):
        state["request_save_sample"] = "1"
        return params
    if name in ("q", "quit"):
        state["request_quit"] = "1"
        return params
    if name in ("p", "pause"):
        state["request_pause"] = "1"
        return params
    if name in ("u", "undistort"):
        state["request_toggle_undistort"] = "1"
        return params
    if name in ("save_calib", "save_params"):
        # Defer the actual write to the main loop (it owns the output path).
        state["request_save_calib"] = "1"
        return params
    if name == "label":
        label = raw_value.strip().replace(" ", "_")
        if not label:
            print("[cmd] ignored empty label")
            return params
        state["label"] = label
        print(f"[cmd] label={label}")
        return params

    binding = TRACKBAR_BINDINGS.get(name)
    if binding is None:
        known = ", ".join(["label"] + sorted(TRACKBAR_BINDINGS))
        print(f"[cmd] unknown parameter '{name}'. Known: {known}")
        return params
    try:
        value = int(float(raw_value))
    except ValueError:
        print(f"[cmd] invalid numeric value for {name}: {raw_value}")
        return params
    trackbar_name, min_value, max_value = binding
    clamped = max(min_value, min(max_value, value))
    if controls_window is not None:
        cv2.setTrackbarPos(trackbar_name, controls_window, clamped)
    if params is not None:
        params = set_param_direct(params, name, clamped)
    print(f"[cmd] {name}={clamped}")
    return params


def apply_command_file(
    controls_window: str | None,
    command_file: Path,
    last_mtime: int | None,
    state: dict[str, str],
    params: CalibrationParams | None = None,
) -> tuple[int | None, CalibrationParams | None]:
    if not command_file.exists():
        return last_mtime, params
    stat = command_file.stat()
    mtime_ns = stat.st_mtime_ns
    if last_mtime is not None and mtime_ns <= last_mtime:
        return last_mtime, params
    for line in command_file.read_text().splitlines():
        params = apply_param_command(controls_window, line, state, params)
    return mtime_ns, params


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
        ("option_scan_y0_pct", params.option_scan_y0_pct, 95),
        ("dynamic_option_roi", params.dynamic_option_roi, 1),
        ("entry_y0_pct", params.entry_y0_pct, 100),
        ("entry_margin_pct", params.entry_margin_pct, 30),
        ("dynamic_option_height_pct", params.dynamic_option_height_pct, 80),
        ("split_option_rois", params.split_option_rois, 1),
        ("option_gap_pct", params.option_gap_pct, 20),
        ("straight_option_width_pct", params.straight_option_width_pct, 60),
        ("option_roi_skew_pct", params.option_roi_skew_pct, 30),
        ("entry_line_tol_pct", params.entry_line_tol_pct, 20),
        ("entry_max_slope_x10", params.entry_max_slope_x10, 30),
        ("center_tol_pct", params.center_tol_pct, 50),
        ("merge_width_factor_x10", params.merge_width_factor_x10, 50),
        ("require_centered", params.require_centered, 1),
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
    updated.option_scan_y0_pct = cv2.getTrackbarPos("option_scan_y0_pct", controls_window)
    updated.dynamic_option_roi = cv2.getTrackbarPos("dynamic_option_roi", controls_window)
    updated.entry_y0_pct = cv2.getTrackbarPos("entry_y0_pct", controls_window)
    updated.entry_margin_pct = cv2.getTrackbarPos("entry_margin_pct", controls_window)
    updated.dynamic_option_height_pct = max(1, cv2.getTrackbarPos("dynamic_option_height_pct", controls_window))
    updated.split_option_rois = cv2.getTrackbarPos("split_option_rois", controls_window)
    updated.option_gap_pct = cv2.getTrackbarPos("option_gap_pct", controls_window)
    updated.straight_option_width_pct = max(6, cv2.getTrackbarPos("straight_option_width_pct", controls_window))
    updated.option_roi_skew_pct = cv2.getTrackbarPos("option_roi_skew_pct", controls_window)
    updated.entry_line_tol_pct = cv2.getTrackbarPos("entry_line_tol_pct", controls_window)
    updated.entry_max_slope_x10 = cv2.getTrackbarPos("entry_max_slope_x10", controls_window)
    updated.center_tol_pct = cv2.getTrackbarPos("center_tol_pct", controls_window)
    updated.merge_width_factor_x10 = max(10, cv2.getTrackbarPos("merge_width_factor_x10", controls_window))
    updated.require_centered = cv2.getTrackbarPos("require_centered", controls_window)
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


def draw_box_pct_alpha(
    frame: np.ndarray,
    x0_pct: int,
    x1_pct: int,
    y0_pct: int,
    y1_pct: int,
    color: tuple[int, int, int],
    alpha: float = 0.18,
    border: int = 2,
) -> None:
    h, w = frame.shape[:2]
    p0 = (int(w * x0_pct / 100.0), int(h * y0_pct / 100.0))
    p1 = (int(w * x1_pct / 100.0), int(h * y1_pct / 100.0))
    fill = frame.copy()
    cv2.rectangle(fill, p0, p1, color, -1)
    cv2.addWeighted(fill, alpha, frame, 1.0 - alpha, 0, frame)
    cv2.rectangle(frame, p0, p1, color, border)


def draw_poly_pct_alpha(
    frame: np.ndarray,
    poly_pct: list,
    color: tuple[int, int, int],
    alpha: float = 0.16,
    border: int = 2,
) -> None:
    h, w = frame.shape[:2]
    pts = np.array([[int(w * x / 100.0), int(h * y / 100.0)] for x, y in poly_pct], dtype=np.int32)
    fill = frame.copy()
    cv2.fillPoly(fill, [pts], color)
    cv2.addWeighted(fill, alpha, frame, 1.0 - alpha, 0, frame)
    cv2.polylines(frame, [pts], True, color, border)


# Per-zone dash colors (BGR). These match the option-ROI fills so a dash and
# the ROI it was assigned to read as the same color.
ZONE_COLORS = {
    "entry": (0, 165, 255),     # orange  - aligned entry-zebra dashes
    "left": (255, 0, 255),      # magenta - left option
    "straight": (255, 255, 0),  # cyan    - straight option
    "right": (255, 160, 0),     # azure   - right option
    "merged": (0, 0, 255),      # red     - oversized/merged blob (rejected)
    "other": (90, 90, 90),      # gray    - detected but outside any ROI
}


def draw_overlay(
    frame: np.ndarray,
    result: DetectionResult,
    params: CalibrationParams,
    undistort_enabled: bool,
    label: str,
    show_header: bool = True,
) -> np.ndarray:
    overlay = frame.copy()
    h, w = overlay.shape[:2]

    # Red: active entry/dash detection band. It stays low and does not define options.
    draw_box_pct_alpha(overlay, 0, 100, params.roi_y0_pct, params.roi_y1_pct, (0, 0, 255), 0.10)
    if result.entry_y_pct is not None:
        # The actual fitted zebra line (cy = slope*cx + intercept) that the entry
        # dashes are aligned to. Green when centered enough to read options.
        y_left = int(result.entry_intercept)
        y_right = int(result.entry_slope * w + result.entry_intercept)
        line_color = (0, 255, 0) if result.entry_centered else (0, 165, 255)
        cv2.line(overlay, (0, y_left), (w, y_right), line_color, 2)

    if result.dashed_detected and params.split_option_rois:
        colors = {
            "left": (255, 0, 255),
            "straight": (255, 255, 0),
            "right": (255, 160, 0),
        }
        for name, poly in result.option_roi_polys.items():
            color = (0, 255, 0) if result.option_valid.get(name, False) else colors[name]
            alpha = 0.22 if result.option_valid.get(name, False) else 0.12
            draw_poly_pct_alpha(overlay, poly, color, alpha)
    elif result.dashed_detected:
        draw_box_pct_alpha(overlay, *result.option_box_pct, (255, 0, 0), 0.12)

    zones = result.box_zones or ["entry"] * len(result.dashed_boxes)
    for (x, y, bw, bh), zone in zip(result.dashed_boxes, zones):
        color = ZONE_COLORS.get(zone, ZONE_COLORS["other"])
        if zone == "other":
            # Stray dashes outside any ROI: draw faint and thin so they no longer
            # clutter the view (they are not part of any detection).
            cv2.rectangle(overlay, (x, y), (x + bw, y + bh), color, 1)
            continue
        border = 3 if zone == "merged" else 2
        fill = overlay.copy()
        cv2.rectangle(fill, (x, y), (x + bw, y + bh), color, -1)
        cv2.addWeighted(fill, 0.22, overlay, 0.78, 0, overlay)
        cv2.rectangle(overlay, (x, y), (x + bw, y + bh), color, border)

    if show_header:
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


def draw_stream_debug_panel(
    width: int,
    result: DetectionResult,
    params: CalibrationParams,
    undistort_enabled: bool,
    label: str,
    paused: bool,
) -> np.ndarray:
    panel = np.zeros((168, width, 3), dtype=np.uint8)

    def put(text: str, x: int, y: int, color: tuple[int, int, int], scale: float = 0.52) -> int:
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)
        return x + int(len(text) * scale * 17)

    entry_y = f"{result.entry_y_pct:.0f}" if result.entry_y_pct is not None else "--"
    state_color = (0, 255, 0) if result.state_name == "READ_OPTIONS" else (
        (0, 200, 255) if result.state_name == "APPROACH_ENTRY" else (180, 180, 180))

    # Row 1: state + the gate that controls when options open.
    x = put(f"{result.state_name}", 12, 26, state_color, 0.6)
    x = put(f"centered={int(result.entry_centered)}", x + 14, 26, (0, 255, 0) if result.entry_centered else (0, 140, 255))
    x = put(f"slope={result.entry_slope:+.2f}", x + 14, 26, (0, 255, 255))
    put(f"entry_y={entry_y}  stable={result.stable_frames}/{params.stable_frames_needed}", x + 14, 26, (0, 255, 255))

    # Row 2: entry-dash count (aligned inliers) + label/flags.
    x = put(f"entry dashes {result.dashed_count}/{params.min_dash_count}", 12, 56, ZONE_COLORS["entry"])
    put(f"label={label}  paused={int(paused)}  undistort={int(undistort_enabled)}", x + 14, 56, (0, 255, 255))

    # Row 3: per-ROI counts, each in its own color (only meaningful once reading).
    x = put("ROI counts:", 12, 86, (200, 200, 200))
    x = put(f"L {result.left_dash}", x + 10, 86, ZONE_COLORS["left"])
    x = put(f"S {result.center_dash}", x + 12, 86, ZONE_COLORS["straight"])
    x = put(f"R {result.right_dash}", x + 12, 86, ZONE_COLORS["right"])
    valid = result.option_valid or {}
    put("valid " + " ".join(f"{k[0].upper()}:{int(valid.get(k, False))}" for k in ("left", "straight", "right")),
        x + 18, 86, (0, 255, 0))

    # Row 4: chosen options + key geometry knobs.
    opts = ",".join(result.options) if result.options else "none"
    put(f"options={opts}   skew={params.option_roi_skew_pct}  gap={params.option_gap_pct}  "
        f"opt_h={params.dynamic_option_height_pct}  margin={params.entry_margin_pct}", 12, 116, (0, 255, 255))

    put("cmd: min_dash_count=3 | roi_skew=12 | center_tol_pct=12 | save_calib=1 (save for robot) | s=1 img | q=1 quit",
        12, 150, (160, 160, 160), 0.46)
    return panel


def compose_stream_dashboard(
    overlay: np.ndarray,
    mask: np.ndarray,
    result: DetectionResult,
    params: CalibrationParams,
    undistort_enabled: bool,
    label: str,
    paused: bool,
) -> np.ndarray:
    """Single H264-friendly view: overlay + Otsu mask + compact debug panel."""
    h, w = overlay.shape[:2]
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask_bgr = cv2.resize(mask_bgr, (w, h), interpolation=cv2.INTER_NEAREST)
    top = np.hstack([overlay, mask_bgr])
    panel = draw_stream_debug_panel(w * 2, result, params, undistort_enabled, label, paused)
    return np.vstack([top, panel])

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
    parser.add_argument("--params-out", type=Path, default=REPO_DIR / "config" / "intersection_params.json",
                        help="Where 'save_calib' writes the tuned detection params for the runtime to load")
    parser.add_argument("--preview-mode", choices=("local", "h264", "none"),
                        default=os.environ.get("STREAM", "local"),
                        help="local uses OpenCV windows/trackbars; h264 streams dashboard and uses commands")
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
    local_preview = args.preview_mode == "local"
    stream_preview = None if local_preview else Preview.from_env("Line Vision Calibrator", fps=args.fps)
    params = CalibrationParams()
    if local_preview:
        cv2.namedWindow(image_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(controls_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(state_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(image_window, args.width, args.height)
        cv2.resizeWindow(mask_window, args.width, args.height)
        cv2.resizeWindow(controls_window, 760, 520)
        cv2.resizeWindow(state_window, 560, 360)
        # Fixed window layout: mask top-left, calibrator below it, controls to the right, state bottom-right
        cv2.moveWindow(mask_window,    0,   0)
        cv2.moveWindow(image_window,   0,   args.height + 30)
        cv2.moveWindow(controls_window, args.width + 10,  0)
        cv2.moveWindow(state_window,   args.width + 10,  args.height + 30)
        create_trackbars(controls_window, params)
    command_queue: queue.Queue[str] = queue.Queue()
    start_stdin_command_thread(command_queue)
    command_file_mtime = None
    state = {"label": args.label}
    print("[cmd] type commands here, e.g. min_dash_count=6, label=true_intersection, or set roi_y0_pct 42")
    if not local_preview:
        print("[cmd] H264 mode: use commands; examples: s=1 save, p=1 pause, q=1 quit, u=1 toggle undistort")
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

        if raw.shape[1] != args.width or raw.shape[0] != args.height:
            raw = cv2.resize(raw, (args.width, args.height), interpolation=cv2.INTER_AREA)

        save_requested = False
        quit_requested = False
        while not command_queue.empty():
            command = command_queue.get_nowait().strip()
            if not local_preview and command in ("q", "quit", "q=1"):
                quit_requested = True
                continue
            if not local_preview and command in ("s", "save", "s=1", "save=1"):
                save_requested = True
                continue
            if not local_preview and command in ("p", "pause", "p=1", "pause=1"):
                paused = not paused
                print(f"[info] paused={paused}")
                continue
            if not local_preview and command in ("u", "undistort", "u=1", "undistort=1"):
                undistort_enabled = not undistort_enabled and camera_matrix is not None and dist_coeffs is not None
                print(f"[info] undistort={undistort_enabled}")
                continue
            params = apply_param_command(controls_window if local_preview else None, command, state, params)
        command_file_mtime, params = apply_command_file(
            controls_window if local_preview else None,
            args.command_file,
            command_file_mtime,
            state,
            params,
        )
        if state.pop("request_save_sample", None):
            save_requested = True
        if state.pop("request_quit", None):
            quit_requested = True
        if state.pop("request_pause", None):
            paused = not paused
            print(f"[info] paused={paused}")
        if state.pop("request_toggle_undistort", None):
            undistort_enabled = not undistort_enabled and camera_matrix is not None and dist_coeffs is not None
            print(f"[info] undistort={undistort_enabled}")
        if local_preview:
            params = read_trackbars(controls_window, params)
        if state.pop("request_save_calib", None):
            save_intersection_params(params, args.params_out)
            print(f"[calib] saved tuned params -> {args.params_out}")
        processed = raw.copy()
        if undistort_enabled:
            processed = cv2.undistort(processed, camera_matrix, dist_coeffs)
        processed = apply_illumination_gain(processed, illumination_gain)

        result = analyze_intersection(processed, params, stable_frames)
        stable_frames = result.stable_frames
        mask = black_mask(processed)
        overlay = draw_overlay(processed, result, params, undistort_enabled, state["label"], show_header=local_preview)
        state_panel = draw_state_panel(result, params, undistort_enabled, state["label"], paused)
        if local_preview:
            cv2.imshow(image_window, overlay)
            cv2.imshow(mask_window, mask)
            if params.show_state_panel:
                cv2.imshow(state_window, state_panel)

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
        else:
            dashboard = compose_stream_dashboard(overlay, mask, result, params, undistort_enabled, state["label"], paused)
            stream_preview.show(dashboard)
            if save_requested:
                save_sample(args.output_dir, raw, processed, mask, overlay, params, result, state["label"])
            if quit_requested:
                break

    if cap is not None:
        cap.release()
    if stream_preview is not None:
        stream_preview.close()
    if local_preview:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
