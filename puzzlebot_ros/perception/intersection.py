#!/usr/bin/env python3
"""Intersection / dashed-marking perception.

ROS-free, single source of truth for intersection detection. Both the runtime
node and the offline calibrator import ``analyze_intersection`` from here so the
robot behaves exactly like what was tuned in the calibrator.

The logic:
  1. Build a black mask (grayscale + Otsu).
  2. Find dash-like rectangles inside a low entry band, with a y-dependent
     minimum area (far dashes are small, near dashes are large).
  3. Estimate the entry-zebra y position and place option ROIs above it.
  4. Split options into left / straight / right and validate each by geometry
     (a left zebra rises toward the center, a right zebra falls away).
  5. Require N stable frames before declaring an intersection.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class IntersectionParams:
    """All intersection tunables. Defaults are the live-tested calibrator set.

    Keep field names identical to the calibrator trackbars / ROS params so a
    value can flow from a slider to YAML to the robot without translation.
    """

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
    # Top of the extended scan band. Dashes between this and roi_y1 are searched
    # so options above the entry zebra can be classified, while only dashes
    # inside [roi_y0, roi_y1] drive the (robust, low) detection trigger.
    option_scan_y0_pct: int = 38
    dynamic_option_roi: int = 1
    entry_y0_pct: int = 58
    entry_margin_pct: int = 10
    dynamic_option_height_pct: int = 22
    split_option_rois: int = 1
    option_gap_pct: int = 4
    straight_option_width_pct: int = 24
    enable_ratio_fallback: int = 0
    ahead_ratio_pct: int = 6
    side_ratio_pct: int = 8


@dataclass
class IntersectionResult:
    dashed_detected: bool
    options: list
    stable_frames: int
    dashed_count: int
    left_dash: int
    center_dash: int
    right_dash: int
    ahead_ratio: float
    left_ratio: float
    right_ratio: float
    dashed_boxes: list = field(default_factory=list)
    entry_y_pct: float | None = None
    option_box_pct: tuple = (0, 0, 0, 0)
    option_roi_boxes: dict = field(default_factory=dict)
    option_counts: dict = field(default_factory=dict)
    option_valid: dict = field(default_factory=dict)
    state_name: str = "FOLLOW_LINE"
    dash_min_area_range: tuple = (0, 0)


def black_mask(frame: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def clamp_box(mask: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> tuple:
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


def build_option_roi_boxes(params: IntersectionParams, option_box: tuple) -> dict:
    x0, x1, y0, y1 = option_box
    gap = max(0, params.option_gap_pct)
    straight_half = max(4, params.straight_option_width_pct // 2)
    straight_x0 = max(x0, 50 - straight_half)
    straight_x1 = min(x1, 50 + straight_half)
    left_x1 = min(straight_x0 - gap, 50 - gap)
    right_x0 = max(straight_x1 + gap, 50 + gap)
    return {
        "left": (x0, max(x0 + 1, left_x1), y0, y1),
        "straight": (straight_x0, max(straight_x0 + 1, straight_x1), y0, y1),
        "right": (min(x1 - 1, right_x0), x1, y0, y1),
    }


def dashes_in_pct_box(dashed: list, frame_w: int, frame_h: int, box_pct: tuple) -> list:
    x0, x1, y0, y1 = box_pct
    px0 = frame_w * x0 / 100.0
    px1 = frame_w * x1 / 100.0
    py0 = frame_h * y0 / 100.0
    py1 = frame_h * y1 / 100.0
    return [d for d in dashed if px0 <= d[0] <= px1 and py0 <= d[1] <= py1]


def aligned_option_pattern(points: list, option: str, min_count: int) -> bool:
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
    # In image coordinates, a left-option zebra usually rises toward the center;
    # a right-option zebra usually falls away from the center.
    if option == "left":
        return slope < -0.10
    if option == "right":
        return slope > 0.10
    return False


def analyze_intersection(
    frame: np.ndarray,
    params: IntersectionParams,
    stable_frames: int,
    mask: np.ndarray | None = None,
) -> IntersectionResult:
    """Run one frame through the intersection detector.

    ``stable_frames`` is the running count from the previous call; the returned
    result carries the updated count, so callers thread it frame to frame.
    Pass ``mask`` to reuse an already-computed black mask.
    """
    h, w = frame.shape[:2]
    if mask is None:
        mask = black_mask(frame)
    roi_y0 = int(h * params.roi_y0_pct / 100.0)
    roi_y1 = int(h * params.roi_y1_pct / 100.0)
    # Scan a wider band than the low trigger band so option dashes above the
    # entry zebra are also found. Dashes are then split into:
    #   - trigger_dashed: inside [roi_y0, roi_y1] -> drive the robust trigger
    #   - dashed (all):   whole scan band         -> classify L/S/R options
    scan_y0 = int(h * min(params.option_scan_y0_pct, params.roi_y0_pct) / 100.0)
    contours, _ = cv2.findContours(mask[scan_y0:roi_y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dashed: list = []
    boxes: list = []
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
        y += scan_y0
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

    # Only dashes in the low band drive the trigger (keeps it robust to
    # background/neighboring-lane clutter higher in the image).
    trigger_dashed = [d for d in dashed if roi_y0 <= d[1] <= roi_y1]
    entry_candidates = [d for d in trigger_dashed if d[1] >= h * params.entry_y0_pct / 100.0]
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

    option_roi_boxes = build_option_roi_boxes(params, option_box_pct)
    option_points = {
        name: dashes_in_pct_box(dashed, w, h, box) for name, box in option_roi_boxes.items()
    }
    option_counts = {name: len(points) for name, points in option_points.items()}
    option_valid = {
        name: aligned_option_pattern(points, name, params.option_min_dash_count)
        for name, points in option_points.items()
    }

    ahead_ratio = black_ratio(
        mask,
        w * params.ahead_x0_pct / 100.0, w * params.ahead_x1_pct / 100.0,
        h * params.roi_y0_pct / 100.0, h * params.roi_y1_pct / 100.0,
    )
    left_ratio = black_ratio(
        mask,
        w * params.left_x0_pct / 100.0, w * params.left_x1_pct / 100.0,
        h * params.side_y0_pct / 100.0, h * params.side_y1_pct / 100.0,
    )
    right_ratio = black_ratio(
        mask,
        w * params.right_x0_pct / 100.0, w * params.right_x1_pct / 100.0,
        h * params.side_y0_pct / 100.0, h * params.side_y1_pct / 100.0,
    )

    # Trigger depends only on the low-band dashes (validated 0 false positives
    # on normal/side_lane/curve/finish frames). Option dashes above the entry
    # are classification-only and intentionally do not trigger the stop.
    raw_detected = len(trigger_dashed) >= params.min_dash_count
    stable_frames = stable_frames + 1 if raw_detected else 0
    dashed_detected = stable_frames >= params.stable_frames_needed

    state_name = (
        "READ_OPTIONS" if dashed_detected
        else ("APPROACH_ENTRY" if raw_detected else "FOLLOW_LINE")
    )

    options: list = []
    if dashed_detected:
        for name in ("left", "straight", "right"):
            if option_valid[name]:
                options.append(name)
        if params.enable_ratio_fallback:
            if "left" not in options and left_ratio > params.side_ratio_pct / 100.0:
                options.append("left")
            if "straight" not in options and ahead_ratio > params.ahead_ratio_pct / 100.0:
                options.append("straight")
            if "right" not in options and right_ratio > params.side_ratio_pct / 100.0:
                options.append("right")

    return IntersectionResult(
        dashed_detected=dashed_detected,
        options=options,
        stable_frames=stable_frames,
        dashed_count=len(trigger_dashed),
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
