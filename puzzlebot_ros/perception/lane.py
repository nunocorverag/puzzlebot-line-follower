#!/usr/bin/env python3
"""Bird's-eye lane following perception.

ROS-free, single source of truth for robust center-line following. Both the
runtime node and the offline calibrator import ``analyze_lane`` from here so the
robot follows exactly what was tuned in the calibrator (same pattern as
``intersection.py``).

The pipeline (why it is robust against off-lane distractor lines and curves):
  1. Warp the ground plane to a bird's-eye (top-down) view via a homography.
     In that view the followed line is near-vertical and the parallel floor
     seams stay parallel and spatially separated, so they are easy to ignore.
  2. Build a black mask with CLAHE normalization + a selectable threshold
     (global Otsu or local adaptive) so it survives lighting changes.
  3. Find the line base with a histogram restricted to a band around the image
     center (the robot's line MUST be there), then climb with sliding windows
     that only follow that line. A parallel seam never enters the window.
  4. Fit a 2nd-order polynomial x = f(y). That yields the lateral offset (for
     steering) AND the curvature (to slow down on bends) in one shot.

Coordinate conventions:
  * Bird's-eye image: origin top-left, robot is at the bottom-center
    (``warp_w / 2``, ``warp_h``). y grows downward = closer to the robot.
  * ``offset`` is ``line_x - center`` evaluated near the bottom: positive means
    the line is to the RIGHT of the robot, so it must steer right.
  * The line center is also mapped back to the original image so the runtime PD
    (which works in original pixels) and the overlay can use it directly.

Metric scale is optional polish: the controller works on the normalized offset
and curvature. Set ``px_per_cm_x10`` (from the known 11.8 cm lane width / 2.2 cm
dash measured in the warped image) to also get the offset in centimeters.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, fields
from pathlib import Path

import cv2
import numpy as np


@dataclass
class LaneParams:
    """All lane-following tunables. Trackbar-friendly ints (``_pct`` = percent of
    the relevant dimension, ``_x10`` = value times ten) so a slider value can
    flow from the calibrator to JSON to the robot without translation, exactly
    like :class:`~puzzlebot_ros.perception.intersection.IntersectionParams`."""

    # --- Bird's-eye warp (source trapezoid in the ORIGINAL image) -----------
    # The trapezoid is symmetric about the image center. Tune the four numbers
    # live until a straight line looks vertical and parallel lines stay parallel
    # in the warped view.
    src_top_y_pct: int = 55        # top edge of the trapezoid (further ahead)
    src_top_half_w_pct: int = 14   # half-width of the top edge (% of image width)
    src_bot_y_pct: int = 95        # bottom edge (closest to the robot)
    src_bot_half_w_pct: int = 42   # half-width of the bottom edge
    warp_w: int = 400              # bird's-eye output width  (px)
    warp_h: int = 600             # bird's-eye output height (px)

    # --- Mask (illumination-robust) ----------------------------------------
    mask_method: int = 0           # 0 = global Otsu, 1 = local adaptive
    use_clahe: int = 1             # contrast-limited adaptive histogram eq
    clahe_clip_x10: int = 20       # CLAHE clip limit (2.0)
    clahe_grid: int = 8            # CLAHE tile grid (NxN)
    adaptive_block: int = 41       # adaptive-threshold block size (forced odd)
    adaptive_c: int = 8            # adaptive-threshold constant subtracted
    blur_ksize: int = 5            # gaussian blur kernel (forced odd)

    # --- Sliding-window line search ----------------------------------------
    nwindows: int = 12             # vertical windows stacked bottom -> top
    window_half_w_pct: int = 12    # window half-width (% of warp_w)
    min_pix: int = 60              # min pixels in a window to recenter it
    base_search_half_w_pct: int = 26  # base histogram is limited to +/- this
                                      # around the center (% of warp_w) -> the
                                      # KEY guard that ignores side lines
    min_fill_pct: int = 1          # below this mask fill the warped view is empty
    max_fill_pct: int = 55         # above this it is noise / over-binarized

    # --- Steering / confidence ---------------------------------------------
    eval_y_pct: int = 88           # where to read the steering offset
                                   # (% of warp_h from the top; near the robot)
    min_windows_conf_pct: int = 40  # need this % of windows with pixels to trust

    # --- Optional metric scale ---------------------------------------------
    px_per_cm_x10: int = 0         # warped px per cm (x10). 0 = metric disabled


@dataclass
class LaneResult:
    detected: bool
    offset_px: float = 0.0          # line_x - center at eval row (warped px)
    offset_norm: float = 0.0        # offset / (warp_w / 2), in [-1, 1]
    offset_cm: float | None = None  # offset in cm if px_per_cm is set
    curvature: float = 0.0          # 2*a from x = a y^2 + b y + c (1/px)
    curvature_norm: float = 0.0     # curvature scaled to a 0..1-ish magnitude
    heading: float = 0.0            # dx/dy at the eval row (line tilt)
    confidence: float = 0.0         # fraction of windows that found the line
    fit: tuple | None = None        # (a, b, c) of x = a y^2 + b y + c, or None
    base_x: float | None = None     # detected line base x (warped px)
    eval_x: float | None = None     # line x at the eval row (warped px)
    lane_center_x_orig: float | None = None  # eval point mapped to original img
    lane_points_orig: list = field(default_factory=list)  # fitted curve, orig
    warped_mask: np.ndarray | None = None     # debug
    window_centers: list = field(default_factory=list)    # debug (warped px)
    fill_pct: float = 0.0


def save_lane_params(params: LaneParams, path) -> None:
    """Persist lane tunables to JSON so the runtime loads exactly what the
    calibrator tuned. Mirrors ``save_intersection_params``."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {f.name: getattr(params, f.name) for f in fields(LaneParams)}
    path.write_text(json.dumps(data, indent=2))


def load_lane_params(path, base: LaneParams | None = None) -> LaneParams:
    """Load lane tunables from JSON. Unknown / missing keys are ignored so the
    file stays forward/backward compatible."""
    params = base or LaneParams()
    path = Path(path)
    if not path.exists():
        return params
    data = json.loads(path.read_text())
    valid = {f.name for f in fields(LaneParams)}
    for key, value in data.items():
        if key in valid:
            setattr(params, key, value)
    return params


# ---------------------------------------------------------------------------
# Homography
# ---------------------------------------------------------------------------
def src_points(params: LaneParams, w: int, h: int) -> np.ndarray:
    """Four source points (original image) ordered TL, TR, BR, BL."""
    cx = w / 2.0
    top_y = h * params.src_top_y_pct / 100.0
    bot_y = h * params.src_bot_y_pct / 100.0
    top_dx = w * params.src_top_half_w_pct / 100.0
    bot_dx = w * params.src_bot_half_w_pct / 100.0
    return np.float32([
        [cx - top_dx, top_y],   # top-left
        [cx + top_dx, top_y],   # top-right
        [cx + bot_dx, bot_y],   # bottom-right
        [cx - bot_dx, bot_y],   # bottom-left
    ])


def dst_points(params: LaneParams) -> np.ndarray:
    """Destination rectangle corners (bird's-eye), ordered TL, TR, BR, BL."""
    w, h = float(params.warp_w), float(params.warp_h)
    return np.float32([[0, 0], [w, 0], [w, h], [0, h]])


def compute_homography(params: LaneParams, w: int, h: int):
    """Return (M, Minv) mapping original <-> bird's-eye."""
    src = src_points(params, w, h)
    dst = dst_points(params)
    m = cv2.getPerspectiveTransform(src, dst)
    minv = cv2.getPerspectiveTransform(dst, src)
    return m, minv


def _warp(image: np.ndarray, m, params: LaneParams) -> np.ndarray:
    return cv2.warpPerspective(
        image, m, (params.warp_w, params.warp_h), flags=cv2.INTER_LINEAR
    )


def warped_black_mask(warped_gray: np.ndarray, params: LaneParams) -> np.ndarray:
    """Illumination-robust black mask on the already-warped grayscale image.

    CLAHE flattens local contrast (so a shadow on one side does not move a
    global threshold), then either Otsu (global, adapts to overall brightness)
    or adaptive thresholding (local, best under strong gradients) binarizes the
    dark line. ``mask_method`` lets us A/B the two against the dataset.
    """
    k = max(1, params.blur_ksize | 1)
    gray = cv2.GaussianBlur(warped_gray, (k, k), 1.4)
    if params.use_clahe:
        clip = max(0.1, params.clahe_clip_x10 / 10.0)
        grid = max(1, params.clahe_grid)
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))
        gray = clahe.apply(gray)
    if params.mask_method == 1:
        block = max(3, params.adaptive_block | 1)
        mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, block, params.adaptive_c,
        )
    else:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


# ---------------------------------------------------------------------------
# Sliding-window line fit
# ---------------------------------------------------------------------------
def _sliding_window_fit(mask: np.ndarray, params: LaneParams):
    """Histogram base (center-restricted) + sliding windows + polynomial fit.

    Returns dict with fit (a, b, c) for x = a y^2 + b y + c, base_x,
    window_centers and the count of windows that actually found the line.
    """
    h, w = mask.shape[:2]
    nwin = max(2, params.nwindows)
    half_w = max(4, int(w * params.window_half_w_pct / 100.0))
    base_half = max(4, int(w * params.base_search_half_w_pct / 100.0))

    # Base: histogram over the bottom third, but only within a band around the
    # center. This is what rejects parallel side lines: the followed line must
    # start near the robot's center, so we never lock onto an off-center seam.
    bottom = mask[int(h * 2 / 3):, :]
    column_sum = np.sum(bottom, axis=0).astype(np.float64)
    center = w // 2
    lo, hi = max(0, center - base_half), min(w, center + base_half)
    band = column_sum[lo:hi]
    if band.size == 0 or band.max() <= 0:
        return {"fit": None, "base_x": None, "window_centers": [], "found": 0,
                "nwin": nwin}
    base_x = lo + int(np.argmax(band))

    nz = mask.nonzero()
    nz_y = np.array(nz[0])
    nz_x = np.array(nz[1])

    win_h = h // nwin
    current_x = base_x
    fit_x, fit_y = [], []
    window_centers = []
    found = 0
    for i in range(nwin):
        y_hi = h - i * win_h
        y_lo = h - (i + 1) * win_h
        x_lo = int(current_x - half_w)
        x_hi = int(current_x + half_w)
        good = ((nz_y >= y_lo) & (nz_y < y_hi)
                & (nz_x >= x_lo) & (nz_x < x_hi)).nonzero()[0]
        cy = (y_lo + y_hi) // 2
        if good.size >= params.min_pix:
            current_x = int(np.mean(nz_x[good]))
            found += 1
            fit_x.extend(nz_x[good].tolist())
            fit_y.extend(nz_y[good].tolist())
            window_centers.append((current_x, cy))
        else:
            # No pixels: keep the last x (the window dead-reckons upward) but do
            # not feed empty data into the fit.
            window_centers.append((current_x, cy))

    fit = None
    if len(fit_y) >= max(50, params.min_pix):
        ys = np.array(fit_y, dtype=np.float64)
        xs = np.array(fit_x, dtype=np.float64)
        # x as a function of y (y is the long axis in the bird's-eye view).
        deg = 2 if found >= 3 else 1
        coeffs = np.polyfit(ys, xs, deg)
        if deg == 1:
            coeffs = np.array([0.0, coeffs[0], coeffs[1]])
        fit = (float(coeffs[0]), float(coeffs[1]), float(coeffs[2]))

    return {"fit": fit, "base_x": base_x, "window_centers": window_centers,
            "found": found, "nwin": nwin}


def _eval_fit(fit, y):
    a, b, c = fit
    return a * y * y + b * y + c


def analyze_lane(frame: np.ndarray, params: LaneParams,
                 m=None, minv=None) -> LaneResult:
    """Full bird's-eye lane analysis on a (preprocessed) BGR frame.

    Pass a cached ``m``/``minv`` homography to avoid recomputing it every frame;
    otherwise it is derived from the current frame size.
    """
    h, w = frame.shape[:2]
    if m is None or minv is None:
        m, minv = compute_homography(params, w, h)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    warped_gray = _warp(gray, m, params)
    mask = warped_black_mask(warped_gray, params)

    fill_pct = 100.0 * float(cv2.countNonZero(mask)) / float(mask.size)
    result = LaneResult(detected=False, warped_mask=mask, fill_pct=fill_pct)
    if not (params.min_fill_pct <= fill_pct <= params.max_fill_pct):
        return result

    sw = _sliding_window_fit(mask, params)
    result.base_x = None if sw["base_x"] is None else float(sw["base_x"])
    result.window_centers = sw["window_centers"]
    result.confidence = sw["found"] / float(sw["nwin"])

    fit = sw["fit"]
    if fit is None or result.confidence < params.min_windows_conf_pct / 100.0:
        return result

    wh = float(params.warp_h)
    ww = float(params.warp_w)
    eval_y = wh * params.eval_y_pct / 100.0
    eval_x = _eval_fit(fit, eval_y)
    center = ww / 2.0

    result.detected = True
    result.fit = fit
    result.eval_x = float(eval_x)
    result.offset_px = float(eval_x - center)
    result.offset_norm = float(np.clip((eval_x - center) / center, -2.0, 2.0))
    result.curvature = float(2.0 * fit[0])
    # Normalize curvature to a ~[-1,1] magnitude over the warp height so it can
    # scale speed without caring about absolute px units.
    result.curvature_norm = float(np.clip(2.0 * fit[0] * wh, -1.0, 1.0))
    result.heading = float(2.0 * fit[0] * eval_y + fit[1])  # dx/dy at eval row
    if params.px_per_cm_x10 > 0:
        result.offset_cm = result.offset_px / (params.px_per_cm_x10 / 10.0)

    # Map the eval point back to the original image so the runtime PD (original
    # pixels) and the overlay can use it directly.
    result.lane_center_x_orig = _birdseye_to_orig((eval_x, eval_y), minv)[0]

    # A few points of the fitted curve, mapped back for the overlay.
    pts = []
    for frac in np.linspace(0.2, 1.0, 6):
        yy = wh * frac
        xx = _eval_fit(fit, yy)
        pts.append(_birdseye_to_orig((xx, yy), minv))
    result.lane_points_orig = pts
    return result


def _birdseye_to_orig(pt, minv):
    src = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, minv)
    return (float(dst[0, 0, 0]), float(dst[0, 0, 1]))


# ---------------------------------------------------------------------------
# Debug drawing
# ---------------------------------------------------------------------------
def draw_lane_overlay(frame: np.ndarray, params: LaneParams,
                      result: LaneResult) -> None:
    """Draw the warp trapezoid and the fitted line back on the original frame."""
    h, w = frame.shape[:2]
    src = src_points(params, w, h).astype(np.int32)
    cv2.polylines(frame, [src.reshape(-1, 1, 2)], True, (0, 200, 255), 2)
    if result.lane_points_orig:
        pts = np.array([[int(x), int(y)] for x, y in result.lane_points_orig],
                       dtype=np.int32)
        color = (0, 255, 0) if result.detected else (0, 0, 255)
        cv2.polylines(frame, [pts.reshape(-1, 1, 2)], False, color, 2)
    if result.lane_center_x_orig is not None:
        x = int(result.lane_center_x_orig)
        cv2.circle(frame, (x, int(h * 0.92)), 8,
                   (0, 255, 0) if result.detected else (0, 0, 255), -1)
    cv2.putText(
        frame,
        f"off={result.offset_norm:+.2f} curv={result.curvature_norm:+.2f} "
        f"conf={result.confidence:.2f} fill={result.fill_pct:.1f}",
        (20, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
    )


def draw_birdseye_debug(result: LaneResult, params: LaneParams) -> np.ndarray:
    """Render the warped mask with the sliding windows and fit for the
    calibrator dashboard. Returns a BGR image (warp_w x warp_h)."""
    if result.warped_mask is None:
        return np.zeros((params.warp_h, params.warp_w, 3), dtype=np.uint8)
    canvas = cv2.cvtColor(result.warped_mask, cv2.COLOR_GRAY2BGR)
    half_w = max(4, int(params.warp_w * params.window_half_w_pct / 100.0))
    for (cx, cy) in result.window_centers:
        cv2.rectangle(canvas, (int(cx - half_w), int(cy - params.warp_h // (2 * max(2, params.nwindows)))),
                      (int(cx + half_w), int(cy + params.warp_h // (2 * max(2, params.nwindows)))),
                      (0, 180, 0), 1)
    if result.fit is not None:
        for frac in np.linspace(0.0, 1.0, 40):
            yy = params.warp_h * frac
            xx = int(_eval_fit(result.fit, yy))
            if 0 <= xx < params.warp_w:
                cv2.circle(canvas, (xx, int(yy)), 2, (0, 0, 255), -1)
    center = params.warp_w // 2
    cv2.line(canvas, (center, 0), (center, params.warp_h), (255, 255, 0), 1)
    if result.eval_x is not None:
        cv2.circle(canvas, (int(result.eval_x),
                            int(params.warp_h * params.eval_y_pct / 100.0)),
                   6, (0, 255, 0) if result.detected else (0, 0, 255), -1)
    return canvas
