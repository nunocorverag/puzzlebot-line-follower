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
    black_thresh: int = 0          # >0: fixed dark threshold (gray < this = line),
                                   #     overrides Otsu/adaptive. The track lines
                                   #     are ALWAYS black, so a fixed dark cut
                                   #     ignores tan floor / white gaps robustly.
                                   #     0 = keep mask_method (Otsu/adaptive).
    use_clahe: int = 1             # contrast-limited adaptive histogram eq
    clahe_clip_x10: int = 20       # CLAHE clip limit (2.0)
    clahe_grid: int = 8            # CLAHE tile grid (NxN)
    adaptive_block: int = 41       # adaptive-threshold block size (forced odd)
    adaptive_c: int = 8            # adaptive-threshold constant subtracted
    blur_ksize: int = 5            # gaussian blur kernel (forced odd)
    line_open_px: int = 5          # remove thin puzzle-piece outlines; the
                                   # painted lane is much thicker and survives
    line_core_px: int = 7          # keep only pixels with this distance-to-edge;
                                   # removes jigsaw seams that survive opening

    # --- Sliding-window line search ----------------------------------------
    nwindows: int = 12             # vertical windows stacked bottom -> top
    window_half_w_pct: int = 12    # window half-width (% of warp_w)
    min_pix: int = 60              # min pixels in a window to recenter it
    base_search_half_w_pct: int = 26  # base histogram is limited to +/- this
                                      # around the center (% of warp_w) -> the
                                      # KEY guard that ignores side lines
    min_fill_pct: int = 1          # below this mask fill the warped view is empty
    max_fill_pct: int = 55         # above this it is noise / over-binarized

    # --- Anti-zebra row reject (key guard at the intersection approach) -----
    # The continuous lane line is VERTICAL in the warp, so it fills only a few
    # px per row (~2.2 cm / lane width). The zebra crossing is a TRANSVERSAL bar
    # of dashes that spans almost the whole warp width. Before the histogram /
    # sliding window we project the mask onto Y (count px per row) and zero out
    # any row whose fill exceeds ``zebra_row_fill_pct`` of the warp width: the
    # dashed cross row vanishes, the continuous line survives, so the follower
    # no longer locks onto the zebra as it approaches the intersection. On a
    # normal straight/curve no row saturates, so this is a no-op there.
    zebra_row_reject: int = 1      # 1 = enable the transversal-row filter
    zebra_row_fill_pct: int = 40   # row fill (% of warp_w) that marks a row as
                                   # a transversal zebra bar -> erased
    zebra_row_close_px: int = 9    # horizontal close (px) to bridge dash gaps
                                   # when measuring row fill (0 = off)

    # --- Dual-line (follow lane CENTER between the two black borders) -------
    # The lane has two black border lines; tracking a single line drifts to one
    # border (the "goes to the right line on curves" bug). With dual_line we find
    # BOTH lines and steer on their midpoint; if only one is visible (tight curve)
    # we offset it by half the lane width to recover the center.
    dual_line: int = 0             # 1 = follow midpoint of the two lines (this
                                   # track follows a single central line, so the
                                   # default is single-line + continuity below)
    min_line_gap_pct: int = 14     # min separation between the two line bases
    lane_half_px: int = 90         # half lane width in warp px (auto-updates when
                                   # both lines are seen; used for 1-line fallback)
    # Temporal continuity: anchor the base histogram near the PREVIOUS frame's
    # line position instead of the image center, so the tracker stays on the same
    # line through a curve instead of jumping to the other border.
    continuity: int = 1
    continuity_search_half_w_pct: int = 12  # when prev_base_x exists, search only
                                            # +/- this % of warp_w around it. This
                                            # prevents a strong side seam from
                                            # hijacking the tracker on approach.
    base_hist_h_pct: int = 18      # bottom slice (% of warp_h) used for the base
                                   # peak; thin = robust to curve smear

    # --- Steering / confidence ---------------------------------------------
    eval_y_pct: int = 88           # where to read the steering offset
                                   # (% of warp_h from the top; near the robot)
    lookahead_y_pct: int = 45      # second, FURTHER-AHEAD read (smaller pct = up =
                                   # further). Its offset minus the near offset is
                                   # the bend, used by the controller feedforward.
    min_windows_conf_pct: int = 40  # need this % of windows with pixels to trust
    fit_max_rmse_px: int = 28       # reject fits made from scattered seams/noise
                                   # (0 disables)

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
    lane_center_far_x_orig: float | None = None  # lookahead point, original img
    lane_points_orig: list = field(default_factory=list)  # fitted curve, orig
    warped_mask: np.ndarray | None = None     # debug (post anti-zebra filter)
    warped_mask_raw: np.ndarray | None = None  # debug (pre anti-zebra filter)
    window_centers: list = field(default_factory=list)    # debug (warped px)
    fill_pct: float = 0.0
    zebra_rows_rejected: int = 0    # rows erased by the anti-zebra filter


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
    # Fixed dark threshold takes precedence: lines are always black, so cutting
    # below a fixed gray level isolates them regardless of tan floor / white gaps
    # (and never flips tan-vs-white the way a global Otsu can). CLAHE is skipped
    # here because it would rescale the very brightness this relies on.
    if getattr(params, "black_thresh", 0) > 0:
        _, mask = cv2.threshold(gray, int(params.black_thresh), 255, cv2.THRESH_BINARY_INV)
        open_px = max(1, int(getattr(params, "line_open_px", 5)) | 1)
        kernel = np.ones((open_px, open_px), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return _keep_thick_line_core(mask, params)
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
    open_px = max(1, int(getattr(params, "line_open_px", 5)) | 1)
    kernel = np.ones((open_px, open_px), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return _keep_thick_line_core(mask, params)


def _keep_thick_line_core(mask: np.ndarray, params: LaneParams) -> np.ndarray:
    """Drop thin puzzle-piece seams while keeping the painted lane core."""
    core_px = int(getattr(params, "line_core_px", 4))
    if core_px <= 0:
        return mask
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    core = (dist >= float(core_px)).astype(np.uint8) * 255
    # Re-grow slightly so the tracker has enough support, but thin seams do not
    # reappear because they had no surviving core pixels.
    grow = max(1, (core_px // 2) | 1)
    kernel = np.ones((grow, grow), np.uint8)
    return cv2.dilate(core, kernel, iterations=1)


def reject_transverse_rows(mask: np.ndarray, params: LaneParams):
    """Erase horizontal (transversal) zebra bars from the warped mask.

    The continuous lane line is near-vertical in the bird's-eye view, so it
    fills only a few px per row. The zebra crossing is a row of dashes that
    spans almost the whole warp width. We project the mask onto Y (px count per
    row); rows whose fill exceeds ``zebra_row_fill_pct`` of the width are zeroed,
    so the dashed cross disappears while the continuous line survives. A small
    horizontal close first bridges the dash gaps so a dashed (not solid) bar
    still registers as dense.

    Returns ``(clean_mask, rows_rejected)``. A straight/curve never saturates a
    row, so this is a no-op there.
    """
    if not getattr(params, "zebra_row_reject", 0):
        return mask, 0
    h, w = mask.shape[:2]
    thresh = max(1, int(w * getattr(params, "zebra_row_fill_pct", 40) / 100.0))
    close_px = int(getattr(params, "zebra_row_close_px", 0))
    if close_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (close_px | 1, 1))
        measured = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    else:
        measured = mask
    row_fill = np.count_nonzero(measured, axis=1)
    dense = row_fill >= thresh
    if not dense.any():
        return mask, 0
    clean = mask.copy()
    clean[dense, :] = 0
    return clean, int(np.count_nonzero(dense))


# ---------------------------------------------------------------------------
# Sliding-window line fit
# ---------------------------------------------------------------------------
def _track_from_base(nz_x, nz_y, base_x, h, w, params):
    """Slide windows upward from ``base_x`` and fit x = a y^2 + b y + c.

    Returns (fit, found, window_centers). Shared by the single- and dual-line
    fitters so both behave identically per line.
    """
    nwin = max(2, params.nwindows)
    half_w = max(4, int(w * params.window_half_w_pct / 100.0))
    win_h = h // nwin
    current_x = base_x
    fit_x, fit_y, centers, found = [], [], [], 0
    for i in range(nwin):
        y_hi = h - i * win_h
        y_lo = h - (i + 1) * win_h
        x_lo, x_hi = int(current_x - half_w), int(current_x + half_w)
        good = ((nz_y >= y_lo) & (nz_y < y_hi)
                & (nz_x >= x_lo) & (nz_x < x_hi)).nonzero()[0]
        cy = (y_lo + y_hi) // 2
        if good.size >= params.min_pix:
            current_x = int(np.mean(nz_x[good]))
            found += 1
            fit_x.extend(nz_x[good].tolist())
            fit_y.extend(nz_y[good].tolist())
        centers.append((current_x, cy))
    fit = None
    if len(fit_y) >= max(50, params.min_pix):
        ys = np.array(fit_y, dtype=np.float64)
        xs = np.array(fit_x, dtype=np.float64)
        deg = 2 if found >= 3 else 1
        coeffs = np.polyfit(ys, xs, deg)
        if deg == 1:
            coeffs = np.array([0.0, coeffs[0], coeffs[1]])
        max_rmse = int(getattr(params, "fit_max_rmse_px", 28))
        if max_rmse > 0:
            pred = coeffs[0] * ys * ys + coeffs[1] * ys + coeffs[2]
            rmse = float(np.sqrt(np.mean((xs - pred) ** 2)))
            if rmse > max_rmse:
                return None, found, centers
        fit = (float(coeffs[0]), float(coeffs[1]), float(coeffs[2]))
    return fit, found, centers


def _dual_line_fit(mask: np.ndarray, params: LaneParams):
    """Find BOTH black border lines and fit their MIDPOINT (the lane center).

    Tracking a single line drifts onto a border in curves; the midpoint is stable.
    If only one line is visible we offset it by half the lane width to keep the
    center. ``lane_half_px`` self-updates whenever both lines are seen.
    """
    h, w = mask.shape[:2]
    nwin = max(2, params.nwindows)
    bottom = mask[int(h * 2 / 3):, :]
    col = np.sum(bottom, axis=0).astype(np.float64)
    if col.size == 0 or col.max() <= 0:
        return {"fit": None, "base_x": None, "window_centers": [], "found": 0,
                "nwin": nwin}

    min_gap = max(8, int(w * params.min_line_gap_pct / 100.0))
    p1 = int(np.argmax(col))
    col2 = col.copy()
    col2[max(0, p1 - min_gap):min(w, p1 + min_gap + 1)] = 0.0
    p2 = int(np.argmax(col2)) if col2.max() > 0.30 * col.max() else None

    nz = mask.nonzero()
    nz_y, nz_x = np.array(nz[0]), np.array(nz[1])
    tracks = []
    for base in [p for p in (p1, p2) if p is not None]:
        fit, found, centers = _track_from_base(nz_x, nz_y, base, h, w, params)
        if fit is not None:
            tracks.append({"base": base, "fit": fit, "found": found, "centers": centers})

    if not tracks:
        return {"fit": None, "base_x": None, "window_centers": [], "found": 0,
                "nwin": nwin}

    if len(tracks) >= 2:
        tracks.sort(key=lambda t: t["base"])
        left, right = tracks[0], tracks[1]
        center_fit = tuple((left["fit"][i] + right["fit"][i]) / 2.0 for i in range(3))
        half = (right["base"] - left["base"]) / 2.0
        if half > 4:                              # learn the lane half-width
            params.lane_half_px = int(0.8 * params.lane_half_px + 0.2 * half)
        base_x = (left["base"] + right["base"]) / 2.0
        found = max(left["found"], right["found"])
        centers = left["centers"] + right["centers"]
    else:
        t = tracks[0]
        half = float(params.lane_half_px)
        # one line only: shift toward center by half a lane (sign from which side)
        sign = +1.0 if t["base"] < w / 2.0 else -1.0
        a, b, c = t["fit"]
        center_fit = (a, b, c + sign * half)
        base_x = t["base"] + sign * half
        found = t["found"]
        centers = t["centers"]

    return {"fit": center_fit, "base_x": base_x, "window_centers": centers,
            "found": found, "nwin": nwin}


def _sliding_window_fit(mask: np.ndarray, params: LaneParams, prev_base_x=None):
    """Histogram base + sliding windows + polynomial fit.

    The base histogram is taken inside a band that is normally centered on the
    image center (rejecting off-center side lines), but with ``continuity`` and a
    ``prev_base_x`` it is centered on the PREVIOUS frame's line so the tracker
    stays on the same line through a curve instead of jumping to the other border.
    """
    h, w = mask.shape[:2]
    nwin = max(2, params.nwindows)
    base_half = max(4, int(w * params.base_search_half_w_pct / 100.0))
    continuity_half = max(4, int(w * getattr(params, "continuity_search_half_w_pct", 12) / 100.0))

    # Base band: a thin slice right at the robot, NOT the whole bottom third. A
    # curved line smears across x over a tall band (so a straighter border wins
    # the peak); the slice nearest the robot keeps the followed line under center.
    band_h = max(0.05, getattr(params, "base_hist_h_pct", 18) / 100.0)
    bottom = mask[int(h * (1.0 - band_h)):, :]
    column_sum = np.sum(bottom, axis=0).astype(np.float64)
    if getattr(params, "continuity", 0) and prev_base_x is not None:
        center = int(prev_base_x)
        half = continuity_half
    else:
        center = w // 2
        half = base_half
    lo, hi = max(0, center - half), min(w, center + half)
    band = column_sum[lo:hi]
    if band.size == 0 or band.max() <= 0:
        return {"fit": None, "base_x": None, "window_centers": [], "found": 0,
                "nwin": nwin}
    base_x = lo + int(np.argmax(band))

    nz = mask.nonzero()
    nz_y, nz_x = np.array(nz[0]), np.array(nz[1])
    fit, found, window_centers = _track_from_base(nz_x, nz_y, base_x, h, w, params)
    return {"fit": fit, "base_x": base_x, "window_centers": window_centers,
            "found": found, "nwin": nwin}


def _eval_fit(fit, y):
    a, b, c = fit
    return a * y * y + b * y + c


def analyze_lane(frame: np.ndarray, params: LaneParams,
                 m=None, minv=None, prev_base_x=None) -> LaneResult:
    """Full bird's-eye lane analysis on a (preprocessed) BGR frame.

    Pass a cached ``m``/``minv`` homography to avoid recomputing it every frame;
    otherwise it is derived from the current frame size.
    """
    h, w = frame.shape[:2]
    if m is None or minv is None:
        m, minv = compute_homography(params, w, h)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    warped_gray = _warp(gray, m, params)
    raw_mask = warped_black_mask(warped_gray, params)

    # Anti-zebra: drop the transversal dash row(s) BEFORE the histogram / sliding
    # window so the follower stays on the continuous (vertical) line and does not
    # lock onto the zebra as it approaches the intersection.
    mask, rows_rejected = reject_transverse_rows(raw_mask, params)

    fill_pct = 100.0 * float(cv2.countNonZero(mask)) / float(mask.size)
    result = LaneResult(detected=False, warped_mask=mask, warped_mask_raw=raw_mask,
                        fill_pct=fill_pct, zebra_rows_rejected=rows_rejected)
    if not (params.min_fill_pct <= fill_pct <= params.max_fill_pct):
        return result

    sw = (_dual_line_fit(mask, params) if getattr(params, "dual_line", 0)
          else _sliding_window_fit(mask, params, prev_base_x))
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

    # Second read further ahead (anticipation). The fit may extrapolate past the
    # windows that found pixels, which is exactly what predicts the upcoming bend;
    # clamp x into the warp so a wild extrapolation can't throw the controller.
    look_y = wh * params.lookahead_y_pct / 100.0
    far_x = float(np.clip(_eval_fit(fit, look_y), 0.0, ww))
    result.lane_center_far_x_orig = _birdseye_to_orig((far_x, look_y), minv)[0]

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
