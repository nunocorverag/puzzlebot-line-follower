"""Robust zebra / intersection detector in GROUND coordinates (cm).

Why this exists
---------------
The legacy detector in ``intersection.py`` reads the zebra geometry on the RAW,
perspective-distorted image with ROIs fixed in image-percent. On a curve approach
the zebra arrives tilted / laterally offset, so the entry-line fit, the "centered"
gate and the option ROIs all break and the robot drives through the cross. Proven
offline on datasets/zebra_{recta,curva,interseccion}: the trigger fired but
``dashed_detected`` held only ~6/42 (curva) and ~6/53 (interseccion).

Approach (validated offline)
----------------------------
Warp to a DEDICATED WIDE bird's-eye view: the lane-following warp trapezoid is too
narrow and clips a curve-approach zebra, so we scale that trapezoid outward about
the image center by ``widen_kx`` -- this keeps the same vanishing point (straight
lines stay vertical, dashes stay uniform) while covering ~kx wider ground. In that
rectified view the dashes become near-uniform rectangles, so we:

  1. find dark blobs, convert each to ground cm (px/cm calibrated from the 2.2 x
     3.15 cm dash used as a ruler),
  2. keep dash-sized blobs (filter in cm -> distance-robust, one threshold set),
  3. RANSAC a *transverse* line through the dash centroids -> the zebra row,
  4. report distance-to-row at the lane center (cm) and the row skew angle. The
     distance is pose-independent, so the robot can slow + STOP the same way out of
     a straight or a curve.

The module is ROS-free (mirrors lane.py / intersection.py) so it can run in the
node and in offline tools alike.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields
from pathlib import Path

import cv2
import numpy as np

from .lane import LaneParams, src_points, warped_black_mask

# Physical dash size (track measurements, cm). Used as the ground ruler.
DASH_CM_FWD = 2.2     # along travel
DASH_CM_LAT = 3.15    # transverse (long side)


@dataclass
class ZebraParams:
    """All zebra-detector tunables. cm-based so they are pose/scale independent."""

    # --- wide warp (relative to the lane warp trapezoid) ---
    widen_kx: float = 2.4          # scale lane trapezoid outward about image center
    warp_w: int = 720
    warp_h: int = 600
    # --- ground scale: BEV px per cm (calibrate with measure_ground_scale) ---
    px_per_cm_x: float = 11.1      # transverse
    px_per_cm_y: float = 23.6      # forward
    # --- dash blob filter (cm) ---
    min_short_cm: float = 0.9
    max_long_cm: float = 7.0
    min_area_cm: float = 2.0
    max_area_cm: float = 22.0
    # --- transverse-row fit ---
    row_tol_cm: float = 2.0        # RANSAC perpendicular inlier tolerance
    min_dashes: int = 3            # inliers to call it a zebra row
    min_span_cm: float = 7.0       # lateral span of the row (~ lane width)
    max_fwd_cm: float = 80.0       # ignore rows farther than this
    # --- options (exit) classification ---
    opt_side_cm: float = 6.0       # |X| beyond this = left/right bucket
    opt_margin_cm: float = 2.0     # look beyond row + this for exits
    opt_min_dashes: int = 2        # dashes needed in a side bucket
    opt_min_span_cm: float = 4.0   # left/right need lateral spread (reject lane-edge columns)
    option_align_deg: float = 18.0  # ONLY read options when the row is within this
                                   # tilt (robot ~square to the cross). A skewed
                                   # approach (coming off a curve) gives garbage
                                   # exits, so we refuse to guess until aligned.
    # --- debounce / motion ---
    stable_frames_needed: int = 3
    slow_distance_cm: float = 30.0  # start slowing when row within this
    stop_distance_cm: float = 10.0  # stop when row within this


@dataclass
class ZebraResult:
    seen: bool = False                     # debounced trigger
    raw_seen: bool = False                 # this-frame trigger (pre-debounce)
    stable_frames: int = 0
    distance_cm: float | None = None       # forward distance to row at lane center
    angle_deg: float | None = None         # row tilt vs transverse (robot skew)
    n_dashes: int = 0
    span_cm: float = 0.0
    options: list = field(default_factory=list)
    n_candidates: int = 0
    # for overlay / debug (all in wide-BEV pixel space)
    dash_px: list = field(default_factory=list)        # [(x,y), ...] all candidates
    row_inlier_idx: list = field(default_factory=list)  # indices into dash_px
    dash_bucket: list = field(default_factory=list)     # per-candidate: row/left/
                                                        # straight/right/none
    option_debug: dict = field(default_factory=dict)    # per-option accept/reject
                                                        # reason (why it offered X)


# --------------------------------------------------------------------------- #
# persistence (same style as lane.py / intersection.py)
# --------------------------------------------------------------------------- #
def save_zebra_params(params: ZebraParams, path) -> None:
    data = {f.name: getattr(params, f.name) for f in fields(params)}
    Path(path).write_text(json.dumps(data, indent=2))


def load_zebra_params(path, base: ZebraParams | None = None) -> ZebraParams:
    params = base or ZebraParams()
    p = Path(path)
    if not p.exists():
        return params
    data = json.loads(p.read_text())
    for f in fields(params):
        if f.name in data:
            setattr(params, f.name, type(getattr(params, f.name))(data[f.name]))
    return params


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
def wide_homography(lane_params: LaneParams, zparams: ZebraParams, w: int, h: int):
    """Wide bird's-eye homography: the lane trapezoid scaled out about cx by kx.

    Scaling both top and bottom half-widths by the same factor keeps the sides on
    lines through the same vanishing point, so the result is still a valid ground
    rectification -- just covering a wider lateral strip.
    """
    src = src_points(lane_params, w, h).astype(np.float64)
    cx = w / 2.0
    src[:, 0] = cx + (src[:, 0] - cx) * zparams.widen_kx
    dst = np.array([[0, 0], [zparams.warp_w, 0],
                    [zparams.warp_w, zparams.warp_h], [0, zparams.warp_h]],
                   dtype=np.float64)
    return cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))


def _bev_blobs(bev_gray, lane_params: LaneParams):
    mask = warped_black_mask(bev_gray, lane_params)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in cnts:
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 4 or bh < 4:
            continue
        blobs.append((x, y, bw, bh))
    return blobs, mask


def _to_cm(blobs, zp: ZebraParams):
    """Wide-BEV bbox -> (Xcm lateral from center, Ycm forward, long_cm, short_cm, (px,py))."""
    out = []
    for (x, y, bw, bh) in blobs:
        cx, cy = x + bw / 2.0, y + bh / 2.0
        Xcm = (cx - zp.warp_w / 2.0) / zp.px_per_cm_x
        Ycm = (zp.warp_h - cy) / zp.px_per_cm_y
        long_cm = max(bw / zp.px_per_cm_x, bh / zp.px_per_cm_y)
        short_cm = min(bw / zp.px_per_cm_x, bh / zp.px_per_cm_y)
        out.append((Xcm, Ycm, long_cm, short_cm, (cx, cy)))
    return out


def _dash_filter(cm, zp: ZebraParams):
    keep = []
    for c in cm:
        Xcm, Ycm, long_cm, short_cm, _ = c
        if short_cm < zp.min_short_cm or long_cm > zp.max_long_cm:
            continue
        if not (zp.min_area_cm <= long_cm * short_cm <= zp.max_area_cm):
            continue
        if not (0 < Ycm <= zp.max_fwd_cm):
            continue
        keep.append(c)
    return keep


def _ransac_transverse_row(cands, zp: ZebraParams):
    """Best transverse line (spans X more than Y) through the dash centroids."""
    pts = np.array([(c[0], c[1]) for c in cands], dtype=np.float64)
    n = len(pts)
    if n < zp.min_dashes:
        return [], None
    best, best_dir = [], None
    for i in range(n):
        for j in range(i + 1, n):
            d = pts[j] - pts[i]
            L = math.hypot(d[0], d[1])
            if L < 1e-3 or abs(d[0]) < abs(d[1]):     # must be mostly transverse
                continue
            ux, uy = d[0] / L, d[1] / L
            nx, ny = -uy, ux
            c0 = nx * pts[i, 0] + ny * pts[i, 1]
            dist = np.abs(pts[:, 0] * nx + pts[:, 1] * ny - c0)
            inl = [k for k in range(n) if dist[k] <= zp.row_tol_cm]
            if len(inl) > len(best):
                best, best_dir = inl, (ux, uy)
    return best, best_dir


def _classify_options(cands, inl_set, row_y, zp: ZebraParams, row_x=0.0):
    """Exits beyond the row, bucketed L/S/R. A real side exit is a TRANSVERSE row
    (spreads in X, shallow in Y); a longitudinal lane-edge column (spreads in Y at
    ~constant X) is rejected so it is not mistaken for a turn.

    ``row_x`` is the lateral center of the detected cross (median X of the row
    inliers). Buckets are measured RELATIVE to it, not to the robot, so a robot
    that stops off-center does not misread a real left/right exit as "straight".

    Returns (options, bucket_of_index, reasons) so the overlay can SHOW which dash
    went to which bucket and WHY each exit was offered or rejected.
    """
    buckets = {"left": [], "straight": [], "right": []}
    bucket_of = ["none"] * len(cands)
    for k, c in enumerate(cands):
        if k in inl_set:
            bucket_of[k] = "row"
            continue
        if c[1] < row_y + zp.opt_margin_cm:
            continue
        X, Y = c[0] - row_x, c[1]   # X relative to the cross center
        name = ("left" if X < -zp.opt_side_cm
                else "right" if X > zp.opt_side_cm else "straight")
        buckets[name].append((X, Y))
        bucket_of[k] = name
    opts, reasons = [], {}
    for name in ("left", "straight", "right"):
        pts = buckets[name]
        if len(pts) < zp.opt_min_dashes:
            reasons[name] = f"reject: {len(pts)}<{zp.opt_min_dashes} dashes"
            continue
        if name in ("left", "right"):
            x_span = max(p[0] for p in pts) - min(p[0] for p in pts)
            y_span = max(p[1] for p in pts) - min(p[1] for p in pts)
            if x_span < zp.opt_min_span_cm or x_span < y_span:
                reasons[name] = (f"reject: not transverse "
                                 f"(xspan{x_span:.0f}<{zp.opt_min_span_cm:.0f} "
                                 f"or <yspan{y_span:.0f})")
                continue
        opts.append(name)
        reasons[name] = f"OK: {len(pts)} dashes"
    return opts, bucket_of, reasons


def analyze_zebra(frame_undistorted, lane_params: LaneParams, zp: ZebraParams,
                  M, stable_frames: int) -> ZebraResult:
    """One frame -> ZebraResult. ``M`` is the wide homography (cache it). Thread
    ``stable_frames`` across calls like the other detectors."""
    bev = cv2.warpPerspective(frame_undistorted, M, (zp.warp_w, zp.warp_h))
    gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
    blobs, _mask = _bev_blobs(gray, lane_params)
    cands = _dash_filter(_to_cm(blobs, zp), zp)
    inl, dir_ = _ransac_transverse_row(cands, zp)

    res = ZebraResult(n_candidates=len(cands),
                      dash_px=[c[4] for c in cands], n_dashes=len(inl))
    raw = False
    if len(inl) >= zp.min_dashes:
        xs = np.array([cands[k][0] for k in inl])
        ys = np.array([cands[k][1] for k in inl])
        span = float(xs.max() - xs.min())
        if span >= zp.min_span_cm:
            raw = True
            if dir_ and abs(dir_[0]) > 1e-6:
                slope = dir_[1] / dir_[0]
                dist = float(np.median(ys - slope * xs))       # y at lane center x=0
                ang = math.degrees(math.atan2(dir_[1], dir_[0]))
                ang = (ang + 90) % 180 - 90                     # normalize to [-90,90]
            else:
                dist, ang = float(np.median(ys)), 0.0
            res.distance_cm = dist
            res.angle_deg = ang
            res.span_cm = span
            res.row_inlier_idx = list(inl)
            # Only trust the L/S/R read when the robot is roughly square to the
            # cross; a skewed row (off a curve) makes the buckets meaningless.
            if abs(ang) <= zp.option_align_deg:
                opts, bucket_of, reasons = _classify_options(
                    cands, set(inl), float(np.median(ys)), zp,
                    row_x=float(np.median(xs)))
                res.options = opts
                res.dash_bucket = bucket_of
                res.option_debug = reasons
            else:
                res.options = []
                res.dash_bucket = ["row" if k in set(inl) else "none"
                                   for k in range(len(cands))]
                res.option_debug = {
                    "_gate": f"skewed a={ang:.0f} > {zp.option_align_deg:.0f} deg"}

    res.raw_seen = raw
    res.stable_frames = stable_frames + 1 if raw else 0
    res.seen = res.stable_frames >= zp.stable_frames_needed
    return res


# Per-bucket dash colors (BGR): entry row + the three exit buckets, so a snapshot
# shows exactly which dashes drove each option.
_BUCKET_COLORS = {
    "row": (0, 0, 255),        # red   = entry zebra row
    "left": (255, 0, 255),     # magenta
    "straight": (255, 255, 0), # cyan
    "right": (255, 160, 0),    # blue
    "none": (0, 200, 255),     # orange = ignored (too close / not an exit)
}


def draw_zebra_overlay(bev, result: ZebraResult):
    """Annotate the wide-BEV: dashes colored by which bucket they fell in (entry
    row / left / straight / right / ignored) plus WHY each exit was offered or
    rejected -- so a recorded snapshot explains the decision on its own."""
    out = bev.copy()
    for k, (px, py) in enumerate(result.dash_px):
        b = result.dash_bucket[k] if k < len(result.dash_bucket) else "none"
        cv2.circle(out, (int(px), int(py)), 6, _BUCKET_COLORS.get(b, (0, 200, 255)), -1)
    d = "--" if result.distance_cm is None else f"{result.distance_cm:.0f}"
    a = "--" if result.angle_deg is None else f"{result.angle_deg:+.0f}"
    cv2.rectangle(out, (0, 0), (out.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(out, f"seen={int(result.seen)} d={d}cm a={a} nd={result.n_dashes} "
                f"opt={','.join(result.options) or '-'}",
                (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    # Why each option was offered/rejected (and the alignment gate).
    y = 44
    for name in ("_gate", "left", "straight", "right"):
        if name in result.option_debug:
            txt = f"{name}: {result.option_debug[name]}"
            col = (0, 255, 0) if result.option_debug[name].startswith("OK") else (180, 180, 255)
            cv2.putText(out, txt, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            y += 18
    # Legend
    cv2.putText(out, "row=red L=mag S=cyan R=blue ign=org",
                (6, out.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)
    return out
