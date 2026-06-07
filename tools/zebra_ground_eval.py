#!/usr/bin/env python3
"""Phase 2 (offline prototype): robust zebra detection in GROUND coordinates.

Pipeline (no narrow-warp clipping -- we project POINTS, not the image):
  1. undistort (like runtime)
  2. black mask on the lower region only (skip horizon clutter)
  3. find dark blobs; project each blob's bbox to BEV px -> ground cm via M
  4. keep dash-sized blobs (filtered by REAL cm size -> distance-robust)
  5. RANSAC a transverse line through the dash centroids -> the zebra row
  6. report: detected?, distance-to-row at lane center (cm), row angle (skew),
     and L/S/R option presence beyond the row (ground buckets)

Threads a debounce count like the live node. Dumps annotated composites.
"""
import os
import sys
import glob
import math

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import (
    LaneParams, load_lane_params, compute_homography,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")

# --- ground scale (phase 1) ---
PPC_X = 7.94    # BEV px per cm, transverse
PPC_Y = 5.68    # BEV px per cm, forward

# --- dash physical size (track measurements) ---
DASH_CM_Y = 2.2     # forward
DASH_CM_X = 3.15    # transverse
DASH_AREA_CM = DASH_CM_X * DASH_CM_Y

# --- detector params (cm-based) ---
SEARCH_Y0_PCT = 45         # ignore image above this (horizon/people)
MIN_DASH_CM = 0.9          # accept blob if min side >= this
MAX_DASH_CM = 6.0          # and max side <= this
MIN_AREA_CM = 2.0
MAX_AREA_CM = 22.0
ROW_TOL_CM = 1.8           # RANSAC perpendicular inlier tolerance
MIN_DASHES = 3             # dashes to call it a zebra row
MIN_SPAN_CM = 7.0          # lateral span of the row
MAX_FWD_CM = 70.0          # ignore rows farther than this
STABLE_NEEDED = 3
STOP_CM = 10.0             # stop when row this close
OPT_SIDE_CM = 6.0          # |X| beyond this = left/right bucket
OPT_MARGIN_CM = 4.0        # look this far beyond the row for exits


def black_mask_lower(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 1.4)
    _, m = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    h = m.shape[0]
    m[: int(h * SEARCH_Y0_PCT / 100.0), :] = 0
    return m


def to_ground(pts_img, M, warp_w, warp_h):
    """img px -> ground cm. Returns (X_cm lateral from center, Y_cm forward)."""
    pts = np.array(pts_img, dtype=np.float32).reshape(-1, 1, 2)
    bev = cv2.perspectiveTransform(pts, M).reshape(-1, 2)
    X = (bev[:, 0] - warp_w / 2.0) / PPC_X
    Y = (warp_h - bev[:, 1]) / PPC_Y
    return np.stack([X, Y], axis=1)


def dash_candidates(frame, M, warp_w, warp_h):
    mask = black_mask_lower(frame)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []  # (Xcm, Ycm, longcm, shortcm, img_cxy)
    h, w = mask.shape[:2]
    for c in cnts:
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 4 or bh < 4:
            continue
        if x <= 1 or x + bw >= w - 1:   # clipped at border = cable/edge
            continue
        cx, cy = x + bw / 2.0, y + bh / 2.0
        corners = [(x, y), (x + bw, y), (x + bw, y + bh), (x, y + bh)]
        g = to_ground(corners + [(cx, cy)], M, warp_w, warp_h)
        gc = g[4]
        if gc[1] <= 0 or gc[1] > MAX_FWD_CM:
            continue
        # ground size from projected quad
        side1 = math.hypot(*(g[1] - g[0]))
        side2 = math.hypot(*(g[2] - g[1]))
        longc, shortc = max(side1, side2), min(side1, side2)
        areac = longc * shortc
        if shortc < MIN_DASH_CM or longc > MAX_DASH_CM:
            continue
        if areac < MIN_AREA_CM or areac > MAX_AREA_CM:
            continue
        cands.append((gc[0], gc[1], longc, shortc, (cx, cy)))
    return cands, mask


def ransac_row(cands):
    """Find the transverse line (in ground cm) with the most dash inliers."""
    pts = np.array([(c[0], c[1]) for c in cands], dtype=np.float64)
    n = len(pts)
    if n < MIN_DASHES:
        return [], None
    best = []
    best_dir = None
    for i in range(n):
        for j in range(i + 1, n):
            d = pts[j] - pts[i]
            L = math.hypot(*d)
            if L < 1e-3:
                continue
            ux, uy = d / L
            # require a mostly-TRANSVERSE line (spans X more than Y) so we don't
            # latch onto the lane (which runs forward in Y)
            if abs(ux) < abs(uy):
                continue
            nx, ny = -uy, ux              # normal
            c0 = nx * pts[i, 0] + ny * pts[i, 1]
            dist = np.abs(pts[:, 0] * nx + pts[:, 1] * ny - c0)
            inl = [k for k in range(n) if dist[k] <= ROW_TOL_CM]
            if len(inl) > len(best):
                best = inl
                best_dir = (ux, uy)
    return best, best_dir


def classify_options(cands, inl_set, row_y, lane_x=0.0):
    left = straight = right = 0
    for k, c in enumerate(cands):
        if k in inl_set:
            continue
        X, Y = c[0], c[1]
        if Y < row_y - 1.0:          # only stuff at/beyond the row
            continue
        dx = X - lane_x
        if dx < -OPT_SIDE_CM:
            left += 1
        elif dx > OPT_SIDE_CM:
            right += 1
        else:
            straight += 1
    opts = []
    if left >= 1:
        opts.append("left")
    if straight >= 1:
        opts.append("straight")
    if right >= 1:
        opts.append("right")
    return opts, (left, straight, right)


def detect(frame, M, params):
    warp_w, warp_h = params.warp_w, params.warp_h
    cands, mask = dash_candidates(frame, M, warp_w, warp_h)
    inl, dir_ = ransac_row(cands)
    res = dict(seen=False, dist_cm=None, angle_deg=None, ndash=len(inl),
               span_cm=0.0, options=[], ncands=len(cands))
    if len(inl) < MIN_DASHES:
        return res, cands, mask, inl
    xs = np.array([cands[k][0] for k in inl])
    ys = np.array([cands[k][1] for k in inl])
    span = float(xs.max() - xs.min())
    if span < MIN_SPAN_CM:
        return res, cands, mask, inl
    # distance to row at lane center (X=0): use line y = m*x + b
    if dir_ is not None and abs(dir_[0]) > 1e-6:
        m = dir_[1] / dir_[0]
        b = float(np.median(ys - m * xs))
        dist = b                      # y at x=0
        angle = math.degrees(math.atan2(dir_[1], dir_[0]))
    else:
        dist = float(np.median(ys)); angle = 0.0
    opts, _counts = classify_options(cands, set(inl), float(np.median(ys)))
    res.update(seen=True, dist_cm=dist, angle_deg=angle, span_cm=span, options=opts)
    return res, cands, mask, inl


def annotate(frame, cands, inl, res):
    out = frame.copy()
    inl_set = set(inl)
    for k, c in enumerate(cands):
        px = (int(c[4][0]), int(c[4][1]))
        col = (0, 0, 255) if k in inl_set else (0, 200, 255)
        cv2.circle(out, px, 5, col, -1)
    txt = (f"seen={int(res['seen'])} dist={res['dist_cm'] if res['dist_cm'] is None else round(res['dist_cm'],1)}cm "
           f"ang={res['angle_deg'] if res['angle_deg'] is None else round(res['angle_deg'],0)} "
           f"nd={res['ndash']} span={round(res['span_cm'],1)} opt={','.join(res['options']) or '-'}")
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(out, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    params = load_lane_params(LANE_JSON, LaneParams())
    out_dir = os.path.expanduser("~/zebra_eval/zebra_ground_out")
    os.makedirs(out_dir, exist_ok=True)
    root = os.path.expanduser("~/zebra_eval")
    cats = ["zebra_recta", "zebra_curva", "zebra_interseccion"]
    M = None
    dump = {"zebra_recta": [13, 18, 24], "zebra_curva": [16, 24, 30],
            "zebra_interseccion": [6, 20, 46]}
    for cat in cats:
        leaves = [d for d, _, fs in os.walk(os.path.join(root, cat))
                  if any(f.startswith("frame_") for f in fs)]
        if not leaves:
            continue
        leaf = sorted(leaves)[0]
        frames = sorted(glob.glob(os.path.join(leaf, "frame_*.jpg")))
        stable = 0
        n_seen = n_stable = 0
        first = None
        print(f"\n=== {cat} ===")
        for fp in frames:
            idx = int(os.path.basename(fp).split("_")[1].split(".")[0])
            img = cv2.imread(fp)
            h, w = img.shape[:2]
            und = cv2.undistort(img, K, D)
            if M is None:
                M, _ = compute_homography(params, w, h)
            res, cands, mask, inl = detect(und, M, params)
            stable = stable + 1 if res["seen"] else 0
            stable_ok = stable >= STABLE_NEEDED
            if res["seen"]:
                n_seen += 1
                if first is None:
                    first = idx
            if stable_ok:
                n_stable += 1
            d = "  -- " if res["dist_cm"] is None else f"{res['dist_cm']:5.1f}"
            a = "  -- " if res["angle_deg"] is None else f"{res['angle_deg']:+5.0f}"
            print(f"  f{idx:03d} seen={int(res['seen'])} stbl={int(stable_ok)} "
                  f"dist={d}cm ang={a} nd={res['ndash']:2d} span={res['span_cm']:5.1f} "
                  f"cands={res['ncands']:2d} opt={','.join(res['options']) or '-'}")
            if idx in dump.get(cat, []):
                cv2.imwrite(os.path.join(out_dir, f"{cat}_f{idx:03d}.jpg"),
                            annotate(und, cands, inl, res))
        print(f"  >> seen {n_seen}/{len(frames)} | stable {n_stable} | first idx={first}")


if __name__ == "__main__":
    main()
