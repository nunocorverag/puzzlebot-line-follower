#!/usr/bin/env python3
"""Phase 2b: zebra detection on a DEDICATED WIDE bird's-eye warp.

Why wide: the lane warp trapezoid is narrow and clips a curve-approach zebra. We
scale the SAME trapezoid outward about the image center by KX (preserves the
vanishing point -> straight lines stay vertical, dashes stay uniform) so we see
~KX wider ground. Detect dash blobs directly in this rectified BEV (uniform size
-> trivial filtering), auto-calibrate px/cm from recta dashes, then RANSAC a
transverse row -> distance (cm) + skew angle + L/S/R options.
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
    LaneParams, load_lane_params, src_points, warped_black_mask,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")

KX = 2.4                 # widen factor about image center
WARP_W = 720             # wide BEV canvas
WARP_H = 600
DASH_CM_Y, DASH_CM_X = 2.2, 3.15

# detector params (cm)
MIN_SHORT_CM, MAX_LONG_CM = 0.9, 7.0
MIN_AREA_CM, MAX_AREA_CM = 2.0, 22.0
ROW_TOL_CM = 2.0
MIN_DASHES = 3
MIN_SPAN_CM = 7.0
MAX_FWD_CM = 80.0
OPT_SIDE_CM = 6.0


def wide_homography(params, w, h):
    src = src_points(params, w, h).astype(np.float64)
    cx = w / 2.0
    src_wide = src.copy()
    src_wide[:, 0] = cx + (src[:, 0] - cx) * KX
    dst = np.array([[0, 0], [WARP_W, 0], [WARP_W, WARP_H], [0, WARP_H]],
                   dtype=np.float64)
    # src_points order: TL, TR, BR, BL  (match dst)
    M = cv2.getPerspectiveTransform(src_wide.astype(np.float32),
                                    dst.astype(np.float32))
    return M


def bev_blobs(frame, K, D, M, params):
    und = cv2.undistort(frame, K, D)
    bev = cv2.warpPerspective(und, M, (WARP_W, WARP_H))
    gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
    mask = warped_black_mask(gray, params)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in cnts:
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 4 or bh < 4:
            continue
        area = cv2.contourArea(c)
        rect = area / float(bw * bh)
        blobs.append((x, y, bw, bh, area, rect))
    return blobs, bev, mask


def calibrate_scale(K, D, M, params, recta_leaf):
    bws, bhs = [], []
    for fp in sorted(glob.glob(os.path.join(recta_leaf, "frame_*.jpg"))):
        idx = int(os.path.basename(fp).split("_")[1].split(".")[0])
        if not (13 <= idx <= 31):
            continue
        blobs, _, _ = bev_blobs(cv2.imread(fp), K, D, M, params)
        for (x, y, bw, bh, area, rect) in blobs:
            if rect >= 0.5 and 0.4 <= bw / float(bh) <= 3.0 and 30 <= area <= 3000:
                bws.append(bw); bhs.append(bh)
    bw_med, bh_med = float(np.median(bws)), float(np.median(bhs))
    return bw_med / DASH_CM_X, bh_med / DASH_CM_Y, len(bws)


def to_cm(blobs, ppc_x, ppc_y):
    out = []
    for (x, y, bw, bh, area, rect) in blobs:
        cx, cy = x + bw / 2.0, y + bh / 2.0
        Xcm = (cx - WARP_W / 2.0) / ppc_x
        Ycm = (WARP_H - cy) / ppc_y
        long_cm = max(bw / ppc_x, bh / ppc_y)
        short_cm = min(bw / ppc_x, bh / ppc_y)
        out.append((Xcm, Ycm, long_cm, short_cm, (cx, cy)))
    return out


def dash_filter(cm):
    return [c for c in cm
            if c[3] >= MIN_SHORT_CM and c[2] <= MAX_LONG_CM
            and MIN_AREA_CM <= c[2] * c[3] <= MAX_AREA_CM
            and 0 < c[1] <= MAX_FWD_CM]


def ransac_row(cands):
    pts = np.array([(c[0], c[1]) for c in cands], dtype=np.float64)
    n = len(pts)
    if n < MIN_DASHES:
        return [], None
    best, best_dir = [], None
    for i in range(n):
        for j in range(i + 1, n):
            d = pts[j] - pts[i]
            L = math.hypot(*d)
            if L < 1e-3 or abs(d[0]) < abs(d[1]):
                continue
            ux, uy = d / L
            nx, ny = -uy, ux
            c0 = nx * pts[i, 0] + ny * pts[i, 1]
            dist = np.abs(pts[:, 0] * nx + pts[:, 1] * ny - c0)
            inl = [k for k in range(n) if dist[k] <= ROW_TOL_CM]
            if len(inl) > len(best):
                best, best_dir = inl, (ux, uy)
    return best, best_dir


def options(cands, inl, row_y):
    L = S = R = 0
    for k, c in enumerate(cands):
        if k in inl or c[1] < row_y - 1.0:
            continue
        if c[0] < -OPT_SIDE_CM:
            L += 1
        elif c[0] > OPT_SIDE_CM:
            R += 1
        else:
            S += 1
    o = []
    if L: o.append("left")
    if S: o.append("straight")
    if R: o.append("right")
    return o


def detect(frame, K, D, M, params, ppc_x, ppc_y):
    blobs, bev, mask = bev_blobs(frame, K, D, M, params)
    cands = dash_filter(to_cm(blobs, ppc_x, ppc_y))
    inl, dir_ = ransac_row(cands)
    r = dict(seen=False, dist=None, ang=None, nd=len(inl), span=0.0,
             opt=[], nc=len(cands))
    if len(inl) >= MIN_DASHES:
        xs = np.array([cands[k][0] for k in inl])
        ys = np.array([cands[k][1] for k in inl])
        span = float(xs.max() - xs.min())
        if span >= MIN_SPAN_CM:
            if dir_ and abs(dir_[0]) > 1e-6:
                m = dir_[1] / dir_[0]
                dist = float(np.median(ys - m * xs))
                ang = math.degrees(math.atan2(dir_[1], dir_[0]))
            else:
                dist, ang = float(np.median(ys)), 0.0
            r.update(seen=True, dist=dist, ang=ang, span=span,
                     opt=options(cands, set(inl), float(np.median(ys))))
    return r, cands, inl, bev, mask


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    params = load_lane_params(LANE_JSON, LaneParams())
    root = os.path.expanduser("~/zebra_eval")
    out_dir = os.path.join(root, "wide_out")
    os.makedirs(out_dir, exist_ok=True)

    def leaf_of(cat):
        ls = [d for d, _, fs in os.walk(os.path.join(root, cat))
              if any(f.startswith("frame_") for f in fs)]
        return sorted(ls)[0]

    img0 = cv2.imread(sorted(glob.glob(os.path.join(leaf_of("zebra_recta"),
                                                    "frame_*.jpg")))[0])
    h, w = img0.shape[:2]
    M = wide_homography(params, w, h)
    ppc_x, ppc_y, ns = calibrate_scale(K, D, M, params, leaf_of("zebra_recta"))
    print(f"[scale] px/cm X={ppc_x:.2f} Y={ppc_y:.2f} (n={ns}) | "
          f"BEV {WARP_W}x{WARP_H} ~= {WARP_W/ppc_x:.0f}x{WARP_H/ppc_y:.0f} cm")

    dump = {"zebra_recta": [13, 18, 24], "zebra_curva": [16, 24, 30],
            "zebra_interseccion": [6, 20, 46]}
    for cat in ["zebra_recta", "zebra_curva", "zebra_interseccion"]:
        leaf = leaf_of(cat)
        frames = sorted(glob.glob(os.path.join(leaf, "frame_*.jpg")))
        stable = nseen = nstab = 0
        first = None
        print(f"\n=== {cat} ===")
        for fp in frames:
            idx = int(os.path.basename(fp).split("_")[1].split(".")[0])
            r, cands, inl, bev, mask = detect(cv2.imread(fp), K, D, M, params,
                                              ppc_x, ppc_y)
            stable = stable + 1 if r["seen"] else 0
            sok = stable >= 3
            if r["seen"]:
                nseen += 1
                first = idx if first is None else first
            if sok:
                nstab += 1
            d = "  --" if r["dist"] is None else f"{r['dist']:5.1f}"
            a = "  --" if r["ang"] is None else f"{r['ang']:+5.0f}"
            print(f"  f{idx:03d} seen={int(r['seen'])} stbl={int(sok)} "
                  f"dist={d}cm ang={a} nd={r['nd']:2d} span={r['span']:5.1f} "
                  f"nc={r['nc']:2d} opt={','.join(r['opt']) or '-'}")
            if idx in dump.get(cat, []):
                ann = bev.copy()
                for k, c in enumerate(cands):
                    p = (int(c[4][0]), int(c[4][1]))
                    cv2.circle(ann, p, 6, (0, 0, 255) if k in set(inl)
                               else (0, 200, 255), -1)
                comp = np.hstack([ann, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
                cv2.imwrite(os.path.join(out_dir, f"{cat}_f{idx:03d}.jpg"), comp)
        print(f"  >> seen {nseen}/{len(frames)} | stable {nstab} | first={first}")


if __name__ == "__main__":
    main()
