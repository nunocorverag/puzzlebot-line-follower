#!/usr/bin/env python3
"""Offline check of the anti-zebra row-reject filter.

Runs analyze_lane over the real camera datasets (zebra_recta / zebra_curva /
zebra_interseccion) and dumps, per frame, a composite:

    [ undistorted + lane overlay | BEV mask RAW | BEV mask CLEAN ]

The RAW panel marks (red) the rows the filter would erase, the CLEAN panel is
the mask actually fed to the histogram / sliding window. Use it to confirm the
filter removes the transversal zebra bar while KEEPING the continuous line, and
that it is a no-op on the plain straight/curve.

Runs ON THE JETSON (laptop has no cv2). Outputs to ~/zebra_eval/zebra_filter_out.
"""
import os
import sys

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import (
    LaneParams, load_lane_params, compute_homography, analyze_lane,
    draw_lane_overlay, draw_birdseye_debug, reject_transverse_rows,
    warped_black_mask,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")


def _leaf(root, cat):
    ls = [d for d, _, fs in os.walk(os.path.join(root, cat))
          if any(f.startswith("frame_") for f in fs)]
    return sorted(ls)[0]


def _mask_panels(result, params):
    """RAW mask (rejected rows in red) and CLEAN mask, BGR, warp-sized."""
    raw = result.warped_mask_raw
    clean = result.warped_mask
    if raw is None:
        raw = clean
    raw_bgr = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    clean_bgr = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
    # Re-derive which rows are dense so we can paint them on the RAW panel.
    h, w = raw.shape[:2]
    thresh = max(1, int(w * params.zebra_row_fill_pct / 100.0))
    close_px = int(params.zebra_row_close_px)
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (close_px | 1, 1))
        measured = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, k)
    else:
        measured = raw
    dense = np.count_nonzero(measured, axis=1) >= thresh
    raw_bgr[dense] = (0, 0, 255)
    cv2.putText(raw_bgr, "RAW", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(clean_bgr, f"CLEAN rej={result.zebra_rows_rejected}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return raw_bgr, clean_bgr


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    lp = load_lane_params(LANE_JSON, LaneParams())
    print(f"zebra_row_reject={lp.zebra_row_reject} fill_pct={lp.zebra_row_fill_pct} "
          f"close_px={lp.zebra_row_close_px}")
    root = os.path.expanduser("~/zebra_eval")
    out = os.path.join(root, "zebra_filter_out")
    os.makedirs(out, exist_ok=True)

    seqs = {"zebra_recta": 32, "zebra_curva": 42, "zebra_interseccion": 53}
    M = Minv = None
    for cat, n in seqs.items():
        try:
            lf = _leaf(root, cat)
        except IndexError:
            print(f"!! no frames for {cat}")
            continue
        prev_base = None
        for idx in range(n):
            fp = os.path.join(lf, f"frame_{idx:05d}.jpg")
            img = cv2.imread(fp)
            if img is None:
                continue
            h, w = img.shape[:2]
            und = cv2.undistort(img, K, D)
            if M is None:
                M, Minv = compute_homography(lp, w, h)
            r = analyze_lane(und, lp, M, Minv, prev_base)
            prev_base = r.base_x if (r.detected and r.confidence >= 0.5) else None
            # Dump frames where the filter fires (the interesting ones) plus a
            # couple of plain references per category.
            interesting = r.zebra_rows_rejected > 0 or idx in (5, 15, 25)
            if not interesting:
                continue
            draw_lane_overlay(und, lp, r)
            raw_bgr, clean_bgr = _mask_panels(r, lp)
            s = und.shape[0] / float(raw_bgr.shape[0])
            raw_bgr = cv2.resize(raw_bgr, (int(raw_bgr.shape[1] * s), und.shape[0]))
            clean_bgr = cv2.resize(clean_bgr, (int(clean_bgr.shape[1] * s), und.shape[0]))
            comp = np.hstack([und, raw_bgr, clean_bgr])
            print(f"{cat} f{idx:03d}: detected={r.detected} off={r.offset_norm:+.2f} "
                  f"conf={r.confidence:.2f} curv={r.curvature_norm:+.2f} "
                  f"fill={r.fill_pct:.1f} rej_rows={r.zebra_rows_rejected} base_x={r.base_x}")
            cv2.imwrite(os.path.join(out, f"{cat}_f{idx:03d}.jpg"), comp)
    print(f"wrote composites to {out}")


if __name__ == "__main__":
    main()
