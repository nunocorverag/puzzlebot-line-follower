#!/usr/bin/env python3
"""Phase 1: measure the BEV ground scale (px per cm, X and Y) using the zebra
dashes as a physical ruler -- each dash is 2.2 cm (forward/Y) x 3.15 cm
(transverse/X) per the track measurements. We undistort + warp a few clean
RECTA frames (zebra near-horizontal, robot square to it), find the dash blobs in
the BEV mask, and report median dash bw/bh in BEV px -> px_per_cm.
"""
import os
import sys
import glob

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import (
    LaneParams, load_lane_params, compute_homography, _warp, warped_black_mask,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")

DASH_CM_Y = 2.2    # forward (along travel)
DASH_CM_X = 3.15   # transverse (long side)


def dash_blobs(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        x, y, bw, bh = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if bw < 4 or bh < 4:
            continue
        rect = area / float(bw * bh)
        asp = max(bw / float(bh), bh / float(bw))
        out.append((x, y, bw, bh, area, rect, asp))
    return out


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    params = load_lane_params(LANE_JSON, LaneParams())
    root = os.path.expanduser("~/zebra_eval/zebra_recta")
    leaves = [d for d, _, fs in os.walk(root) if any(f.startswith("frame_") for f in fs)]
    leaf = sorted(leaves)[0]
    frames = sorted(glob.glob(os.path.join(leaf, "frame_*.jpg")))

    M = None
    bws, bhs = [], []
    # frames 13-31 are where the recta zebra is cleanly in view (from the eval)
    for fp in frames:
        idx = int(os.path.basename(fp).split("_")[1].split(".")[0])
        if not (13 <= idx <= 31):
            continue
        img = cv2.imread(fp)
        h, w = img.shape[:2]
        und = cv2.undistort(img, K, D)
        if M is None:
            M, _ = compute_homography(params, w, h)
        bev = _warp(und, M, params)
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        mask = warped_black_mask(gray, params)
        blobs = dash_blobs(mask)
        # keep compact, dash-like blobs (reject the big lane stripe / seams)
        for (x, y, bw, bh, area, rect, asp) in blobs:
            if rect >= 0.55 and asp <= 2.2 and 20 <= area <= 2000 and bh <= 40 and bw <= 60:
                bws.append(bw); bhs.append(bh)

    if not bws:
        print("no dash blobs found"); return
    bw_med = float(np.median(bws)); bh_med = float(np.median(bhs))
    ppc_x = bw_med / DASH_CM_X
    ppc_y = bh_med / DASH_CM_Y
    print(f"samples={len(bws)}")
    print(f"dash BEV px: bw_med={bw_med:.1f}  bh_med={bh_med:.1f}")
    print(f"px_per_cm_X (transverse) = {ppc_x:.2f}   (bw {bw_med:.1f}px / {DASH_CM_X}cm)")
    print(f"px_per_cm_Y (forward)    = {ppc_y:.2f}   (bh {bh_med:.1f}px / {DASH_CM_Y}cm)")
    print(f"px_per_cm_x10 (X, for lane.py offset_cm) = {round(ppc_x*10)}")
    print(f"warp coverage: W={params.warp_w}px ~= {params.warp_w/ppc_x:.1f}cm  "
          f"H={params.warp_h}px ~= {params.warp_h/ppc_y:.1f}cm")


if __name__ == "__main__":
    main()
