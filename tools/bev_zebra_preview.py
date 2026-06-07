#!/usr/bin/env python3
"""Warp zebra dataset frames to bird's-eye (matching runtime: undistort -> warp)
and dump composites [undistorted | BEV | BEV black-mask] so we can SEE whether the
zebra dashes become uniform/detectable in BEV and whether the warp is good enough.
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


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    params = load_lane_params(LANE_JSON, LaneParams())
    out_dir = os.path.expanduser("~/zebra_eval/bev_out")
    os.makedirs(out_dir, exist_ok=True)

    # representative frames per category (where the zebra is in view)
    picks = {
        "zebra_recta":       ["frame_00013", "frame_00018", "frame_00024"],
        "zebra_curva":       ["frame_00016", "frame_00024", "frame_00030"],
        "zebra_interseccion":["frame_00006", "frame_00020", "frame_00046"],
    }
    root = os.path.expanduser("~/zebra_eval")
    M = None
    for cat, names in picks.items():
        leaves = [d for d, _, fs in os.walk(os.path.join(root, cat))
                  if any(f.startswith("frame_") for f in fs)]
        if not leaves:
            print("missing", cat); continue
        leaf = sorted(leaves)[0]
        for nm in names:
            fp = os.path.join(leaf, nm + ".jpg")
            img = cv2.imread(fp)
            if img is None:
                print("no", fp); continue
            h, w = img.shape[:2]
            und = cv2.undistort(img, K, D)
            if M is None:
                M, _ = compute_homography(params, w, h)
            bev = _warp(und, M, params)
            gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
            mask = warped_black_mask(gray, params)
            mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            # resize undistorted to warp height for side-by-side
            wh = bev.shape[0]
            uw = int(und.shape[1] * wh / und.shape[0])
            und_r = cv2.resize(und, (uw, wh))
            comp = np.hstack([und_r, bev, mask3])
            op = os.path.join(out_dir, f"{cat}_{nm}.jpg")
            cv2.imwrite(op, comp)
            print("wrote", op, "bev", bev.shape)


if __name__ == "__main__":
    main()
