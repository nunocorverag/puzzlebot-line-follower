#!/usr/bin/env python3
"""Validate perception/zebra.py over the zebra datasets (trigger + distance +
options). Dumps annotated wide-BEV composites and a per-category summary."""
import os
import sys
import glob

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import LaneParams, load_lane_params
from puzzlebot_ros.perception.zebra import (
    ZebraParams, wide_homography, analyze_zebra, draw_zebra_overlay,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    lp = load_lane_params(LANE_JSON, LaneParams())
    zp = ZebraParams()
    root = os.path.expanduser("~/zebra_eval")
    out_dir = os.path.join(root, "module_out")
    os.makedirs(out_dir, exist_ok=True)

    def leaf_of(cat):
        ls = [d for d, _, fs in os.walk(os.path.join(root, cat))
              if any(f.startswith("frame_") for f in fs)]
        return sorted(ls)[0]

    img0 = cv2.imread(sorted(glob.glob(os.path.join(leaf_of("zebra_recta"),
                                                    "frame_*.jpg")))[0])
    h, w = img0.shape[:2]
    M = wide_homography(lp, zp, w, h)

    dump = {"zebra_recta": [18, 24], "zebra_curva": [16, 24, 30],
            "zebra_interseccion": [6, 20, 46]}
    for cat in ["zebra_recta", "zebra_curva", "zebra_interseccion"]:
        leaf = leaf_of(cat)
        frames = sorted(glob.glob(os.path.join(leaf, "frame_*.jpg")))
        stable = nseen = nopt = 0
        first = None
        opt_hist = {}
        print(f"\n=== {cat} ===")
        for fp in frames:
            idx = int(os.path.basename(fp).split("_")[1].split(".")[0])
            und = cv2.undistort(cv2.imread(fp), K, D)
            r = analyze_zebra(und, lp, zp, M, stable)
            stable = r.stable_frames
            if r.seen:
                nseen += 1
                first = idx if first is None else first
                key = ",".join(r.options) or "-"
                opt_hist[key] = opt_hist.get(key, 0) + 1
                if r.options:
                    nopt += 1
            if idx in dump.get(cat, []):
                bev = cv2.warpPerspective(und, M, (zp.warp_w, zp.warp_h))
                cv2.imwrite(os.path.join(out_dir, f"{cat}_f{idx:03d}.jpg"),
                            draw_zebra_overlay(bev, r))
        print(f"  seen {nseen}/{len(frames)} | first idx={first} | "
              f"frames-with-options {nopt}")
        print(f"  option histogram (stable frames): {opt_hist}")


if __name__ == "__main__":
    main()
