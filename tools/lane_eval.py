#!/usr/bin/env python3
"""Run analyze_lane over curve/straight frames and dump the bird's-eye debug
(mask + sliding windows + fit) so we can SEE where the follower locks on and
whether it drifts to the wrong (right) line in a curve."""
import os
import sys
import glob

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import (
    LaneParams, load_lane_params, compute_homography, analyze_lane,
    draw_lane_overlay, draw_birdseye_debug,
)

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    lp = load_lane_params(LANE_JSON, LaneParams())
    root = os.path.expanduser("~/zebra_eval")
    out = os.path.join(root, "lane_out")
    os.makedirs(out, exist_ok=True)

    def leaf(cat):
        ls = [d for d, _, fs in os.walk(os.path.join(root, cat))
              if any(f.startswith("frame_") for f in fs)]
        return sorted(ls)[0]

    # Run EVERY frame in sequence (threading prev_base for continuity); dump the
    # picked ones. This mirrors the live node where base flows frame to frame.
    seqs = {"zebra_recta": range(0, 32), "zebra_curva": range(0, 42)}
    picks = {"zebra_recta": [5, 10], "zebra_curva": [3, 8, 12, 16, 20]}
    M = Minv = None
    for cat, idxs in seqs.items():
        lf = leaf(cat)
        prev_base = None
        for idx in idxs:
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
            if idx not in picks[cat]:
                continue
            draw_lane_overlay(und, lp, r)
            bev = draw_birdseye_debug(r, lp) if r.warped_mask is not None else None
            if bev is not None:
                s = und.shape[0] / float(bev.shape[0])
                bev = cv2.resize(bev, (int(bev.shape[1] * s), und.shape[0]))
                comp = np.hstack([und, bev])
            else:
                comp = und
            print(f"{cat} f{idx:03d}: detected={r.detected} off={r.offset_norm:+.2f} "
                  f"conf={r.confidence:.2f} curv={r.curvature_norm:+.2f} fill={r.fill_pct:.1f} "
                  f"base_x={r.base_x}")
            cv2.imwrite(os.path.join(out, f"{cat}_f{idx:03d}.jpg"), comp)


if __name__ == "__main__":
    main()
