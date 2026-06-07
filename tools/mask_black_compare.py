#!/usr/bin/env python3
"""Compare masks on the WIDE zebra BEV: current Otsu (warped_black_mask) vs a
BLACK-only threshold. The track lines are always BLACK; the floor is tan and the
gaps/other sections are white. Otsu adapts and can grab tan-vs-white (flooding the
mask with non-line area); a fixed dark threshold keeps ONLY the black lines.

Dumps [BEV | Otsu mask | BLACK mask] composites so we can see the difference.
"""
import os
import sys
import glob

import cv2
import numpy as np

SRC = "/home/puzzlebot/ros2_ws/src/puzzlebot_ros"
sys.path.insert(0, SRC)
from puzzlebot_ros.perception.lane import LaneParams, load_lane_params, warped_black_mask
from puzzlebot_ros.perception.zebra import ZebraParams, wide_homography

CAM = os.path.join(SRC, "config", "camera_params.npz")
LANE_JSON = os.path.join(SRC, "config", "lane_params.json")

# Black-line threshold: pixels darker than this (0-255 gray) are "line". Tan floor
# sits well above this; white is far above. Tune if needed.
BLACK_T = 90


def black_mask_fixed(bev_bgr, t=BLACK_T):
    gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.4)
    _, m = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY_INV)  # dark -> 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def main():
    data = np.load(CAM)
    K, D = data["camera_matrix"], data["dist_coeffs"]
    lp = load_lane_params(LANE_JSON, LaneParams())
    zp = ZebraParams()
    root = os.path.expanduser("~/zebra_eval")
    out = os.path.join(root, "mask_cmp")
    os.makedirs(out, exist_ok=True)

    def leaf(cat):
        ls = [d for d, _, fs in os.walk(os.path.join(root, cat))
              if any(f.startswith("frame_") for f in fs)]
        return sorted(ls)[0]

    picks = {"zebra_recta": [18], "zebra_curva": [16, 24, 30],
             "zebra_interseccion": [6, 20]}
    M = None
    for cat, idxs in picks.items():
        lf = leaf(cat)
        for idx in idxs:
            fp = os.path.join(lf, f"frame_{idx:05d}.jpg")
            img = cv2.imread(fp)
            if img is None:
                continue
            h, w = img.shape[:2]
            und = cv2.undistort(img, K, D)
            if M is None:
                M = wide_homography(lp, zp, w, h)
            bev = cv2.warpPerspective(und, M, (zp.warp_w, zp.warp_h))
            otsu = warped_black_mask(cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY), lp)
            blk = black_mask_fixed(bev)
            comp = np.hstack([bev,
                              cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR),
                              cv2.cvtColor(blk, cv2.COLOR_GRAY2BGR)])
            cv2.putText(comp, "BEV", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(comp, "OTSU", (zp.warp_w + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(comp, f"BLACK<{BLACK_T}", (2 * zp.warp_w + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            op = os.path.join(out, f"{cat}_f{idx:03d}.jpg")
            cv2.imwrite(op, comp)
            print("wrote", op)


if __name__ == "__main__":
    main()
