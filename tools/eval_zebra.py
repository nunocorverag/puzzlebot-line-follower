#!/usr/bin/env python3
"""Offline eval of the intersection detector over a folder of raw camera frames.

Threads stable_frames like the live node so entry_seen / raw_detected reflect the
real debounced behaviour. Prints a per-frame table + a summary so we can see how
the CURRENT detector behaves on recta / curva / interseccion captures.
"""
import sys
import glob
import os

import cv2

sys.path.insert(0, "/home/puzzlebot/ros2_ws/src/puzzlebot_ros")
from puzzlebot_ros.perception.intersection import (
    IntersectionParams,
    analyze_intersection,
)


def run(folder: str, params: IntersectionParams) -> None:
    frames = sorted(glob.glob(os.path.join(folder, "frame_*.jpg")))
    if not frames:
        print(f"  (no frames in {folder})")
        return
    stable = 0
    n_seen = n_detected = n_centered = 0
    first_seen = None
    for i, fp in enumerate(frames):
        img = cv2.imread(fp)
        if img is None:
            continue
        r = analyze_intersection(img, params, stable)
        stable = r.stable_frames
        if r.entry_seen:
            n_seen += 1
            if first_seen is None:
                first_seen = i
        if r.dashed_detected:
            n_detected += 1
        if r.entry_centered:
            n_centered += 1
        ey = f"{r.entry_y_pct:5.1f}" if r.entry_y_pct is not None else "  -- "
        print(
            f"  {os.path.basename(fp):18s} "
            f"trig={r.stable_frames:2d} seen={int(r.entry_seen)} "
            f"det={int(r.dashed_detected)} cent={int(r.entry_centered)} "
            f"dash={r.dashed_count:2d} ey%={ey} slope={r.entry_slope:+.3f} "
            f"opt={','.join(r.options) if r.options else '-'}"
        )
    total = len(frames)
    print(
        f"  >> {os.path.basename(folder.rstrip('/'))}: {total} frames | "
        f"entry_seen {n_seen} | dashed_detected {n_detected} | "
        f"centered {n_centered} | first_seen idx={first_seen}"
    )


def main() -> None:
    params = IntersectionParams()
    roots = sys.argv[1:] or ["zebra_recta", "zebra_curva", "zebra_interseccion"]
    for root in roots:
        # accept either the category dir (with a timestamp subdir) or a leaf dir
        leaves = [d for d, _, fs in os.walk(root) if any(f.startswith("frame_") for f in fs)]
        for leaf in sorted(leaves):
            print(f"\n=== {leaf} ===")
            run(leaf, params)


if __name__ == "__main__":
    main()
