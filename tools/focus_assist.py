#!/usr/bin/env python3
"""Live focus assistant for the CSI camera (twist the lens to maximize sharpness).

Sharpness is scene-dependent, so there is no universal "good" number: you point
at a fixed, textured target (the checkerboard or printed text) at the working
distance and TURN THE LENS to MAXIMIZE the score. The tool keeps a decaying
peak so you can find the maximum, tells you if you are going up/down, and flags
when you are at the peak ("no muevas").

Set focus BEFORE calibrating the camera, and don't touch it afterwards (changing
focus slightly changes the intrinsics).

Preview streams over the unified Preview (H264 by default).

  python3 tools/focus_assist.py
  python3 tools/focus_assist.py --roi 0.4
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.camera import open_csi_capture  # noqa: E402
from puzzlebot_ros.perception.stream import Preview  # noqa: E402

PEAK_DECAY = 0.998        # per-frame relaxation of the held peak (~slow)
EMA_ALPHA = 0.3           # smoothing of the live score
AT_PEAK_RATIO = 0.96      # >= this fraction of peak = "you're sharp"


def sharpness(gray) -> float:
    """Variance of the Laplacian — the classic focus metric (higher = sharper)."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def center_roi(shape, frac: float):
    h, w = shape[:2]
    rw, rh = int(w * frac), int(h * frac)
    x0, y0 = (w - rw) // 2, (h - rh) // 2
    return x0, y0, x0 + rw, y0 + rh


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--roi", type=float, default=0.5,
                        help="center region fraction used to measure focus (0-1)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    cap = open_csi_capture(width=args.width, height=args.height, fps=args.fps, downscale=True)
    if cap is None:
        print("[error] camera unavailable")
        return 1
    preview = Preview.from_env("Focus Assist", fps=args.fps)

    peak = 1e-6
    ema = None
    print("[info] Apunta a un objetivo con textura y gira el lente para MAXIMIZAR. Ctrl+C para salir.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            x0, y0, x1, y1 = center_roi(frame.shape, args.roi)
            gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
            score = sharpness(gray)

            ema = score if ema is None else (EMA_ALPHA * score + (1 - EMA_ALPHA) * ema)
            peak = max(ema, peak * PEAK_DECAY)
            ratio = ema / peak if peak > 0 else 0.0

            # Direction: is the smoothed score above its own recent trend?
            if ratio >= AT_PEAK_RATIO:
                msg, col = "EN EL PICO - no muevas", (0, 255, 0)
            elif score > ema:
                msg, col = "subiendo: sigue girando", (0, 220, 255)
            else:
                msg, col = "bajando: gira al otro lado", (0, 140, 255)

            # ROI box.
            cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 255), 1)

            # Vertical bar = current / peak.
            bh = int((frame.shape[0] - 80) * min(1.0, ratio))
            bx = frame.shape[1] - 36
            cv2.rectangle(frame, (bx, 40), (bx + 20, frame.shape[0] - 40), (60, 60, 60), 1)
            cv2.rectangle(frame, (bx, frame.shape[0] - 40 - bh), (bx + 20, frame.shape[0] - 40), col, -1)

            lines = [
                f"focus: {ema:7.0f}",
                f"pico : {peak:7.0f}  ({ratio*100:3.0f}%)",
                msg,
            ]
            y = 28
            for i, line in enumerate(lines):
                c = col if i == 2 else (255, 255, 255)
                cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)
                y += 30
            preview.show(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        preview.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
