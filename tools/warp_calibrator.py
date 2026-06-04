#!/usr/bin/env python3
"""Bird's-eye warp calibrator for the lane follower.

ROS-free. Tunes the homography + sliding-window params in
``puzzlebot_ros/perception/lane.py`` live (Jetson CSI over H264) or offline on
saved dataset frames (``--image``), then persists them to
``config/lane_params.json`` so the runtime follower loads exactly what was tuned.

Dashboard (left -> right): original frame with the warp trapezoid + fitted line,
and the bird's-eye view with the sliding windows + fit. Tune until a STRAIGHT
line looks vertical and parallel lines stay parallel in the bird's-eye view.

Live tuning uses a command file (same mechanism as the line calibrator):
  scripts/set_warp_param.sh src_top_half_w_pct 16
  scripts/set_warp_param.sh mask_method 1      # 0=Otsu, 1=adaptive
  scripts/set_warp_param.sh save_lane 1        # -> config/lane_params.json
  scripts/set_warp_param.sh q 1                # quit
Any LaneParams field name is accepted; plus: save_lane, q/quit, p/pause,
u/undistort, reset.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import fields
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.lane import (  # noqa: E402
    LaneParams,
    analyze_lane,
    compute_homography,
    draw_birdseye_debug,
    draw_lane_overlay,
    load_lane_params,
    save_lane_params,
)
from puzzlebot_ros.perception.camera import (  # noqa: E402
    build_gstreamer_pipeline,
    load_camera_params,
    load_illumination_gain,
    preprocess_frame,
)
from puzzlebot_ros.perception.stream import Preview  # noqa: E402

DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_ILLUMINATION_PARAMS = REPO_DIR / "config" / "illumination_flatfield.npz"
DEFAULT_OUTPUT = REPO_DIR / "config" / "lane_params.json"
DEFAULT_COMMAND_FILE = REPO_DIR / "debug_dataset" / "warp_commands.txt"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gstreamer", action="store_true", help="open the CSI camera")
    p.add_argument("--image", type=Path, help="run offline on a single image")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    p.add_argument("--illumination-params", type=Path, default=DEFAULT_ILLUMINATION_PARAMS)
    p.add_argument("--no-undistort", action="store_true")
    p.add_argument("--lane-params", type=Path, default=DEFAULT_OUTPUT,
                   help="load these params if present (continue tuning)")
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                   help="where save_lane writes")
    p.add_argument("--command-file", type=Path, default=DEFAULT_COMMAND_FILE)
    p.add_argument("--stream-fps", type=int, default=15)
    return p.parse_args()


def open_capture(args: argparse.Namespace):
    if args.image:
        return None
    cap = cv2.VideoCapture(
        build_gstreamer_pipeline(args.width, args.height, args.fps), cv2.CAP_GSTREAMER
    ) if args.gstreamer else cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[warn] could not open capture")
        return None
    return cap


def apply_command(line: str, params: LaneParams, state: dict) -> LaneParams:
    line = line.strip()
    if not line or line.startswith("#"):
        return params
    parts = line.replace("=", " ").split()
    if len(parts) < 1:
        return params
    name = parts[0]
    value = parts[1] if len(parts) > 1 else "1"
    if name in ("save_lane", "save"):
        state["save"] = True
        return params
    if name in ("q", "quit"):
        state["quit"] = True
        return params
    if name in ("p", "pause"):
        state["pause"] = not state.get("pause", False)
        return params
    if name in ("u", "undistort"):
        state["undistort"] = not state.get("undistort", True)
        print(f"[cmd] undistort={state['undistort']}")
        return params
    if name == "reset":
        print("[cmd] reset to defaults")
        return LaneParams()
    valid = {f.name for f in fields(LaneParams)}
    if name not in valid:
        print(f"[cmd] unknown '{name}'. Known: {', '.join(sorted(valid))}")
        return params
    try:
        setattr(params, name, int(float(value)))
        print(f"[cmd] {name}={getattr(params, name)}")
        state["dirty_homography"] = True
    except ValueError:
        print(f"[cmd] bad value for {name}: {value}")
    return params


def watch_command_file(path: Path, last_mtime, params, state):
    if not path.exists():
        return last_mtime, params
    mtime = path.stat().st_mtime_ns
    if last_mtime is not None and mtime <= last_mtime:
        return last_mtime, params
    for line in path.read_text().splitlines():
        params = apply_command(line, params, state)
    return mtime, params


def compose_dashboard(frame, result, params: LaneParams) -> np.ndarray:
    """Original (with trapezoid + fit) beside the bird's-eye debug view."""
    left = frame.copy()
    draw_lane_overlay(left, params, result)
    bird = draw_birdseye_debug(result, params)
    h = left.shape[0]
    bw = int(bird.shape[1] * h / bird.shape[0])
    bird = cv2.resize(bird, (bw, h))
    dash = np.hstack([left, bird])
    # Footer with the live warp values so the operator sees what they're tuning.
    txt = (f"top_y={params.src_top_y_pct} top_hw={params.src_top_half_w_pct} "
           f"bot_y={params.src_bot_y_pct} bot_hw={params.src_bot_half_w_pct} "
           f"mask={'adapt' if params.mask_method else 'otsu'} clahe={params.use_clahe}")
    cv2.putText(dash, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return dash


def main() -> int:
    args = parse_args()
    params = load_lane_params(args.lane_params)
    print(f"[info] loaded lane params from {args.lane_params}"
          if args.lane_params.exists() else "[info] starting from LaneParams defaults")

    camera_matrix, dist_coeffs = (None, None)
    if not args.no_undistort:
        camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    gain = load_illumination_gain(args.illumination_params)

    state = {"quit": False, "pause": False, "undistort": not args.no_undistort,
             "save": False, "dirty_homography": True}
    preview = Preview.from_env(window="WarpCalibrator", fps=args.stream_fps)

    cap = open_capture(args)
    static = cv2.imread(str(args.image)) if args.image else None
    if cap is None and static is None:
        print("[error] no camera and no --image"); return 1

    # Ignore stale commands already in the file (e.g. a 'q' from a prior session).
    last_mtime = (args.command_file.stat().st_mtime_ns
                  if args.command_file.exists() else None)
    m = minv = None
    frame_size = None
    last_frame = static
    try:
        while not state["quit"]:
            last_mtime, params = watch_command_file(args.command_file, last_mtime, params, state)

            # Resize to the calibration resolution BEFORE undistort: camera_params
            # is for args.width x args.height (640x480); the CSI delivers native
            # 1280x720, so feeding it raw warps the undistort + leaves black bars.
            size = (args.width, args.height)
            if cap is not None and not state["pause"]:
                ok, raw = cap.read()
                if not ok:
                    continue
                cm = camera_matrix if state["undistort"] else None
                dc = dist_coeffs if state["undistort"] else None
                last_frame = preprocess_frame(raw, cm, dc, gain, size=size)
            elif static is not None:
                cm = camera_matrix if state["undistort"] else None
                dc = dist_coeffs if state["undistort"] else None
                last_frame = preprocess_frame(static.copy(), cm, dc, gain, size=size)

            if last_frame is None:
                continue
            h, w = last_frame.shape[:2]
            if state["dirty_homography"] or frame_size != (w, h):
                m, minv = compute_homography(params, w, h)
                frame_size = (w, h)
                state["dirty_homography"] = False

            result = analyze_lane(last_frame, params, m, minv)
            preview.show(compose_dashboard(last_frame, result, params))

            if state["save"]:
                save_lane_params(params, args.output)
                print(f"[saved] {args.output}")
                state["save"] = False
    finally:
        if cap is not None:
            cap.release()
        preview.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
