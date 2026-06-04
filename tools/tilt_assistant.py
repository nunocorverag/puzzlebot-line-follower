#!/usr/bin/env python3
"""Camera tilt (pitch) assistant for the Puzzlebot.

ROS-free. As you physically tilt the camera, this estimates the pitch angle in
degrees (0 = perfectly horizontal) from the vanishing point of the track's two
parallel lines, using the calibrated intrinsics (``camera_params.npz``). It also
overlays the horizon plus the two working bands so you can pick a tilt that
serves BOTH jobs at once:

  * lower band  -> ground (line + dashes) used by the bird's-eye warp,
  * upper band  -> far field (traffic light, signs, far curve preview).

How it works: the two straight lane lines are parallel on the ground, so in the
image they meet at a vanishing point ``v``. With the principal point ``cy`` and
focal length ``fy``, the pitch is ``atan((cy - v_y) / fy)``: horizontal camera
=> vanishing point at the image center => 0 deg; tilt down => it rises => +deg.

Stand the robot on a STRAIGHT section for a reading. Place signs near/far to
confirm they land in the upper band. Run over H264 like the other tools.

Commands (via scripts/set_tilt_param.sh or the command file):
  save 1   -> write the current pitch to config/camera_pose.json
  u 1      -> toggle undistort
  q 1      -> quit
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from math import atan2, degrees, hypot
from pathlib import Path

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.intersection import black_mask  # noqa: E402
from puzzlebot_ros.perception.camera import (  # noqa: E402
    build_gstreamer_pipeline,
    load_camera_params,
    load_illumination_gain,
    preprocess_frame,
)
from puzzlebot_ros.perception.stream import Preview  # noqa: E402

DEFAULT_CAMERA_PARAMS = REPO_DIR / "config" / "camera_params.npz"
DEFAULT_ILLUMINATION_PARAMS = REPO_DIR / "config" / "illumination_flatfield.npz"
DEFAULT_OUTPUT = REPO_DIR / "config" / "camera_pose.json"
DEFAULT_COMMAND_FILE = REPO_DIR / "debug_dataset" / "tilt_commands.txt"
DEFAULT_SNAPSHOT_DIR = REPO_DIR / "debug_dataset" / "tilt_session"
RELEVEL_TOL_DEG = 1.0   # within this delta the camera is back at the setpoint

# Working bands (fractions of image height), matching the lane/sign split.
GROUND_BAND = (0.55, 0.95)   # bird's-eye warp source region
FAR_BAND = (0.0, 0.50)       # signs / traffic light region


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gstreamer", action="store_true")
    p.add_argument("--image", type=Path)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--camera-params", type=Path, default=DEFAULT_CAMERA_PARAMS)
    p.add_argument("--illumination-params", type=Path, default=DEFAULT_ILLUMINATION_PARAMS)
    p.add_argument("--camera-height-cm", type=float, default=0.0,
                   help="if set, also prints approx ground distances")
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--command-file", type=Path, default=DEFAULT_COMMAND_FILE)
    p.add_argument("--stream-fps", type=int, default=15)
    p.add_argument("--snapshot-dir", type=Path, default=DEFAULT_SNAPSHOT_DIR)
    p.add_argument("--snapshot-interval", type=float, default=0.8,
                   help="min seconds between captures (debounce)")
    p.add_argument("--stable-tol-deg", type=float, default=2.0,
                   help="capture only when pitch varies less than this (held still)")
    p.add_argument("--stable-time", type=float, default=0.5,
                   help="seconds the pitch must stay stable before a capture")
    p.add_argument("--min-pitch-gap-deg", type=float, default=1.0,
                   help="skip a capture unless the pitch differs this much from the last")
    p.add_argument("--auto-start", action="store_true",
                   help="start capturing immediately (default: wait for 'start')")
    p.add_argument("--relevel", action="store_true",
                   help="re-leveling mode: compare live pitch to the saved setpoint")
    p.add_argument("--target-pitch", type=float, default=None,
                   help="override the setpoint for --relevel (deg)")
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


def estimate_vanishing_point(frame):
    """Return (vx, vy, lines) from the two dominant straight lane lines, or None.

    Robustness: search only the lower (clean) region, keep near-vertical segments,
    split them by slope sign (left vs right edge) and fit ONE line per side by
    length-weighted least squares (stable vs. a median of noisy short segments
    from signs/clutter). Reject near-parallel pairs (ill-conditioned).
    """
    h, w = frame.shape[:2]
    mask = black_mask(frame)
    mask[: int(h * 0.45), :] = 0          # only the near lane (clean, no clutter)
    segments = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=50,
                               minLineLength=int(h * 0.20), maxLineGap=30)
    if segments is None:
        return None
    # Accumulate endpoints per side, weighted by segment length.
    sides = {"l": {"y": [], "x": [], "w": []}, "r": {"y": [], "x": [], "w": []}}
    for x1, y1, x2, y2 in segments[:, 0]:
        if abs(y2 - y1) < 5:
            continue
        a = (x2 - x1) / float(y2 - y1)
        if abs(a) > 2.5:                  # too horizontal -> skip
            continue
        length = hypot(x2 - x1, y2 - y1)
        s = sides["l"] if a < 0 else sides["r"]
        s["y"] += [y1, y2]; s["x"] += [x1, x2]; s["w"] += [length, length]
    fits = {}
    for key, s in sides.items():
        if len(s["y"]) < 4 or sum(s["w"]) < h * 0.5:
            return None                  # not enough reliable line on a side
        a, b = np.polyfit(s["y"], s["x"], 1, w=s["w"])  # x = a*y + b
        fits[key] = (float(a), float(b))
    (a1, b1), (a2, b2) = fits["l"], fits["r"]
    if abs(a1 - a2) < 0.15:              # near-parallel -> vanishing point unstable
        return None
    vy = (b2 - b1) / (a1 - a2)
    vx = a1 * vy + b1
    return float(vx), float(vy), ((a1, b1), (a2, b2))


def ground_distance_cm(row_y, cy, fy, pitch_rad, height_cm):
    """Approx ground distance ahead for an image row, given camera height."""
    alpha = atan2(row_y - cy, fy)         # angle below the optical axis
    depression = pitch_rad + alpha        # below horizontal
    if depression <= 0.02:
        return None                       # at/above horizon -> not on the ground
    return height_cm / np.tan(depression)


def _band_fill(frame, y0f, y1f):
    h = frame.shape[0]
    mask = black_mask(frame)
    band = mask[int(h * y0f):int(h * y1f), :]
    return float(cv2.countNonZero(band)) / float(max(1, band.size))


def score_pose(frame, vp, pitch_deg):
    """Rate a tilt for the dual-purpose camera (0..1) + the metrics behind it.

    Good tilt = the GROUND band has the line, the horizon sits above it so the
    FAR band can show signs, and the pitch is a moderate downward angle.
    """
    h = frame.shape[0]
    ground_fill = _band_fill(frame, *GROUND_BAND)
    # Line present in the ground band, but not the whole band flooded black.
    s_ground = float(np.clip(ground_fill / 0.04, 0, 1)) * float(np.clip((0.35 - ground_fill) / 0.10, 0, 1))
    # Horizon above the ground band -> far band is real receding road for signs.
    vy_frac = (vp[1] / h) if vp is not None else 1.0
    s_far = float(np.clip((GROUND_BAND[0] - vy_frac) / GROUND_BAND[0], 0, 1))
    # Prefer a moderate downward pitch (~5..18 deg).
    s_pitch = float(np.clip(1.0 - abs(pitch_deg - 11.0) / 11.0, 0, 1))
    score = 0.4 * s_ground + 0.3 * s_far + 0.3 * s_pitch
    return score, {"ground_fill": round(ground_fill, 4), "vy_frac": round(vy_frac, 3),
                   "s_ground": round(s_ground, 2), "s_far": round(s_far, 2),
                   "s_pitch": round(s_pitch, 2)}


def draw_relevel(frame, current_pitch, target_pitch, have_estimate):
    h, w = frame.shape[:2]
    delta = current_pitch - target_pitch
    ok = have_estimate and abs(delta) <= RELEVEL_TOL_DEG
    color = (0, 255, 0) if ok else (0, 0, 255)
    if not have_estimate:
        msg = "RE-LEVEL: stand on a STRAIGHT section"
    elif ok:
        msg = f"LEVEL OK  ({current_pitch:+.1f} vs target {target_pitch:+.1f})"
    else:
        arrow = "tilt DOWN" if delta < 0 else "tilt UP"
        msg = f"{arrow} {abs(delta):.1f} deg  (now {current_pitch:+.1f}, target {target_pitch:+.1f})"
    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, 6)
    cv2.putText(frame, msg, (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def draw(frame, vp, pitch_deg, conf, fy, cy, height_cm):
    h, w = frame.shape[:2]
    # Working bands.
    for (y0f, y1f), color, label in [
        (GROUND_BAND, (0, 180, 0), "GROUND (warp)"),
        (FAR_BAND, (0, 160, 255), "FAR (signs/light)"),
    ]:
        y0, y1 = int(h * y0f), int(h * y1f)
        cv2.rectangle(frame, (0, y0), (w - 1, y1), color, 2)
        cv2.putText(frame, label, (8, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
    cv2.line(frame, (0, int(cy)), (w, int(cy)), (120, 120, 120), 1)  # optical axis row

    if vp is not None:
        vx, vy, ((a1, b1), (a2, b2)) = vp
        for (a, b) in ((a1, b1), (a2, b2)):
            p0 = (int(a * (h - 1) + b), h - 1)
            p1 = (int(a * vy + b), int(vy))
            cv2.line(frame, p0, p1, (255, 0, 255), 2)
        cv2.line(frame, (0, int(vy)), (w, int(vy)), (0, 255, 255), 2)  # horizon
        cv2.circle(frame, (int(vx), int(vy)), 7, (0, 0, 255), -1)

    level = "HORIZONTAL" if abs(pitch_deg) < 1.5 else ("DOWN" if pitch_deg > 0 else "UP")
    txt = (f"pitch={pitch_deg:+.1f} deg [{level}]  conf={conf:.2f}"
           if vp is not None else
           "no estimate - stand on a STRAIGHT section")
    cv2.putText(frame, txt, (8, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2)
    if vp is not None and height_cm > 0:
        gd_bot = ground_distance_cm(h * GROUND_BAND[1], cy, fy,
                                    np.radians(pitch_deg), height_cm)
        gd_top = ground_distance_cm(h * GROUND_BAND[0], cy, fy,
                                    np.radians(pitch_deg), height_cm)
        d = (f"ground band ~{gd_bot:.0f}..{gd_top:.0f} cm ahead"
             if gd_bot and gd_top else "ground band reaches the horizon")
        cv2.putText(frame, d, (8, h - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0), 2)


def watch_commands(path: Path, last_mtime, state):
    if not path.exists():
        return last_mtime
    mtime = path.stat().st_mtime_ns
    if last_mtime is not None and mtime <= last_mtime:
        return last_mtime
    for line in path.read_text().splitlines():
        name = line.replace("=", " ").split()[0] if line.strip() else ""
        if name in ("q", "quit"):
            state["quit"] = True
        elif name in ("u", "undistort"):
            state["undistort"] = not state["undistort"]
        elif name in ("save",):
            state["save"] = True
        elif name in ("mark", "snap"):
            state["mark"] = True
        elif name in ("start", "arm", "go"):
            state["armed"] = True
            print("[cmd] capture ARMED")
        elif name in ("stop", "disarm", "pause"):
            state["armed"] = False
            print("[cmd] capture STOPPED")
    return mtime


def main() -> int:
    args = parse_args()
    camera_matrix, dist_coeffs = load_camera_params(args.camera_params)
    if camera_matrix is None:
        print("[error] tilt estimate needs camera_params.npz intrinsics")
        return 1
    fy = float(camera_matrix[1, 1])
    cy = float(camera_matrix[1, 2])
    gain = load_illumination_gain(args.illumination_params)

    # Re-leveling target: CLI override, else the saved camera_pose.json.
    target_pitch = args.target_pitch
    if args.relevel and target_pitch is None and args.output.exists():
        try:
            target_pitch = float(json.loads(args.output.read_text()).get("pitch_deg"))
        except (ValueError, TypeError, json.JSONDecodeError):
            target_pitch = None
    if args.relevel and target_pitch is None:
        print("[error] --relevel needs a saved setpoint (run normally + save first) "
              "or --target-pitch"); return 1
    if args.relevel:
        print(f"[relevel] target pitch = {target_pitch:+.2f} deg")

    state = {"quit": False, "undistort": True, "save": False, "mark": False,
             "armed": args.auto_start}
    preview = Preview.from_env(window="TiltAssistant", fps=args.stream_fps)
    cap = open_capture(args)
    static = cv2.imread(str(args.image)) if args.image else None
    if cap is None and static is None:
        print("[error] no camera and no --image"); return 1

    snapshots = []          # (score, pitch, metrics, overlay_path) for the ranking
    last_snap_t = 0.0
    last_captured_pitch = None
    pitch_hist = []         # last few pitch readings (count-based stability)
    if not args.relevel:
        args.snapshot_dir.mkdir(parents=True, exist_ok=True)
    log_fp = (args.snapshot_dir / "tilt_log.jsonl") if not args.relevel else None

    # Ignore any stale commands already in the file (e.g. the 'q' that closed a
    # previous session) by seeding last_mtime to the current file state.
    last_mtime = (args.command_file.stat().st_mtime_ns
                  if args.command_file.exists() else None)
    last_pitch = 0.0
    pitch_buffer = []       # recent raw pitches -> median display (kills jitter)
    try:
        while not state["quit"]:
            last_mtime = watch_commands(args.command_file, last_mtime, state)
            if cap is not None:
                ok, raw = cap.read()
                if not ok:
                    continue
            else:
                raw = static.copy()
            cm = camera_matrix if state["undistort"] else None
            dc = dist_coeffs if state["undistort"] else None
            # Resize to the calibration resolution BEFORE undistort: camera_params
            # is for args.width x args.height (640x480); the CSI delivers native
            # 1280x720, so feeding it raw warps the undistort + leaves black bars.
            frame = preprocess_frame(raw, cm, dc, gain, size=(args.width, args.height))

            vp = estimate_vanishing_point(frame)
            conf = 0.0
            if vp is not None:
                _, vy, _ = vp
                raw_pitch = degrees(atan2(cy - vy, fy))
                pitch_buffer.append(raw_pitch)
                if len(pitch_buffer) > 9:
                    pitch_buffer.pop(0)
                last_pitch = float(np.median(pitch_buffer))  # robust to outliers
                conf = 1.0
            pitch_deg = last_pitch

            score, metrics = (0.0, {})
            if vp is not None and not args.relevel:
                score, metrics = score_pose(frame, vp, pitch_deg)

            # Stability: capture only when the camera is HELD still, so we never
            # grab a blurry mid-motion frame. Track recent pitch readings.
            now_t = time.time()
            stable = False
            if vp is not None:
                # Count-based (frame-rate independent): the last few valid pitch
                # readings within tolerance => held still. The loop can run slow
                # over H264, so a time window was unreliable.
                pitch_hist.append(pitch_deg)
                if len(pitch_hist) > 5:
                    pitch_hist.pop(0)
                if len(pitch_hist) >= 4:
                    stable = (max(pitch_hist) - min(pitch_hist)) <= args.stable_tol_deg
            else:
                pitch_hist = []

            raw_for_save = frame.copy()
            draw(frame, vp, pitch_deg, conf, fy, cy, args.camera_height_cm)
            if vp is not None and not args.relevel:
                status = ("ARMED" if state["armed"] else "stopped")
                hold = "HELD" if stable else "moving..."
                cv2.putText(frame, f"score={score:.2f}  [{status}] {hold}", (8, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if state["armed"] else (0, 165, 255), 2)
            if args.relevel:
                draw_relevel(frame, pitch_deg, target_pitch, vp is not None)
            preview.show(frame)

            # Snapshot when armed + held still at a NEW pitch (debounced), or on
            # a manual 'mark'. This is what keeps one clean frame per held pose.
            new_pose = (last_captured_pitch is None
                        or abs(pitch_deg - last_captured_pitch) >= args.min_pitch_gap_deg)
            auto_due = (state["armed"] and stable and new_pose
                        and now_t - last_snap_t >= args.snapshot_interval)
            if (not args.relevel and vp is not None and (auto_due or state["mark"])):
                last_snap_t = now_t
                last_captured_pitch = pitch_deg
                state["mark"] = False
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                name = f"tilt_{stamp}_p{pitch_deg:+05.1f}_s{score:.2f}"
                over = args.snapshot_dir / f"{name}_overlay.jpg"
                cv2.imwrite(str(over), frame)
                cv2.imwrite(str(args.snapshot_dir / f"{name}_raw.jpg"), raw_for_save)
                rec = {"ts": stamp, "pitch_deg": round(pitch_deg, 2),
                       "score": round(score, 3), **metrics,
                       "overlay": over.name}
                if log_fp is not None:
                    with open(log_fp, "a") as f:
                        f.write(json.dumps(rec) + "\n")
                snapshots.append((score, pitch_deg, metrics, over.name))

            if state["save"]:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps({"pitch_deg": round(pitch_deg, 2)}, indent=2))
                print(f"[saved] {args.output} pitch={pitch_deg:+.2f}")
                state["save"] = False
    finally:
        if cap is not None:
            cap.release()
        preview.close()

    # Ranked recommendation from the captured session.
    if snapshots:
        snapshots.sort(key=lambda s: s[0], reverse=True)
        summary = [{"rank": i + 1, "score": round(s, 3), "pitch_deg": round(p, 2),
                    "overlay": ov, **m} for i, (s, p, m, ov) in enumerate(snapshots[:5])]
        (args.snapshot_dir / "tilt_summary.json").write_text(json.dumps(summary, indent=2))
        print("\n=== Best tilts this session (higher score = better dual-purpose) ===")
        for row in summary:
            print(f"  #{row['rank']}  score={row['score']:.2f}  "
                  f"pitch={row['pitch_deg']:+.1f} deg  -> {row['overlay']}")
        print(f"\nPull with scripts/pull_calibration_dataset.sh, review "
              f"{args.snapshot_dir.name}/, then save the chosen pitch.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
