#!/usr/bin/env python3
"""Offline replay of analyze_lane + PD control on a follower_session.

Reads JPGs from a session in filename order, runs analyze_lane with chained
prev_base_x (matching the runtime), applies the PD + curve-guard law, and
prints per-frame:
    t  base_x  off  curv  w_pd  w_out  guard  csv_w

Usage (Jetson or any machine with cv2):
    python3 tools/replay_lane.py datasets/follower_session/YYYYMMDD_HHMMSS

Override control params: append key=value pairs, e.g. kp=0.002 ff_gain=1.5
No ROS needed.
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "puzzlebot_ros"))

import cv2  # noqa: E402
from perception.lane import (  # noqa: E402
    analyze_lane,
    compute_homography,
    LaneParams,
    load_lane_params,
)

# ---------------------------------------------------------------------------
# Default control params — mirrors line_follower.py + control_params.json
# ---------------------------------------------------------------------------
CTRL = dict(
    kp=0.0018,
    kd=0.0,
    ff_gain=1.0,
    max_w=0.60,
    max_v=0.10,
    lane_curve_min_turn_w=0.075,
    lane_hold_curve_min_curv=0.55,
    lane_curve_dropout_s=2.0,
    lane_hold_curve_s=1.20,
    lane_curve_refresh_conf=0.35,
    lane_hold_conf=0.50,
    lane_curve_hold_assist_conf=0.80,
    lane_base_edge_margin_pct=12,
)


def _f(v, d=0.0):
    try:
        return float(v)
    except Exception:
        return d


def _ts(name: str) -> float:
    """Wall-clock seconds from filename follow_YYYYMMDD_HHMMSS_mmm_STATE.jpg."""
    parts = Path(name).stem.split("_")
    if len(parts) >= 4:
        try:
            hhmmss, mmm = parts[2], parts[3]
            return (int(hhmmss[0:2]) * 3600
                    + int(hhmmss[2:4]) * 60
                    + int(hhmmss[4:6])
                    + int(mmm) / 1000.0)
        except Exception:
            pass
    return 0.0


def _load_csv(sess: Path):
    p = sess / "controller_data.csv"
    if not p.exists():
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def _reset_state():
    return dict(
        prev_base_x=None,
        last_error=0.0,
        last_deriv=0.0,
        last_t=None,
        hold_curv=0.0,
        hold_signed=0.0,
        hold_center_x=None,
        hold_far_x=None,
        hold_t=None,
    )


def run_replay(sess_path: str, overrides: dict) -> None:
    sess = Path(sess_path)
    cfg = {**CTRL, **overrides}

    # Lane params from saved JSON (falls back to defaults)
    lp_path = REPO / "config" / "lane_params.json"
    params = load_lane_params(str(lp_path)) if lp_path.exists() else LaneParams()

    # CSV for ground-truth w comparison (keyed by t_rel rounded to 2 dp)
    csv_rows = _load_csv(sess)
    csv_map: dict[float, dict] = {}
    # Map of t_rel → base_x at each HOLD→FOLLOW transition (seed for continuity)
    follow_starts: dict[float, float | None] = {}
    if csv_rows:
        t0_csv = _f(csv_rows[0].get("t"))
        prev_state = ""
        for r in csv_rows:
            t_key = round(_f(r.get("t")) - t0_csv, 2)
            csv_map[t_key] = r
            st = r.get("state", "")
            if st == "FOLLOW" and prev_state != "FOLLOW":
                bx_csv = r.get("base_x")
                follow_starts[t_key] = _f(bx_csv) if bx_csv and bx_csv != "999" else None
            prev_state = st

    jpgs = sorted(
        [f for f in sess.iterdir() if f.suffix == ".jpg"],
        key=lambda p: p.name,
    )
    if not jpgs:
        print("(no JPG frames)")
        return

    t0 = _ts(jpgs[0].name)
    edge_margin = params.warp_w * cfg["lane_base_edge_margin_pct"] / 100.0

    # Pre-compute homography from first loadable frame
    m_h = minv_h = None
    for jpg in jpgs:
        img0 = cv2.imread(str(jpg))
        if img0 is not None:
            m_h, minv_h = compute_homography(params, img0.shape[1], img0.shape[0])
            break

    # ---- state (reset at each HOLD→FOLLOW transition) ----
    prev_base_x: float | None = None
    last_good_base_t: float | None = None   # wall_t of last accepted base
    base_hold_s = 0.5                        # mirror lane_base_hold_s default
    last_error = last_deriv = 0.0
    last_t: float | None = None
    hold_curv = hold_signed = 0.0
    hold_center_x: float | None = None
    hold_far_x: float | None = None
    hold_t: float | None = None
    last_reset_fs: float | None = None  # track which follow_start we already reset

    hdr = (f"{'t':>7}  {'base_x':>6}  {'off':>6}  {'curv':>6}  "
           f"{'w_pd':>7}  {'w_out':>7}  {'guard':>5}  {'csv_w':>7}")
    print(f"=== replay {sess.name} ===")
    print(hdr)
    print("-" * 70)

    for jpg in jpgs:
        wall_t = _ts(jpg.name)
        t_rel = wall_t - t0

        # Determine state from CSV
        t_key = round(t_rel, 2)
        csv_row = csv_map.get(t_key) or (
            csv_map.get(min(csv_map, key=lambda k: abs(k - t_rel)))
            if csv_map else None
        )
        csv_state = csv_row.get("state", "") if csv_row else ""

        # Reset control state at HOLD→FOLLOW transitions; seed prev_base_x
        # from the CSV so the continuity tracker starts on the right line.
        if follow_starts:
            closest_fs = min(follow_starts, key=lambda k: abs(k - t_key))
            at_follow_start = (abs(t_key - closest_fs) < 0.25
                               and csv_state == "FOLLOW"
                               and closest_fs != last_reset_fs)
            if at_follow_start:
                prev_base_x = follow_starts[closest_fs]
                last_good_base_t = wall_t if prev_base_x is not None else None
                last_error = last_deriv = 0.0
                last_t = None
                hold_curv = hold_signed = 0.0
                hold_center_x = hold_far_x = hold_t = None
                last_reset_fs = closest_fs

        # Skip non-FOLLOW frames (print a marker instead)
        if csv_state and "FOLLOW" not in csv_state:
            continue

        frame = cv2.imread(str(jpg))
        if frame is None:
            continue
        h_img, w_img = frame.shape[:2]
        cx = w_img / 2.0

        lr = analyze_lane(frame, params, m=m_h, minv=minv_h, prev_base_x=prev_base_x)

        # ---- acceptance (simplified: conf + edge only) ----
        min_conf = params.min_windows_conf_pct / 100.0
        now_good = lr.detected and lr.confidence >= min_conf
        edge_reject = False
        bx = lr.base_x
        if now_good and bx is not None:
            if bx < edge_margin or bx > (params.warp_w - edge_margin):
                edge_reject = True
        accept = now_good and not edge_reject

        # ---- update prev_base_x (sticky during brief dropouts, like runtime) ----
        if accept and bx is not None:
            prev_base_x = bx
            last_good_base_t = wall_t
        elif not accept:
            # keep prev_base_x for up to base_hold_s (mirrors _lane_prev_base sticky logic)
            if last_good_base_t is None or (wall_t - last_good_base_t) > base_hold_s:
                prev_base_x = None

        # ---- curve hold update ----
        sc = float(lr.curvature_norm) if lr.detected else 0.0
        if accept and lr.lane_center_x_orig is not None:
            conf = lr.confidence
            same_dir = (hold_signed == 0.0 or hold_signed * sc >= 0.0)
            recent_hold = (hold_t is not None
                           and (wall_t - hold_t) <= cfg["lane_hold_curve_s"])
            assist_weak = conf <= cfg["lane_curve_hold_assist_conf"]
            sign_flip = (recent_hold and assist_weak
                         and abs(sc) >= cfg["lane_hold_curve_min_curv"]
                         and hold_signed * sc < 0.0)
            flat = (recent_hold and assist_weak
                    and abs(sc) < cfg["lane_hold_curve_min_curv"])

            if sign_flip or flat:
                # keep hold far_x for curve_term (assist from hold)
                pass
            elif abs(sc) >= cfg["lane_hold_curve_min_curv"] or not recent_hold:
                hold_curv = abs(sc)
                hold_signed = sc
                hold_center_x = lr.lane_center_x_orig
                hold_far_x = lr.lane_center_far_x_orig
                hold_t = wall_t
        elif not accept and lr.detected and lr.lane_center_x_orig is not None:
            # weak curve refresh
            same_dir = (hold_signed == 0.0 or hold_signed * sc >= 0.0)
            old_hold = (hold_t is None
                        or (wall_t - hold_t) > cfg["lane_hold_curve_s"])
            if (lr.confidence >= cfg["lane_curve_refresh_conf"]
                    and abs(sc) >= cfg["lane_hold_curve_min_curv"]
                    and (same_dir or old_hold)):
                hold_curv = abs(sc)
                hold_signed = sc
                hold_center_x = lr.lane_center_x_orig
                hold_far_x = lr.lane_center_far_x_orig
                hold_t = wall_t

        # ---- steering targets ----
        steer_x: float | None = None
        far_x: float | None = None
        if accept and lr.lane_center_x_orig is not None:
            steer_x = lr.lane_center_x_orig
            # check assist (sign_flip/flat handled above — not resetting steer_x here)
            conf = lr.confidence
            same_dir = (hold_signed == 0.0 or hold_signed * sc >= 0.0)
            recent_hold = (hold_t is not None
                           and (wall_t - hold_t) <= cfg["lane_hold_curve_s"])
            assist_weak = conf <= cfg["lane_curve_hold_assist_conf"]
            sign_flip = (recent_hold and assist_weak
                         and abs(sc) >= cfg["lane_hold_curve_min_curv"]
                         and hold_signed * sc < 0.0)
            flat = (recent_hold and assist_weak
                    and abs(sc) < cfg["lane_hold_curve_min_curv"])
            if (sign_flip or flat) and hold_far_x is not None:
                far_x = hold_far_x
            else:
                far_x = lr.lane_center_far_x_orig

        # ---- PD math ----
        w_pd = 0.0
        w_out = 0.0
        guard_fired = False
        dt = (wall_t - last_t) if last_t is not None else 0.05

        if steer_x is not None and dt > 0:
            line_err = cx - steer_x
            raw_d = (line_err - last_error) / dt
            deriv = 0.7 * last_deriv + 0.3 * raw_d

            curve_term = 0.0
            if far_x is not None:
                curve_term = (cx - far_x) - line_err

            w_raw = (cfg["kp"] * line_err
                     + cfg["kd"] * deriv
                     + cfg["kp"] * cfg["ff_gain"] * curve_term)
            w_out = max(-cfg["max_w"], min(cfg["max_w"], w_raw))
            w_pd = w_out

            last_error = line_err
            last_deriv = deriv

            # ---- curve direction guard (constant floor) ----
            curve_age = (wall_t - hold_t) if hold_t is not None else 1e9
            if (cfg["lane_curve_min_turn_w"] > 0.0
                    and hold_curv >= cfg["lane_hold_curve_min_curv"]
                    and curve_age <= cfg["lane_curve_dropout_s"]
                    and abs(hold_signed) > 1e-3):
                c_dir = 1.0 if hold_signed > 0.0 else -1.0
                min_w = min(abs(cfg["lane_curve_min_turn_w"]), cfg["max_w"])
                if w_out * c_dir < min_w:
                    w_out = c_dir * min_w
                    guard_fired = True

        last_t = wall_t

        # ---- lookup csv_w ----
        csv_w = ""
        if csv_map:
            closest = min(csv_map, key=lambda k: abs(k - t_rel))
            if abs(closest - t_rel) < 0.15:
                csv_w = f"{_f(csv_map[closest].get('w')):+.3f}"

        bx_s = f"{bx:6.1f}" if bx is not None else "   ---"
        off_s = f"{lr.offset_norm:+6.3f}" if lr.detected else "   ---"
        curv_s = f"{sc:+6.3f}" if lr.detected else "   ---"

        print(f"{t_rel:7.2f}  {bx_s}  {off_s}  {curv_s}  "
              f"{w_pd:+7.3f}  {w_out:+7.3f}  {'Y' if guard_fired else '-':>5}  {csv_w:>7}")

    print("=" * 70)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    sess = sys.argv[1]
    overrides: dict = {}
    for arg in sys.argv[2:]:
        if "=" in arg:
            k, v = arg.split("=", 1)
            try:
                overrides[k] = float(v)
            except ValueError:
                pass
    run_replay(sess, overrides)


if __name__ == "__main__":
    main()
