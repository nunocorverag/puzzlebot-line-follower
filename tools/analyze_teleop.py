#!/usr/bin/env python3
"""Cruza una demostracion teleop con la percepcion del carril.

Para cada frame grabado por tools/recorder.py (con teleop_cmd.csv), corre
analyze_lane (encadenando prev_base_x como el runtime) y lo empareja con el
(v, w) que el humano mando. Sirve para ver "cuando la linea estaba en offset X
con curvatura Y, el humano giro w=Z" y calibrar el control a esa demostracion.

CORRE EN LA JETSON (necesita cv2). Uso:
    python3 tools/analyze_teleop.py datasets/recordings/<SESSION>
    python3 tools/analyze_teleop.py datasets/recordings/<SESSION> --out demo.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.lane import (  # noqa: E402
    LaneParams, analyze_lane, compute_homography, load_lane_params,
)

DEFAULT_LANE_PARAMS = REPO_DIR / "config" / "lane_params.json"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path, help="datasets/recordings/<SESSION>")
    ap.add_argument("--lane-params", type=Path, default=DEFAULT_LANE_PARAMS)
    ap.add_argument("--out", type=Path, default=None, help="CSV de salida")
    args = ap.parse_args()

    cmd_csv = args.session / "teleop_cmd.csv"
    if not cmd_csv.exists():
        print(f"falta {cmd_csv} (grabaste con el recorder nuevo?)")
        return 1
    rows = list(csv.DictReader(open(cmd_csv)))
    params = load_lane_params(args.lane_params, LaneParams())

    out_rows = []
    m = minv = None
    prev_base = None
    for r in rows:
        fp = args.session / r["frame"]
        if not fp.exists():
            continue
        frame = cv2.imread(str(fp))
        if frame is None:
            continue
        if m is None:
            h, w = frame.shape[:2]
            m, minv = compute_homography(params, w, h)
        res = analyze_lane(frame, params, m, minv, prev_base)
        prev_base = res.base_x if res.base_x is not None else prev_base
        out_rows.append({
            "t": float(r["t"]), "v": float(r["v"]), "w": float(r["w"]),
            "detected": int(res.detected),
            "off": round(res.offset_norm, 3),
            "curv": round(res.curvature_norm, 3),
            "conf": round(res.confidence, 2),
            "base_x": None if res.base_x is None else round(res.base_x, 1),
            "heading": round(res.heading, 4),
        })

    print(f"frames procesados: {len(out_rows)}")
    print("\n  t      v      w     det  off    curv  conf  base   heading")
    for o in out_rows:
        print(f"{o['t']:6.2f} {o['v']:+.3f} {o['w']:+.3f}  {o['detected']}  "
              f"{o['off']:+.2f} {o['curv']:+.2f} {o['conf']:.2f} "
              f"{str(o['base_x']):>6s} {o['heading']:+.4f}")

    # Resumen de la relacion humano vs percepcion en curva.
    curve = [o for o in out_rows if abs(o["curv"]) >= 0.4 and o["detected"]]
    if curve:
        ws = [abs(o["w"]) for o in curve]
        print(f"\nEN CURVA (|curv|>=0.4, {len(curve)} frames): "
              f"|w| humano: min={min(ws):.3f} max={max(ws):.3f} "
              f"mean={sum(ws)/len(ws):.3f}")

    if args.out:
        with open(args.out, "w", newline="") as fh:
            wri = csv.DictWriter(fh, fieldnames=list(out_rows[0].keys()))
            wri.writeheader()
            wri.writerows(out_rows)
        print(f"\nguardado -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
