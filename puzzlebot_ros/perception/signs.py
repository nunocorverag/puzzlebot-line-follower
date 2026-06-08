"""YOLO traffic-sign detector for the Manchester track.

Detects the painted signs (stop / give_way / workers / turn_left / turn_right /
go_straight) with the trained ``best.pt`` and maps them to driving actions. The
node consumes this to: slow on workers, stop on stop/give_way, and AUTO-decide
the turn at the next cross on the arrow signs.

ROS-free (mirrors lane.py / zebra.py). DEGRADES GRACEFULLY: if ultralytics or the
weights are missing, the detector simply reports "nothing" and the follower keeps
working exactly as before -- so enabling signs can never break line following.

Performance: inference runs only every ``every_n`` frames on the UPPER band of the
frame (signs are above the floor) at a small ``imgsz``, so it does not slow the
30 Hz control loop.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class SignParams:
    model_path: str = ""           # path to best.pt
    conf: float = 0.55             # min YOLO confidence
    band_top_pct: int = 0          # inference ROI: vertical band [top, bot] of frame
    band_bot_pct: int = 60         # signs live in the upper part (above the floor)
    imgsz: int = 320               # inference size (small = fast)
    every_n: int = 5               # run inference 1 of every N frames (~6 Hz @30)
    min_box_pct: float = 1.2       # min box area (% of ROI) -> ignore far/tiny signs
    stable_needed: int = 2         # same canonical class this many detections -> commit


# Tolerant mapping: match by substring so small naming differences in best.pt
# (e.g. "turnLeft", "left_arrow", "roadworks", "yield") still resolve.
_ALIASES = (
    ("left", "turn_left"), ("right", "turn_right"),
    ("straight", "go_straight"), ("ahead", "go_straight"), ("forward", "go_straight"),
    ("stop", "stop"),
    ("work", "workers"), ("men", "workers"), ("worker", "workers"),
    ("give", "give_way"), ("yield", "give_way"),
)
CANONICAL = {"turn_left", "turn_right", "go_straight", "stop", "workers", "give_way"}


@dataclass
class SignResult:
    name: str | None = None        # canonical name or None
    conf: float = 0.0
    box: tuple | None = None       # (x1, y1, x2, y2) in full-frame px
    area_pct: float = 0.0          # box area as % of the ROI


def _canon(raw) -> str | None:
    r = str(raw).lower()
    for key, val in _ALIASES:
        if key in r:
            return val
    return None


class SignDetector:
    def __init__(self, params: SignParams, log: Callable[[str], None] = print):
        self.p = params
        self._log = log
        self._model = None
        self._ok = False
        self._names = {}
        self._frame_i = 0
        self._last = SignResult()
        self._stable_name = None
        self._stable_count = 0
        if not params.model_path:
            log("[signs] disabled (no model_path)")
            return
        try:
            from ultralytics import YOLO
            self._model = YOLO(params.model_path)
            self._names = dict(self._model.names)
            self._ok = True
            log(f"[signs] model loaded: {params.model_path} classes={self._names}")
        except Exception as exc:                       # noqa: BLE001 (degrade on any error)
            self._log(f"[signs] DISABLED (could not load model): {exc}")

    @property
    def ok(self) -> bool:
        return self._ok

    def detect(self, frame) -> SignResult:
        """Return the most confident, debounced sign (name None if none).

        Only actually infers every ``every_n`` frames; in between it returns the
        last result so callers can poll every frame cheaply.
        """
        if not self._ok:
            return SignResult()
        self._frame_i += 1
        if self._frame_i % max(1, self.p.every_n) != 0:
            return self._last

        h, w = frame.shape[:2]
        y0 = max(0, int(h * self.p.band_top_pct / 100.0))
        y1 = min(h, int(h * self.p.band_bot_pct / 100.0))
        roi = frame[y0:y1, :]
        if roi.size == 0:
            self._last = SignResult()
            return self._last
        try:
            preds = self._model.predict(roi, imgsz=self.p.imgsz,
                                        conf=self.p.conf, verbose=False)
        except Exception as exc:                       # noqa: BLE001
            self._log(f"[signs] predict failed: {exc}", )
            self._last = SignResult()
            return self._last

        roi_area = float(roi.shape[0] * roi.shape[1]) or 1.0
        best = SignResult()
        for r in preds:
            for b in getattr(r, "boxes", []):
                cf = float(b.conf[0])
                name = _canon(self._names.get(int(b.cls[0]), int(b.cls[0])))
                if name is None:
                    continue
                x1, yb1, x2, yb2 = (float(v) for v in b.xyxy[0])
                area_pct = 100.0 * ((x2 - x1) * (yb2 - yb1)) / roi_area
                if area_pct < self.p.min_box_pct:
                    continue
                if cf > best.conf:
                    best = SignResult(name=name, conf=cf,
                                      box=(x1, yb1 + y0, x2, yb2 + y0),
                                      area_pct=area_pct)

        # Debounce: only surface a sign after it is seen stable_needed times.
        if best.name is not None and best.name == self._stable_name:
            self._stable_count += 1
        else:
            self._stable_name = best.name
            self._stable_count = 1 if best.name is not None else 0
        self._last = best if self._stable_count >= self.p.stable_needed else SignResult()
        return self._last


def draw_sign_overlay(frame, result: SignResult):
    """Draw the detected sign box + label on the frame (in place)."""
    import cv2
    if result.name is None or result.box is None:
        return frame
    x1, y1, x2, y2 = (int(v) for v in result.box)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, f"{result.name} {result.conf:.2f}", (x1, max(12, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame
