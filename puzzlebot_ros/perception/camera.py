#!/usr/bin/env python3
"""Single source of truth for CSI camera capture and frame preprocessing.

ROS-free (only ``cv2`` / ``numpy``) so the runtime nodes, the standalone tools
and offline laptop runs all import the exact same code. Before this module the
GStreamer pipeline was copy-pasted in ``line_follower``, ``traffic_light``,
``pictures`` and the calibrator, each with subtle differences.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

import cv2
import numpy as np


def build_gstreamer_pipeline(
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    sensor_id: int = 0,
    capture_width: int = 1280,
    capture_height: int = 720,
    downscale: bool = False,
) -> str:
    """Canonical ``nvarguscamerasrc`` pipeline string for the CSI camera.

    Captures at the sensor's native ``capture_width``x``capture_height`` and
    delivers BGR frames through an ``appsink``. By default the frame keeps the
    capture resolution and the caller resizes; set ``downscale=True`` to have
    ``nvvidconv`` resize to ``width``x``height`` in hardware.
    """
    out_caps = f", width={width}, height={height}" if downscale else ""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw, format=BGRx{out_caps} ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink max-buffers=1 drop=true"
    )


def open_csi_capture(
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    sensor_id: int = 0,
    downscale: bool = False,
    log: Callable[[str], None] = print,
) -> cv2.VideoCapture | None:
    """Open the CSI camera via GStreamer. Returns ``None`` if it cannot open.

    The CSI camera is single-owner: if another process already holds it this
    fails with "Failed to create CaptureSession". Free it first (the run
    scripts call ``stop_demo.sh``); a stuck ``nvargus-daemon`` needs
    ``sudo systemctl restart nvargus-daemon`` on the Jetson.
    """
    pipeline = build_gstreamer_pipeline(
        width=width, height=height, fps=fps, sensor_id=sensor_id, downscale=downscale
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    # isOpened() can be a false positive when the CSI is busy: GStreamer builds
    # the pipeline but nvargus fails the CaptureSession, so no frames ever come.
    # Verify we can actually grab a frame before declaring success.
    got_frame = False
    if cap.isOpened():
        for _ in range(10):
            ok, _frame = cap.read()
            if ok and _frame is not None:
                got_frame = True
                break
    if not got_frame:
        cap.release()
        log("[camera] Could not grab from CSI camera — it is BUSY (another camera "
            "process is running) or nvargus is stuck. Free it first, or run "
            "'sudo systemctl restart nvargus-daemon' on the Jetson.")
        return None
    log(f"[camera] CSI camera open @ {width}x{height} (capture native, downscale={downscale}).")
    return cap


def load_camera_params(path: Path) -> tuple[np.ndarray | None, np.ndarray | None]:
    path = Path(path)
    if not path.exists():
        print(f"[warn] camera params not found: {path}")
        return None, None
    data = np.load(str(path))
    print(f"[info] loaded camera params: {path}")
    return data["camera_matrix"], data["dist_coeffs"]


def load_illumination_gain(path: Path) -> np.ndarray | None:
    path = Path(path)
    if not path.exists():
        print(f"[warn] illumination params not found: {path}")
        return None
    data = np.load(str(path))
    print(f"[info] loaded illumination params: {path}")
    return data["gain"].astype(np.float32)


def apply_illumination_gain(frame: np.ndarray, gain: np.ndarray | None) -> np.ndarray:
    if gain is None:
        return frame
    if gain.shape[:2] != frame.shape[:2]:
        gain = cv2.resize(gain, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
    corrected = frame.astype(np.float32) * gain
    return np.clip(corrected, 0, 255).astype(np.uint8)


def preprocess_frame(
    frame: np.ndarray,
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
    gain: np.ndarray | None = None,
    rotate180: bool = False,
    size: tuple[int, int] | None = None,
) -> np.ndarray:
    """Apply the standard frame conditioning in one place.

    Order: optional 180° flip, optional resize, optional undistort, optional
    illumination flat-field gain. ``size`` is ``(width, height)``.
    """
    if rotate180:
        frame = cv2.flip(frame, -1)
    if size is not None and (frame.shape[1], frame.shape[0]) != size:
        frame = cv2.resize(frame, size)
    if camera_matrix is not None and dist_coeffs is not None:
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    if gain is not None:
        frame = apply_illumination_gain(frame, gain)
    return frame
