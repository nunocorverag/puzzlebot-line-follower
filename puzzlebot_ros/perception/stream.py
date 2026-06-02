#!/usr/bin/env python3
"""Unified preview/streaming for every camera tool.

Before this module only ``line_follower`` could stream over H264; every other
tool used a local ``cv2.imshow`` window that needs ``ssh -X`` and is laggy over
WiFi. ``Preview`` gives all of them the same three modes behind one ``show()``
call, picked by env vars so the run scripts stay uniform.

ROS-free (only ``cv2``); the optional ``log`` callable lets ROS nodes route
messages through their logger.
"""

from __future__ import annotations

import os
from typing import Callable

import cv2


def _resolve_h264_host(explicit: str = "") -> str:
    """Laptop IP for the UDP sink: explicit arg, else $H264_HOST."""
    return (explicit or os.environ.get("H264_HOST", "")).strip()


class H264Streamer:
    """Hardware-encoded H264/RTP-over-UDP sink (``nvv4l2h264enc``).

    Far lower bandwidth than MJPEG because it compresses between frames. The
    writer opens lazily on the first frame (it needs the frame size). Receive on
    the laptop with ``scripts/view_h264_stream.sh``.
    """

    def __init__(
        self,
        host: str,
        port: int = 5000,
        bitrate: int = 2_000_000,
        fps: int = 15,
        log: Callable[[str], None] = print,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.bitrate = int(bitrate)
        self.fps = max(1, int(fps))
        self._log = log
        self._writer: cv2.VideoWriter | None = None
        self._init_failed = False

    def _ensure_writer(self, frame) -> bool:
        if self._writer is not None:
            return self._writer.isOpened()
        if self._init_failed:
            return False
        h, w = frame.shape[:2]
        pipeline = (
            "appsrc is-live=true do-timestamp=true ! "
            f"video/x-raw,format=BGR,width={w},height={h},framerate={self.fps}/1 ! "
            "videoconvert ! video/x-raw,format=BGRx ! "
            "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
            f"nvv4l2h264enc insert-sps-pps=1 idrinterval={self.fps} "
            f"bitrate={self.bitrate} maxperf-enable=1 ! "
            "h264parse ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.host} port={self.port} sync=false async=false"
        )
        writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(self.fps), (w, h), True)
        if not writer.isOpened():
            self._init_failed = True
            self._log("[stream] Could not open H264 GStreamer writer; check nvv4l2h264enc.")
            return False
        self._writer = writer
        self._log(
            f"[stream] H264 UDP -> {self.host}:{self.port} "
            f"@ {self.fps}fps {self.bitrate // 1000}kbps"
        )
        return True

    def write(self, frame) -> bool:
        """Push a frame. Returns False if the writer is unavailable."""
        if not self._ensure_writer(frame):
            return False
        self._writer.write(frame)
        return True

    def release(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


class Preview:
    """One preview surface with three modes: ``h264`` | ``local`` | ``none``.

    - ``h264``  : stream to the laptop (auto-falls back to ``local`` if the
                  encoder can't open or no host is set).
    - ``local`` : ``cv2.imshow`` window (needs a DISPLAY / ``ssh -X``).
    - ``none``  : headless, ``show()`` is a no-op.
    """

    def __init__(
        self,
        window: str = "Preview",
        mode: str = "local",
        h264_host: str = "",
        h264_port: int = 5000,
        h264_bitrate: int = 2_000_000,
        fps: int = 15,
        log: Callable[[str], None] = print,
    ) -> None:
        self.window = window
        self.mode = (mode or "local").strip().lower()
        self.fps = max(1, int(fps))
        self._log = log
        self._streamer: H264Streamer | None = None
        self._window_created = False

        if self.mode == "h264":
            host = _resolve_h264_host(h264_host)
            if not host:
                self._log("[stream] mode=h264 needs H264_HOST (laptop IP); using local window.")
                self.mode = "local"
            else:
                self._streamer = H264Streamer(
                    host, h264_port, h264_bitrate, self.fps, log=log
                )
        if self.mode == "local" and not os.environ.get("DISPLAY"):
            self._log("[stream] mode=local but no DISPLAY; running headless (none).")
            self.mode = "none"

    @classmethod
    def from_env(
        cls,
        window: str = "Preview",
        fps: int = 15,
        log: Callable[[str], None] = print,
    ) -> "Preview":
        """Build from the shared env vars set by the run scripts.

        ``STREAM`` (h264|local|none), ``H264_HOST``, ``H264_PORT``,
        ``H264_BITRATE``. Default mode is ``local`` so a tool run bare on a
        Jetson with a monitor still shows a window.
        """
        return cls(
            window=window,
            mode=os.environ.get("STREAM", "local"),
            h264_host=os.environ.get("H264_HOST", ""),
            h264_port=int(os.environ.get("H264_PORT", "5000")),
            h264_bitrate=int(os.environ.get("H264_BITRATE", "2000000")),
            fps=fps,
            log=log,
        )

    def show(self, frame) -> None:
        if self.mode == "none":
            return
        if self.mode == "h264":
            if self._streamer is not None and self._streamer.write(frame):
                return
            # Encoder unavailable: degrade to a local window for the session.
            self._log("[stream] H264 unavailable; falling back to local window.")
            self.mode = "local" if os.environ.get("DISPLAY") else "none"
            if self.mode == "none":
                return
        if not self._window_created:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            self._window_created = True
        cv2.imshow(self.window, frame)
        cv2.waitKey(1)

    def close(self) -> None:
        if self._streamer is not None:
            self._streamer.release()
        if self._window_created:
            cv2.destroyWindow(self.window)
            self._window_created = False
