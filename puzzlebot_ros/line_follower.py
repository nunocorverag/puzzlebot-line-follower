#!/usr/bin/env python3

import csv
import json
import math
import os
import socket
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String

import cv2
import numpy as np
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from puzzlebot_ros.perception.intersection import (
    IntersectionParams,
    analyze_intersection,
    load_intersection_params,
)
from puzzlebot_ros.perception.lane import (
    LaneParams,
    analyze_lane,
    compute_homography,
    draw_birdseye_debug,
    draw_lane_overlay,
    load_lane_params,
    save_lane_params,
)
from puzzlebot_ros.perception.zebra import (
    ZebraParams,
    analyze_zebra,
    draw_zebra_overlay,
    load_zebra_params,
    save_zebra_params,
    wide_homography,
)
from dataclasses import fields as dataclass_fields
from puzzlebot_ros.perception.camera import open_csi_capture
from puzzlebot_ros.perception.stream import H264Streamer


# MJPEG server -------------------------------------------------
# Event-driven: each client connection blocks until a NEW frame is published
# and sends it exactly once. The previous implementation busy-looped and
# re-sent the same frame as fast as the socket allowed, flooding the link with
# duplicate frames and causing the laggy stream.
class _MJPEGHandler(BaseHTTPRequestHandler):
    _cond = threading.Condition()
    _frame = None       # latest JPEG bytes
    _seq = 0            # increments on every new frame

    @classmethod
    def update_frame(cls, data):
        with cls._cond:
            cls._frame = data
            cls._seq += 1
            cls._cond.notify_all()

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Connection', 'close')
        self.end_headers()
        last_seq = -1
        while True:
            with _MJPEGHandler._cond:
                while _MJPEGHandler._seq == last_seq:
                    _MJPEGHandler._cond.wait(timeout=5.0)
                data = _MJPEGHandler._frame
                last_seq = _MJPEGHandler._seq
            if not data:
                continue
            try:
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                    + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n'
                )
            except (BrokenPipeError, ConnectionResetError):
                break

    def log_message(self, *_):          # silence the default server logging
        pass


def _start_mjpeg_server(port=8080):
    srv = ThreadingHTTPServer(('0.0.0.0', port), _MJPEGHandler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv
# --------------------------------------------------------------

class AutonomousRacer(Node):

    def __init__(self):
        super().__init__('autonomous_racer')

        # =========================================================
        # Publishers
        # =========================================================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/traffic_state', 10)
        self.intersection_prompt_pub = self.create_publisher(String, '/intersection_prompt', 10)
        self.create_subscription(String, '/intersection_decision', self._intersection_decision_cb, 10)

        # =========================================================
        # Camera Setup (Jetson)
        # =========================================================
        self.cap = open_csi_capture(width=640, height=480, fps=30, downscale=True,
                                    log=self.get_logger().info)
        if self.cap is None:
            self.get_logger().error("Could not open camera")
            return

        self.declare_parameter('use_undistort', True)
        self.declare_parameter('camera_params_path', '')
        self.declare_parameter('use_illumination_correction', True)
        self.declare_parameter('illumination_params_path', '')
        # MJPEG stream tuning. Lower quality / fps / a capped width all cut
        # bandwidth, which is the usual cause of a laggy preview over WiFi.
        self.declare_parameter('stream_fps', 15)
        self.declare_parameter('stream_quality', 60)
        self.declare_parameter('stream_max_width', 0)   # 0 = keep full width
        self._stream_quality = int(self.get_parameter('stream_quality').value)
        self._stream_max_width = int(self.get_parameter('stream_max_width').value)
        stream_fps = max(1, int(self.get_parameter('stream_fps').value))
        self._stream_min_period = 1.0 / stream_fps
        self._last_stream_time = None

        # Optional hardware-encoded H264/RTP-over-UDP stream. Far lower bandwidth
        # than MJPEG because it compresses between frames using the Jetson's
        # nvv4l2h264enc encoder. Opt-in: stream_mode=h264 + h264_host=<laptop IP>.
        # Receive on the laptop with scripts/view_h264_stream.sh.
        self.declare_parameter('stream_mode', 'mjpeg')   # 'mjpeg' or 'h264'
        self.declare_parameter('h264_host', '')          # laptop IP for the UDP sink
        self.declare_parameter('h264_port', 5000)
        self.declare_parameter('h264_bitrate', 2000000)  # bits/s
        self._stream_mode = str(self.get_parameter('stream_mode').value).strip().lower()
        self._h264_host = str(self.get_parameter('h264_host').value).strip()
        self._h264_port = int(self.get_parameter('h264_port').value)
        self._h264_bitrate = int(self.get_parameter('h264_bitrate').value)
        self._h264_streamer = None
        if self._stream_mode == 'h264' and not self._h264_host:
            self.get_logger().error(
                'stream_mode=h264 needs h264_host (laptop IP). Falling back to MJPEG.'
            )
            self._stream_mode = 'mjpeg'
        elif self._stream_mode == 'h264':
            self._h264_streamer = H264Streamer(
                self._h264_host, self._h264_port, self._h264_bitrate,
                fps=stream_fps, log=self.get_logger().info,
            )

        self.declare_parameter('show_window', False)
        requested_window = bool(self.get_parameter('show_window').value)
        self.show_window = requested_window and bool(os.environ.get('DISPLAY'))
        if requested_window and not self.show_window:
            self.get_logger().warn('show_window requested, but DISPLAY is not set; running headless.')
        self.illumination_gain = self._load_illumination_gain()
        self.camera_matrix, self.dist_coeffs = self._load_camera_params()

        # =========================================================
        # Traffic Light Variables
        # =========================================================
        self.min_area = 500
        self.threshold_frames = 3

        self.current_state = "RED"
        self.last_state = "UNKNOWN"

        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0

        # =========================================================
        # Line Tracking Variables
        # =========================================================
        self.last_bottom_center = None
        self.last_top_center = None
        self.max_jump_distance = 80
        self.time_line_lost = None

        # Detection validation (robustness). Otsu always binarizes *something*,
        # so a uniform / low-contrast ROI (just floor, or a soft shadow) produces
        # phantom contours that yank the steering. Before trusting any candidate
        # we require the ROI to have enough contrast AND a sane foreground fill.
        self.declare_parameter('line_min_contrast', 18.0)   # min std-dev of ROI gray
        self.declare_parameter('line_min_fill_pct', 0.4)    # min % of ROI that is "line"
        self.declare_parameter('line_max_fill_pct', 70.0)   # above this it's noise/shadow
        self._line_min_contrast = float(self.get_parameter('line_min_contrast').value)
        self._line_min_fill_pct = float(self.get_parameter('line_min_fill_pct').value)
        self._line_max_fill_pct = float(self.get_parameter('line_max_fill_pct').value)

        # Intersection detection uses the shared perception module so the robot
        # behaves exactly like what was tuned in the calibrator. The tuned values
        # are loaded from the JSON the calibrator writes via 'save_calib'.
        self.declare_parameter('intersection_params_path', '')
        self.intersection_params = self._load_intersection_params()
        self.intersection_stable = 0          # consecutive-frame counter (threaded into the detector)
        self.intersection_result = None       # latest detector result while a prompt is pending
        self.intersection_phase = None        # None | 'approach' | 'wait'
        self.intersection_pending = False
        self.intersection_options = []
        self.intersection_decision = None
        self.last_prompt_time = None
        self.commit_direction = None
        self.commit_until = None             # MAX (safety) time for the maneuver
        self._commit_min_until = None        # MIN time before re-acquire can end it
        self.intersection_cooldown_until = None

        # Approach-and-center: when the intersection is first detected the robot
        # keeps following the line at a slow creep until the entry zebra reaches
        # the target depth AND is centered, so it always stops at the same spot.
        # NOTE: the motors have a deadband ~0.08-0.10 m/s (0.05 does not move the
        # robot, ~0.10 does). approach_speed must stay above it or the creep
        # never actually drives.
        self.declare_parameter('approach_target_entry_y_pct', 82)
        self._approach_target_entry_y_pct = float(self.get_parameter('approach_target_entry_y_pct').value)
        self._approach_speed = 0.06

        # Robust zebra detector (bird's-eye, ground-coordinate). When enabled it
        # replaces the legacy raw-image entry-line geometry for the intersection
        # trigger + the stop: it detects the zebra row in a dedicated WIDE warp and
        # measures the forward DISTANCE in cm, so the robot stops the same way out
        # of a straight or a curve (the legacy geometry was noise on curves and the
        # robot drove through). Set use_zebra_bev:=false to fall back to legacy.
        self.declare_parameter('use_zebra_bev', True)
        self._use_zebra_bev = bool(self.get_parameter('use_zebra_bev').value)
        self.declare_parameter('zebra_params_path', '')
        self.zebra_params = self._load_zebra_params()
        self._zebra_M = None              # cached wide homography
        self._zebra_frame_size = None
        self._zebra_stable = 0
        self.zebra_result = None
        self._zebra_opt_votes = {}        # exit -> frames seen during APPROACH
        self.declare_parameter('zebra_opt_min_votes', 2)
        self._zebra_opt_min_votes = int(self.get_parameter('zebra_opt_min_votes').value)

        # Testing aid: ignore the traffic-light supervisor so the robot drives
        # without needing to see a real GREEN light.
        self.declare_parameter('ignore_traffic_light', False)
        self._ignore_traffic_light = bool(self.get_parameter('ignore_traffic_light').value)

        # Motion master switch for safe testing. Starts disabled so the robot
        # never moves until you explicitly enable it from the terminal via
        # /drive_enable (scripts/set_drive_jetson.sh on|off). Perception and the
        # stream keep running while disabled, so you can watch detection.
        self.declare_parameter('start_driving', False)
        self._drive_enabled = bool(self.get_parameter('start_driving').value)
        self.create_subscription(Bool, '/drive_enable', self._drive_enable_cb, 10)

        # ---------------------------------------------------------
        # Per-lane persistent anchors for the top ROI's 3 lines.
        # Each slot stores the last known (cx, cy) for that lane.
        # They are initialized lazily on first detection.
        # ---------------------------------------------------------
        self.last_left_anchor   = None   # Left  limit circle (yellow)
        self.last_middle_anchor = None   # Center prediction circle (red)
        self.last_right_anchor  = None   # Right limit circle  (purple)

        # Max pixel distance a candidate may jump per frame for each anchor.
        # Keep this somewhat generous so the tracker can still recover from
        # occlusion; set it tighter than the inter-lane spacing.
        self.anchor_max_jump = 100

        # =========================================================
        # PD Controller (live-tunable via the param tuner / rqt_reconfigure /
        # scripts/set_gain_jetson.sh). Saved gains persist in control_params.json.
        # =========================================================
        self._control_params_path = self._config_save_path('control_params.json')
        saved = {}
        found = self._find_config('control_params.json')
        if found is not None:
            try:
                saved = json.loads(found.read_text())
                self.get_logger().info(f'Loaded control gains: {found}')
            except (OSError, json.JSONDecodeError) as exc:
                self.get_logger().warn(f'control_params.json unreadable ({exc})')
        else:
            self.get_logger().warn('control_params.json not found; using defaults')
        self.declare_parameter('kp', float(saved.get('kp', 0.003)))
        self.declare_parameter('kd', float(saved.get('kd', 0.008)))
        self.declare_parameter('max_v', float(saved.get('max_v', 0.08)))
        self.declare_parameter('max_w', float(saved.get('max_w', 0.6)))
        # Curve feedforward: steer ahead by the bend (far offset - near offset),
        # weighted by ff_gain and the SAME kp. 0 = pure feedback (old behavior);
        # ~1 = anticipate the curve. It is the bend term, so straights are
        # unaffected and the existing straight-line PD tuning is preserved.
        self.declare_parameter('ff_gain', float(saved.get('ff_gain', 1.0)))
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.ff_gain = float(self.get_parameter('ff_gain').value)

        # Robust-intersection knobs (all live-tunable + persisted in
        # control_params.json). See docs/RUNBOOK.md "Intersections".
        #   k_align         : heading-align gain in APPROACH (rotate the zebra
        #                     horizontal); 0 = lateral-only centering.
        #   slow speed      : FOLLOW speed cap once a zebra is seen (kills the
        #                     feedforward overshoot before the cross).
        #   commit_*        : the open-loop turn maneuver (NEEDS on-robot tuning).
        #   min_travel      : distance to cover after a turn before the next cross
        #                     can fire (double-intersection guard).
        self.declare_parameter('k_align', float(saved.get('k_align', 0.6)))
        self.declare_parameter('intersection_slow_speed', float(saved.get('intersection_slow_speed', 0.08)))
        self.declare_parameter('approach_speed', float(saved.get('approach_speed', 0.06)))
        self.declare_parameter('approach_align_slope', float(saved.get('approach_align_slope', 0.15)))
        self.declare_parameter('approach_timeout_s', float(saved.get('approach_timeout_s', 10.0)))
        self.declare_parameter('commit_speed', float(saved.get('commit_speed', 0.08)))
        self.declare_parameter('commit_turn_w', float(saved.get('commit_turn_w', 0.6)))
        self.declare_parameter('commit_duration', float(saved.get('commit_duration', 2.0)))
        self.declare_parameter('commit_duration_straight', float(saved.get('commit_duration_straight', 1.5)))
        # Closed-loop commit: keep turning/crossing until the lane is RE-ACQUIRED
        # (after a min time to clear the cross), capped by commit_duration above so
        # a missed line can't spin forever. 0 = old pure open-loop (time only).
        self.declare_parameter('commit_min_s', float(saved.get('commit_min_s', 0.8)))
        self.declare_parameter('commit_closed_loop', bool(saved.get('commit_closed_loop', True)))
        self.declare_parameter('intersection_min_travel_m', float(saved.get('intersection_min_travel_m', 0.25)))
        # Square-up-in-place: at the cross, if we stopped skewed (came off a curve)
        # rotate IN PLACE (v=0, safe -- no arcing) to face the cross before reading
        # options/asking. Uses k_align as the gain (rad of zebra angle -> w); flip
        # k_align's sign live if it turns the wrong way.
        self.declare_parameter('align_in_place', bool(saved.get('align_in_place', True)))
        self.declare_parameter('align_tol_deg', float(saved.get('align_tol_deg', 12.0)))
        self.declare_parameter('align_timeout_s', float(saved.get('align_timeout_s', 4.0)))
        self._k_align = float(self.get_parameter('k_align').value)
        self._intersection_slow_speed = float(self.get_parameter('intersection_slow_speed').value)
        self._approach_speed = float(self.get_parameter('approach_speed').value)
        self._approach_align_slope = float(self.get_parameter('approach_align_slope').value)
        self._approach_timeout_s = float(self.get_parameter('approach_timeout_s').value)
        self._commit_speed = float(self.get_parameter('commit_speed').value)
        self._commit_turn_w = float(self.get_parameter('commit_turn_w').value)
        self._commit_duration = float(self.get_parameter('commit_duration').value)
        self._commit_duration_straight = float(self.get_parameter('commit_duration_straight').value)
        self._commit_min_s = float(self.get_parameter('commit_min_s').value)
        self._commit_closed_loop = bool(self.get_parameter('commit_closed_loop').value)
        self._intersection_min_travel_m = float(self.get_parameter('intersection_min_travel_m').value)
        self._align_in_place = bool(self.get_parameter('align_in_place').value)
        self._align_tol_deg = float(self.get_parameter('align_tol_deg').value)
        self._align_timeout_s = float(self.get_parameter('align_timeout_s').value)
        self._align_start_time = None        # for the square-up-in-place timeout
        self._approach_start_time = None     # for the APPROACH timeout
        self._dist_since_commit = 1e9        # distance proxy since last turn (m)
        self._near_intersection = False      # zebra seen this frame (slow zone)

        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time = self.get_clock().now()

        # Line-lost recovery (robustness). When both ROIs lose the line we no
        # longer drive blindly straight (that runs off the track on a curve).
        # Instead we pivot toward the side the line was last seen, encoded by the
        # sign of the last PD error, until it re-acquires or the search times out.
        self.declare_parameter('recover_seconds', 3.0)  # search this long, then stop
        self.declare_parameter('recover_turn', 0.4)      # angular speed while searching
        self.declare_parameter('recover_speed', 0.03)    # tiny forward creep while searching
        self._recover_seconds = float(self.get_parameter('recover_seconds').value)
        self._recover_turn = float(self.get_parameter('recover_turn').value)
        self._recover_speed = float(self.get_parameter('recover_speed').value)

        # =========================================================
        # Bird's-eye lane follower (robust primary path)
        # =========================================================
        # Warps the ground to a top-down view and tracks the center line with a
        # center-restricted sliding window, so parallel floor seams / off-lane
        # lines can't hijack steering. Falls back to the legacy two-ROI detector
        # when disabled or not confident (e.g. warp not tuned yet), so the robot
        # always follows *something*. Tune the warp live in the calibrator.
        self.declare_parameter('use_birdseye', True)
        self.declare_parameter('lane_params_path', '')
        self.declare_parameter('curve_slow_gain', 0.6)   # speed *= 1 - gain*|curv|
        self.declare_parameter('curve_min_scale', 0.4)   # never below this fraction
        self._use_birdseye = bool(self.get_parameter('use_birdseye').value)
        self._curve_slow_gain = float(self.get_parameter('curve_slow_gain').value)
        self._curve_min_scale = float(self.get_parameter('curve_min_scale').value)
        self.lane_params = self._load_lane_params()
        # Expose every LaneParams field as a live ROS param (lane.<field>) so the
        # warp can be tuned live (param tuner / rqt) and saved back to JSON.
        self._lane_param_names = []
        for f in dataclass_fields(LaneParams):
            name = f"lane.{f.name}"
            self.declare_parameter(name, int(getattr(self.lane_params, f.name)))
            self._lane_param_names.append(name)
        # Same for the zebra detector (zebra.<field>) so the stop distance, slow
        # distance, etc. can be tuned live and saved to zebra_params.json.
        self._zebra_param_names = []
        for f in dataclass_fields(ZebraParams):
            name = f"zebra.{f.name}"
            self.declare_parameter(name, getattr(self.zebra_params, f.name))
            self._zebra_param_names.append(name)
        self._lane_M = None              # cached homography (lazy, per frame size)
        self._lane_prev_base = None      # previous lane base x (continuity)
        self._lane_Minv = None
        self._lane_frame_size = None

        # ---------------------------------------------------------
        # Diagnostics: status HUD + optional controller CSV log.
        # ---------------------------------------------------------
        self._last_lane_result = None    # latest LaneResult for the HUD/CSV
        self.declare_parameter('controller_log', False)
        self.declare_parameter('controller_log_path', '')
        self._csv_fp = None
        self._csv_writer = None
        self._event_fp = None
        self._t0 = self.get_clock().now()
        if bool(self.get_parameter('controller_log').value):
            log_path = str(self.get_parameter('controller_log_path').value).strip() or \
                str(self._snapshot_dir() / 'controller_data.csv')
            try:
                Path(log_path).parent.mkdir(parents=True, exist_ok=True)
                self._csv_fp = open(log_path, 'w', newline='')
                self._csv_writer = csv.writer(self._csv_fp)
                self._csv_writer.writerow([
                    't', 'state', 'phase', 'commit', 'pending', 'decision', 'options',
                    'lane_src', 'off', 'conf', 'curv', 'base_x',
                    'z_seen', 'z_dist_cm', 'z_angle_deg', 'z_ndashes', 'z_span_cm',
                    'z_options', 'near_intersection',
                    'error', 'deriv', 'v', 'w',
                    'kp', 'kd', 'ff_gain', 'max_v', 'max_w',
                    'stop_cm', 'slow_cm', 'commit_min_s', 'commit_turn_w'])
                self.get_logger().info(f'[LOG] controller CSV -> {log_path}')
            except OSError as exc:
                self.get_logger().error(f'[LOG] could not open CSV ({exc}); disabled')
                self._csv_fp = self._csv_writer = None
        self.declare_parameter('session_log', True)
        self.declare_parameter('session_log_path', '')
        if bool(self.get_parameter('session_log').value):
            log_path = str(self.get_parameter('session_log_path').value).strip()
            if not log_path:
                log_path = str(self._snapshot_dir() / 'events.jsonl')
            try:
                Path(log_path).parent.mkdir(parents=True, exist_ok=True)
                self._event_fp = open(log_path, 'a', buffering=1)
                self.get_logger().info(f'[LOG] session events -> {log_path}')
            except OSError as exc:
                self.get_logger().error(f'[LOG] could not open events log ({exc}); disabled')
                self._event_fp = None

        # Operator reset for the intersection state machine
        # (scripts/set_intersection_jetson.sh reset): clears phase/decision/commit.
        self.create_subscription(Bool, '/intersection_reset',
                                 self._intersection_reset_cb, 10)

        # Live lane metrics for the tuner / Foxglove / PlotJuggler:
        # [off, conf, curv, v, w].
        self.lane_status_pub = self.create_publisher(Float32MultiArray, '/lane_status', 10)
        # Persist current tunables to JSON on demand (tuner 'save' key).
        self.create_subscription(Bool, '/save_params', self._save_params_cb, 10)

        # Live telemetry over UDP for the laptop dashboard (no ROS on the laptop,
        # same pattern as the H264 stream). Broadcasts a compact JSON status; the
        # dashboard (tools/dashboard.py) renders the state machine + detection +
        # params + a rolling log. Reuses h264_host if telemetry_host is empty.
        self.declare_parameter('telemetry_host', '')
        self.declare_parameter('telemetry_port', 5055)
        thost = str(self.get_parameter('telemetry_host').value).strip() or self._h264_host
        self._telemetry_addr = None
        self._telemetry_sock = None
        if thost:
            self._telemetry_addr = (thost, int(self.get_parameter('telemetry_port').value))
            self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info(f'Telemetry UDP -> {self._telemetry_addr}')
        self._telemetry_last = self.get_clock().now()

        # Periodic snapshot recorder (tuner 'r' -> /recorder_enable). Saves the
        # annotated frame every snapshot_interval s for offline review; pull with
        # scripts/pull_follower_snapshots.sh.
        self.declare_parameter('snapshot_interval', float(saved.get('snapshot_interval', 0.5)))
        self.declare_parameter('record_on_start', False)
        self._snapshot_interval = float(self.get_parameter('snapshot_interval').value)
        self._recording = bool(self.get_parameter('record_on_start').value)
        self._last_snap_t = self.get_clock().now()
        self._snap_count = 0
        self.create_subscription(Bool, '/recorder_enable', self._recorder_cb, 10)

        # Live PD/warp tuning via ros2 param set (param tuner / rqt_reconfigure /
        # scripts/set_gain_jetson.sh). Registered LAST so it sees all declarations.
        self.add_on_set_parameters_callback(self._on_set_params)

        # Timer (30 Hz)
        self.timer = self.create_timer(0.033, self.control_loop)

        # MJPEG server (access from the PC: http://10.10.0.100:8080)
        _start_mjpeg_server(port=8080)
        self.get_logger().info("Autonomous Racer Started: Lines + Traffic Lights")
        self.get_logger().info("MJPEG stream available at http://10.10.0.100:8080")

        # Quiet the per-frame INFO spam ([VISION]/[MATH]/[LANE]/[TRACKING]/[ACTION])
        # unless verbose; warnings (line lost, intersection, transitions) still show.
        # The video HUD is the live state display.
        self.declare_parameter('verbose', False)
        if not bool(self.get_parameter('verbose').value):
            self.get_logger().set_level(LoggingSeverity.WARN)

    def _package_config_path(self, filename):
        if get_package_share_directory is None:
            return None
        try:
            return Path(get_package_share_directory("puzzlebot_ros")) / "config" / filename
        except Exception:
            return None

    def _find_config(self, filename):
        """First existing config path (source tree, then installed/share)."""
        candidates = [
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config') / filename,
            Path(__file__).resolve().parents[1] / 'config' / filename,
        ]
        pkg = self._package_config_path(filename)
        if pkg is not None:
            candidates.append(pkg)
        for c in candidates:
            if c.exists():
                return c
        return None

    def _config_save_path(self, filename):
        """Where to WRITE config: the source tree (persists + gets synced)."""
        src = Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config')
        base = src if src.is_dir() else (Path(__file__).resolve().parents[1] / 'config')
        return base / filename

    def _load_camera_params(self):
        if not bool(self.get_parameter('use_undistort').value):
            self.get_logger().info('Camera undistortion disabled.')
            return None, None

        configured_path = str(self.get_parameter('camera_params_path').value).strip()
        candidate_paths = []
        if configured_path:
            candidate_paths.append(Path(configured_path).expanduser())
        candidate_paths.extend([
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config/camera_params.npz'),
            Path(__file__).resolve().parents[1] / 'config' / 'camera_params.npz',
        ])
        package_config = self._package_config_path('camera_params.npz')
        if package_config is not None:
            candidate_paths.append(package_config)

        for params_path in candidate_paths:
            if params_path.exists():
                data = np.load(str(params_path))
                self.get_logger().info(f'Loaded camera calibration: {params_path}')
                return data['camera_matrix'], data['dist_coeffs']

        self.get_logger().warn('Camera calibration not found; running without undistort.')
        return None, None

    def _load_intersection_params(self):
        configured_path = str(self.get_parameter('intersection_params_path').value).strip()
        candidate_paths = []
        if configured_path:
            candidate_paths.append(Path(configured_path).expanduser())
        candidate_paths.extend([
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config/intersection_params.json'),
            Path(__file__).resolve().parents[1] / 'config' / 'intersection_params.json',
        ])
        package_config = self._package_config_path('intersection_params.json')
        if package_config is not None:
            candidate_paths.append(package_config)
        for params_path in candidate_paths:
            if params_path.exists():
                params = load_intersection_params(params_path)
                self.get_logger().info(f'Loaded intersection calibration: {params_path}')
                return params
        self.get_logger().warn('Intersection calibration JSON not found; using built-in defaults.')
        return IntersectionParams()

    def _load_zebra_params(self):
        configured_path = str(self.get_parameter('zebra_params_path').value).strip()
        candidate_paths = []
        if configured_path:
            candidate_paths.append(Path(configured_path).expanduser())
        candidate_paths.extend([
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config/zebra_params.json'),
            Path(__file__).resolve().parents[1] / 'config' / 'zebra_params.json',
        ])
        package_config = self._package_config_path('zebra_params.json')
        if package_config is not None:
            candidate_paths.append(package_config)
        for params_path in candidate_paths:
            if params_path.exists():
                params = load_zebra_params(params_path)
                self.get_logger().info(f'Loaded zebra calibration: {params_path}')
                return params
        self.get_logger().warn('Zebra calibration JSON not found; using built-in defaults.')
        return ZebraParams()

    def _load_lane_params(self):
        configured_path = str(self.get_parameter('lane_params_path').value).strip()
        candidate_paths = []
        if configured_path:
            candidate_paths.append(Path(configured_path).expanduser())
        candidate_paths.extend([
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config/lane_params.json'),
            Path(__file__).resolve().parents[1] / 'config' / 'lane_params.json',
        ])
        package_config = self._package_config_path('lane_params.json')
        if package_config is not None:
            candidate_paths.append(package_config)
        for params_path in candidate_paths:
            if params_path.exists():
                params = load_lane_params(params_path)
                self.get_logger().info(f'Loaded lane calibration: {params_path}')
                return params
        self.get_logger().warn(
            'Lane calibration JSON not found; using built-in warp defaults '
            '(tune in the calibrator). Bird\'s-eye falls back to legacy ROIs '
            'until confident.'
        )
        return LaneParams()

    def _load_illumination_gain(self):
        if not bool(self.get_parameter('use_illumination_correction').value):
            self.get_logger().info('Illumination correction disabled.')
            return None

        configured_path = str(self.get_parameter('illumination_params_path').value).strip()
        candidate_paths = []
        if configured_path:
            candidate_paths.append(Path(configured_path).expanduser())
        candidate_paths.extend([
            Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros/config/illumination_flatfield.npz'),
            Path(__file__).resolve().parents[1] / 'config' / 'illumination_flatfield.npz',
        ])
        package_config = self._package_config_path('illumination_flatfield.npz')
        if package_config is not None:
            candidate_paths.append(package_config)

        for params_path in candidate_paths:
            if params_path.exists():
                data = np.load(str(params_path))
                self.get_logger().info(f'Loaded illumination calibration: {params_path}')
                return data['gain'].astype(np.float32)

        self.get_logger().warn('Illumination calibration not found; running without flat-field correction.')
        return None

    def _apply_illumination_gain(self, frame):
        if self.illumination_gain is None:
            return frame
        gain = self.illumination_gain
        if gain.shape[:2] != frame.shape[:2]:
            gain = cv2.resize(gain, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
        corrected = frame.astype(np.float32) * gain
        return np.clip(corrected, 0, 255).astype(np.uint8)

    def _drive_enable_cb(self, msg):
        self._drive_enabled = bool(msg.data)
        self.get_logger().info(f"[DRIVE] enabled={self._drive_enabled}")

    def _intersection_decision_cb(self, msg):
        decision = msg.data.strip().lower()
        aliases = {
            'left': 'left', 'l': 'left', 'izquierda': 'left', 'i': 'left',
            'straight': 'straight', 's': 'straight', 'forward': 'straight',
            'front': 'straight', 'adelante': 'straight', 'recto': 'straight',
            'right': 'right', 'r': 'right', 'derecha': 'right', 'd': 'right',
        }
        normalized = aliases.get(decision)
        if normalized is None:
            self.get_logger().warn(
                f"Intersection decision '{msg.data}' ignored. Use left, straight, or right."
            )
            self._event('decision_ignored', raw=msg.data, reason='unknown')
            return
        # Only enforce the option list when we actually classified some options.
        # If detection fired but no direction could be validated, trust the operator.
        if self.intersection_pending and self.intersection_options and normalized not in self.intersection_options:
            self.get_logger().warn(
                f"Decision '{normalized}' not in current options: {', '.join(self.intersection_options)}"
            )
            self._event('decision_rejected', decision=normalized,
                        allowed=list(self.intersection_options))
            return
        self.intersection_decision = normalized
        self.get_logger().info(f"Intersection decision received: {normalized}")
        self._event('decision_received', decision=normalized)

    def _intersection_reset_cb(self, msg):
        """Operator escape hatch: clear the whole intersection state machine so
        the robot drops back to plain line following (e.g. stuck in WAIT)."""
        if not msg.data:
            return
        self.intersection_phase = None
        self.intersection_decision = None
        self.intersection_pending = False
        self.intersection_options = []
        self.commit_direction = None
        self.commit_until = None
        self.intersection_result = None
        self.intersection_stable = 0
        self.last_prompt_time = None
        self.intersection_cooldown_until = None
        self.get_logger().warn('[INTERSECTION] state RESET by operator -> FOLLOW')
        self._event('intersection_reset')

    def _on_set_params(self, params):
        """Apply live PD/curve/warp tuning from ros2 param set without a restart."""
        lane_changed = False
        for p in params:
            if p.name == 'kp':
                self.kp = float(p.value)
            elif p.name == 'kd':
                self.kd = float(p.value)
            elif p.name == 'max_v':
                self.max_v = float(p.value)
            elif p.name == 'max_w':
                self.max_w = float(p.value)
            elif p.name == 'ff_gain':
                self.ff_gain = float(p.value)
            elif p.name == 'snapshot_interval':
                self._snapshot_interval = float(p.value)   # live recorder rate (s)
            elif p.name == 'curve_slow_gain':
                self._curve_slow_gain = float(p.value)
            elif p.name == 'curve_min_scale':
                self._curve_min_scale = float(p.value)
            elif p.name == 'k_align':
                self._k_align = float(p.value)
            elif p.name == 'intersection_slow_speed':
                self._intersection_slow_speed = float(p.value)
            elif p.name == 'approach_align_slope':
                self._approach_align_slope = float(p.value)
            elif p.name == 'approach_timeout_s':
                self._approach_timeout_s = float(p.value)
            elif p.name == 'commit_speed':
                self._commit_speed = float(p.value)
            elif p.name == 'commit_turn_w':
                self._commit_turn_w = float(p.value)
            elif p.name == 'commit_duration':
                self._commit_duration = float(p.value)
            elif p.name == 'commit_duration_straight':
                self._commit_duration_straight = float(p.value)
            elif p.name == 'intersection_min_travel_m':
                self._intersection_min_travel_m = float(p.value)
            elif p.name == 'commit_min_s':
                self._commit_min_s = float(p.value)
            elif p.name == 'commit_closed_loop':
                self._commit_closed_loop = bool(p.value)
            elif p.name == 'align_in_place':
                self._align_in_place = bool(p.value)
            elif p.name == 'align_tol_deg':
                self._align_tol_deg = float(p.value)
            elif p.name == 'align_timeout_s':
                self._align_timeout_s = float(p.value)
            elif p.name.startswith('lane.'):
                field = p.name[len('lane.'):]
                if hasattr(self.lane_params, field):
                    setattr(self.lane_params, field, int(p.value))
                    lane_changed = True
            elif p.name.startswith('zebra.'):
                field = p.name[len('zebra.'):]
                if hasattr(self.zebra_params, field):
                    cur = getattr(self.zebra_params, field)
                    setattr(self.zebra_params, field, type(cur)(p.value))
                    if field in ('widen_kx', 'warp_w', 'warp_h'):
                        self._zebra_M = None          # geometry -> rebuild warp
                        self._zebra_frame_size = None
        if lane_changed:
            # Trapezoid/size may have moved -> rebuild the homography next frame.
            self._lane_M = None
            self._lane_frame_size = None
        return SetParametersResult(successful=True)

    def _save_params_cb(self, msg):
        """Persist current tunables to config JSON (warp + control gains)."""
        if not msg.data:
            return
        try:
            lane_path = self._config_save_path('lane_params.json')
            save_lane_params(self.lane_params, lane_path)
            self._control_params_path.parent.mkdir(parents=True, exist_ok=True)
            self._control_params_path.write_text(json.dumps({
                'kp': self.kp, 'kd': self.kd,
                'max_v': self.max_v, 'max_w': self.max_w,
                'ff_gain': self.ff_gain,
                'curve_slow_gain': self._curve_slow_gain,
                'curve_min_scale': self._curve_min_scale,
                'k_align': self._k_align,
                'intersection_slow_speed': self._intersection_slow_speed,
                'approach_align_slope': self._approach_align_slope,
                'approach_timeout_s': self._approach_timeout_s,
                'commit_speed': self._commit_speed,
                'commit_turn_w': self._commit_turn_w,
                'commit_duration': self._commit_duration,
                'commit_duration_straight': self._commit_duration_straight,
                'commit_min_s': self._commit_min_s,
                'commit_closed_loop': self._commit_closed_loop,
                'intersection_min_travel_m': self._intersection_min_travel_m,
                'snapshot_interval': self._snapshot_interval,
            }, indent=2))
            zebra_path = self._config_save_path('zebra_params.json')
            save_zebra_params(self.zebra_params, zebra_path)
            self.get_logger().warn(
                f'[SAVE] wrote {lane_path.name} + {self._control_params_path.name} '
                f'+ {zebra_path.name}')
        except OSError as exc:
            self.get_logger().error(f'[SAVE] failed: {exc}')

    def _phase_label(self):
        """High-level state for the HUD/CSV: (text, BGR color)."""
        if not self._drive_enabled:
            return ("HOLD: drive OFF", (0, 165, 255))
        effective = "GREEN" if self._ignore_traffic_light else self.current_state
        if effective == "RED":
            return ("STOP: red light", (0, 0, 255))
        if self.intersection_phase == 'wait':
            return ("WAIT: decision", (0, 0, 255))
        if self.intersection_phase == 'approach':
            return ("APPROACH", (0, 255, 255))
        if self.commit_direction is not None:
            return (f"COMMIT {self.commit_direction}", (255, 160, 0))
        if self.time_line_lost is not None:
            return ("RECOVER: line lost", (0, 128, 255))
        return ("FOLLOW", (0, 255, 0))

    def _draw_status_hud(self, frame, cmd):
        """Translucent top banner: state, drive/light, lane metrics, gains, cmd."""
        h, w = frame.shape[:2]
        label, color = self._phase_label()
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 54), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        lr = self._last_lane_result
        src = "BEV" if lr is not None else "legacy"
        off = lr.offset_norm if lr is not None else 0.0
        conf = lr.confidence if lr is not None else 0.0
        curv = lr.curvature_norm if lr is not None else 0.0
        light = "IGN" if self._ignore_traffic_light else self.current_state

        zextra = ""
        if self._use_zebra_bev and self.zebra_result is not None and self.zebra_result.seen:
            zr = self.zebra_result
            zd = "?" if zr.distance_cm is None else f"{zr.distance_cm:.0f}"
            zextra = f"  ZEB:{zd}cm[{','.join(zr.options) or '-'}]"

        cv2.putText(frame, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        line2 = (f"drive:{'ON' if self._drive_enabled else 'off'}  light:{light}  "
                 f"{src} off:{off:+.2f} conf:{conf:.2f} curv:{curv:+.2f}  "
                 f"v:{cmd.linear.x:.3f} w:{cmd.angular.z:+.2f}{zextra}")
        cv2.putText(frame, line2, (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        gains = (f"kp:{self.kp:.4f} kd:{self.kd:.4f} ff:{self.ff_gain:.2f} "
                 f"mv:{self.max_v:.2f} mw:{self.max_w:.2f}")
        cv2.putText(frame, gains, (max(10, w - 420), 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        if self._recording:
            cv2.putText(frame, f"REC {self._snap_count}", (max(10, w - 110), 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    def _recorder_cb(self, msg):
        self._recording = bool(msg.data)
        self.get_logger().warn(
            f'[REC] recording {"ON" if self._recording else "off"} '
            f'(total snaps: {self._snap_count})')
        self._event('recorder', enabled=self._recording, snaps=self._snap_count)

    def _event(self, name, **fields):
        if self._event_fp is None:
            return
        now = self.get_clock().now()
        lr = self._last_lane_result
        zr = self.zebra_result
        row = {
            't': round((now - self._t0).nanoseconds * 1e-9, 3),
            'event': name,
            'state': self._phase_label()[0],
            'phase': self.intersection_phase,
            'commit': self.commit_direction,
            'pending': bool(self.intersection_pending),
            'decision': self.intersection_decision,
            'options': list(self.intersection_options),
            'lane': None if lr is None else {
                'detected': bool(lr.detected),
                'off': round(float(lr.offset_norm), 3),
                'conf': round(float(lr.confidence), 3),
                'curv': round(float(lr.curvature_norm), 3),
                'base_x': None if lr.base_x is None else round(float(lr.base_x), 1),
            },
            'zebra': None if zr is None else {
                'seen': bool(zr.seen),
                'dist': None if zr.distance_cm is None else round(float(zr.distance_cm), 1),
                'angle': None if zr.angle_deg is None else round(float(zr.angle_deg), 1),
                'ndash': int(zr.n_dashes),
                'span': round(float(zr.span_cm), 1),
                'options': list(zr.options),
                'debug': getattr(zr, 'option_debug', {}),
            },
        }
        row.update(fields)
        try:
            self._event_fp.write(json.dumps(row, sort_keys=True) + '\n')
        except OSError:
            pass

    def _snapshot_dir(self):
        base = Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros')
        if not base.is_dir():
            base = Path(__file__).resolve().parents[1]
        return base / 'debug_dataset' / 'follower_session'

    def _maybe_snapshot(self, now, frame):
        if not self._recording or self._snapshot_interval <= 0:
            return
        if (now - self._last_snap_t).nanoseconds * 1e-9 < self._snapshot_interval:
            return
        self._last_snap_t = now
        try:
            d = self._snapshot_dir()
            d.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            state = self._phase_label()[0].split(':')[0].replace(' ', '')
            # Glue the bird's-eye debug (mask + sliding windows + fit) beside the
            # annotated frame so a pulled snapshot shows exactly what the lane
            # search saw -- the only way to diagnose curve failures offline.
            out = frame
            lr = self._last_lane_result
            if lr is not None and lr.warped_mask is not None:
                bev = draw_birdseye_debug(lr, self.lane_params)
                scale = frame.shape[0] / float(bev.shape[0])
                bev = cv2.resize(bev, (int(bev.shape[1] * scale), frame.shape[0]))
                out = cv2.hconcat([frame, bev])
            # Glue the WIDE zebra BEV with the detected row, so a recorded session
            # shows exactly what the zebra detector saw (the lane BEV above is a
            # different, narrower warp and does not show the zebra detection).
            if (self._use_zebra_bev and self._zebra_M is not None
                    and self.zebra_result is not None):
                zp = self.zebra_params
                zbev = cv2.warpPerspective(frame, self._zebra_M, (zp.warp_w, zp.warp_h))
                zbev = draw_zebra_overlay(zbev, self.zebra_result)
                zscale = frame.shape[0] / float(zbev.shape[0])
                zbev = cv2.resize(zbev, (int(zbev.shape[1] * zscale), frame.shape[0]))
                out = cv2.hconcat([out, zbev])
            cv2.imwrite(str(d / f'follow_{stamp}_{state}.jpg'), out)
            self._snap_count += 1
        except OSError as exc:
            self.get_logger().error(f'[REC] snapshot failed: {exc}')

    def _publish_lane_status(self, cmd):
        lr = self._last_lane_result
        msg = Float32MultiArray()
        msg.data = [
            float(lr.offset_norm) if lr else 0.0,
            float(lr.confidence) if lr else 0.0,
            float(lr.curvature_norm) if lr else 0.0,
            float(cmd.linear.x), float(cmd.angular.z)]
        self.lane_status_pub.publish(msg)

    def _publish_telemetry(self, now, cmd):
        """Broadcast a compact JSON status to the laptop dashboard (~10 Hz)."""
        if self._telemetry_sock is None:
            return
        if (now - self._telemetry_last).nanoseconds * 1e-9 < 0.1:
            return
        self._telemetry_last = now
        lr = self._last_lane_result
        zr = self.zebra_result
        zp = self.zebra_params
        zebra = None
        if self._use_zebra_bev and zr is not None:
            zebra = {
                'seen': bool(zr.seen),
                'dist': None if zr.distance_cm is None else round(zr.distance_cm, 1),
                'angle': None if zr.angle_deg is None else round(zr.angle_deg, 0),
                'ndash': int(zr.n_dashes),
                'span': round(zr.span_cm, 1),
                'options': list(zr.options),
                'debug': getattr(zr, 'option_debug', {}),
            }
        data = {
            't': round((now - self._t0).nanoseconds * 1e-9, 1),
            'state': self._phase_label()[0],
            'drive': bool(self._drive_enabled),
            'light': 'IGN' if self._ignore_traffic_light else self.current_state,
            'phase': self.intersection_phase,
            'commit': self.commit_direction,
            'options': list(self.intersection_options),
            'zebra': zebra,
            'lane': {
                'off': round(float(lr.offset_norm), 2) if lr else None,
                'conf': round(float(lr.confidence), 2) if lr else None,
                'curv': round(float(lr.curvature_norm), 2) if lr else None,
                'src': 'BEV' if lr is not None else 'legacy',
            },
            'cmd': {'v': round(float(cmd.linear.x), 3), 'w': round(float(cmd.angular.z), 2)},
            'params': {
                'use_zebra_bev': bool(self._use_zebra_bev),
                'kp': self.kp, 'kd': self.kd, 'max_v': self.max_v, 'max_w': self.max_w,
                'ff_gain': self.ff_gain,
                'slow_cm': zp.slow_distance_cm, 'stop_cm': zp.stop_distance_cm,
                'approach_v': self._approach_speed,
            },
        }
        try:
            self._telemetry_sock.sendto(json.dumps(data).encode(), self._telemetry_addr)
        except OSError:
            pass

    def _log_controller_row(self, now, cmd):
        if self._csv_writer is None:
            return
        t = (now - self._t0).nanoseconds * 1e-9
        lr = self._last_lane_result
        zr = self.zebra_result
        zp = self.zebra_params
        self._csv_writer.writerow([
            f"{t:.3f}", self._phase_label()[0], self.intersection_phase or '',
            self.commit_direction or '', bool(self.intersection_pending),
            self.intersection_decision or '', '|'.join(self.intersection_options),
            'BEV' if lr is not None else 'legacy',
            f"{(lr.offset_norm if lr else 0.0):+.3f}",
            f"{(lr.confidence if lr else 0.0):.2f}",
            f"{(lr.curvature_norm if lr else 0.0):+.3f}",
            '' if (lr is None or lr.base_x is None) else f"{lr.base_x:.1f}",
            bool(zr.seen) if zr is not None else False,
            '' if (zr is None or zr.distance_cm is None) else f"{zr.distance_cm:.1f}",
            '' if (zr is None or zr.angle_deg is None) else f"{zr.angle_deg:.1f}",
            int(zr.n_dashes) if zr is not None else 0,
            '' if zr is None else f"{zr.span_cm:.1f}",
            '' if zr is None else '|'.join(zr.options),
            bool(self._near_intersection),
            f"{self.last_error:.1f}", f"{self.last_derivative:.1f}",
            f"{cmd.linear.x:.3f}", f"{cmd.angular.z:+.3f}",
            f"{self.kp:.4f}", f"{self.kd:.4f}", f"{self.ff_gain:.3f}",
            f"{self.max_v:.3f}", f"{self.max_w:.3f}",
            f"{zp.stop_distance_cm:.1f}", f"{zp.slow_distance_cm:.1f}",
            f"{self._commit_min_s:.2f}", f"{self._commit_turn_w:.2f}"])

    def _publish_stream_frame(self, frame):
        """Throttle and push the annotated frame to the active stream (MJPEG or H264)."""
        now = self.get_clock().now()
        if (self._last_stream_time is not None
                and (now - self._last_stream_time).nanoseconds * 1e-9 < self._stream_min_period):
            return
        self._last_stream_time = now

        if self._stream_mode == 'h264':
            if self._h264_streamer is not None and self._h264_streamer.write(frame):
                return
            # Writer unavailable: degrade to MJPEG for the rest of the session.
            self._stream_mode = 'mjpeg'
            self.get_logger().error('H264 stream unavailable; using MJPEG at :8080 instead.')

        stream_frame = frame
        if self._stream_max_width and frame.shape[1] > self._stream_max_width:
            scale = self._stream_max_width / float(frame.shape[1])
            stream_frame = cv2.resize(
                frame,
                (self._stream_max_width, int(frame.shape[0] * scale)),
                interpolation=cv2.INTER_AREA,
            )
        ok, jpeg = cv2.imencode('.jpg', stream_frame, [cv2.IMWRITE_JPEG_QUALITY, self._stream_quality])
        if ok:
            _MJPEGHandler.update_frame(jpeg.tobytes())

    def _analyze_intersection(self, frame):
        """Run the shared intersection detector, threading the stability count."""
        result = analyze_intersection(frame, self.intersection_params, self.intersection_stable)
        self.intersection_stable = result.stable_frames
        return result

    def _publish_intersection_prompt(self, result):
        option_text = ', '.join(result.options) if result.options else 'none'
        msg = String()
        msg.data = (
            f"Intersection detected. Options: {option_text}. "
            "Reply with: ros2 topic pub --once /intersection_decision "
            "std_msgs/msg/String \"{data: 'left'}\""
        )
        self.intersection_prompt_pub.publish(msg)
        self.get_logger().warn(
            f"[INTERSECTION] Waiting for decision. Options: {option_text}. "
            f"dash={result.dashed_count} L:{result.left_dash} S:{result.center_dash} R:{result.right_dash}"
        )

    def _publish_zebra_prompt(self, zres):
        opts = zres.options if (zres is not None and zres.options) else []
        option_text = ', '.join(opts) if opts else 'none (decide manually)'
        msg = String()
        msg.data = (
            f"Intersection detected. Options: {option_text}. "
            "Reply with: ros2 topic pub --once /intersection_decision "
            "std_msgs/msg/String \"{data: 'left'}\""
        )
        self.intersection_prompt_pub.publish(msg)
        d = '?' if (zres is None or zres.distance_cm is None) else f"{zres.distance_cm:.0f}"
        self.get_logger().warn(
            f"[ZEBRA] Waiting for decision. Options: {option_text}. dist={d}cm")

    def _run_zebra_phase(self, frame, now):
        """Robust bird's-eye / ground-coordinate intersection handling.

        Mirrors the legacy phase machine (None -> approach -> wait -> commit) but
        the trigger and the stop are driven by the zebra ROW DISTANCE in cm
        (pose-independent), not the noisy raw-image entry-line geometry. Returns
        True if it consumed the frame (WAIT: published a stop and early-returned).
        """
        zp = self.zebra_params

        # Suppress detection while committing a turn or inside the post-turn
        # travel/time cooldown (same double-cross guard as the legacy path).
        cooldown_active = (
            (self.intersection_cooldown_until is not None
             and now < self.intersection_cooldown_until)
            or self._dist_since_commit < self._intersection_min_travel_m
        )
        if cooldown_active or self.commit_direction is not None:
            self._zebra_stable = 0
            self.zebra_result = None
        else:
            h, w = frame.shape[:2]
            if self._zebra_M is None or self._zebra_frame_size != (w, h):
                self._zebra_M = wide_homography(self.lane_params, zp, w, h)
                self._zebra_frame_size = (w, h)
            zres = analyze_zebra(frame, self.lane_params, zp, self._zebra_M,
                                 self._zebra_stable)
            self._zebra_stable = zres.stable_frames
            self.zebra_result = zres

        zres = self.zebra_result
        dist = zres.distance_cm if zres is not None else None
        # Slow-zone: zebra debounced-seen AND within the slow distance.
        self._near_intersection = bool(
            zres is not None and zres.seen and dist is not None
            and dist <= zp.slow_distance_cm)

        # Enter APPROACH once the zebra is seen and within slowing range.
        if (zres is not None and zres.seen and dist is not None
                and dist <= zp.slow_distance_cm and self.intersection_phase is None):
            self.intersection_phase = 'approach'
            self._zebra_opt_votes = {}
            self._approach_start_time = now
            self.get_logger().info(f'[ZEBRA] seen @ {dist:.0f}cm -> APPROACH')
            self._event('approach_start', dist_cm=round(float(dist), 1))

        # APPROACH -> WAIT by distance (or timeout back to FOLLOW).
        if self.intersection_phase == 'approach':
            # Vote options over the whole approach (a single frame flaps; an exit
            # seen in >= zebra_opt_min_votes frames sticks). Fixes "saw left but
            # not right": once right is detected in any couple of frames it stays.
            if zres is not None:
                for o in zres.options:
                    self._zebra_opt_votes[o] = self._zebra_opt_votes.get(o, 0) + 1
                self.intersection_options = [
                    o for o in ('left', 'straight', 'right')
                    if self._zebra_opt_votes.get(o, 0) >= self._zebra_opt_min_votes]
            arrived = dist is not None and dist <= zp.stop_distance_cm
            timed_out = (
                self._approach_start_time is not None
                and (now - self._approach_start_time).nanoseconds * 1e-9
                > self._approach_timeout_s)
            if arrived:
                self.intersection_phase = 'wait'
                self.intersection_pending = True
                self.intersection_decision = None
                self.last_prompt_time = None
                self._approach_start_time = None
                self._align_start_time = None
                self.get_logger().info(
                    f'[ZEBRA] at cross ({dist:.0f}cm) -> WAIT')
                self._event('wait_start', dist_cm=round(float(dist), 1),
                            voted_options=list(self.intersection_options),
                            option_votes=dict(self._zebra_opt_votes))
            elif timed_out:
                self.intersection_phase = None
                self._approach_start_time = None
                self.intersection_cooldown_until = now + Duration(seconds=2.0)
                self.get_logger().warn('[ZEBRA] APPROACH timed out -> FOLLOW')
                self._event('approach_timeout')

        # WAIT: stopped at the cross. First SQUARE UP in place if we arrived skewed
        # (came off a curve) -- rotate with v=0 (safe, no arcing) to face the cross
        # so options read correctly, THEN prompt + wait for a decision.
        if self.intersection_phase == 'wait':
            zr = self.zebra_result
            za = zr.angle_deg if zr is not None else None
            if self._align_start_time is None:
                self._align_start_time = now
            align_elapsed = (now - self._align_start_time).nanoseconds * 1e-9
            need_align = (self._align_in_place and za is not None
                          and abs(za) > self._align_tol_deg
                          and align_elapsed < self._align_timeout_s)
            if need_align and self._drive_enabled and self.intersection_decision is None:
                tw = Twist()
                tw.angular.z = max(-self.max_w, min(self.max_w,
                                                    -self._k_align * math.radians(za)))
                self.cmd_pub.publish(tw)
                self.get_logger().info(
                    f'[ZEBRA] WAIT square-up in place: za={za:.0f} w={tw.angular.z:+.2f}',
                    throttle_duration_sec=0.5)
                self._draw_status_hud(frame, tw)
                self._log_controller_row(now, tw)
                self._publish_telemetry(now, tw)
                self._publish_stream_frame(frame)
                if self.show_window:
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(1)
                return True

            should_prompt = (self.last_prompt_time is None
                             or (now - self.last_prompt_time).nanoseconds * 1e-9 > 1.0)
            if should_prompt:
                self._publish_zebra_prompt(zres)
                self.last_prompt_time = now

            if self.intersection_decision is None:
                self.cmd_pub.publish(Twist())
                self._draw_status_hud(frame, Twist())
                self._log_controller_row(now, Twist())
                self._publish_telemetry(now, Twist())
                self._publish_stream_frame(frame)
                if self.show_window:
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(1)
                return True

            self.commit_direction = self.intersection_decision
            dur = (self._commit_duration_straight
                   if self.commit_direction == 'straight' else self._commit_duration)
            self.commit_until = now + Duration(seconds=dur)
            self._commit_min_until = now + Duration(seconds=self._commit_min_s)
            self._dist_since_commit = 0.0
            self._approach_start_time = None
            self.intersection_phase = None
            self.intersection_pending = False
            self.intersection_options = []
            self.intersection_decision = None
            self._zebra_stable = 0
            self.zebra_result = None
            self.intersection_cooldown_until = now + Duration(seconds=1.5)
            self._event('commit_start', direction=self.commit_direction,
                        duration_s=float(dur), min_s=float(self._commit_min_s))
        return False

    def _draw_intersection_overlay(self, frame, result):
        h, w = frame.shape[:2]
        cv2.putText(frame, 'INTERSECTION - WAITING', (20, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        opt_text = ', '.join(result.options) if result.options else 'none'
        cv2.putText(frame, f"options: {opt_text}", (20, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        # Red entry/trigger band + orange detected entry line.
        ry0 = int(h * self.intersection_params.roi_y0_pct / 100.0)
        ry1 = int(h * self.intersection_params.roi_y1_pct / 100.0)
        cv2.rectangle(frame, (0, ry0), (w, ry1), (0, 0, 255), 2)
        if result.entry_y_pct is not None:
            # Fitted zebra line the entry dashes align to; green once centered.
            y_left = int(result.entry_intercept)
            y_right = int(result.entry_slope * w + result.entry_intercept)
            line_color = (0, 255, 0) if result.entry_centered else (0, 165, 255)
            cv2.line(frame, (0, y_left), (w, y_right), line_color, 2)
            if result.entry_center_x is not None:
                entry_y = int(result.entry_slope * result.entry_center_x + result.entry_intercept)
                entry_pt = (int(result.entry_center_x), entry_y)
                cv2.circle(frame, entry_pt, 9, line_color, -1)
        # Option ROIs: green when geometrically validated, else each in its own
        # color (left=magenta, straight=cyan, right=azure).
        roi_colors = {'left': (255, 0, 255), 'straight': (255, 255, 0), 'right': (255, 160, 0)}
        for name, poly in result.option_roi_polys.items():
            pts = np.array(
                [[int(w * x / 100.0), int(h * y / 100.0)] for x, y in poly],
                dtype=np.int32,
            )
            color = (0, 255, 0) if result.option_valid.get(name, False) else roi_colors.get(name, (255, 0, 255))
            cv2.polylines(frame, [pts], True, color, 2)
        # Color each detected dash by the zone it was assigned to (matches the
        # offline calibrator palette). Strays ('other') stay faint and thin.
        zone_colors = {
            'entry': (0, 165, 255), 'left': (255, 0, 255), 'straight': (255, 255, 0),
            'right': (255, 160, 0), 'merged': (0, 0, 255), 'other': (90, 90, 90),
        }
        zones = result.box_zones or ['entry'] * len(result.dashed_boxes)
        for (x, y, bw, bh), zone in zip(result.dashed_boxes, zones):
            color = zone_colors.get(zone, zone_colors['other'])
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 1 if zone == 'other' else 2)
        cv2.putText(frame, f"dash:{result.dashed_count} L:{result.left_dash} S:{result.center_dash} R:{result.right_dash} centered:{int(result.entry_centered)}",
                    (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

    # =============================================================
    # TRAFFIC LIGHT DETECTOR
    # =============================================================
    def detect_color(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_area = 0
        largest_contour = None

        for c in contours:
            area = cv2.contourArea(c)
            if area > largest_area:
                largest_area = area
                largest_contour = c

        return largest_area, largest_contour, mask

    # =============================================================
    # ANCHOR ASSIGNMENT HELPER
    # Matches a pool of candidates to three named anchors using a
    # greedy nearest-neighbour approach.  Each candidate can only
    # be consumed once, and candidates that are too far from any
    # anchor are ignored.  If an anchor has no matching candidate
    # its previous position is kept (freeze-last-known).
    # =============================================================
    def _assign_to_anchors(self, candidates, anchor_left, anchor_middle, anchor_right, x_start, y_start):
        """
        candidates : list of dicts with keys 'cx', 'cy', 'area'
                     (cx/cy already in full-frame coordinates)
        anchor_*   : (cx, cy) or None  – last known position for that lane
        x_start    : left edge of the ROI (used to build a default x order
                     when anchors are not yet initialised)

        Returns (new_left, new_middle, new_right) where each value is
        either an updated (cx, cy) tuple or the unchanged anchor value.
        """
        MAX_JUMP = self.anchor_max_jump

        # If anchors are not yet set, do a one-time bootstrap by sorting
        # the candidates by X and assigning left->middle->right in order.
        if anchor_left is None and anchor_middle is None and anchor_right is None:
            if len(candidates) >= 3:
                by_x = sorted(candidates, key=lambda c: c['cx'])[:3]
                by_x.sort(key=lambda c: c['cx'])
                return (
                    (by_x[0]['cx'], by_x[0]['cy']),
                    (by_x[1]['cx'], by_x[1]['cy']),
                    (by_x[2]['cx'], by_x[2]['cy']),
                )
            # Not enough candidates yet; can't bootstrap.
            return None, None, None

        remaining = list(candidates)   # mutable pool
        new_left   = anchor_left
        new_middle = anchor_middle
        new_right  = anchor_right

        def best_match(anchor, pool):
            """Return (index_in_pool, candidate) closest to anchor, or None."""
            if anchor is None or not pool:
                return None
            best_idx, best_dist = None, float('inf')
            for i, c in enumerate(pool):
                dist = np.hypot(c['cx'] - anchor[0], c['cy'] - anchor[1])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_dist <= MAX_JUMP:
                return best_idx
            return None

        # Greedy pass: process anchors in order of distance to their nearest
        # candidate so the closest pairing is resolved first (avoids stealing).
        def min_dist_to_pool(anchor, pool):
            if anchor is None or not pool:
                return float('inf')
            return min(np.hypot(c['cx'] - anchor[0], c['cy'] - anchor[1]) for c in pool)

        anchor_slots = [
            ('left',   anchor_left),
            ('middle', anchor_middle),
            ('right',  anchor_right),
        ]
        # Sort by proximity so the tightest pair is matched first
        anchor_slots.sort(key=lambda s: min_dist_to_pool(s[1], remaining))

        results = {'left': anchor_left, 'middle': anchor_middle, 'right': anchor_right}
        for name, anchor in anchor_slots:
            idx = best_match(anchor, remaining)
            if idx is not None:
                c = remaining.pop(idx)
                results[name] = (c['cx'], c['cy'])
            # else: keep last known position (freeze)

        return results['left'], results['middle'], results['right']

    # =============================================================
    # ROI LINE DETECTOR
    # =============================================================
    def detect_line_in_roi(
        self, frame, x_start, x_end, y_start, y_end,
        last_center, reference_x, draw_color=(0, 255, 0), force_middle_of_three=False
    ):
        roi = frame[y_start:y_end, x_start:x_end]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)

        _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Robustness guard: bail out before trusting Otsu on a ROI that has no
        # real line. Low contrast => uniform surface; an out-of-range fill ratio
        # => either nothing (noise speckle) or a big shadow/over-binarized blob.
        # In either case we return no candidate so the caller freezes its last
        # known position instead of chasing a phantom.
        roi_contrast = float(blurred.std())
        fill_pct = 100.0 * float(np.count_nonzero(mask)) / float(mask.size)
        if (roi_contrast < self._line_min_contrast
                or not (self._line_min_fill_pct <= fill_pct <= self._line_max_fill_pct)):
            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), draw_color, 1)
            return None, mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 100: continue

            x_box, y_box, w_box, h_box = cv2.boundingRect(c)
            if h_box < 5: continue
            if w_box > h_box * 4.0: continue  # Aspect ratio filter

            moments = cv2.moments(c)
            if moments["m00"] == 0: continue

            cx = int(moments["m10"] / moments["m00"]) + x_start
            cy = int(moments["m01"] / moments["m00"]) + y_start

            valid_candidates.append({'cx': cx, 'cy': cy, 'area': area})

        best_candidate = None

        # =========================================================
        # 3-LINE TRACKING (Top ROI)
        # Uses per-lane anchors so each circle follows its own line
        # independently.  If a line disappears its anchor is frozen
        # at the last known position; it resumes tracking as soon as
        # a matching candidate re-appears within anchor_max_jump px.
        # =========================================================
        if force_middle_of_three:
            new_left, new_middle, new_right = self._assign_to_anchors(
                valid_candidates,
                self.last_left_anchor,
                self.last_middle_anchor,
                self.last_right_anchor,
                x_start,
                y_start,
            )

            # Persist whatever we resolved (including frozen values)
            self.last_left_anchor   = new_left
            self.last_middle_anchor = new_middle
            self.last_right_anchor  = new_right

            # Draw ROI boundary
            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), draw_color, 2)

            # Draw each anchor with its colour (greyed-out / smaller if frozen
            # this frame, full-size if updated).  We mark frozen anchors with a
            # hollow circle so the operator can see which ones are dead-reckoning.
            updated_names = set()
            for cand in valid_candidates:
                for name, anchor in [('left', new_left), ('middle', new_middle), ('right', new_right)]:
                    if anchor is not None:
                        dist = np.hypot(cand['cx'] - anchor[0], cand['cy'] - anchor[1])
                        if dist < 2:   # effectively the same point
                            updated_names.add(name)

            def draw_anchor(pt, color_solid, color_frozen, name):
                if pt is None:
                    return
                is_live = name in updated_names
                if is_live:
                    cv2.circle(frame, pt, 10, color_solid, -1)
                else:
                    # Hollow circle = frozen / estimated position
                    cv2.circle(frame, pt, 10, color_frozen, 2)

            draw_anchor(new_left,   (0, 255, 255), (0, 180, 180), 'left')    # Yellow / dim cyan
            draw_anchor(new_middle, (0, 0, 255),   (0, 0, 160),   'middle')  # Red    / dim red
            draw_anchor(new_right,  (200, 0, 200), (120, 0, 120), 'right')   # Purple / dim purple

            best_candidate = new_middle  # Steering uses the centre lane
            return best_candidate, mask

        # =========================================================
        # ANCHORING FALLBACK (Bottom ROI or < 3 lines)
        # =========================================================
        else:
            best_score = float('inf')
            if last_center is None:
                last_center = (int(reference_x), y_end)
                allowed_jump = float('inf')
            else:
                allowed_jump = self.max_jump_distance

            for cand in valid_candidates:
                cx, cy, area = cand['cx'], cand['cy'], cand['area']
                dist_to_ref = abs(cx - reference_x)
                dist_to_last = np.sqrt((cx - last_center[0])**2 + (cy - last_center[1])**2)

                if dist_to_last > allowed_jump: continue

                score = (dist_to_ref * 0.4) + (dist_to_last * 0.6) - (area * 0.001)
                if score < best_score:
                    best_score = score
                    best_candidate = (cx, cy)

            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), draw_color, 2)
            if best_candidate is not None:
                cv2.circle(frame, best_candidate, 10, draw_color, -1)

            return best_candidate, mask

    # =============================================================
    # MAIN LOOP
    # =============================================================
    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received from camera!")
            return

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        frame = self._apply_illumination_gain(frame)

        h, w = frame.shape[:2]
        frame_center_x = w / 2.0
        now = self.get_clock().now()

        # ---------------------------------------------------------
        # 1. TRAFFIC LIGHT PERCEPTION
        # ---------------------------------------------------------
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([8, 255, 255])) + \
                   cv2.inRange(hsv, np.array([172, 150, 100]), np.array([180, 255, 255]))
        yellow_mask = cv2.inRange(hsv, np.array([20, 150, 120]), np.array([32, 255, 255]))
        green_mask  = cv2.inRange(hsv, np.array([40, 120, 120]), np.array([85, 255, 255]))

        red_area,    _, _ = self.detect_color(red_mask)
        yellow_area, _, _ = self.detect_color(yellow_mask)
        green_area,  _, _ = self.detect_color(green_mask)

        detected_color = "UNKNOWN"
        if max(red_area, yellow_area, green_area) > self.min_area:
            if   red_area    > yellow_area and red_area    > green_area:  detected_color = "RED"
            elif yellow_area > red_area    and yellow_area > green_area:  detected_color = "YELLOW"
            elif green_area  > red_area    and green_area  > yellow_area: detected_color = "GREEN"

        self.get_logger().info(
            f"[VISION] Areas -> R:{red_area:.0f} Y:{yellow_area:.0f} G:{green_area:.0f} "
            f"| Raw Detect: {detected_color} | Active State: {self.current_state}",
            throttle_duration_sec=1.0,
        )

        if detected_color == "RED":
            self.red_count    += 1; self.yellow_count  = 0; self.green_count = 0
        elif detected_color == "YELLOW":
            self.yellow_count += 1; self.red_count     = 0; self.green_count = 0
        elif detected_color == "GREEN":
            self.green_count  += 1; self.red_count     = 0; self.yellow_count = 0
        else:
            self.red_count = 0; self.yellow_count = 0; self.green_count = 0

        if   self.red_count    >= self.threshold_frames: self.current_state = "RED"
        elif self.yellow_count >= self.threshold_frames: self.current_state = "YELLOW"
        elif self.green_count  >= self.threshold_frames: self.current_state = "GREEN"

        if self.current_state != self.last_state:
            self.get_logger().info(
                f"[TRAFFIC LIGHT] >>> Switched from {self.last_state} to {self.current_state} <<<"
            )
            self.last_state = self.current_state

        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # ---------------------------------------------------------
        # 2. INTERSECTION / DASHED-LINE PERCEPTION
        # ---------------------------------------------------------
        # Robust path (default): bird's-eye, ground-coordinate zebra detector. It
        # owns the trigger + the stop (by distance in cm). Legacy raw-image path
        # below runs only when use_zebra_bev:=false.
        if self._use_zebra_bev:
            if self._run_zebra_phase(frame, now):
                return
        else:
            # Suppress detection while committing a turn, during the time cooldown,
            # OR until we have driven far enough past the last cross (distance proxy
            # that stops a double intersection from re-firing the one we just left).
            cooldown_active = (
                (self.intersection_cooldown_until is not None
                 and now < self.intersection_cooldown_until)
                or self._dist_since_commit < self._intersection_min_travel_m
            )
            if cooldown_active or self.commit_direction is not None:
                self.intersection_stable = 0
                result = None
            else:
                result = self._analyze_intersection(frame)
                self.intersection_result = result

            # FOLLOW slow-zone: the moment a zebra is SEEN (debounced, regardless of
            # centering) we slow down and relax the curve feedforward below, so the
            # robot does not overshoot the cross before it can center. Independent of
            # whether we commit to APPROACH this frame.
            self._near_intersection = bool(result is not None and result.entry_seen)

            # Enter APPROACH on the centering-INDEPENDENT trigger (entry_seen). Coming
            # out of a curve the robot is skewed and would never satisfy the old
            # centered `dashed_detected` gate; APPROACH then actively straightens it.
            if (result is not None and result.entry_seen
                    and self.intersection_phase is None):
                self.intersection_phase = 'approach'
                self.intersection_options = result.options
                self._approach_start_time = now
                self.get_logger().info('[INTERSECTION] seen -> APPROACH (center + align)')

            if self.intersection_phase in ('approach', 'wait') and self.intersection_result is not None:
                self.intersection_options = self.intersection_result.options
                self._draw_intersection_overlay(frame, self.intersection_result)

            # APPROACH: arrived once the entry zebra is at the target depth AND centered
            # (now reachable because the motion section actively aligns heading). A
            # timeout drops back to FOLLOW so a bad detection can't strand the robot.
            if self.intersection_phase == 'approach':
                r = self.intersection_result
                # Coming out of a curve the robot is NOT aligned, and a diff-drive
                # robot can't strafe to fix that. So we do NOT require alignment: we
                # just STOP when the zebra is close, in whatever pose. The skew is
                # absorbed later by the turn maneuver + lane re-acquisition.
                arrived = (
                    r is not None and r.entry_y_pct is not None
                    and r.entry_y_pct >= self._approach_target_entry_y_pct
                )
                timed_out = (
                    self._approach_start_time is not None
                    and (now - self._approach_start_time).nanoseconds * 1e-9 > self._approach_timeout_s
                )
                if arrived:
                    self.intersection_phase = 'wait'
                    self.intersection_pending = True
                    self.intersection_decision = None
                    self.last_prompt_time = None
                    self._approach_start_time = None
                    self.get_logger().info('[INTERSECTION] at cross + aligned -> WAIT')
                elif timed_out:
                    self.intersection_phase = None
                    self._approach_start_time = None
                    self.intersection_cooldown_until = now + Duration(seconds=2.0)
                    self.get_logger().warn('[INTERSECTION] APPROACH timed out -> FOLLOW')

            # WAIT: stopped at the intersection, prompting until a decision arrives.
            if self.intersection_phase == 'wait':
                draw_result = self.intersection_result
                should_prompt = self.last_prompt_time is None or (now - self.last_prompt_time).nanoseconds * 1e-9 > 1.0
                if should_prompt and draw_result is not None:
                    self._publish_intersection_prompt(draw_result)
                    self.last_prompt_time = now

                if self.intersection_decision is None:
                    self.cmd_pub.publish(Twist())
                    self._draw_status_hud(frame, Twist())
                    self._log_controller_row(now, Twist())
                    self._publish_stream_frame(frame)
                    if self.show_window:
                        cv2.imshow("Frame", frame)
                        cv2.waitKey(1)
                    return

                self.commit_direction = self.intersection_decision
                dur = (self._commit_duration_straight
                       if self.commit_direction == 'straight' else self._commit_duration)
                self.commit_until = now + Duration(seconds=dur)
                self._commit_min_until = now + Duration(seconds=self._commit_min_s)
                self._dist_since_commit = 0.0        # start the double-cross travel guard
                self._approach_start_time = None
                self.intersection_phase = None
                self.intersection_pending = False
                self.intersection_options = []
                self.intersection_decision = None
                self.intersection_stable = 0
                self.intersection_result = None
                self.intersection_cooldown_until = now + Duration(seconds=1.5)

        # ---------------------------------------------------------
        # 3. LINE PERCEPTION + BASE CONTROL
        # ---------------------------------------------------------
        base_linear_x  = 0.0
        target_angular_z = 0.0
        steering_center_x = None
        steering_far_x = None       # lookahead point (BEV only) -> curve feedforward
        lane_curvature = 0.0
        bottom_candidate = top_candidate = None
        self._last_lane_result = None    # set below when the bird's-eye path runs

        # Primary path: bird's-eye lane follower. Only while normally following;
        # during an intersection approach/commit we keep the legacy ROI logic
        # that centers on the zebra entry.
        # In the zebra-BEV path we keep following the LANE during APPROACH (the
        # robot creeps toward the cross while staying on the line); the legacy path
        # instead centers on the entry zebra, so it only runs birdseye when FOLLOW.
        lane_follow_phase_ok = self.intersection_phase is None or (
            self._use_zebra_bev and self.intersection_phase == 'approach')
        # Run the bird's-eye detector during FOLLOW, the zebra APPROACH, AND the
        # COMMIT maneuver. During COMMIT it does not steer (the turn overrides
        # below) but it provides the lane RE-ACQUIRED signal that ends the commit.
        run_birdseye = self._use_birdseye and (
            lane_follow_phase_ok or self.commit_direction is not None)
        lane_ok = False
        if run_birdseye:
            if self._lane_M is None or self._lane_frame_size != (w, h):
                self._lane_M, self._lane_Minv = compute_homography(self.lane_params, w, h)
                self._lane_frame_size = (w, h)
            lane_result = analyze_lane(frame, self.lane_params, self._lane_M,
                                       self._lane_Minv, self._lane_prev_base)
            self._last_lane_result = lane_result
            # Thread the base x to the next frame for continuity (stay on the same
            # line through a curve); drop it when the line is lost so it re-acquires
            # from the center next time.
            if lane_result.detected and lane_result.confidence >= 0.5:
                self._lane_prev_base = lane_result.base_x
            else:
                self._lane_prev_base = None
            draw_lane_overlay(frame, self.lane_params, lane_result)
            if lane_result.detected and lane_result.lane_center_x_orig is not None:
                lane_ok = True
                steering_center_x = lane_result.lane_center_x_orig
                steering_far_x = lane_result.lane_center_far_x_orig
                lane_curvature = abs(lane_result.curvature_norm)
                self.time_line_lost = None
                self.get_logger().info(
                    f"[LANE] off={lane_result.offset_norm:+.2f} "
                    f"curv={lane_result.curvature_norm:+.2f} conf={lane_result.confidence:.2f}",
                    throttle_duration_sec=1.0,
                )

        # Fallback path: legacy two-ROI detector. Also used during approach and
        # whenever the bird's-eye view is not confident (e.g. warp not yet tuned).
        if not lane_ok:
            bottom_y_start, bottom_y_end = int(h * 0.60), h
            bottom_x_start, bottom_x_end = int(w * 0.25), int(w * 0.75)
            approach_entry_center_x = None
            if (self.intersection_phase == "approach"
                    and self.intersection_result is not None
                    and self.intersection_result.entry_center_x is not None):
                approach_entry_center_x = float(self.intersection_result.entry_center_x)
                roi_width = bottom_x_end - bottom_x_start
                roi_left = approach_entry_center_x - roi_width / 2.0
                bottom_x_start = int(max(0, min(w - roi_width, roi_left)))
                bottom_x_end = bottom_x_start + roi_width
                cv2.line(frame, (int(approach_entry_center_x), bottom_y_start),
                         (int(approach_entry_center_x), bottom_y_end), (0, 255, 255), 2)

            top_y_start, top_y_end = int(h * 0.25), int(h * 0.50)
            top_x_start, top_x_end = int(w * 0.10), int(w * 0.90)

            if approach_entry_center_x is not None:
                bottom_reference_x = approach_entry_center_x
            else:
                bottom_reference_x = self.last_bottom_center[0] if self.last_bottom_center else frame_center_x

            bottom_candidate, bottom_mask = self.detect_line_in_roi(
                frame,
                bottom_x_start, bottom_x_end, bottom_y_start, bottom_y_end,
                self.last_bottom_center, reference_x=bottom_reference_x,
                draw_color=(0, 255, 0), force_middle_of_three=False
            )

            if bottom_candidate:
                top_reference_x = bottom_candidate[0]
            else:
                top_reference_x = self.last_top_center[0] if self.last_top_center else frame_center_x

            top_candidate, top_mask = self.detect_line_in_roi(
                frame,
                top_x_start, top_x_end, top_y_start, top_y_end,
                self.last_top_center, reference_x=top_reference_x,
                draw_color=(0, 0, 255),
                force_middle_of_three=True
            )

            if bottom_candidate is not None: self.last_bottom_center = bottom_candidate
            if top_candidate    is not None: self.last_top_center    = top_candidate

            bot_str = f"({bottom_candidate[0]}, {bottom_candidate[1]})" if bottom_candidate else "NONE"
            top_str = f"({top_candidate[0]},    {top_candidate[1]})"    if top_candidate    else "NONE"
            self.get_logger().info(
                f"[TRACKING] Bottom Line: {bot_str} | Top Line: {top_str}",
                throttle_duration_sec=1.0,
            )

            if bottom_candidate is not None:
                self.time_line_lost = None
                bottom_cx, bottom_cy = bottom_candidate
                steering_center_x = bottom_cx
                if top_candidate is not None:
                    top_cx, top_cy = top_candidate
                    steering_center_x += (top_cx - bottom_cx) * 0.15
                    cv2.line(frame, (bottom_cx, bottom_cy), (top_cx, top_cy), (255, 255, 0), 2)

            elif top_candidate is not None:
                self.time_line_lost = None
                steering_center_x = top_candidate[0]
                base_linear_x = 0.04
                self.get_logger().info(
                    "[CONTROL] Using top candidate only (bottom lost).",
                    throttle_duration_sec=1.0,
                )

            else:
                if self.time_line_lost is None:
                    self.time_line_lost = now
                elapsed_time = (now - self.time_line_lost).nanoseconds * 1e-9

                # Turn toward the side the line was last seen instead of driving
                # straight off the track. The PD error sign encodes that side:
                # error>0 => line was left of center => turn left (+w); <0 => right.
                recover_dir = 1.0 if self.last_error > 0 else (-1.0 if self.last_error < 0 else 0.0)
                if elapsed_time < self._recover_seconds:
                    base_linear_x    = self._recover_speed
                    target_angular_z = self._recover_turn * recover_dir
                    self.get_logger().warn(
                        f"[CONTROL] LINE LOST {elapsed_time:.1f}s -> searching dir={recover_dir:+.0f}",
                        throttle_duration_sec=0.5,
                    )
                else:
                    base_linear_x    = 0.0
                    target_angular_z = 0.0
                    self.get_logger().warn(
                        "[CONTROL] LINE LOST -> stopped (search timed out)",
                        throttle_duration_sec=2.0,
                    )

            if approach_entry_center_x is not None:
                steering_center_x = approach_entry_center_x
                self.time_line_lost = None
                self.get_logger().info(
                    f"[INTERSECTION] Steering to entry center x={approach_entry_center_x:.1f}",
                    throttle_duration_sec=1.0,
                )

        # PD Math
        if steering_center_x is not None:
            line_error = frame_center_x - steering_center_x
            dt = (now - self.last_time).nanoseconds * 1e-9

            if dt > 0:
                raw_derivative = (line_error - self.last_error) / dt
                derivative     = (0.7 * self.last_derivative) + (0.3 * raw_derivative)

                # Curve feedforward (BEV only): the bend = how much more the line
                # is offset further ahead than right at the robot. Steering ahead
                # by it makes the robot turn INTO the curve instead of chasing the
                # near edge. It is a difference, so on a straight it is ~0 and the
                # straight-line tuning is untouched.
                curve_term = 0.0
                if steering_far_x is not None:
                    far_error = frame_center_x - steering_far_x
                    curve_term = far_error - line_error
                if self._near_intersection:
                    curve_term = 0.0   # relax anticipation near a cross (no overshoot)
                w_out = (self.kp * line_error) + (self.kd * derivative) \
                    + (self.kp * self.ff_gain * curve_term)

                curve_factor = max(0.4, 1.0 - (abs(line_error) / frame_center_x))

                if base_linear_x == 0.0:
                    base_linear_x = self.max_v * curve_factor

                target_angular_z = max(-self.max_w, min(self.max_w, w_out))

                self.get_logger().info(
                    f"[MATH] Error: {line_error:.1f} | Deriv: {derivative:.1f} | "
                    f"Curve Fact: {curve_factor:.2f} -> Raw W: {w_out:.3f}",
                    throttle_duration_sec=1.0,
                )

                self.last_error      = line_error
                self.last_derivative = derivative
                self.last_time       = now

        # Slow down proportionally to the path curvature. The bird's-eye fit
        # gives a real curvature estimate; it is 0 on the legacy path, so this
        # is a no-op there and the legacy curve_factor still applies.
        if lane_curvature > 0.0:
            base_linear_x *= max(self._curve_min_scale,
                                 1.0 - self._curve_slow_gain * lane_curvature)

        # Slow-zone: cap speed while a zebra is in view (FOLLOW only), so the robot
        # closes on the cross slowly enough to center instead of overshooting.
        if self._near_intersection and self.intersection_phase is None:
            base_linear_x = min(base_linear_x, self._intersection_slow_speed)

        if self.commit_direction is not None:
            lr = self._last_lane_result
            reacquired = (lr is not None and lr.detected
                          and lr.confidence >= 0.5)
            past_min = (self._commit_min_until is None
                        or now >= self._commit_min_until)
            past_max = (self.commit_until is not None
                        and now >= self.commit_until)
            # End the maneuver when the lane is RE-ACQUIRED (after a min time to
            # clear the cross), or at the safety cap. A cross has no line to
            # follow, so we drive the turn/cross open-loop ONLY until the line of
            # the chosen branch reappears -- then hand straight back to FOLLOW.
            if past_max or (self._commit_closed_loop and past_min and reacquired):
                self.get_logger().info(
                    f"[INTERSECTION] commit {self.commit_direction} done -> FOLLOW "
                    f"(reacquired={reacquired}, timeout={past_max})")
                self._event('commit_end', direction=self.commit_direction,
                            reacquired=bool(reacquired), timeout=bool(past_max))
                self.commit_direction = None
                self.commit_until = None
                self._commit_min_until = None
            else:
                base_linear_x = self._commit_speed
                if self.commit_direction == 'left':
                    target_angular_z = self._commit_turn_w
                elif self.commit_direction == 'right':
                    target_angular_z = -self._commit_turn_w
                else:
                    target_angular_z = 0.0
                self.get_logger().info(
                    f"[INTERSECTION] Committing {self.commit_direction}: "
                    f"V={base_linear_x:.2f}, W={target_angular_z:.2f}",
                    throttle_duration_sec=0.5)

        # During APPROACH: drive a fixed creep AND actively align heading so the
        # robot straightens onto the zebra (works whether it arrived from a curve
        # or a straight). w_align rotates the fitted entry line toward horizontal;
        # the legacy centering above already handles lateral offset.
        if self.intersection_phase == 'approach':
            if self._use_zebra_bev:
                # Keep the LANE steering (computed above) so the robot stays on the
                # line while creeping to the cross; just brake by distance. NOTE: a
                # pure zebra-angle "square-up" was tried and made it WORSE -- it
                # turned the wrong way and, with no lateral control, arced off the
                # cross and lost the zebra (session 191531). Lane steering at least
                # keeps it on the line into the cross; the option classifier is now
                # robust to a small lateral offset (measured relative to the cross
                # center), so we no longer need to fight for a perfect pose here.
                zr = self.zebra_result
                dist = zr.distance_cm if zr is not None else None
                zp = self.zebra_params
                if dist is not None and dist <= zp.stop_distance_cm:
                    base_linear_x = 0.0
                else:
                    base_linear_x = self._approach_speed
                self.get_logger().info(
                    f"[ZEBRA] APPROACH creep: V={base_linear_x:.3f} "
                    f"W={target_angular_z:.2f} dist="
                    f"{'?' if dist is None else f'{dist:.0f}'}cm",
                    throttle_duration_sec=1.0,
                )
            else:
                r = self.intersection_result
                close = (r is not None and r.entry_y_pct is not None
                         and r.entry_y_pct >= self._approach_target_entry_y_pct)
                # Far: creep toward the zebra. Close: stop forward and rotate IN
                # PLACE to make it horizontal, so an off-axis approach stops AT the
                # cross instead of arcing through it.
                base_linear_x = 0.0 if close else self._approach_speed
                if r is not None and r.entry_y_pct is not None:
                    w_align = -self._k_align * r.entry_slope
                    target_angular_z = max(-self.max_w,
                                           min(self.max_w, target_angular_z + w_align))
                self.get_logger().info(
                    f"[INTERSECTION] APPROACH {'align-in-place' if close else 'creep'}: "
                    f"V={base_linear_x:.3f} W={target_angular_z:.2f} "
                    f"slope={0.0 if r is None else r.entry_slope:.3f}",
                    throttle_duration_sec=1.0,
                )

        # ---------------------------------------------------------
        # 5. SUPERVISOR OVERRIDE (Traffic Light Scale)
        # ---------------------------------------------------------
        cmd = Twist()

        # In testing, ignore_traffic_light forces a GREEN supervisor so the robot
        # drives without needing to see a real light.
        effective_state = "GREEN" if self._ignore_traffic_light else self.current_state

        if effective_state == "RED":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("[ACTION] Stopped for RED light.", throttle_duration_sec=1.0)
        elif effective_state == "YELLOW":
            cmd.linear.x  = base_linear_x * 0.5
            cmd.angular.z = target_angular_z
            self.get_logger().info(
                f"[ACTION] Throttled for YELLOW. Cmd -> V: {cmd.linear.x:.3f}, W: {cmd.angular.z:.3f}",
                throttle_duration_sec=1.0,
            )
        else:  # GREEN
            cmd.linear.x  = base_linear_x
            cmd.angular.z = target_angular_z
            self.get_logger().info(
                f"[ACTION] Normal Drive (GREEN). Cmd -> V: {cmd.linear.x:.3f}, W: {cmd.angular.z:.3f}",
                throttle_duration_sec=1.0,
            )

        # Master motion switch: if driving is disabled, hold still regardless of
        # what the controller computed (perception keeps running below).
        if not self._drive_enabled:
            cmd = Twist()
            self.get_logger().info("[DRIVE] disabled -> holding still", throttle_duration_sec=2.0)

        self.cmd_pub.publish(cmd)

        # Distance proxy for the double-intersection guard: integrate commanded
        # speed at the timer rate (30 Hz). Stays huge until a commit resets it.
        self._dist_since_commit = min(100.0,
                                      self._dist_since_commit + abs(cmd.linear.x) * 0.033)

        self.get_logger().debug("-" * 50)

        # Debug Visuals
        cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), h), (0, 255, 255), 2)
        self._draw_status_hud(frame, cmd)
        self._log_controller_row(now, cmd)
        self._publish_lane_status(cmd)
        self._publish_telemetry(now, cmd)
        self._maybe_snapshot(now, frame)
        if self.show_window:
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

        # Push the annotated frame to the MJPEG stream.
        self._publish_stream_frame(frame)

    def destroy_node(self):
        self.cap.release()
        if self._h264_streamer is not None:
            self._h264_streamer.release()
        if self._csv_fp is not None:
            self._csv_fp.close()
        if self._event_fp is not None:
            self._event_fp.close()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousRacer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()