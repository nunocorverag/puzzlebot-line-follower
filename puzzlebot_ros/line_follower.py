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
from puzzlebot_ros.perception.signs import (
    SignParams,
    SignDetector,
    draw_sign_overlay,
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

        # Live debug view: stream the full composite (camera + lane BEV + zebra
        # BEV/mask + overlays) instead of just the camera, so a single video shows
        # everything the robot sees. Toggle live with /autonomous_racer stream_debug.
        self.declare_parameter('stream_debug', True)
        self._stream_debug = bool(self.get_parameter('stream_debug').value)

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
        traffic_saved = {}
        traffic_found = self._find_config('control_params.json')
        if traffic_found is not None:
            try:
                traffic_saved = json.loads(traffic_found.read_text())
            except (OSError, json.JSONDecodeError):
                traffic_saved = {}
        self.declare_parameter('traffic_light_roi_y_pct', int(traffic_saved.get('traffic_light_roi_y_pct', 55)))
        self.declare_parameter('traffic_light_min_area', float(traffic_saved.get('traffic_light_min_area', 80.0)))
        self.declare_parameter('traffic_light_max_area', float(traffic_saved.get('traffic_light_max_area', 5000.0)))
        self.declare_parameter('traffic_light_min_circularity', float(traffic_saved.get('traffic_light_min_circularity', 0.65)))
        self.declare_parameter('traffic_light_aspect_tol', float(traffic_saved.get('traffic_light_aspect_tol', 0.35)))
        self.declare_parameter('traffic_light_min_fill', float(traffic_saved.get('traffic_light_min_fill', 0.45)))
        self.declare_parameter('traffic_light_max_fill', float(traffic_saved.get('traffic_light_max_fill', 1.15)))
        # Require the light to sit on the gray screen/plate (measured S~45, V~101):
        # the ring just outside the disc must be grayish. Rejects loose colored
        # objects (red cable, chair) not inside the panel.
        self.declare_parameter('traffic_light_require_plate', bool(traffic_saved.get('traffic_light_require_plate', True)))
        self.declare_parameter('traffic_light_plate_max_sat', float(traffic_saved.get('traffic_light_plate_max_sat', 70.0)))
        self.declare_parameter('traffic_light_plate_min_val', float(traffic_saved.get('traffic_light_plate_min_val', 45.0)))
        self.declare_parameter('traffic_light_plate_max_val', float(traffic_saved.get('traffic_light_plate_max_val', 210.0)))
        self.declare_parameter('traffic_light_position_classify', bool(traffic_saved.get('traffic_light_position_classify', False)))
        self.declare_parameter('traffic_light_position_map', str(traffic_saved.get('traffic_light_position_map', 'GREEN,YELLOW,RED')))
        self.declare_parameter('traffic_light_position_anchors_pct',
                               str(traffic_saved.get('traffic_light_position_anchors_pct', '27,50,73')))
        self.declare_parameter('traffic_light_position_max_slot_error_pct',
                               float(traffic_saved.get('traffic_light_position_max_slot_error_pct', 8.0)))
        self.declare_parameter('traffic_light_plate_min_area',
                               float(traffic_saved.get('traffic_light_plate_min_area', 900.0)))
        self.declare_parameter('traffic_light_action_min_radius_px',
                               float(traffic_saved.get('traffic_light_action_min_radius_px', 10.0)))
        self.declare_parameter('traffic_light_action_max_radius_px',
                               float(traffic_saved.get('traffic_light_action_max_radius_px', 80.0)))
        self.declare_parameter('traffic_light_action_min_distance_cm',
                               float(traffic_saved.get('traffic_light_action_min_distance_cm', 12.0)))
        self.declare_parameter('traffic_light_action_max_distance_cm',
                               float(traffic_saved.get('traffic_light_action_max_distance_cm', 45.0)))
        self.declare_parameter('traffic_light_distance_k_cm_px',
                               float(traffic_saved.get('traffic_light_distance_k_cm_px', 360.0)))
        self._tl_require_plate = bool(self.get_parameter('traffic_light_require_plate').value)
        self._tl_plate_max_sat = float(self.get_parameter('traffic_light_plate_max_sat').value)
        self._tl_plate_min_val = float(self.get_parameter('traffic_light_plate_min_val').value)
        self._tl_plate_max_val = float(self.get_parameter('traffic_light_plate_max_val').value)
        self._tl_position_classify = bool(self.get_parameter('traffic_light_position_classify').value)
        self._tl_position_map = self._parse_tl_position_map(
            str(self.get_parameter('traffic_light_position_map').value)
        )
        self._tl_position_anchors = self._parse_tl_position_anchors(
            str(self.get_parameter('traffic_light_position_anchors_pct').value)
        )
        self._tl_position_max_slot_error = (
            float(self.get_parameter('traffic_light_position_max_slot_error_pct').value) / 100.0
        )
        self._tl_plate_min_area = float(self.get_parameter('traffic_light_plate_min_area').value)
        self._tl_action_min_radius_px = float(self.get_parameter('traffic_light_action_min_radius_px').value)
        self._tl_action_max_radius_px = float(self.get_parameter('traffic_light_action_max_radius_px').value)
        self._tl_action_min_distance_cm = float(self.get_parameter('traffic_light_action_min_distance_cm').value)
        self._tl_action_max_distance_cm = float(self.get_parameter('traffic_light_action_max_distance_cm').value)
        self._tl_distance_k_cm_px = float(self.get_parameter('traffic_light_distance_k_cm_px').value)
        self._tl_roi_y_pct = int(self.get_parameter('traffic_light_roi_y_pct').value)
        self._tl_min_area = float(self.get_parameter('traffic_light_min_area').value)
        self._tl_max_area = float(self.get_parameter('traffic_light_max_area').value)
        self._tl_min_circularity = float(self.get_parameter('traffic_light_min_circularity').value)
        self._tl_aspect_tol = float(self.get_parameter('traffic_light_aspect_tol').value)
        self._tl_min_fill = float(self.get_parameter('traffic_light_min_fill').value)
        self._tl_max_fill = float(self.get_parameter('traffic_light_max_fill').value)
        self._traffic_light_candidate = None

        # Default GREEN: with the optional light (default) the robot drives unless a
        # RED is actually seen. In strict mode this is corrected by the HSV machine.
        self.current_state = "GREEN"
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
        self._zebra_opt_votes = {}        # exit -> frames seen during ADVANCE
        self.declare_parameter('zebra_opt_min_votes', 2)
        self._zebra_opt_min_votes = int(self.get_parameter('zebra_opt_min_votes').value)
        # DETECT -> ADVANCE -> READ flow (odometry-based). On the first stable sight
        # of the entry row within detect_distance, the robot freezes a travel target
        # and ADVANCES that distance by odometry to the READING window (read_distance
        # to the entry; can be small/negative = on top of the cross) where the side
        # exits are actually visible -- instead of stopping at the entry and trying
        # to read from the worst spot. Both tunable live.
        self.declare_parameter('detect_distance_cm', 22.0)
        self.declare_parameter('read_distance_cm', 6.0)
        # ADVANCE crosses the FIRST row by vision, then creeps briefly to the
        # reading spot instead of stopping before the row.
        #   read_cross_jump_cm: a sudden z_dist INCREASE this large (after coming
        #     down close) means we crossed the first row and now see the next one.
        #   read_after_entry_max_cm: short cap after that crossing, so ADVANCE
        #     does not chase the exit row. read_advance_extra_cm remains as an
        #     optional tighter cap for deliberate odometry-only tuning.
        self.declare_parameter('read_advance_extra_cm', 0.0)
        self.declare_parameter('read_after_entry_max_cm', 6.0)
        self.declare_parameter('read_cross_jump_cm', 8.0)
        self._detect_distance_cm = float(self.get_parameter('detect_distance_cm').value)
        self._read_distance_cm = float(self.get_parameter('read_distance_cm').value)
        self._read_advance_extra_cm = float(self.get_parameter('read_advance_extra_cm').value)
        self._read_after_entry_max_cm = float(self.get_parameter('read_after_entry_max_cm').value)
        self._read_cross_jump_cm = float(self.get_parameter('read_cross_jump_cm').value)
        self._adv_at_entry = False        # phase-2 flag (reached entry, now crossing)
        self._adv_extra_odom0 = 0.0
        self._adv_entry_target_m = 0.0
        self._adv_prev_dist = None         # previous-frame z_dist (cross-jump detect)
        # ADVANCE goes STRAIGHT by default (gain 0): steering by the row/lane over a
        # cross grabs the edge dashes and veers off. Raise advance_center_gain only
        # if you want gentle centring on the zebra row center (sign tunable).
        self.declare_parameter('advance_center_gain', 0.0)
        self.declare_parameter('advance_lane_keep_gain', 1.0)
        self.declare_parameter('advance_lane_keep_max_w', 0.12)
        self._advance_center_gain = float(self.get_parameter('advance_center_gain').value)
        self._advance_lane_keep_gain = float(self.get_parameter('advance_lane_keep_gain').value)
        self._advance_lane_keep_max_w = float(self.get_parameter('advance_lane_keep_max_w').value)
        self._adv_odom0 = 0.0             # odometry mark at DETECT
        self._adv_target_m = 0.0          # distance to advance to the reading window

        # Testing aid: ignore the traffic-light supervisor so the robot drives
        # without needing to see a real GREEN light.
        self.declare_parameter('ignore_traffic_light', False)
        self._ignore_traffic_light = bool(self.get_parameter('ignore_traffic_light').value)
        # OPTIONAL traffic light (default): drive by default (as if GREEN) and only
        # OBEY the light when one is actually seen -- a sustained RED stops, and when
        # the light leaves view it returns to GREEN. This is what the user wants:
        # "the light is off at the start; only act on it if it appears." Set
        # traffic_light_optional:=false for STRICT mode (must see GREEN to move).
        self.declare_parameter('traffic_light_optional', True)
        self._traffic_light_optional = bool(self.get_parameter('traffic_light_optional').value)
        self._tl_unknown_count = 0

        # --- YOLO traffic signs (best.pt) -------------------------------------
        # Gated by use_signs (default False so it can NEVER break line following;
        # turn it on once the model is confirmed on the robot). Behaviours:
        #   workers          -> slow to workers_speed_factor for workers_slow_s
        #   stop             -> stop for stop_seconds, then continue (once)
        #   give_way         -> stop for giveway_seconds, then continue (once)
        #   turn_left/right  -> AUTO-decide that direction at the NEXT cross
        #   go_straight      -> AUTO-decide straight at the next cross
        # The signs are read in the UPPER band only and every few frames, so they
        # do not slow the loop. Detection degrades to no-op if the model is absent.
        self.declare_parameter('use_signs', False)
        self.declare_parameter('signs_model_path', '')
        self.declare_parameter('signs_conf', 0.55)
        self.declare_parameter('workers_speed_factor', 0.5)
        self.declare_parameter('workers_min_speed', 0.04)
        self.declare_parameter('workers_slow_s', 4.0)
        self.declare_parameter('stop_seconds', 3.0)
        self.declare_parameter('giveway_seconds', 1.5)
        self.declare_parameter('sign_cooldown_s', 6.0)   # don't re-fire same sign
        self.declare_parameter('sign_forget_s', 15.0)    # discard pending_turn if sign not seen
        # stop/give_way only ACT when the sign is CLOSE (its box is big enough = near).
        # Arrow signs must latch earlier so the turn survives until the next cross.
        # area_pct is a distance proxy: bigger box => closer sign.
        self.declare_parameter('sign_act_area_pct', 6.0)
        self.declare_parameter('sign_turn_act_area_pct', 1.4)
        self._sign_act_area_pct = float(self.get_parameter('sign_act_area_pct').value)
        self._sign_turn_act_area_pct = float(self.get_parameter('sign_turn_act_area_pct').value)
        self._use_signs = bool(self.get_parameter('use_signs').value)
        self._workers_speed_factor = float(self.get_parameter('workers_speed_factor').value)
        self._workers_min_speed = float(self.get_parameter('workers_min_speed').value)
        self._workers_slow_s = float(self.get_parameter('workers_slow_s').value)
        self._stop_seconds = float(self.get_parameter('stop_seconds').value)
        self._giveway_seconds = float(self.get_parameter('giveway_seconds').value)
        self._sign_cooldown_s = float(self.get_parameter('sign_cooldown_s').value)
        self._sign_forget_s = float(self.get_parameter('sign_forget_s').value)
        self._sign_detector = None
        if self._use_signs:
            mp = str(self.get_parameter('signs_model_path').value).strip()
            if not mp:
                found = self._find_config('best.pt')
                mp = str(found) if found is not None else ''
            self._sign_detector = SignDetector(
                SignParams(model_path=mp,
                           conf=float(self.get_parameter('signs_conf').value)),
                log=self.get_logger().info)
        self._sign_result = None             # last SignResult (for HUD/telemetry)
        self._pending_turn = None            # 'left'/'right'/'straight' from a sign
        self._pending_turn_until = None      # timeout: discard pending_turn if sign not seen
        self._workers_until = None           # slow-zone end time from a workers sign
        self._stopsign_until = None          # hold-still end time (stop/give_way)
        self._sign_last_fired = {}           # sign name -> last action time (cooldown)

        # Motion master switch for safe testing. Starts disabled so the robot
        # never moves until you explicitly enable it from the terminal via
        # /drive_enable (scripts/set_drive_jetson.sh on|off). Perception and the
        # stream keep running while disabled, so you can watch detection.
        self.declare_parameter('start_driving', False)
        self._drive_enabled = bool(self.get_parameter('start_driving').value)
        self.create_subscription(Bool, '/drive_enable', self._drive_enable_cb, 10)
        # When driving is disabled, RELEASE /cmd_vel (send a short STOP burst, then
        # stay silent) so an external teleop (cmd_vel_udp_bridge) can drive without
        # fighting the follower's 30 Hz zeros. The 'd' toggle thus arbitrates who
        # drives. The stop burst guarantees the robot halts even with no teleop.
        self.declare_parameter('release_cmd_when_off', True)
        self._release_cmd_when_off = bool(self.get_parameter('release_cmd_when_off').value)
        self._drive_off_stop_ticks = 0

        # Odometry: a monotonically-growing travelled-distance estimate (metres),
        # used to advance/cross exact distances at intersections independent of the
        # camera. Primary source = the robot's measured /robot_vel; if that is not
        # arriving (no motor agent / encoders), we fall back to integrating the
        # commanded speed in the control loop. Mark a point and read the delta.
        self._odom_dist = 0.0             # integrated from the COMMAND (reliable)
        self._last_odom_t = None          # for real-dt integration (loop rate varies)
        self._robot_vel_fresh_t = None    # last time /robot_vel arrived (telemetry)
        self._robot_vel_last_x = 0.0
        self.create_subscription(Twist, '/robot_vel', self._robot_vel_cb, 10)

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
        self.declare_parameter('kd', float(saved.get('kd', 0.0)))
        self.declare_parameter('max_v', float(saved.get('max_v', 0.08)))
        self.declare_parameter('max_w', float(saved.get('max_w', 0.6)))
        # Curve feedforward: steer ahead by the bend (far offset - near offset),
        # weighted by ff_gain and the SAME kp. 0 = pure feedback (old behavior);
        # ~1 = anticipate the curve. It is the bend term, so straights are
        # unaffected and the existing straight-line PD tuning is preserved.
        self.declare_parameter('ff_gain', float(saved.get('ff_gain', 1.0)))
        # Curve steering from the line's heading (tilt at the eval row). A teleop
        # demonstration showed heading is the reliable curve signal -- correctly
        # signed (positive = left) and growing from ~0.11 on a straight to ~0.5 in a
        # tight curve, unlike curvature_norm which flips sign mid-curve. kp*offset
        # alone under-steered 2-6x (reached ~0.05-0.19 while the human held ~0.30).
        # w += curve_heading_gain * heading, beyond a straight-residual deadband.
        self.declare_parameter('curve_heading_gain', float(saved.get('curve_heading_gain', 0.5)))
        self.declare_parameter('curve_heading_deadband', float(saved.get('curve_heading_deadband', 0.20)))
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.ff_gain = float(self.get_parameter('ff_gain').value)
        self._curve_heading_gain = float(self.get_parameter('curve_heading_gain').value)
        self._curve_heading_deadband = float(self.get_parameter('curve_heading_deadband').value)

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
        self.declare_parameter('commit_turn_pre_advance_cm', float(saved.get('commit_turn_pre_advance_cm', 10.0)))
        # Commit must CROSS the intersection before re-acquiring. The robot stops
        # ~10 cm before the first dashed row and the cross is ~26 cm deep (double
        # cross), so straight must travel ~36 cm before it looks for the continuing
        # line; turns must clear the cross too. At commit_speed 0.08 m/s: ~36 cm =
        # ~4.5 s (straight), turns ~2 s. The re-acquire (which is trivially true
        # while sitting on a line) is only allowed AFTER this min, so it no longer
        # cuts the commit short.
        self.declare_parameter('commit_duration', float(saved.get('commit_duration', 3.5)))
        self.declare_parameter('commit_duration_straight', float(saved.get('commit_duration_straight', 6.0)))
        # Closed-loop commit: keep turning/crossing until the lane is RE-ACQUIRED
        # (after a min time to clear the cross), capped by commit_duration above so
        # a missed line can't spin forever. 0 = old pure open-loop (time only).
        self.declare_parameter('commit_min_s', float(saved.get('commit_min_s', 2.0)))
        # STRAIGHT needs a longer minimum: it must CROSS the whole zebra (~26 cm)
        # before handing back to FOLLOW, or it re-acquires a side line of the cross
        # mid-way and turns instead of going through. Only affects 'straight'; the
        # left/right turn timing is unchanged so curves are not touched.
        self.declare_parameter('commit_straight_min_s', float(saved.get('commit_straight_min_s', 4.5)))
        self.declare_parameter('commit_closed_loop', bool(saved.get('commit_closed_loop', True)))
        self.declare_parameter('intersection_min_travel_m', float(saved.get('intersection_min_travel_m', 0.25)))
        # Square-up-in-place: at the cross, if we stopped skewed (came off a curve)
        # rotate IN PLACE (v=0, safe -- no arcing) to face the cross before reading
        # options/asking. Uses k_align as the gain (rad of zebra angle -> w); flip
        # k_align's sign live if it turns the wrong way.
        self.declare_parameter('align_in_place', bool(saved.get('align_in_place', False)))
        self.declare_parameter('align_tol_deg', float(saved.get('align_tol_deg', 12.0)))
        self.declare_parameter('align_max_w', float(saved.get('align_max_w', 0.20)))
        self.declare_parameter('align_prior_min_frames', int(saved.get('align_prior_min_frames', 5)))
        self.declare_parameter('align_prior_heading_deg', float(saved.get('align_prior_heading_deg', 8.0)))
        self.declare_parameter('align_prior_curv', float(saved.get('align_prior_curv', 0.30)))
        self.declare_parameter('align_timeout_s', float(saved.get('align_timeout_s', 0.8)))
        # The zebra angle (za) ON the cross is noisy: a robot that came in STRAIGHT
        # (small lane offset + curvature just before the cross) still reads za~-13,
        # which is detector noise, not a real skew -> it would rotate in place for
        # free and leave the cross crooked. Skip the square-up when we arrived
        # straight; only align when the approach was genuinely off a curve.
        self.declare_parameter('align_skip_when_straight', bool(saved.get('align_skip_when_straight', True)))
        self.declare_parameter('align_straight_off', float(saved.get('align_straight_off', 0.18)))
        self.declare_parameter('align_straight_curv', float(saved.get('align_straight_curv', 0.25)))
        self._align_skip_when_straight = bool(self.get_parameter('align_skip_when_straight').value)
        self._align_straight_off = float(self.get_parameter('align_straight_off').value)
        self._align_straight_curv = float(self.get_parameter('align_straight_curv').value)
        self._adv_came_straight = False    # set at DETECT from the pre-cross lane
        self._k_align = float(self.get_parameter('k_align').value)
        self._intersection_slow_speed = float(self.get_parameter('intersection_slow_speed').value)
        self._approach_speed = float(self.get_parameter('approach_speed').value)
        self._approach_align_slope = float(self.get_parameter('approach_align_slope').value)
        self._approach_timeout_s = float(self.get_parameter('approach_timeout_s').value)
        self._commit_speed = float(self.get_parameter('commit_speed').value)
        self._commit_turn_w = float(self.get_parameter('commit_turn_w').value)
        self._commit_turn_pre_advance_cm = float(self.get_parameter('commit_turn_pre_advance_cm').value)
        self._commit_odom0 = 0.0
        self._commit_start_time = None
        self._commit_duration = float(self.get_parameter('commit_duration').value)
        self._commit_duration_straight = float(self.get_parameter('commit_duration_straight').value)
        self._commit_min_s = float(self.get_parameter('commit_min_s').value)
        self._commit_straight_min_s = float(self.get_parameter('commit_straight_min_s').value)
        self._commit_closed_loop = bool(self.get_parameter('commit_closed_loop').value)
        self._intersection_min_travel_m = float(self.get_parameter('intersection_min_travel_m').value)
        self._align_in_place = bool(self.get_parameter('align_in_place').value)
        self._align_tol_deg = float(self.get_parameter('align_tol_deg').value)
        self._align_max_w = float(self.get_parameter('align_max_w').value)
        self._align_prior_min_frames = int(self.get_parameter('align_prior_min_frames').value)
        self._align_prior_heading_deg = float(self.get_parameter('align_prior_heading_deg').value)
        self._align_prior_curv = float(self.get_parameter('align_prior_curv').value)
        self._align_prior_samples = []
        self._adv_align_prior_curved = False
        self._adv_align_prior = {}
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
        self.declare_parameter('curve_memory_s', 1.20)   # keep slowing briefly after a tight curve
        self._use_birdseye = bool(self.get_parameter('use_birdseye').value)
        self._curve_slow_gain = float(self.get_parameter('curve_slow_gain').value)
        self._curve_min_scale = float(self.get_parameter('curve_min_scale').value)
        self._curve_memory_s = float(self.get_parameter('curve_memory_s').value)
        self._curve_hold_until = None
        self._curve_hold_value = 0.0
        # --- Robust curve ARC handler -------------------------------------------
        # On this track every curve is a ~45 deg LEFT bend. Instead of fighting it
        # with the PD (which anticipated or under-steered), when a tight curve is
        # CONFIRMED by the curvature MAGNITUDE (reliable; its sign flips) we drive a
        # fixed arc -- forward + left -- matching the hand-driven demo (v~0.08,
        # w~0.30), and exit when the line straightens and re-centers. All live-tunable.
        self.declare_parameter('curve_arc_enabled', bool(saved.get('curve_arc_enabled', True)))
        self.declare_parameter('curve_arc_v', float(saved.get('curve_arc_v', 0.08)))        # forward speed during arc
        self.declare_parameter('curve_arc_w', float(saved.get('curve_arc_w', 0.30)))        # +left angular during arc
        self.declare_parameter('curve_arc_enter', float(saved.get('curve_arc_enter', 0.60)))    # |curv| to enter the arc
        self.declare_parameter('curve_arc_exit', float(saved.get('curve_arc_exit', 0.30)))     # |curv| under this (centered) -> exit
        self.declare_parameter('curve_arc_exit_frames', int(saved.get('curve_arc_exit_frames', 3)))
        self.declare_parameter('curve_arc_min_s', float(saved.get('curve_arc_min_s', 0.6)))     # arc at least this long
        self.declare_parameter('curve_arc_max_s', float(saved.get('curve_arc_max_s', 4.0)))     # safety cap
        # Pre-advance: on entry, drive STRAIGHT this long before starting the left
        # turn, so the robot goes further into the curve first instead of cutting it
        # ("advance straight, then turn"). Raise it to turn later / go straighter.
        self.declare_parameter('curve_arc_pre_s', float(saved.get('curve_arc_pre_s', 0.6)))
        # Post sequence (after the main turn): advance STRAIGHT curve_arc_post_s, then
        # a short second LEFT turn (curve_arc_recenter_s) to settle back onto center,
        # then hand back to the PD. The user's "advance a bit, then turn again".
        self.declare_parameter('curve_arc_post_s', float(saved.get('curve_arc_post_s', 0.4)))
        self.declare_parameter('curve_arc_recenter_s', float(saved.get('curve_arc_recenter_s', 0.3)))
        self._curve_arc_enabled = bool(self.get_parameter('curve_arc_enabled').value)
        self._curve_arc_v = float(self.get_parameter('curve_arc_v').value)
        self._curve_arc_w = float(self.get_parameter('curve_arc_w').value)
        self._curve_arc_enter = float(self.get_parameter('curve_arc_enter').value)
        self._curve_arc_exit = float(self.get_parameter('curve_arc_exit').value)
        self._curve_arc_exit_frames = int(self.get_parameter('curve_arc_exit_frames').value)
        self._curve_arc_min_s = float(self.get_parameter('curve_arc_min_s').value)
        self._curve_arc_max_s = float(self.get_parameter('curve_arc_max_s').value)
        self._curve_arc_pre_s = float(self.get_parameter('curve_arc_pre_s').value)
        self._curve_arc_post_s = float(self.get_parameter('curve_arc_post_s').value)
        self._curve_arc_recenter_s = float(self.get_parameter('curve_arc_recenter_s').value)
        self._curve_arc_active = False
        self._curve_arc_start = None
        self._curve_arc_enter_count = 0
        self._curve_arc_exit_count = 0
        self._curve_arc_phase = 'turn'      # 'turn' (pre+left) | 'post' (advance+recenter)
        self._curve_arc_post_start = None
        # --- BLIND TURN (primary tight-curve handler; replaces the timer arc) ----
        # Follow closed-loop while the line is visible; when it is LOST, turn toward
        # the side it was going (auto left/right) until it re-appears near center.
        # Event-driven: the line decides when to stop turning, not a timer. The main
        # knob is blind_turn_w (turn rate). It only turns AFTER losing the line.
        self.declare_parameter('blind_turn_enabled', bool(saved.get('blind_turn_enabled', True)))
        self.declare_parameter('blind_turn_w', float(saved.get('blind_turn_w', 0.35)))      # blind turn rate (rad/s)
        self.declare_parameter('blind_turn_v', float(saved.get('blind_turn_v', 0.05)))      # slow forward while blind
        self.declare_parameter('blind_conf', float(saved.get('blind_conf', 0.5)))           # BEV conf = "line visible"
        self.declare_parameter('blind_reacquire_off', float(saved.get('blind_reacquire_off', 0.45)))  # |off| to call it back
        self.declare_parameter('blind_enter_frames', int(saved.get('blind_enter_frames', 3)))  # lost frames before turning
        self.declare_parameter('blind_max_s', float(saved.get('blind_max_s', 3.0)))         # safety cap
        self._blind_turn_enabled = bool(self.get_parameter('blind_turn_enabled').value)
        self._blind_turn_w = float(self.get_parameter('blind_turn_w').value)
        self._blind_turn_v = float(self.get_parameter('blind_turn_v').value)
        self._blind_conf = float(self.get_parameter('blind_conf').value)
        self._blind_reacquire_off = float(self.get_parameter('blind_reacquire_off').value)
        self._blind_enter_frames = int(self.get_parameter('blind_enter_frames').value)
        self._blind_max_s = float(self.get_parameter('blind_max_s').value)
        self._curve_side = 0.0          # smoothed turn-side memory (+left / -right)
        self._blind_lost_frames = 0
        self._blind_active = False
        self._blind_start = None
        self._blind_dir = 0.0
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

        # Heading hysteresis near a cross. When the BEV fit confidence drops in
        # the slow-zone (zebra in view) we do NOT fall through to the legacy ROI
        # detector (which can lock onto the zebra / side lines and jerk). Instead
        # we HOLD the last confident steering target and drive straight on it for
        # up to lane_hold_s, giving the anti-zebra filter time to re-acquire the
        # continuous line through the transition. No-op away from a cross.
        self.declare_parameter('lane_hold_near_cross', bool(saved.get('lane_hold_near_cross', True)))
        self.declare_parameter('lane_hold_conf', float(saved.get('lane_hold_conf', 0.5)))
        self.declare_parameter('lane_hold_s', float(saved.get('lane_hold_s', 1.5)))
        self.declare_parameter('lane_hold_curve_s', float(saved.get('lane_hold_curve_s', 1.20)))
        self.declare_parameter('lane_hold_curve_min_curv', float(saved.get('lane_hold_curve_min_curv', 0.55)))
        self._lane_hold_near_cross = bool(self.get_parameter('lane_hold_near_cross').value)
        self._lane_hold_conf = float(self.get_parameter('lane_hold_conf').value)
        self._lane_hold_s = float(self.get_parameter('lane_hold_s').value)
        self._lane_hold_curve_s = float(self.get_parameter('lane_hold_curve_s').value)
        self._lane_hold_curve_min_curv = float(self.get_parameter('lane_hold_curve_min_curv').value)
        self._lane_hold_center_x = None  # last confident steering center (orig px)
        self._lane_hold_far_x = None
        self._lane_hold_curvature = 0.0
        self._lane_hold_signed_curvature = 0.0
        self._lane_hold_time = None      # when it was captured (for the timeout)

        # Branch guard near a cross. At a fork the BEV can briefly drop the line
        # (anti-zebra row reject) and then RE-ACQUIRE onto the diverging side
        # branch with HIGH confidence -> the robot turns off instead of going
        # straight. Two defenses, BOTH gated on _near_intersection so the normal
        # straight/curve is untouched:
        #   - sticky base: keep _lane_prev_base anchored through brief dropouts so
        #     re-acquisition stays in the narrow continuity corridor (not the wide
        #     center band that grabs the branch). Dropped after lane_base_hold_s.
        #   - base-jump reject: ignore a detected base that jumped more than
        #     lane_base_max_jump_pct of the warp width from the last good base.
        self.declare_parameter('lane_base_hold_s', float(saved.get('lane_base_hold_s', 1.0)))
        self.declare_parameter('lane_base_max_jump_pct', int(saved.get('lane_base_max_jump_pct', 15)))
        self.declare_parameter('lane_curve_max_jump_pct', int(saved.get('lane_curve_max_jump_pct', 10)))
        self.declare_parameter('lane_curve_guard_conf', float(saved.get('lane_curve_guard_conf', 0.80)))
        self.declare_parameter('lane_curve_guard_max_offset', float(saved.get('lane_curve_guard_max_offset', 0.35)))
        self.declare_parameter('lane_curve_hold_assist_conf', float(saved.get('lane_curve_hold_assist_conf', 0.80)))
        self._lane_base_hold_s = float(self.get_parameter('lane_base_hold_s').value)
        self._lane_base_max_jump_pct = int(self.get_parameter('lane_base_max_jump_pct').value)
        self._lane_curve_max_jump_pct = int(self.get_parameter('lane_curve_max_jump_pct').value)
        self._lane_curve_guard_conf = float(self.get_parameter('lane_curve_guard_conf').value)
        self._lane_curve_guard_max_offset = float(self.get_parameter('lane_curve_guard_max_offset').value)
        self._lane_curve_hold_assist_conf = float(self.get_parameter('lane_curve_hold_assist_conf').value)
        self._lane_good_base = None      # last accepted base x (warped px)
        self._lane_good_base_time = None # when it was accepted (for the timeout)

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
        self._loop_stage = 'init'
        self._last_commit_tick_t = None
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

        # Timer (30 Hz). Use the SAFE wrapper so an unhandled exception in the
        # control loop can never (a) crash the timer callback (which would freeze
        # the camera read + leave the last cmd_vel latched -> runaway), nor
        # (b) leave the robot driving. On error we publish STOP and recover next tick.
        self.timer = self.create_timer(0.033, self._safe_control_loop)

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
        was = self._drive_enabled
        self._drive_enabled = bool(msg.data)
        if was and not self._drive_enabled:
            # transition ON->OFF: queue a STOP burst before releasing /cmd_vel
            self._drive_off_stop_ticks = 15
        self.get_logger().info(f"[DRIVE] enabled={self._drive_enabled}")

    def _robot_vel_cb(self, msg):
        """Record /robot_vel for telemetry only. Its content proved unreliable for
        distance (it read ~0 while the robot was clearly moving), so odometry is
        integrated from the COMMAND in _odom_tick instead."""
        self._robot_vel_fresh_t = self.get_clock().now()
        self._robot_vel_last_x = float(msg.linear.x)

    def _odom_tick(self, now, cmd):
        """Integrate the COMMANDED forward speed into travelled distance, using the
        REAL elapsed time between calls (the loop rate varies a lot -- the debug
        composite stream slows it well below 30 Hz -- so a fixed dt under-counted
        ~3x and the ADVANCE never reached its target -> timeout -> never asked)."""
        if self._last_odom_t is not None:
            dt = (now - self._last_odom_t).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._odom_dist += abs(float(cmd.linear.x)) * dt
        self._last_odom_t = now

    def _odom_m(self):
        return self._odom_dist

    def _run_signs(self, frame, now):
        """Detect a sign and LATCH its action. No-op if signs are disabled. Each
        sign re-fires at most once per sign_cooldown_s so it doesn't retrigger."""
        if not self._use_signs or self._sign_detector is None:
            return
        res = self._sign_detector.detect(frame)
        self._sign_result = res
        
        # Log all detections for debugging and events
        if res.all_detections:
            signs_summary = ", ".join([
                f"{s['name']}(c:{s['conf']:.2f},a:{s['area_pct']:.1f}%,sc:{s['score']:.2f})"
                for s in res.all_detections
            ])
            self.get_logger().info(
                f"[SIGN] Detected: {signs_summary} | Selected: {res.name or 'none'}",
                throttle_duration_sec=2.0
            )
            # Event: log all detections with vote details
            detections_for_event = []
            for s in res.all_detections:
                det = {
                    'name': s['name'],
                    'conf': round(s['conf'], 3),
                    'area_pct': round(s['area_pct'], 2),
                    'score': round(s['score'], 3)
                }
                if s.get('original_name'):
                    det['corrected_from'] = s['original_name']
                if s.get('vote_details'):
                    det['votes'] = s['vote_details']
                detections_for_event.append(det)
            
            self._event('signs_detected', 
                       detections=detections_for_event,
                       selected=res.name,
                       selected_conf=round(res.conf, 3) if res.name else None,
                       selected_area=round(res.area_pct, 2) if res.name else None)
        
        # Check if we have a pending turn from a directional sign
        if self._pending_turn is not None:
            # The forget timer ONLY runs while purely FOLLOWING the line. In ANY
            # other state -- approach/ADVANCE, wait/READ, commit, or while HELD by a
            # red light / stop hold / drive disabled -- the sign is out of the FOV by
            # design, so the countdown is FROZEN. Otherwise time spent at a light or
            # advancing onto the cross would drop the turn before we can use it.
            is_following = (self.intersection_phase is None
                            and self.commit_direction is None)
            held_in_place = (
                self.current_state == 'RED'
                or (self._stopsign_until is not None and now < self._stopsign_until)
                or not self._drive_enabled)
            counter_active = is_following and not held_in_place
            # If the sign is still visible, refresh the timeout
            if res.name in ('turn_left', 'turn_right', 'go_straight'):
                self._pending_turn_until = now + Duration(seconds=self._sign_forget_s)
            # Only count down (and possibly discard) while purely FOLLOWING
            elif not counter_active:
                self._pending_turn_until = now + Duration(seconds=self._sign_forget_s)
            elif (self._pending_turn_until is not None
                  and now >= self._pending_turn_until):
                discarded_turn = self._pending_turn
                self.get_logger().warn(
                    f"[SIGN] Discarding pending turn '{discarded_turn}' - sign not seen for {self._sign_forget_s}s"
                )
                self._event('sign_timeout', 
                           discarded_turn=discarded_turn,
                           timeout_s=self._sign_forget_s)
                self._pending_turn = None
                self._pending_turn_until = None
        
        if res.name is None:
            return
        last = self._sign_last_fired.get(res.name)
        if last is not None and (now - last).nanoseconds * 1e-9 < self._sign_cooldown_s:
            return
        
        name = res.name
        if name in ('turn_left', 'turn_right', 'go_straight'):
            # Directional signs latch earlier than STOP/give_way. In real runs the
            # arrow often leaves the FOV before the zebra READ window; waiting for
            # the stop-sign distance threshold means no pending_turn is ever set.
            if res.area_pct < self._sign_turn_act_area_pct:
                self.get_logger().info(
                    f"[SIGN] {name} seen far (area {res.area_pct:.1f}% < "
                    f"{self._sign_turn_act_area_pct:.1f}%) -> waiting to get closer",
                    throttle_duration_sec=1.0)
                return
            direction = {'turn_left': 'left', 'turn_right': 'right',
                         'go_straight': 'straight'}[name]
            if self._pending_turn is not None and self._pending_turn != direction:
                self.get_logger().warn(
                    f"[SIGN] ignoring conflicting {name}->{direction}; "
                    f"pending {self._pending_turn} is already latched",
                    throttle_duration_sec=1.0)
                self._event('sign_conflict_ignored',
                            sign_name=name,
                            direction=direction,
                            pending_turn=self._pending_turn,
                            conf=round(res.conf, 3),
                            area_pct=round(res.area_pct, 2))
                return
            self._pending_turn = direction
            self._pending_turn_until = now + Duration(seconds=self._sign_forget_s)
            self._sign_last_fired[name] = now
            self.get_logger().warn(
                f"[SIGN] {name} ({res.conf:.2f}, area {res.area_pct:.1f}%) -> auto-{self._pending_turn} at next cross "
                f"(expires in {self._sign_forget_s}s if not seen)")
            self._event('sign_action', 
                       sign_name=name,
                       action='pending_turn',
                       direction=self._pending_turn,
                       conf=round(res.conf, 3),
                       area_pct=round(res.area_pct, 2),
                       threshold_pct=self._sign_turn_act_area_pct,
                       expires_s=self._sign_forget_s)
        elif name == 'workers':
            self._workers_until = now + Duration(seconds=self._workers_slow_s)
            self._sign_last_fired[name] = now
            self.get_logger().warn(f"[SIGN] workers ({res.conf:.2f}) -> slowing")
            self._event('sign_action',
                       sign_name=name,
                       action='slow_zone',
                       conf=round(res.conf, 3),
                       duration_s=self._workers_slow_s)
        elif name in ('stop', 'give_way'):
            # Only act when the sign is CLOSE (box big enough). Far away we wait.
            if res.area_pct < self._sign_act_area_pct:
                self.get_logger().info(
                    f"[SIGN] {name} seen far (area {res.area_pct:.1f}<"
                    f"{self._sign_act_area_pct:.1f}) -> waiting to get closer",
                    throttle_duration_sec=1.0)
                return
            if self._stopsign_until is None:        # not already holding
                dur = self._stop_seconds if name == 'stop' else self._giveway_seconds
                self._stopsign_until = now + Duration(seconds=dur)
                self._sign_last_fired[name] = now
                self.get_logger().warn(
                    f"[SIGN] {name} ({res.conf:.2f}, area {res.area_pct:.1f}) -> hold {dur:.1f}s")
                self._event('sign_action',
                           sign_name=name,
                           action='stop_hold',
                           conf=round(res.conf, 3),
                           area_pct=round(res.area_pct, 2),
                           duration_s=dur)

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
        # The OPERATOR is the authority: a manual decision is always obeyed, even
        # if it is not in the auto-detected options (those are a best-effort hint
        # and can be wrong/incomplete -- e.g. a real straight read as left-only).
        if (self.intersection_pending and self.intersection_options
                and normalized not in self.intersection_options):
            self.get_logger().warn(
                f"Decision '{normalized}' not in detected options "
                f"({', '.join(self.intersection_options)}); obeying operator anyway."
            )
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
            elif p.name == 'curve_heading_gain':
                self._curve_heading_gain = float(p.value)
            elif p.name == 'curve_heading_deadband':
                self._curve_heading_deadband = float(p.value)
            elif p.name == 'blind_turn_enabled':
                self._blind_turn_enabled = bool(p.value)
            elif p.name == 'blind_turn_w':
                self._blind_turn_w = float(p.value)
            elif p.name == 'blind_turn_v':
                self._blind_turn_v = float(p.value)
            elif p.name == 'blind_conf':
                self._blind_conf = float(p.value)
            elif p.name == 'blind_reacquire_off':
                self._blind_reacquire_off = float(p.value)
            elif p.name == 'blind_enter_frames':
                self._blind_enter_frames = int(p.value)
            elif p.name == 'blind_max_s':
                self._blind_max_s = float(p.value)
            elif p.name == 'curve_arc_enabled':
                self._curve_arc_enabled = bool(p.value)
            elif p.name == 'curve_arc_v':
                self._curve_arc_v = float(p.value)
            elif p.name == 'curve_arc_w':
                self._curve_arc_w = float(p.value)
            elif p.name == 'curve_arc_enter':
                self._curve_arc_enter = float(p.value)
            elif p.name == 'curve_arc_exit':
                self._curve_arc_exit = float(p.value)
            elif p.name == 'curve_arc_exit_frames':
                self._curve_arc_exit_frames = int(p.value)
            elif p.name == 'curve_arc_min_s':
                self._curve_arc_min_s = float(p.value)
            elif p.name == 'curve_arc_max_s':
                self._curve_arc_max_s = float(p.value)
            elif p.name == 'curve_arc_pre_s':
                self._curve_arc_pre_s = float(p.value)
            elif p.name == 'curve_arc_post_s':
                self._curve_arc_post_s = float(p.value)
            elif p.name == 'curve_arc_recenter_s':
                self._curve_arc_recenter_s = float(p.value)
            elif p.name == 'snapshot_interval':
                self._snapshot_interval = float(p.value)   # live recorder rate (s)
            elif p.name == 'curve_slow_gain':
                self._curve_slow_gain = float(p.value)
            elif p.name == 'curve_min_scale':
                self._curve_min_scale = float(p.value)
            elif p.name == 'curve_memory_s':
                self._curve_memory_s = float(p.value)
            elif p.name == 'lane_hold_near_cross':
                self._lane_hold_near_cross = bool(p.value)
            elif p.name == 'lane_hold_conf':
                self._lane_hold_conf = float(p.value)
            elif p.name == 'lane_hold_s':
                self._lane_hold_s = float(p.value)
            elif p.name == 'lane_hold_curve_s':
                self._lane_hold_curve_s = float(p.value)
            elif p.name == 'lane_hold_curve_min_curv':
                self._lane_hold_curve_min_curv = float(p.value)
            elif p.name == 'lane_base_hold_s':
                self._lane_base_hold_s = float(p.value)
            elif p.name == 'lane_base_max_jump_pct':
                self._lane_base_max_jump_pct = int(p.value)
            elif p.name == 'lane_curve_max_jump_pct':
                self._lane_curve_max_jump_pct = int(p.value)
            elif p.name == 'lane_curve_guard_conf':
                self._lane_curve_guard_conf = float(p.value)
            elif p.name == 'lane_curve_guard_max_offset':
                self._lane_curve_guard_max_offset = float(p.value)
            elif p.name == 'lane_curve_hold_assist_conf':
                self._lane_curve_hold_assist_conf = float(p.value)
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
            elif p.name == 'commit_turn_pre_advance_cm':
                self._commit_turn_pre_advance_cm = float(p.value)
            elif p.name == 'commit_duration':
                self._commit_duration = float(p.value)
            elif p.name == 'commit_duration_straight':
                self._commit_duration_straight = float(p.value)
            elif p.name == 'intersection_min_travel_m':
                self._intersection_min_travel_m = float(p.value)
            elif p.name == 'commit_min_s':
                self._commit_min_s = float(p.value)
            elif p.name == 'commit_straight_min_s':
                self._commit_straight_min_s = float(p.value)
            elif p.name == 'commit_closed_loop':
                self._commit_closed_loop = bool(p.value)
            elif p.name == 'advance_lane_keep_gain':
                self._advance_lane_keep_gain = float(p.value)
            elif p.name == 'advance_lane_keep_max_w':
                self._advance_lane_keep_max_w = float(p.value)
            elif p.name == 'stream_debug':
                self._stream_debug = bool(p.value)
            elif p.name == "traffic_light_roi_y_pct":
                self._tl_roi_y_pct = int(p.value)
            elif p.name == "traffic_light_min_area":
                self._tl_min_area = float(p.value)
            elif p.name == "traffic_light_max_area":
                self._tl_max_area = float(p.value)
            elif p.name == "traffic_light_min_circularity":
                self._tl_min_circularity = float(p.value)
            elif p.name == "traffic_light_aspect_tol":
                self._tl_aspect_tol = float(p.value)
            elif p.name == "traffic_light_min_fill":
                self._tl_min_fill = float(p.value)
            elif p.name == "traffic_light_max_fill":
                self._tl_max_fill = float(p.value)
            elif p.name == "traffic_light_require_plate":
                self._tl_require_plate = bool(p.value)
            elif p.name == "traffic_light_plate_max_sat":
                self._tl_plate_max_sat = float(p.value)
            elif p.name == "traffic_light_plate_min_val":
                self._tl_plate_min_val = float(p.value)
            elif p.name == "traffic_light_plate_max_val":
                self._tl_plate_max_val = float(p.value)
            elif p.name == "traffic_light_position_classify":
                self._tl_position_classify = bool(p.value)
            elif p.name == "traffic_light_position_map":
                self._tl_position_map = self._parse_tl_position_map(str(p.value))
            elif p.name == "traffic_light_position_anchors_pct":
                self._tl_position_anchors = self._parse_tl_position_anchors(str(p.value))
            elif p.name == "traffic_light_position_max_slot_error_pct":
                self._tl_position_max_slot_error = float(p.value) / 100.0
            elif p.name == "traffic_light_plate_min_area":
                self._tl_plate_min_area = float(p.value)
            elif p.name == "traffic_light_action_min_radius_px":
                self._tl_action_min_radius_px = float(p.value)
            elif p.name == "traffic_light_action_max_radius_px":
                self._tl_action_max_radius_px = float(p.value)
            elif p.name == "traffic_light_action_min_distance_cm":
                self._tl_action_min_distance_cm = float(p.value)
            elif p.name == "traffic_light_action_max_distance_cm":
                self._tl_action_max_distance_cm = float(p.value)
            elif p.name == "traffic_light_distance_k_cm_px":
                self._tl_distance_k_cm_px = float(p.value)
            elif p.name == 'workers_speed_factor':
                self._workers_speed_factor = float(p.value)
            elif p.name == 'workers_min_speed':
                self._workers_min_speed = float(p.value)
            elif p.name == 'workers_slow_s':
                self._workers_slow_s = float(p.value)
            elif p.name == 'stop_seconds':
                self._stop_seconds = float(p.value)
            elif p.name == 'giveway_seconds':
                self._giveway_seconds = float(p.value)
            elif p.name == 'sign_cooldown_s':
                self._sign_cooldown_s = float(p.value)
            elif p.name == 'sign_forget_s':
                self._sign_forget_s = float(p.value)
            elif p.name == 'sign_act_area_pct':
                self._sign_act_area_pct = float(p.value)
            elif p.name == 'sign_turn_act_area_pct':
                self._sign_turn_act_area_pct = float(p.value)
            elif p.name == 'detect_distance_cm':
                self._detect_distance_cm = float(p.value)
            elif p.name == 'read_distance_cm':
                self._read_distance_cm = float(p.value)
            elif p.name == 'read_advance_extra_cm':
                self._read_advance_extra_cm = float(p.value)
            elif p.name == 'read_after_entry_max_cm':
                self._read_after_entry_max_cm = float(p.value)
            elif p.name == 'read_cross_jump_cm':
                self._read_cross_jump_cm = float(p.value)
            elif p.name == 'advance_center_gain':
                self._advance_center_gain = float(p.value)
            elif p.name == 'align_in_place':
                self._align_in_place = bool(p.value)
            elif p.name == 'align_tol_deg':
                self._align_tol_deg = float(p.value)
            elif p.name == 'align_max_w':
                self._align_max_w = float(p.value)
            elif p.name == 'align_prior_min_frames':
                self._align_prior_min_frames = int(p.value)
            elif p.name == 'align_prior_heading_deg':
                self._align_prior_heading_deg = float(p.value)
            elif p.name == 'align_prior_curv':
                self._align_prior_curv = float(p.value)
            elif p.name == 'align_timeout_s':
                self._align_timeout_s = float(p.value)
            elif p.name == 'align_skip_when_straight':
                self._align_skip_when_straight = bool(p.value)
            elif p.name == 'align_straight_off':
                self._align_straight_off = float(p.value)
            elif p.name == 'align_straight_curv':
                self._align_straight_curv = float(p.value)
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
                'curve_heading_gain': self._curve_heading_gain,
                'curve_heading_deadband': self._curve_heading_deadband,
                'curve_slow_gain': self._curve_slow_gain,
                'curve_min_scale': self._curve_min_scale,
                'curve_memory_s': self._curve_memory_s,
                'curve_arc_enabled': self._curve_arc_enabled,
                'curve_arc_v': self._curve_arc_v,
                'curve_arc_w': self._curve_arc_w,
                'curve_arc_enter': self._curve_arc_enter,
                'curve_arc_exit': self._curve_arc_exit,
                'curve_arc_exit_frames': self._curve_arc_exit_frames,
                'curve_arc_min_s': self._curve_arc_min_s,
                'curve_arc_max_s': self._curve_arc_max_s,
                'curve_arc_pre_s': self._curve_arc_pre_s,
                'curve_arc_post_s': self._curve_arc_post_s,
                'curve_arc_recenter_s': self._curve_arc_recenter_s,
                'blind_turn_enabled': self._blind_turn_enabled,
                'blind_turn_w': self._blind_turn_w,
                'blind_turn_v': self._blind_turn_v,
                'blind_conf': self._blind_conf,
                'blind_reacquire_off': self._blind_reacquire_off,
                'blind_enter_frames': self._blind_enter_frames,
                'blind_max_s': self._blind_max_s,
                'lane_hold_near_cross': self._lane_hold_near_cross,
                'lane_hold_conf': self._lane_hold_conf,
                'lane_hold_s': self._lane_hold_s,
                'lane_hold_curve_s': self._lane_hold_curve_s,
                'lane_hold_curve_min_curv': self._lane_hold_curve_min_curv,
                'lane_base_hold_s': self._lane_base_hold_s,
                'lane_base_max_jump_pct': self._lane_base_max_jump_pct,
                'lane_curve_max_jump_pct': self._lane_curve_max_jump_pct,
                'lane_curve_guard_conf': self._lane_curve_guard_conf,
                'lane_curve_guard_max_offset': self._lane_curve_guard_max_offset,
                'lane_curve_hold_assist_conf': self._lane_curve_hold_assist_conf,
                'k_align': self._k_align,
                'intersection_slow_speed': self._intersection_slow_speed,
                'approach_align_slope': self._approach_align_slope,
                'approach_timeout_s': self._approach_timeout_s,
                'commit_speed': self._commit_speed,
                'commit_turn_w': self._commit_turn_w,
                'commit_turn_pre_advance_cm': self._commit_turn_pre_advance_cm,
                'commit_duration': self._commit_duration,
                'commit_duration_straight': self._commit_duration_straight,
                'commit_min_s': self._commit_min_s,
                'commit_straight_min_s': self._commit_straight_min_s,
                'commit_closed_loop': self._commit_closed_loop,
                'sign_turn_act_area_pct': self._sign_turn_act_area_pct,
                'sign_act_area_pct': self._sign_act_area_pct,
                'sign_cooldown_s': self._sign_cooldown_s,
                'sign_forget_s': self._sign_forget_s,
                'workers_min_speed': self._workers_min_speed,
                'advance_lane_keep_gain': self._advance_lane_keep_gain,
                'advance_lane_keep_max_w': self._advance_lane_keep_max_w,
                'align_in_place': self._align_in_place,
                'align_tol_deg': self._align_tol_deg,
                'align_max_w': self._align_max_w,
                'align_prior_min_frames': self._align_prior_min_frames,
                'align_prior_heading_deg': self._align_prior_heading_deg,
                'align_prior_curv': self._align_prior_curv,
                'intersection_min_travel_m': self._intersection_min_travel_m,
                'align_skip_when_straight': self._align_skip_when_straight,
                'align_straight_off': self._align_straight_off,
                'align_straight_curv': self._align_straight_curv,
                'read_after_entry_max_cm': self._read_after_entry_max_cm,
                'traffic_light_roi_y_pct': self._tl_roi_y_pct,
                'traffic_light_min_area': self._tl_min_area,
                'traffic_light_max_area': self._tl_max_area,
                'traffic_light_min_circularity': self._tl_min_circularity,
                'traffic_light_aspect_tol': self._tl_aspect_tol,
                'traffic_light_min_fill': self._tl_min_fill,
                'traffic_light_max_fill': self._tl_max_fill,
                'traffic_light_require_plate': self._tl_require_plate,
                'traffic_light_plate_max_sat': self._tl_plate_max_sat,
                'traffic_light_plate_min_val': self._tl_plate_min_val,
                'traffic_light_plate_max_val': self._tl_plate_max_val,
                'traffic_light_position_classify': self._tl_position_classify,
                'traffic_light_position_map': ','.join(self._tl_position_map),
                'traffic_light_position_anchors_pct': ','.join(f'{anchor * 100.0:.1f}' for anchor in self._tl_position_anchors),
                'traffic_light_position_max_slot_error_pct': self._tl_position_max_slot_error * 100.0,
                'traffic_light_plate_min_area': self._tl_plate_min_area,
                'traffic_light_action_min_radius_px': self._tl_action_min_radius_px,
                'traffic_light_action_max_radius_px': self._tl_action_max_radius_px,
                'traffic_light_action_min_distance_cm': self._tl_action_min_distance_cm,
                'traffic_light_action_max_distance_cm': self._tl_action_max_distance_cm,
                'traffic_light_distance_k_cm_px': self._tl_distance_k_cm_px,
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
        if self.intersection_phase == "wait":
            return ("READ: decision", (0, 0, 255))
        if self.intersection_phase == "approach":
            return ("ADVANCE", (0, 255, 255))
        if self.commit_direction is not None:
            return (f"COMMIT {self.commit_direction}", (255, 160, 0))
        if self.time_line_lost is not None:
            return ("RECOVER: line lost", (0, 128, 255))
        return ("FOLLOW", (0, 255, 0))

    def _draw_traffic_light_overlay(self, frame, cand):
        if cand is None:
            return
        color_map = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255), "GREEN": (0, 255, 0)}
        col = color_map.get(cand.get("color"), (255, 255, 255))
        cx, cy = cand["center"]
        radius = cand["radius"]
        x, y, bw, bh = cand["bbox"]
        plate = cand.get("plate_bbox")
        if plate is not None:
            px, py, pw, ph = plate
            cv2.rectangle(frame, (px, py), (px + pw, py + ph), (180, 180, 180), 1)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), col, 2)
        cv2.circle(frame, (int(cx), int(cy)), int(radius), col, 2)
        slot = cand.get('slot')
        pos = f" pos={slot:.2f}" if isinstance(slot, (int, float)) else ""
        dist = cand.get('distance_cm')
        dist_txt = f" d={dist:.0f}cm" if isinstance(dist, (int, float)) else ""
        act_txt = "ACT" if cand.get('actionable', False) else "IGN"
        hsv_name = cand.get('hsv_color') or cand.get('color')
        txt = (f"TL {cand['color']} {act_txt} hsv={hsv_name}{pos}{dist_txt} "
               f"c={cand['circularity']:.2f} fill={cand['fill']:.2f}")
        cv2.putText(frame, txt, (x, max(18, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

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

        tlextra = ""
        tl = self._traffic_light_candidate
        if tl is not None:
            cx, cy = tl["center"]
            tlextra = " TL:{}@{:.0f},{:.0f} r{:.0f}".format(
                tl["color"], cx, cy, tl["radius"])

        zextra = ""
        if self._use_zebra_bev and self.zebra_result is not None and self.zebra_result.seen:
            zr = self.zebra_result
            zd = "?" if zr.distance_cm is None else f"{zr.distance_cm:.0f}"
            zextra = f"  ZEB:{zd}cm[{','.join(zr.options) or '-'}]"

        cv2.putText(frame, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        line2 = (f"drive:{'ON' if self._drive_enabled else 'off'}  light:{light}  "
                 f"{src} off:{off:+.2f} conf:{conf:.2f} curv:{curv:+.2f}  "
                 f"v:{cmd.linear.x:.3f} w:{cmd.angular.z:+.2f}{tlextra}{zextra}")
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

    def _write_loop_exception(self, exc, traceback_text):
        """Persist control-loop exceptions where session pulls can capture them."""
        try:
            self._event(
                'loop_exception',
                error=repr(exc),
                loop_stage=getattr(self, '_loop_stage', 'unknown'),
                traceback=traceback_text[-4000:],
            )
        except Exception:  # noqa: BLE001
            pass
        try:
            path = self._snapshot_dir() / 'control_exceptions.log'
            path.parent.mkdir(parents=True, exist_ok=True)
            t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            with open(path, 'a', buffering=1) as fp:
                fp.write(
                    f"\n=== control_loop exception t={t:.3f} "
                    f"stage={getattr(self, '_loop_stage', 'unknown')} "
                    f"state={self._phase_label()[0]} "
                    f"phase={self.intersection_phase} "
                    f"commit={self.commit_direction} ===\n"
                )
                fp.write(traceback_text)
        except Exception:  # noqa: BLE001
            pass

    def _commit_debug_tick(self, now, stage, cmd=None, force=False, **fields):
        if self.commit_direction is None:
            return
        last = self._last_commit_tick_t
        if (not force and last is not None
                and (now - last).nanoseconds * 1e-9 < 0.25):
            return
        self._last_commit_tick_t = now
        lr = self._last_lane_result
        payload = {
            'stage': stage,
            'direction': self.commit_direction,
            'cmd_v': None if cmd is None else round(float(cmd.linear.x), 3),
            'cmd_w': None if cmd is None else round(float(cmd.angular.z), 3),
            'lane_detected': bool(lr is not None and lr.detected),
            'lane_conf': None if lr is None else round(float(lr.confidence), 3),
            'lane_off': None if lr is None else round(float(lr.offset_norm), 3),
            'drive_enabled': bool(self._drive_enabled),
            'light': self.current_state,
        }
        payload.update(fields)
        self._event('commit_tick', **payload)

    def _snapshot_dir(self):
        base = Path('/home/puzzlebot/ros2_ws/src/puzzlebot_ros')
        if not base.is_dir():
            base = Path(__file__).resolve().parents[1]
        return base / 'debug_dataset' / 'follower_session'

    def _build_debug_composite(self, frame):
        """[camera+HUD | lane BEV (mask+windows+fit) | wide zebra BEV (row+why)].

        Fixed panel sizes (black placeholders when a result is missing) so the
        composite keeps a CONSTANT size every frame -- required for the H264
        encoder, and what lets us watch everything the robot sees live.
        """
        h = frame.shape[0]
        panels = [frame]

        lp = self.lane_params
        lr = self._last_lane_result
        if lr is not None and lr.warped_mask is not None:
            bev = draw_birdseye_debug(lr, lp)
        else:
            bev = np.zeros((lp.warp_h, lp.warp_w, 3), np.uint8)
            cv2.putText(bev, 'lane: --', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (120, 120, 120), 1)
        s = h / float(bev.shape[0])
        panels.append(cv2.resize(bev, (max(1, int(bev.shape[1] * s)), h)))

        if self._use_zebra_bev:
            zp = self.zebra_params
            if self._zebra_M is not None:
                zbev = cv2.warpPerspective(frame, self._zebra_M, (zp.warp_w, zp.warp_h))
                if self.zebra_result is not None:
                    zbev = draw_zebra_overlay(zbev, self.zebra_result)
            else:
                zbev = np.zeros((zp.warp_h, zp.warp_w, 3), np.uint8)
                cv2.putText(zbev, 'zebra: --', (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (120, 120, 120), 1)
            s = h / float(zbev.shape[0])
            panels.append(cv2.resize(zbev, (max(1, int(zbev.shape[1] * s)), h)))
        return cv2.hconcat(panels)

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
            cv2.imwrite(str(d / f'follow_{stamp}_{state}.jpg'),
                        self._build_debug_composite(frame))
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
            'sign': (self._sign_result.name if self._sign_result else None),
            'pending_turn': self._pending_turn,
            'odom': round(self._odom_m(), 2),
            'advance': (round((self._odom_m() - self._adv_odom0) * 100, 0)
                        if self.intersection_phase == 'approach' else None),
            'advance_target': (round(self._adv_target_m * 100, 0)
                               if self.intersection_phase == 'approach' else None),
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
                'curve_heading_gain': self._curve_heading_gain,
                'curve_heading_deadband': self._curve_heading_deadband,
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
        try:
            self._csv_fp.flush()
        except OSError:
            pass

    def _publish_stream_frame(self, frame):
        """Throttle and push the annotated frame to the active stream (MJPEG or H264).

        With stream_debug on, push the full debug composite (camera + lane BEV +
        zebra BEV) so the live stream shows everything the robot sees, not just the
        camera. The composite has a constant size, so the H264 encoder is happy.
        """
        now = self.get_clock().now()
        if (self._last_stream_time is not None
                and (now - self._last_stream_time).nanoseconds * 1e-9 < self._stream_min_period):
            return
        self._last_stream_time = now

        if self._stream_debug:
            try:
                frame = self._build_debug_composite(frame)
            except Exception as exc:  # never let the overlay kill the stream
                self.get_logger().warn(f'[stream] debug composite failed: {exc}',
                                       throttle_duration_sec=5.0)

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

        # DETECT: first stable sight of the entry row within detect range -> start
        # ADVANCE. Freeze a travel target from the MEASURED distance now, then drive
        # that far by ODOMETRY to the reading window, so we no longer need the row
        # in view while moving onto the cross (it leaves the camera when close).
        # ROBUSTNESS: Only trigger if:
        # 1. Zebra is seen and within range
        # 2. No intersection phase active
        # 3. Cooldown expired (prevent re-trigger after recent commit)
        # 4. Sufficient stability (prevent false triggers)
        # NOTE: pending_turn is OK - it will be used as auto-decision at the intersection
        cooldown_ok = (self.intersection_cooldown_until is None 
                      or now >= self.intersection_cooldown_until)
        stable_frames = int(zres.stable_frames) if zres is not None else 0
        stable_ok = stable_frames >= 3  # Require at least 3 consecutive frames
        dashes_ok = (
            zres is not None
            and int(zres.n_dashes) >= int(zp.trigger_min_dashes))
        angle_ok = (
            zres is not None and zres.angle_deg is not None
            and abs(float(zres.angle_deg)) <= float(zp.trigger_max_angle_deg))
        center_ok = (
            zres is not None
            and (float(zp.trigger_max_center_cm) <= 0.0
                 or zres.row_center_cm is None
                 or abs(float(zres.row_center_cm)) <= float(zp.trigger_max_center_cm)))
        trigger_ok = stable_ok and dashes_ok and angle_ok and center_ok
        
        # Debug: log why approach is rejected
        if (zres is not None and zres.seen and dist is not None
                and dist <= self._detect_distance_cm 
                and self.intersection_phase is None):
            if not self._drive_enabled:
                self.get_logger().info(
                    "[ZEBRA] Approach blocked: drive disabled",
                    throttle_duration_sec=2.0)
            elif not cooldown_ok:
                self.get_logger().info(
                    f"[ZEBRA] Approach blocked: cooldown active",
                    throttle_duration_sec=2.0)
            elif not stable_ok:
                self.get_logger().info(
                    f"[ZEBRA] Approach blocked: insufficient stability ({stable_frames}/3 frames)",
                    throttle_duration_sec=2.0)
            elif not dashes_ok:
                nd = 0 if zres is None else int(zres.n_dashes)
                self.get_logger().info(
                    f"[ZEBRA] Approach blocked: weak row dashes={nd} "
                    f"< {int(zp.trigger_min_dashes)}",
                    throttle_duration_sec=1.0)
            elif not angle_ok:
                za = '?' if zres.angle_deg is None else f'{zres.angle_deg:.1f}'
                self.get_logger().info(
                    f"[ZEBRA] Approach blocked: skewed row angle={za}deg "
                    f"> {zp.trigger_max_angle_deg:.1f}deg",
                    throttle_duration_sec=1.0)
            elif not center_ok:
                rc = '?' if zres.row_center_cm is None else f'{zres.row_center_cm:.1f}'
                self.get_logger().info(
                    f"[ZEBRA] Approach blocked: row center={rc}cm "
                    f"> {zp.trigger_max_center_cm:.1f}cm",
                    throttle_duration_sec=1.0)
        
        if (zres is not None and zres.seen and dist is not None
                and dist <= self._detect_distance_cm 
                and self.intersection_phase is None
                and self._drive_enabled
                and cooldown_ok and trigger_ok):
            self.intersection_phase = 'approach'   # ADVANCE
            self._zebra_opt_votes = {}
            self._adv_odom0 = self._odom_m()
            self._adv_target_m = max(0.0, (dist - self._read_distance_cm) / 100.0)
            self._adv_entry_target_m = max(0.0, (dist + 2.0) / 100.0)
            self._adv_at_entry = False
            self._adv_prev_dist = dist         # seed the cross-jump detector
            self._approach_start_time = now
            # Did we arrive STRAIGHT? Read the pre-cross lane (last FOLLOW frame):
            # small offset + curvature => the za skew on the cross is noise, so the
            # WAIT square-up must NOT fire (FIX 2). Off a curve this stays False.
            lr = self._last_lane_result
            pre_off = abs(lr.offset_norm) if (lr is not None and lr.detected) else 0.0
            pre_curv = abs(lr.curvature_norm) if (lr is not None and lr.detected) else 0.0
            prior = list(self._align_prior_samples)
            prior_frames = len(prior)
            if prior:
                prior_heading = float(np.median([p["heading_deg"] for p in prior]))
                prior_abs_heading = float(np.median([abs(p["heading_deg"]) for p in prior]))
                prior_curv = float(np.median([p["curv"] for p in prior]))
                prior_off = float(np.median([p["off"] for p in prior]))
            else:
                prior_heading = prior_abs_heading = prior_curv = prior_off = 0.0
            self._adv_align_prior_curved = (
                prior_frames >= self._align_prior_min_frames
                and (prior_abs_heading >= self._align_prior_heading_deg
                     or prior_curv >= self._align_prior_curv))
            self._adv_align_prior = {
                "frames": prior_frames,
                "heading_deg": round(prior_heading, 1),
                "abs_heading_deg": round(prior_abs_heading, 1),
                "curv": round(prior_curv, 3),
                "off": round(prior_off, 3),
                "curved": bool(self._adv_align_prior_curved),
            }
            self._adv_came_straight = (not self._adv_align_prior_curved
                                       and pre_off < self._align_straight_off
                                       and pre_curv < self._align_straight_curv)
            self.get_logger().info(
                f'[ZEBRA] detected @ {dist:.0f}cm -> ADVANCE to first row '
                f'(came_straight={self._adv_came_straight}, off={pre_off:.2f})')
            self._event('approach_start', dist_cm=round(float(dist), 1),
                        came_straight=bool(self._adv_came_straight),
                        align_prior=dict(self._adv_align_prior))

        # ADVANCE -> READ once we have driven to the reading window (or timeout).
        if self.intersection_phase == 'approach':
            # Vote options over the whole advance (a single frame flaps; an exit
            # seen in >= zebra_opt_min_votes frames sticks).
            if zres is not None:
                for o in zres.options:
                    self._zebra_opt_votes[o] = self._zebra_opt_votes.get(o, 0) + 1
                self.intersection_options = [
                    o for o in ('left', 'straight', 'right')
                    if self._zebra_opt_votes.get(o, 0) >= self._zebra_opt_min_votes]
            # VISION-FIRST crossing of the ENTRY row. Reaching read_distance only
            # arms the jump detector; READ starts after the row is crossed. Once
            # crossed, creep a short distance until the continuous straight line is
            # visible, or stop at read_after_entry_max_cm so we do not chase the
            # exit row. read_advance_extra_cm remains as an optional extra cap.
            advanced = self._odom_m() - self._adv_odom0
            straight_seen = bool(zres is not None and "straight" in zres.options)
            if not self._adv_at_entry:
                near = (self._adv_prev_dist is not None
                        and self._adv_prev_dist
                        <= self._read_distance_cm + self._read_cross_jump_cm)
                crossed = (dist is not None and near
                           and dist - self._adv_prev_dist > self._read_cross_jump_cm)
                odom_crossed = advanced >= self._adv_entry_target_m
                if crossed or odom_crossed:
                    self._adv_at_entry = True
                    self._adv_extra_odom0 = self._odom_m()
                    mode = "z_dist jump" if crossed else "odom fallback"
                    dtxt = "?" if dist is None else f"{dist:.0f}"
                    self.get_logger().info(
                        f"[ZEBRA] crossed entry row by {mode} (dist={dtxt}cm)")
                arrived = False
            else:
                extra_cm = (self._odom_m() - self._adv_extra_odom0) * 100.0
                cap_cm = max(0.0, self._read_after_entry_max_cm)
                if self._read_advance_extra_cm > 0.0:
                    cap_cm = min(cap_cm, self._read_advance_extra_cm)
                if straight_seen:
                    self._zebra_opt_votes["straight"] = max(
                        self._zebra_opt_votes.get("straight", 0),
                        self._zebra_opt_min_votes)
                    self.intersection_options = [
                        o for o in ("left", "straight", "right")
                        if self._zebra_opt_votes.get(o, 0) >= self._zebra_opt_min_votes]
                arrived = straight_seen or extra_cm >= cap_cm
            if dist is not None:
                self._adv_prev_dist = dist
            timed_out = (
                self._approach_start_time is not None
                and (now - self._approach_start_time).nanoseconds * 1e-9
                > self._approach_timeout_s)
            if arrived:
                self.intersection_phase = 'wait'   # READ
                self.intersection_pending = True
                self.intersection_decision = None
                self.last_prompt_time = None
                self._approach_start_time = None
                self._align_start_time = None
                self.get_logger().info(
                    f'[ZEBRA] at reading window (advanced {advanced*100:.0f}cm) -> READ')
                self._event('wait_start',
                            dist_cm=None if dist is None else round(float(dist), 1),
                            voted_options=list(self.intersection_options),
                            option_votes=dict(self._zebra_opt_votes))
            elif timed_out:
                self.intersection_phase = None
                self._approach_start_time = None
                self.intersection_cooldown_until = now + Duration(seconds=2.0)
                self.get_logger().warn('[ZEBRA] ADVANCE timed out -> FOLLOW')
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
            # FIX 2: the za ON the cross is noisy. If we arrived STRAIGHT (small
            # pre-cross lane offset/curvature), the za skew is detector noise, not a
            # real heading error -- so do NOT rotate in place for free. Only square
            # up when the approach was genuinely off a curve.
            skip_straight = self._align_skip_when_straight and self._adv_came_straight
            need_align = (self._align_in_place and za is not None
                          and abs(za) > self._align_tol_deg
                          and align_elapsed < self._align_timeout_s
                          and self._adv_align_prior_curved
                          and not skip_straight)
            if skip_straight and za is not None and abs(za) > self._align_tol_deg:
                self.get_logger().info(
                    f'[ZEBRA] WAIT square-up SKIPPED (came straight, za={za:.0f} noise)',
                    throttle_duration_sec=1.0)
            if need_align and self._drive_enabled and self.intersection_decision is None:
                tw = Twist()
                limit_w = min(self.max_w, self._align_max_w)
                tw.angular.z = max(-limit_w, min(limit_w,
                                                    -self._k_align * math.radians(za)))
                self.cmd_pub.publish(tw)
                self.get_logger().info(
                    f"[ZEBRA] WAIT square-up in place: za={za:.0f} w={tw.angular.z:+.2f} prior={self._adv_align_prior}",
                    throttle_duration_sec=0.5)
                self._draw_status_hud(frame, tw)
                self._maybe_snapshot(now, frame)
                self._log_controller_row(now, tw)
                self._publish_telemetry(now, tw)
                self._publish_stream_frame(frame)
                if self.show_window:
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(1)
                return True

            # AUTO-decision from a traffic sign: if an arrow sign latched a turn,
            # take it here instead of waiting for the operator (fluid, no stop for
            # input). Manual 1/2/3 still works and overrides if pressed.
            if self.intersection_decision is None and self._pending_turn is not None:
                self.intersection_decision = self._pending_turn
                self.get_logger().warn(
                    f"[SIGN] auto-deciding {self._pending_turn} at cross")
                self._pending_turn = None

            should_prompt = (self.last_prompt_time is None
                             or (now - self.last_prompt_time).nanoseconds * 1e-9 > 1.0)
            if should_prompt:
                self._publish_zebra_prompt(zres)
                self.last_prompt_time = now

            if self.intersection_decision is None:
                self.cmd_pub.publish(Twist())
                self._draw_status_hud(frame, Twist())
                self._maybe_snapshot(now, frame)
                self._log_controller_row(now, Twist())
                self._publish_telemetry(now, Twist())
                self._publish_stream_frame(frame)
                if self.show_window:
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(1)
                return True

            self.commit_direction = self.intersection_decision
            is_straight = self.commit_direction == 'straight'
            dur = self._commit_duration_straight if is_straight else self._commit_duration
            min_s = self._commit_straight_min_s if is_straight else self._commit_min_s
            pre_turn_cm = 0.0 if is_straight else self._commit_turn_pre_advance_cm
            self.commit_until = now + Duration(seconds=dur)
            self._commit_min_until = now + Duration(seconds=min_s)
            self._dist_since_commit = 0.0
            self._commit_odom0 = self._odom_m()
            self._commit_start_time = now
            self._approach_start_time = None
            self.intersection_phase = None
            self._last_commit_tick_t = None
            self.intersection_pending = False
            self.intersection_options = []
            self.intersection_decision = None
            self._zebra_stable = 0
            self.zebra_result = None
            self.intersection_cooldown_until = now + Duration(seconds=1.5)
            self._event('commit_start', direction=self.commit_direction,
                        duration_s=float(dur), min_s=float(min_s),
                        pre_advance_cm=float(pre_turn_cm))
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
    def _parse_tl_position_map(self, value):
        colors = [part.strip().upper() for part in str(value).split(',') if part.strip()]
        valid = {"GREEN", "YELLOW", "RED"}
        if len(colors) != 3 or any(color not in valid for color in colors):
            self.get_logger().warn(
                f"Invalid traffic_light_position_map={value!r}; using GREEN,YELLOW,RED"
            )
            return ["GREEN", "YELLOW", "RED"]
        return colors

    def _parse_tl_position_anchors(self, value):
        try:
            anchors = [float(part.strip()) / 100.0 for part in str(value).split(',') if part.strip()]
        except ValueError:
            anchors = []
        if len(anchors) != 3 or any(anchor <= 0.0 or anchor >= 1.0 for anchor in anchors):
            self.get_logger().warn(
                f"Invalid traffic_light_position_anchors_pct={value!r}; using 27,50,73"
            )
            return [0.27, 0.50, 0.73]
        return anchors

    def _estimate_tl_plate_bbox(self, hsv, cand):
        if hsv is None or cand is None:
            return None
        h, w = hsv.shape[:2]
        cx, cy = cand["center"]
        radius = max(1.0, float(cand["radius"]))
        roi_y = int(h * max(1, min(100, self._tl_roi_y_pct)) / 100.0)
        y0 = max(0, int(cy - 8.0 * radius))
        y1 = min(roi_y, int(cy + 8.0 * radius))
        x0 = max(0, int(cx - 8.0 * radius))
        x1 = min(w, int(cx + 8.0 * radius))
        if y1 <= y0 or x1 <= x0:
            return None

        roi = hsv[y0:y1, x0:x1]
        gray = cv2.inRange(
            roi,
            np.array([0, 0, int(self._tl_plate_min_val)], dtype=np.uint8),
            np.array([180, int(self._tl_plate_max_sat), int(self._tl_plate_max_val)], dtype=np.uint8),
        )
        kernel = np.ones((5, 5), np.uint8)
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel, iterations=2)
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0.0
        local_cx = cx - x0
        local_cy = cy - y0
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self._tl_plate_min_area:
                continue
            px, py, pw, ph = cv2.boundingRect(c)
            if not (px <= local_cx <= px + pw and py <= local_cy <= py + ph):
                continue
            if pw < 2.4 * radius or ph < 4.0 * radius:
                continue
            if area > best_area:
                best_area = area
                best = (x0 + px, y0 + py, pw, ph)
        return best

    def _classify_tl_candidate_by_position(self, cand, hsv):
        if cand is None:
            return None
        cand = dict(cand)
        cand["hsv_color"] = cand.get("color")
        if not self._tl_position_classify:
            return cand

        plate = self._estimate_tl_plate_bbox(hsv, cand)
        if plate is None:
            return None
        _px, _py, _pw, _ph = plate
        _cx, cy = cand["center"]
        h = hsv.shape[0] if hsv is not None else 1
        roi_y = int(h * max(1, min(100, self._tl_roi_y_pct)) / 100.0)
        rel_y = float(cy) / float(max(1, roi_y))
        slot_idx = min(range(3), key=lambda idx: abs(rel_y - self._tl_position_anchors[idx]))
        slot_err = abs(rel_y - self._tl_position_anchors[slot_idx])
        if slot_err > self._tl_position_max_slot_error:
            return None

        position_color = self._tl_position_map[slot_idx]
        hsv_color = cand.get("hsv_color")
        if hsv_color in {"RED", "YELLOW", "GREEN"} and hsv_color != position_color:
            return None

        cand["color"] = position_color
        cand["slot"] = float(rel_y)
        cand["slot_error"] = float(slot_err)
        cand["plate_bbox"] = tuple(int(v) for v in plate)
        return cand

    def _annotate_tl_actionability(self, cand):
        if cand is None:
            return None
        cand = dict(cand)
        radius = max(1e-3, float(cand.get("radius", 0.0)))
        distance_cm = float(self._tl_distance_k_cm_px) / radius
        radius_ok = self._tl_action_min_radius_px <= radius <= self._tl_action_max_radius_px
        distance_ok = self._tl_action_min_distance_cm <= distance_cm <= self._tl_action_max_distance_cm
        cand["distance_cm"] = distance_cm
        cand["actionable"] = bool(radius_ok and distance_ok)
        cand["action_reason"] = "ok" if cand["actionable"] else (
            f"dist={distance_cm:.1f}cm r={radius:.1f}px"
        )
        return cand

    def detect_color(self, mask, color_name="UNKNOWN", hsv=None):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        h, _w = mask.shape[:2]
        roi_y = int(h * max(1, min(100, self._tl_roi_y_pct)) / 100.0)
        if roi_y < h:
            mask[roi_y:, :] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            area = float(cv2.contourArea(c))
            if not (self._tl_min_area <= area <= self._tl_max_area):
                continue
            perim = float(cv2.arcLength(c, True))
            if perim <= 1e-3:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = bw / float(max(1, bh))
            if not (1.0 - self._tl_aspect_tol <= aspect <= 1.0 + self._tl_aspect_tol):
                continue
            circularity = 4.0 * math.pi * area / (perim * perim)
            if circularity < self._tl_min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            circle_area = math.pi * radius * radius
            fill = area / circle_area if circle_area > 1e-3 else 0.0
            if not (self._tl_min_fill <= fill <= self._tl_max_fill):
                continue
            # The light must sit ON the gray screen/plate: the ring just outside the
            # disc must be grayish (low saturation, mid value). This rejects loose
            # colored objects (red cable, chair) that are NOT inside the panel.
            # ENHANCED: Also check that the object is in the central region (not at edges)
            if self._tl_require_plate and hsv is not None:
                # Reject objects too far to the left or right (e.g., HDMI cable on side)
                # Use 10%-90% to allow slightly off-center traffic lights
                if cx < _w * 0.10 or cx > _w * 0.90:
                    continue   # too far to the side -> reject
                
                # Reject objects in bottom half (traffic lights are always in upper half)
                if cy > h * 0.5:
                    continue   # too low -> reject
                
                # Check ring around the light (gray plate)
                ring = np.zeros((h, _w), np.uint8)
                cv2.circle(ring, (int(cx), int(cy)), int(2.2 * radius), 255, -1)
                cv2.circle(ring, (int(cx), int(cy)), int(1.4 * radius), 0, -1)
                _, s_ring, v_ring, _ = cv2.mean(hsv, mask=ring)
                # Use configurable plate saturation threshold (allows tuning)
                if not (s_ring <= self._tl_plate_max_sat
                        and self._tl_plate_min_val <= v_ring <= self._tl_plate_max_val):
                    continue   # not on the gray plate -> reject
                
                # Additional check: verify there's a large gray rectangular area around it
                # (real traffic lights are mounted on a big gray panel, reflections are not)
                plate_h = int(8.0 * radius)
                plate_w = int(3.0 * radius)
                py0 = max(0, int(cy - plate_h / 2))
                py1 = min(h, int(cy + plate_h / 2))
                px0 = max(0, int(cx - plate_w / 2))
                px1 = min(_w, int(cx + plate_w / 2))
                if py1 > py0 and px1 > px0:
                    plate_roi = hsv[py0:py1, px0:px1]
                    gray_in_plate = cv2.inRange(
                        plate_roi,
                        np.array([0, 0, int(self._tl_plate_min_val)], dtype=np.uint8),
                        np.array([180, int(self._tl_plate_max_sat), int(self._tl_plate_max_val)], dtype=np.uint8),
                    )
                    gray_area = cv2.countNonZero(gray_in_plate)
                    plate_area = (py1 - py0) * (px1 - px0)
                    gray_ratio = gray_area / float(max(1, plate_area))
                    # At least 30% of the surrounding area must be gray (traffic light panel)
                    if gray_ratio < 0.30:
                        continue   # not enough gray plate around -> likely a reflection
            cand = {
                "color": color_name, "area": area, "center": (float(cx), float(cy)),
                "radius": float(radius), "bbox": (int(x), int(y), int(bw), int(bh)),
                "circularity": float(circularity), "fill": float(fill),
            }
            if best is None or cand["area"] > best["area"]:
                best = cand

        return (0.0 if best is None else best["area"]), best, mask

    # =============================================================
    # ANCHOR ASSIGNMENT HELPER
    # Matches a pool of candidates to three named anchors using a
    # greedy nearest-neighbour approach. Each candidate can only
    # be consumed once, and candidates that are too far from any
    # anchor are ignored. If an anchor has no matching candidate
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
    def _safe_control_loop(self):
        """Crash-proof wrapper around control_loop. An unhandled exception in the
        timer callback would otherwise (a) stop the timer from firing (camera
        appears to freeze: cap.read() is no longer called) and (b) leave the LAST
        cmd_vel latched on the robot -> it keeps driving forward into a wall.
        Here we catch everything, publish a STOP, and let the NEXT tick recover."""
        try:
            self.control_loop()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            tb = traceback.format_exc()
            self._write_loop_exception(exc, tb)
            self.get_logger().error(
                f"[CONTROL] loop exception -> STOP + recover: {exc}\n"
                f"{tb}")
            try:
                self.cmd_pub.publish(Twist())          # fail safe: stop the robot
            except Exception:                          # noqa: BLE001
                pass

    def control_loop(self):
        self._loop_stage = 'camera_read'
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received from camera!")
            return

        self._loop_stage = 'preprocess'
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        frame = self._apply_illumination_gain(frame)

        h, w = frame.shape[:2]
        frame_center_x = w / 2.0
        now = self.get_clock().now()

        # YOLO traffic signs: detect + latch actions (no-op if disabled).
        # During COMMIT the decision is locked; skip inference so a slow/bad sign
        # frame cannot stall the camera/control loop mid-turn.
        self._loop_stage = 'signs'
        if self.commit_direction is None:
            self._run_signs(frame, now)

        self._loop_stage = 'traffic_light'

        # ---------------------------------------------------------
        # 1. TRAFFIC LIGHT PERCEPTION
        # ---------------------------------------------------------
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # Wider RED/YELLOW so the painted light disc is caught (the old S>=150 was
        # too strict and missed the red/amber disc). The circular-shape filter +
        # upper ROI keep floor/tan/clutter out. Green kept (it already worked).
        # RED: Increased min saturation to 120 to reject low-saturation red objects (HDMI cable)
        red_mask = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255])) + \
                   cv2.inRange(hsv, np.array([168, 120, 80]), np.array([180, 255, 255]))
        # YELLOW: Widened range H[15..42] and lowered saturation to 60 for better far detection
        yellow_mask = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([42, 255, 255]))
        # GREEN: Lowered saturation from 120 to 80 and value from 120 to 80 for better far detection
        green_mask  = cv2.inRange(hsv, np.array([40, 80, 80]), np.array([90, 255, 255]))

        # Don't let the RED of a YOLO sign (e.g. the STOP octagon) be read as a red
        # traffic LIGHT: blank the detected sign's box (+margin) from the color masks.
        sr = self._sign_result
        if sr is not None and sr.box is not None:
            bx1, by1, bx2, by2 = (int(v) for v in sr.box)
            m = 12
            y0b, y1b = max(0, by1 - m), min(h, by2 + m)
            x0b, x1b = max(0, bx1 - m), min(w, bx2 + m)
            red_mask[y0b:y1b, x0b:x1b] = 0
            yellow_mask[y0b:y1b, x0b:x1b] = 0
            green_mask[y0b:y1b, x0b:x1b] = 0

        red_area, red_cand, _ = self.detect_color(red_mask, "RED", hsv)
        yellow_area, yellow_cand, _ = self.detect_color(yellow_mask, "YELLOW", hsv)
        green_area, green_cand, _ = self.detect_color(green_mask, "GREEN", hsv)

        detected_color = "UNKNOWN"
        action_color = "UNKNOWN"
        raw_candidates = [red_cand, yellow_cand, green_cand]
        candidates = [
            annotated for annotated in (
                self._annotate_tl_actionability(classified)
                for classified in (
                    self._classify_tl_candidate_by_position(cand, hsv)
                    for cand in raw_candidates
                )
            )
            if annotated is not None
        ]
        best_cand = max(candidates, key=lambda item: item["area"], default=None)
        self._traffic_light_candidate = best_cand
        if best_cand is not None:
            detected_color = best_cand["color"]
            if best_cand.get("actionable", False):
                action_color = detected_color
            self._draw_traffic_light_overlay(frame, best_cand)

        self.get_logger().info(
            f"[VISION] Areas -> R:{red_area:.0f} Y:{yellow_area:.0f} G:{green_area:.0f} "
            f"| Raw Detect: {detected_color} | Action: {action_color} | Active State: {self.current_state} "
            f"| TL: {self._traffic_light_candidate}",
            throttle_duration_sec=1.0,
        )

        if action_color == "RED":
            self.red_count    += 1; self.yellow_count  = 0; self.green_count = 0
            self._tl_unknown_count = 0
        elif action_color == "YELLOW":
            self.yellow_count += 1; self.red_count     = 0; self.green_count = 0
            self._tl_unknown_count = 0
        elif action_color == "GREEN":
            self.green_count  += 1; self.red_count     = 0; self.yellow_count = 0
            self._tl_unknown_count = 0
        else:
            self.red_count = 0; self.yellow_count = 0; self.green_count = 0
            self._tl_unknown_count += 1

        if   self.red_count    >= self.threshold_frames: self.current_state = "RED"
        elif self.yellow_count >= self.threshold_frames: self.current_state = "YELLOW"
        elif self.green_count  >= self.threshold_frames: self.current_state = "GREEN"
        # OPTIONAL light: no light in view for a while -> drive (GREEN). So a RED
        # only holds while the light is actually visible; it never strands the robot.
        elif (self._traffic_light_optional
              and self._tl_unknown_count >= self.threshold_frames):
            self.current_state = "GREEN"

        if self.current_state != self.last_state:
            self.get_logger().info(
                f"[TRAFFIC LIGHT] >>> Switched from {self.last_state} to {self.current_state} <<<"
            )
            self.last_state = self.current_state

        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        self._loop_stage = 'intersection'

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
            # ROBUSTNESS: Same protections as zebra BEV path
            stable_ok_legacy = self.intersection_stable >= 3
            
            if (result is not None and result.entry_seen
                    and self.intersection_phase is None):
                if not stable_ok_legacy:
                    self.get_logger().info(
                        f"[INTERSECTION] Approach blocked: insufficient stability ({self.intersection_stable}/3 frames)",
                        throttle_duration_sec=2.0)
                elif stable_ok_legacy:
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
                self._commit_odom0 = self._odom_m()  # baseline for turn pre-advance
                self._commit_start_time = now
                self._approach_start_time = None
                self.intersection_phase = None
                self._last_commit_tick_t = None
                self.intersection_pending = False
                self.intersection_options = []
                self.intersection_decision = None
                self.intersection_stable = 0
                self.intersection_result = None
                self.intersection_cooldown_until = now + Duration(seconds=1.5)

        self._loop_stage = 'lane_control'

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
            now_good = lane_result.detected and lane_result.confidence >= 0.5
            base_x = lane_result.base_x
            near = self._near_intersection

            # Reject a base that jumped too far from the last good base.
            # Near a cross this prevents side-branch lock. In a tight curve it
            # prevents grabbing puzzle seams / outer edges that still score as a
            # medium-confidence lane fit.
            base_jumped = False
            curve_fit_outlier = False
            recent_base_dt = None
            if self._lane_good_base_time is not None:
                recent_base_dt = (now - self._lane_good_base_time).nanoseconds * 1e-9
            recent_base = (
                self._lane_good_base is not None
                and recent_base_dt is not None
                and recent_base_dt <= max(self._lane_base_hold_s, self._lane_hold_curve_s)
            )
            if (now_good and base_x is not None
                    and self._lane_good_base is not None
                    and recent_base):
                in_curve = (
                    abs(float(lane_result.curvature_norm)) >= self._lane_hold_curve_min_curv
                    or self._lane_hold_curvature >= self._lane_hold_curve_min_curv
                )
                weak_curve_fit = lane_result.confidence < self._lane_curve_guard_conf
                jump_reason = None
                max_jump_pct = None
                if near and self._lane_base_max_jump_pct > 0 and recent_base_dt <= self._lane_base_hold_s:
                    max_jump_pct = self._lane_base_max_jump_pct
                    jump_reason = "side-branch guard"
                else:
                    if (self._lane_curve_max_jump_pct > 0
                            and (in_curve or weak_curve_fit)):
                        max_jump_pct = self._lane_curve_max_jump_pct
                        jump_reason = "curve continuity guard"
                    if (self._lane_curve_guard_max_offset > 0.0
                            and in_curve
                            and weak_curve_fit
                            and abs(float(lane_result.offset_norm)) > self._lane_curve_guard_max_offset):
                        curve_fit_outlier = True
                        self.get_logger().warn(
                            f"[LANE] curve fit outlier off={lane_result.offset_norm:+.2f} "
                            f"conf={lane_result.confidence:.2f} rejected",
                            throttle_duration_sec=0.5)
                if max_jump_pct is not None:
                    max_jump = self.lane_params.warp_w * max_jump_pct / 100.0
                    if abs(base_x - self._lane_good_base) > max_jump:
                        base_jumped = True
                        self.get_logger().warn(
                            f"[LANE] base jump {self._lane_good_base:.0f}->{base_x:.0f} "
                            f"(> {max_jump:.0f}px) rejected -- {jump_reason}",
                            throttle_duration_sec=0.5)
            accept = now_good and not base_jumped and not curve_fit_outlier

            # Thread the base x to the next frame for continuity (stay on the same
            # line through a curve). Near a cross keep it STICKY through brief
            # dropouts (up to lane_base_hold_s) so re-acquisition stays in the
            # narrow continuity corridor instead of grabbing the side branch from
            # the wide center band. Away from a cross, drop it immediately as
            # before so a genuine line loss re-acquires from center.
            if accept:
                self._lane_prev_base = base_x
                self._lane_good_base = base_x
                self._lane_good_base_time = now
            elif ((near or base_jumped or curve_fit_outlier) and self._lane_good_base_time is not None
                  and recent_base_dt is not None
                  and recent_base_dt <= max(self._lane_base_hold_s, self._lane_hold_curve_s)):
                pass  # sticky: keep _lane_prev_base anchored on the last good line
            else:
                self._lane_prev_base = None
            draw_lane_overlay(frame, self.lane_params, lane_result)
            if accept and lane_result.lane_center_x_orig is not None:
                lane_ok = True
                steering_center_x = lane_result.lane_center_x_orig
                steering_far_x = lane_result.lane_center_far_x_orig
                lane_curvature = abs(lane_result.curvature_norm)
                self.time_line_lost = None
                # Capture the last CONFIDENT heading for the near-cross hysteresis.
                if lane_result.confidence >= self._lane_hold_conf:
                    signed_curvature = float(lane_result.curvature_norm)
                    recent_curve_hold = (
                        self._lane_hold_time is not None
                        and self._lane_hold_curvature >= self._lane_hold_curve_min_curv
                        and (now - self._lane_hold_time).nanoseconds * 1e-9 <= self._lane_hold_curve_s
                    )
                    weak_fit = lane_result.confidence <= self._lane_curve_hold_assist_conf
                    sign_flip = (
                        recent_curve_hold
                        and weak_fit
                        and abs(signed_curvature) >= self._lane_hold_curve_min_curv
                        and self._lane_hold_signed_curvature * signed_curvature < 0.0
                    )
                    flattened = (
                        recent_curve_hold
                        and weak_fit
                        and lane_curvature < self._lane_hold_curve_min_curv
                    )
                    # Do not let a weak exit-frame erase a strong curve target.
                    # In this lab the polynomial briefly flips curvature sign at
                    # the apex even though the robot still needs the same turn.
                    if ((sign_flip or flattened) and self._lane_hold_far_x is not None):
                        steering_far_x = self._lane_hold_far_x
                        lane_curvature = max(lane_curvature, self._lane_hold_curvature)
                        self.get_logger().warn(
                            f"[LANE] assist curve lookahead from hold "
                            f"(conf={lane_result.confidence:.2f}, "
                            f"curv={signed_curvature:+.2f}->{self._lane_hold_signed_curvature:+.2f})",
                            throttle_duration_sec=0.5)
                    elif lane_curvature >= self._lane_hold_curve_min_curv or not recent_curve_hold:
                        self._lane_hold_center_x = steering_center_x
                        self._lane_hold_far_x = steering_far_x
                        self._lane_hold_curvature = lane_curvature
                        self._lane_hold_time = now
                        self._lane_hold_signed_curvature = signed_curvature
                self.get_logger().info(
                    f"[LANE] off={lane_result.offset_norm:+.2f} "
                    f"curv={lane_result.curvature_norm:+.2f} conf={lane_result.confidence:.2f}",
                    throttle_duration_sec=1.0,
                )
                if self.commit_direction is None and self.intersection_phase in (None, "approach"):
                    self._align_prior_samples.append({
                        "off": abs(float(lane_result.offset_norm)),
                        "curv": abs(float(lane_result.curvature_norm)),
                        "heading_deg": math.degrees(math.atan(float(lane_result.heading))),
                    })
                    self._align_prior_samples = self._align_prior_samples[-20:]

        # Heading hysteresis: in the slow-zone (zebra in view) the BEV fit can
        # briefly lose confidence as the cross enters the ROI. Rather than drop to
        # the legacy ROI detector (which would lock onto the zebra/side lines and
        # jerk the robot), HOLD the last confident steering target and drive
        # straight on it for up to lane_hold_s. The anti-zebra filter keeps the
        # continuous line, so this just bridges the brief dropout. No-op away from
        # a cross (near_intersection is False there).
        if (not lane_ok and self._lane_hold_near_cross and self._near_intersection
                and self._lane_hold_center_x is not None
                and self._lane_hold_time is not None
                and (now - self._lane_hold_time).nanoseconds * 1e-9 <= self._lane_hold_s):
            lane_ok = True
            steering_center_x = self._lane_hold_center_x
            steering_far_x = None        # no anticipation -> go straight
            lane_curvature = 0.0
            self.time_line_lost = None
            self.get_logger().warn(
                f"[LANE] hold heading near cross (cx={steering_center_x:.0f})",
                throttle_duration_sec=0.5,
            )
        elif (not lane_ok and self._use_birdseye and self._lane_hold_center_x is not None
              and self._lane_hold_time is not None
              and self._lane_hold_curvature >= self._lane_hold_curve_min_curv
              and (now - self._lane_hold_time).nanoseconds * 1e-9 <= self._lane_hold_curve_s):
            lane_ok = True
            steering_center_x = self._lane_hold_center_x
            steering_far_x = self._lane_hold_far_x
            lane_curvature = self._lane_hold_curvature
            self.time_line_lost = None
            self.get_logger().warn(
                f"[LANE] hold BEV curve target (cx={steering_center_x:.0f}, "
                f"curv={lane_curvature:.2f})",
                throttle_duration_sec=0.5,
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

        # ---- Robust curve ARC handler (see __init__) --------------------------
        # When a tight curve is CONFIRMED by curvature magnitude, stop using the PD
        # and drive a fixed forward+left arc (matching the hand-driven demo) until
        # the line straightens and re-centers. Hardcoded LEFT: every curve on this
        # track is a left bend, so we never depend on the (flaky) curvature sign.
        curve_arc = False
        if self._curve_arc_enabled and not self._blind_turn_enabled:
            lr_arc = self._last_lane_result
            arc_detected = lr_arc is not None and lr_arc.detected
            curv_mag = abs(float(lr_arc.curvature_norm)) if arc_detected else 0.0
            off_mag = abs(float(lr_arc.offset_norm)) if arc_detected else 1.0
            in_follow = (self.intersection_phase is None and self.commit_direction is None
                         and not self._near_intersection and self.time_line_lost is None)
            if self._curve_arc_active:
                if self._curve_arc_phase == 'post':
                    # advance straight, then a short second left turn, then exit
                    post_elapsed = (now - self._curve_arc_post_start).nanoseconds * 1e-9
                    if post_elapsed >= (self._curve_arc_post_s + self._curve_arc_recenter_s) \
                            or not in_follow:
                        self._curve_arc_active = False
                        self.get_logger().warn("[CURVE] arc+recenter done -> FOLLOW",
                                               throttle_duration_sec=0.5)
                    else:
                        curve_arc = True
                else:
                    elapsed = (now - self._curve_arc_start).nanoseconds * 1e-9
                    straightened = (arc_detected and curv_mag < self._curve_arc_exit
                                    and off_mag < 0.25)
                    if elapsed >= self._curve_arc_max_s or not in_follow:
                        self._curve_arc_active = False       # hard exit (cap / left FOLLOW)
                        self._curve_arc_exit_count = 0
                        self.get_logger().warn(
                            f"[CURVE] arc done ({elapsed:.1f}s) -> FOLLOW",
                            throttle_duration_sec=0.5)
                    elif elapsed >= self._curve_arc_min_s and straightened:
                        self._curve_arc_exit_count += 1
                        curve_arc = True
                        if self._curve_arc_exit_count >= self._curve_arc_exit_frames:
                            # main turn finished -> run the advance + re-center sequence
                            self._curve_arc_phase = 'post'
                            self._curve_arc_post_start = now
                            self._curve_arc_exit_count = 0
                            self.get_logger().warn(
                                f"[CURVE] turn done ({elapsed:.1f}s) -> advance+recenter",
                                throttle_duration_sec=0.5)
                    else:
                        self._curve_arc_exit_count = 0
                        curve_arc = True
            elif in_follow and arc_detected and curv_mag >= self._curve_arc_enter:
                self._curve_arc_enter_count += 1
                if self._curve_arc_enter_count >= 2:   # 2-frame debounce
                    self._curve_arc_active = True
                    self._curve_arc_start = now
                    self._curve_arc_phase = 'turn'
                    self._curve_arc_post_start = None
                    self._curve_arc_enter_count = 0
                    self._curve_arc_exit_count = 0
                    curve_arc = True
                    self.get_logger().warn(
                        f"[CURVE] arc START (curv={curv_mag:.2f}) -> forward+left",
                        throttle_duration_sec=0.5)
            else:
                self._curve_arc_enter_count = 0

        if curve_arc:
            base_linear_x = self._curve_arc_v
            if self._curve_arc_phase == 'post':
                # advance straight first, then the short second LEFT turn to re-center
                post_elapsed = (now - self._curve_arc_post_start).nanoseconds * 1e-9
                if post_elapsed < self._curve_arc_post_s:
                    target_angular_z = 0.0                 # advance a bit more
                else:
                    target_angular_z = self._curve_arc_w   # second turn to re-acquire center
            else:
                arc_elapsed = (now - self._curve_arc_start).nanoseconds * 1e-9
                if arc_elapsed < self._curve_arc_pre_s:
                    target_angular_z = 0.0                 # advance STRAIGHT into the curve first
                else:
                    target_angular_z = self._curve_arc_w   # main LEFT turn (curves here are left)
            steering_center_x = None               # bypass the PD below
            self.last_error = 0.0
            self.last_derivative = 0.0
            self.last_time = now

        # ---- BLIND TURN until re-acquire (robust tight-curve handler) ----------
        # Tight 45-deg curves sweep the line out of the warp, so we LOSE it mid-turn
        # -- no controller follows what it can't see. So: follow closed-loop (PD)
        # while the line is VISIBLE, remembering which way it is going; the moment we
        # LOSE it, turn that way at a fixed rate (slow forward) until it comes back
        # near center, then resume PD. Event-driven (no timers), auto left/right, and
        # it only turns AFTER losing the line, so it never cuts the curve early.
        if self._blind_turn_enabled:
            lr_b = self._last_lane_result
            bev_ok = (lr_b is not None and lr_b.detected
                      and lr_b.confidence >= self._blind_conf
                      and lr_b.lane_center_x_orig is not None)
            blind_follow = (self.intersection_phase is None and self.commit_direction is None
                            and not self._near_intersection)
            if bev_ok and blind_follow:
                # remember the turn side from where the line is (err sign == w sign)
                err_b = frame_center_x - float(lr_b.lane_center_x_orig)
                side = 1.0 if err_b > 0 else (-1.0 if err_b < 0 else 0.0)
                self._curve_side = 0.6 * self._curve_side + 0.4 * side
                self._blind_lost_frames = 0
            elif blind_follow:
                self._blind_lost_frames += 1

            if self._blind_active:
                elapsed_b = (now - self._blind_start).nanoseconds * 1e-9
                off_b = abs(float(lr_b.offset_norm)) if bev_ok else 1.0
                reacquired = bev_ok and off_b <= self._blind_reacquire_off
                if reacquired or elapsed_b >= self._blind_max_s or not blind_follow:
                    self._blind_active = False
                    self.get_logger().warn(
                        f"[BLIND] done (reacquired={reacquired}, {elapsed_b:.1f}s) -> FOLLOW",
                        throttle_duration_sec=0.5)
                else:
                    base_linear_x = self._blind_turn_v
                    target_angular_z = self._blind_dir * self._blind_turn_w
                    steering_center_x = None          # bypass PD + suppress recover
                    self.time_line_lost = None
                    self.last_error = 0.0
                    self.last_derivative = 0.0
                    self.last_time = now
            elif (blind_follow and self._blind_lost_frames >= self._blind_enter_frames
                  and abs(self._curve_side) >= 0.3):
                # line lost while following -> commit to a blind turn toward its side
                self._blind_active = True
                self._blind_start = now
                self._blind_dir = 1.0 if self._curve_side > 0 else -1.0
                base_linear_x = self._blind_turn_v
                target_angular_z = self._blind_dir * self._blind_turn_w
                steering_center_x = None
                self.time_line_lost = None
                self.last_error = 0.0
                self.last_derivative = 0.0
                self.last_time = now
                self.get_logger().warn(
                    f"[BLIND] line lost -> turning {'LEFT' if self._blind_dir > 0 else 'RIGHT'} "
                    f"(side={self._curve_side:+.2f})", throttle_duration_sec=0.5)

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

                # Heading curve term (see __init__): the reliable curve signal from
                # the teleop demo. Steer proportional to the line tilt beyond a
                # straight-residual deadband, so tight curves get the ~0.30 the human
                # used instead of the 0.05-0.19 kp*offset produced. Only on a fresh,
                # confident BEV fit, away from intersections (those steer separately).
                lr = self._last_lane_result
                if (self._curve_heading_gain > 0.0
                        and lr is not None and lr.detected
                        and lr.confidence >= 0.5
                        and not self._near_intersection
                        and self.commit_direction is None
                        and self.intersection_phase is None):
                    heading = float(lr.heading)
                    if abs(heading) >= self._curve_heading_deadband:
                        w_out += self._curve_heading_gain * heading

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

        # Slow down proportionally to curvature, with short memory. Tight curves
        # can produce one flat/fragmented fit while the robot is still physically
        # in the turn; releasing speed immediately is what caused the jump from
        # ~0.04 to ~0.09 m/s in the curve sessions.
        effective_curvature = lane_curvature
        if lane_curvature >= self._lane_hold_curve_min_curv:
            self._curve_hold_value = max(self._curve_hold_value, lane_curvature)
            self._curve_hold_until = now + Duration(seconds=self._curve_memory_s)
        elif self._curve_hold_until is not None and now < self._curve_hold_until:
            effective_curvature = max(effective_curvature, self._curve_hold_value)
        else:
            self._curve_hold_until = None
            self._curve_hold_value = 0.0
        if effective_curvature > 0.0 and not curve_arc:
            base_linear_x *= max(self._curve_min_scale,
                                 1.0 - self._curve_slow_gain * effective_curvature)

        # Slow-zone: cap speed while a zebra is in view (FOLLOW only), so the robot
        # closes on the cross slowly enough to center instead of overshooting.
        if self._near_intersection and self.intersection_phase is None:
            base_linear_x = min(base_linear_x, self._intersection_slow_speed)

        if self.commit_direction is not None:
            lr = self._last_lane_result
            # ROBUST re-acquisition for turns: require high confidence AND reasonable offset
            # to avoid grabbing edge lines during the turn
            is_turn = self.commit_direction in ('left', 'right')
            if is_turn:
                # For turns: strict criteria to avoid edge detection
                reacquired = (lr is not None and lr.detected
                              and lr.confidence >= 0.7
                              and abs(lr.offset_norm) < 0.6)
            else:
                # For straight: more lenient (original criteria)
                reacquired = (lr is not None and lr.detected
                              and lr.confidence >= 0.5)
            
            past_min = (self._commit_min_until is None
                        or now >= self._commit_min_until)
            past_max = (self.commit_until is not None
                        and now >= self.commit_until)
            commit_elapsed = (0.0 if self._commit_start_time is None else
                              (now - self._commit_start_time).nanoseconds * 1e-9)
            straight_early_handoff = False
            if (not is_turn and reacquired and lr is not None and lr.detected
                    and commit_elapsed >= 1.5):
                # On this lab the straight branch can immediately become a curve.
                # Holding w=0 for the full straight min time hands FOLLOW a large
                # offset. If the lane is already confidently reacquired and starts
                # drifting/curving hard, return to FOLLOW early so it can steer.
                straight_early_handoff = (
                    abs(lr.offset_norm) >= 0.45
                    or abs(lr.curvature_norm) >= 0.55)
            # End the maneuver when the lane is RE-ACQUIRED (after a min time to
            # clear the cross), or at the safety cap. A cross has no line to
            # follow, so we drive the turn/cross open-loop ONLY until the line of
            # the chosen branch reappears -- then hand straight back to FOLLOW.
            if (past_max
                    or (self._commit_closed_loop and reacquired
                        and (past_min or straight_early_handoff))):
                self.get_logger().info(
                    f"[INTERSECTION] commit {self.commit_direction} done -> FOLLOW "
                    f"(reacquired={reacquired}, timeout={past_max})")
                self._event('commit_end', direction=self.commit_direction,
                            reacquired=bool(reacquired), timeout=bool(past_max),
                            early_handoff=bool(straight_early_handoff),
                            elapsed_s=round(float(commit_elapsed), 3))
                self.commit_direction = None
                self.commit_until = None
                self._commit_min_until = None
                self._commit_start_time = None
            else:
                base_linear_x = self._commit_speed
                pre_cm = (self._commit_turn_pre_advance_cm
                          if self.commit_direction in ("left", "right") else 0.0)
                pre_elapsed = commit_elapsed
                pre_odom_cm = (self._odom_m() - self._commit_odom0) * 100.0
                pre_time_s = pre_cm / max(1e-3, self._commit_speed * 100.0)
                pre_done = (pre_odom_cm >= pre_cm or pre_elapsed >= pre_time_s)
                self._commit_debug_tick(
                    now, 'command_calc',
                    pre_cm=round(float(pre_cm), 2),
                    pre_odom_cm=round(float(pre_odom_cm), 2),
                    pre_elapsed_s=round(float(pre_elapsed), 3),
                    pre_time_s=round(float(pre_time_s), 3),
                    pre_done=bool(pre_done),
                    reacquired=bool(reacquired),
                    past_min=bool(past_min),
                    past_max=bool(past_max),
                )
                if not pre_done:
                    target_angular_z = 0.0
                elif self.commit_direction == 'left':
                    target_angular_z = self._commit_turn_w
                elif self.commit_direction == 'right':
                    target_angular_z = -self._commit_turn_w
                else:
                    target_angular_z = 0.0
                # Detailed logging for turn commits
                if is_turn:
                    lane_info = "no_lane"
                    if lr is not None and lr.detected:
                        lane_info = f"conf={lr.confidence:.2f},off={lr.offset_norm:.2f}"
                    self.get_logger().info(
                        f"[INTERSECTION] Committing {self.commit_direction}: "
                        f"V={base_linear_x:.2f}, W={target_angular_z:.2f}, "
                        f"pre={min(pre_cm, pre_odom_cm):.0f}/{pre_cm:.0f}cm t={pre_elapsed:.1f}/{pre_time_s:.1f}s, "
                        f"lane={lane_info}, reacq={reacquired}",
                        throttle_duration_sec=0.5)
                else:
                    self.get_logger().info(
                        f"[INTERSECTION] Committing {self.commit_direction}: "
                        f"V={base_linear_x:.2f}, W={target_angular_z:.2f}, "
                        f"pre={min(pre_cm, pre_odom_cm):.0f}/{pre_cm:.0f}cm t={pre_elapsed:.1f}/{pre_time_s:.1f}s",
                        throttle_duration_sec=0.5)

        # During APPROACH: drive a fixed creep AND actively align heading so the
        # robot straightens onto the zebra (works whether it arrived from a curve
        # or a straight). w_align rotates the fitted entry line toward horizontal;
        # the legacy centering above already handles lateral offset.
        if self.intersection_phase == 'approach':
            lane_keep_w = target_angular_z
            if self._use_zebra_bev:
                # ADVANCE: do NOT use the lane follower here (it flakes over the
                # cross and grabs the side dashes -> veers left). Hold centre with
                # the zebra ROW CENTER (stable) and go STRAIGHT if the row is lost.
                # Before the entry row is crossed, keep a capped slice of lane
                # steering so curved approaches do not drive straight off-line.
                # End ADVANCE by ODOMETRY at the reading window (not the camera
                # distance, which leaves view up close).
                base_linear_x = self._approach_speed
                zr = self.zebra_result
                rc = zr.row_center_cm if (zr is not None and zr.seen) else None
                if rc is not None:
                    target_angular_z = max(-self.max_w, min(
                        self.max_w, -self._advance_center_gain * rc))
                else:
                    target_angular_z = 0.0   # row lost: keep heading straight
                if (not self._adv_at_entry and lane_ok
                        and self._advance_lane_keep_gain > 0.0):
                    keep = max(-self._advance_lane_keep_max_w,
                               min(self._advance_lane_keep_max_w,
                                   self._advance_lane_keep_gain * lane_keep_w))
                    target_angular_z = max(-self.max_w, min(
                        self.max_w, target_angular_z + keep))
                advanced = self._odom_m() - self._adv_odom0
                self.get_logger().info(
                    f"[ZEBRA] ADVANCE: V={base_linear_x:.3f} W={target_angular_z:.2f} "
                    f"rc={'?' if rc is None else f'{rc:.0f}'}cm "
                    f"adv={advanced*100:.0f}/{self._adv_target_m*100:.0f}cm",
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
        # Once COMMIT starts, finish it. Traffic-light or sign holds seen during
        # the turn must not overwrite W and strand the robot inside the cross.
        effective_state = ("GREEN" if (self._ignore_traffic_light or self.commit_direction is not None)
                           else self.current_state)

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

        # --- TRAFFIC SIGN overrides (after the light, before the drive switch) ---
        # workers: cap/scale speed while the slow window is active.
        if self.commit_direction is None and self._workers_until is not None:
            if now < self._workers_until:
                cmd.linear.x *= self._workers_speed_factor
                if cmd.linear.x > 0.0:
                    cmd.linear.x = max(cmd.linear.x, self._workers_min_speed)
                self.get_logger().info("[SIGN] workers: slowing", throttle_duration_sec=1.0)
            else:
                self._workers_until = None
        # stop / give_way: hold still for the configured time, then release.
        if self.commit_direction is None and self._stopsign_until is not None:
            if now < self._stopsign_until:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("[SIGN] holding for stop/give-way",
                                       throttle_duration_sec=1.0)
            else:
                self._stopsign_until = None
                self.get_logger().warn("[SIGN] hold done -> resume")

        # Master motion switch: if driving is disabled, hold still regardless of
        # what the controller computed (perception keeps running below).
        publish_cmd = True
        if not self._drive_enabled:
            cmd = Twist()
            if self._release_cmd_when_off:
                # Send a short STOP burst on the way down, then RELEASE /cmd_vel
                # (stay silent) so an external teleop can drive. 'd' arbitrates.
                if self._drive_off_stop_ticks > 0:
                    self._drive_off_stop_ticks -= 1
                else:
                    publish_cmd = False
                self.get_logger().info("[DRIVE] disabled -> /cmd_vel released (teleop can drive)",
                                       throttle_duration_sec=2.0)
            else:
                self.get_logger().info("[DRIVE] disabled -> holding still", throttle_duration_sec=2.0)

        self._loop_stage = 'publish_cmd'
        self._commit_debug_tick(now, 'before_publish_cmd', cmd=cmd, force=True)
        if publish_cmd:
            self.cmd_pub.publish(cmd)
        self._commit_debug_tick(now, 'after_publish_cmd', cmd=cmd, force=True)

        # Odometry: integrate the commanded speed into travelled distance.
        self._odom_tick(now, cmd)

        # Distance proxy for the double-intersection guard: integrate commanded
        # speed at the timer rate (30 Hz). Stays huge until a commit resets it.
        self._dist_since_commit = min(100.0,
                                      self._dist_since_commit + abs(cmd.linear.x) * 0.033)

        self.get_logger().debug("-" * 50)

        # Debug Visuals
        cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), h), (0, 255, 255), 2)
        if self._sign_result is not None:
            draw_sign_overlay(frame, self._sign_result)
        self._draw_status_hud(frame, cmd)
        self._loop_stage = 'controller_csv'
        self._commit_debug_tick(now, 'before_controller_csv', cmd=cmd, force=True)
        self._log_controller_row(now, cmd)
        self._commit_debug_tick(now, 'after_controller_csv', cmd=cmd, force=True)
        self._loop_stage = 'lane_status'
        self._publish_lane_status(cmd)
        self._loop_stage = 'telemetry'
        self._publish_telemetry(now, cmd)
        self._loop_stage = 'snapshot'
        self._commit_debug_tick(now, 'before_snapshot', cmd=cmd, force=True)
        self._maybe_snapshot(now, frame)
        self._commit_debug_tick(now, 'after_snapshot', cmd=cmd, force=True)
        if self.show_window:
            self._loop_stage = 'imshow'
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

        # Push the annotated frame to the MJPEG stream.
        self._loop_stage = 'stream'
        self._commit_debug_tick(now, 'before_stream', cmd=cmd, force=True)
        self._publish_stream_frame(frame)
        self._commit_debug_tick(now, 'after_stream', cmd=cmd, force=True)
        self._loop_stage = 'done'

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
