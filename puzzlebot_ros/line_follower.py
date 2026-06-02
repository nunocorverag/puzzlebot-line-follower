#!/usr/bin/env python3

import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

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
        self.commit_until = None
        self.intersection_cooldown_until = None

        # Approach-and-center: when the intersection is first detected the robot
        # keeps following the line at a slow creep until the entry zebra reaches
        # the target depth AND is centered, so it always stops at the same spot.
        # NOTE: the motors have a deadband ~0.08-0.10 m/s (0.05 does not move the
        # robot, ~0.10 does). approach_speed must stay above it or the creep
        # never actually drives.
        self.declare_parameter('approach_target_entry_y_pct', 82)
        self.declare_parameter('approach_speed', 0.10)
        self._approach_target_entry_y_pct = float(self.get_parameter('approach_target_entry_y_pct').value)
        self._approach_speed = float(self.get_parameter('approach_speed').value)

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
        # PD Controller
        # =========================================================
        self.kp = 0.003
        self.kd = 0.008

        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time = self.get_clock().now()

        self.max_v = 0.08
        self.max_w = 0.6

        # Timer (30 Hz)
        self.timer = self.create_timer(0.033, self.control_loop)

        # MJPEG server (access from the PC: http://10.10.0.100:8080)
        _start_mjpeg_server(port=8080)
        self.get_logger().info("Autonomous Racer Started: Lines + Traffic Lights")
        self.get_logger().info("MJPEG stream available at http://10.10.0.100:8080")

    def _package_config_path(self, filename):
        if get_package_share_directory is None:
            return None
        try:
            return Path(get_package_share_directory("puzzlebot_ros")) / "config" / filename
        except Exception:
            return None

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
            return
        # Only enforce the option list when we actually classified some options.
        # If detection fired but no direction could be validated, trust the operator.
        if self.intersection_pending and self.intersection_options and normalized not in self.intersection_options:
            self.get_logger().warn(
                f"Decision '{normalized}' not in current options: {', '.join(self.intersection_options)}"
            )
            return
        self.intersection_decision = normalized
        self.get_logger().info(f"Intersection decision received: {normalized}")

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
            f"| Raw Detect: {detected_color} | Active State: {self.current_state}"
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
        cooldown_active = (
            self.intersection_cooldown_until is not None
            and now < self.intersection_cooldown_until
        )
        if cooldown_active or self.commit_direction is not None:
            # Suppress detection while committing a turn or cooling down.
            self.intersection_stable = 0
            result = None
        else:
            result = self._analyze_intersection(frame)
            self.intersection_result = result

        # The detector requires stable_frames_needed consecutive (and centered)
        # frames, so dashed_detected is already debounced. The first time it
        # fires we enter APPROACH: keep following the line at a creep until the
        # zebra is close and centered, only then stop and ask for a decision.
        if (result is not None and result.dashed_detected
                and self.intersection_phase is None):
            self.intersection_phase = 'approach'
            self.intersection_options = result.options
            self.get_logger().info('[INTERSECTION] detected -> APPROACH_CENTER')

        if self.intersection_phase in ('approach', 'wait') and self.intersection_result is not None:
            self.intersection_options = self.intersection_result.options
            self._draw_intersection_overlay(frame, self.intersection_result)

        # APPROACH: arrived once the entry zebra is at the target depth and the
        # fitted line is centered under the camera. Motion itself is handled by
        # the line follower below (capped to a slow creep in the supervisor).
        if self.intersection_phase == 'approach':
            r = self.intersection_result
            arrived = (
                r is not None and r.entry_y_pct is not None
                and r.entry_y_pct >= self._approach_target_entry_y_pct
                and r.entry_centered
            )
            if arrived:
                self.intersection_phase = 'wait'
                self.intersection_pending = True
                self.intersection_decision = None
                self.last_prompt_time = None
                self.get_logger().info('[INTERSECTION] centered -> WAIT for decision')

        # WAIT: stopped at the intersection, prompting until a decision arrives.
        if self.intersection_phase == 'wait':
            draw_result = self.intersection_result
            should_prompt = self.last_prompt_time is None or (now - self.last_prompt_time).nanoseconds * 1e-9 > 1.0
            if should_prompt and draw_result is not None:
                self._publish_intersection_prompt(draw_result)
                self.last_prompt_time = now

            if self.intersection_decision is None:
                self.cmd_pub.publish(Twist())
                self._publish_stream_frame(frame)
                if self.show_window:
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(1)
                return

            self.commit_direction = self.intersection_decision
            self.commit_until = now + Duration(seconds=1.0)
            self.intersection_phase = None
            self.intersection_pending = False
            self.intersection_options = []
            self.intersection_decision = None
            self.intersection_stable = 0
            self.intersection_result = None
            self.intersection_cooldown_until = now + Duration(seconds=3.0)

        # ---------------------------------------------------------
        # 3. LINE PERCEPTION
        # ---------------------------------------------------------
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
        self.get_logger().info(f"[TRACKING] Bottom Line: {bot_str} | Top Line: {top_str}")

        # ---------------------------------------------------------
        # 4. BASE CONTROL CALCULATION (Line Follower)
        # ---------------------------------------------------------
        base_linear_x  = 0.0
        target_angular_z = 0.0
        steering_center_x = None

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
            self.get_logger().info("[CONTROL] Using top candidate only (bottom lost).")

        else:
            if self.time_line_lost is None:
                self.time_line_lost = now
            elapsed_time = (now - self.time_line_lost).nanoseconds * 1e-9
            self.get_logger().warn(f"[CONTROL] LINE LOST! Elapsed time: {elapsed_time:.2f}s")

            if elapsed_time < 5.0:
                base_linear_x    = 0.04
                target_angular_z = 0.0
            else:
                base_linear_x    = 0.0
                target_angular_z = 0.0

        if approach_entry_center_x is not None:
            steering_center_x = approach_entry_center_x
            self.time_line_lost = None
            self.get_logger().info(
                f"[INTERSECTION] Steering to entry center x={approach_entry_center_x:.1f}"
            )

        # PD Math
        if steering_center_x is not None:
            line_error = frame_center_x - steering_center_x
            dt = (now - self.last_time).nanoseconds * 1e-9

            if dt > 0:
                raw_derivative = (line_error - self.last_error) / dt
                derivative     = (0.7 * self.last_derivative) + (0.3 * raw_derivative)
                w_out          = (self.kp * line_error) + (self.kd * derivative)

                curve_factor = max(0.4, 1.0 - (abs(line_error) / frame_center_x))

                if base_linear_x == 0.0:
                    base_linear_x = self.max_v * curve_factor

                target_angular_z = max(-self.max_w, min(self.max_w, w_out))

                self.get_logger().info(
                    f"[MATH] Error: {line_error:.1f} | Deriv: {derivative:.1f} | "
                    f"Curve Fact: {curve_factor:.2f} -> Raw W: {w_out:.3f}"
                )

                self.last_error      = line_error
                self.last_derivative = derivative
                self.last_time       = now

        if self.commit_direction is not None:
            if self.commit_until is not None and now < self.commit_until:
                base_linear_x = 0.04
                if self.commit_direction == 'left':
                    target_angular_z = 0.25
                elif self.commit_direction == 'right':
                    target_angular_z = -0.25
                else:
                    target_angular_z = 0.0
                self.get_logger().info(
                    f"[INTERSECTION] Committing {self.commit_direction}: "
                    f"V={base_linear_x:.2f}, W={target_angular_z:.2f}"
                )
            else:
                self.commit_direction = None
                self.commit_until = None

        # During APPROACH keep steering (angular, for centering) but drive a
        # fixed creep speed above the motor deadband so it actually closes on the
        # zebra and stops at a repeatable spot.
        if self.intersection_phase == 'approach':
            base_linear_x = self._approach_speed
            self.get_logger().info(
                f"[INTERSECTION] APPROACH creep: V={base_linear_x:.3f} W={target_angular_z:.2f}"
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
            self.get_logger().info("[ACTION] Stopped for RED light.")
        elif effective_state == "YELLOW":
            cmd.linear.x  = base_linear_x * 0.5
            cmd.angular.z = target_angular_z
            self.get_logger().info(
                f"[ACTION] Throttled for YELLOW. Cmd -> V: {cmd.linear.x:.3f}, W: {cmd.angular.z:.3f}"
            )
        else:  # GREEN
            cmd.linear.x  = base_linear_x
            cmd.angular.z = target_angular_z
            self.get_logger().info(
                f"[ACTION] Normal Drive (GREEN). Cmd -> V: {cmd.linear.x:.3f}, W: {cmd.angular.z:.3f}"
            )

        # Master motion switch: if driving is disabled, hold still regardless of
        # what the controller computed (perception keeps running below).
        if not self._drive_enabled:
            cmd = Twist()
            self.get_logger().info("[DRIVE] disabled -> holding still")

        self.cmd_pub.publish(cmd)

        self.get_logger().info("-" * 50)

        # Debug Visuals
        cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), h), (0, 255, 255), 2)
        if self.show_window:
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

        # Push the annotated frame to the MJPEG stream.
        self._publish_stream_frame(frame)

    def destroy_node(self):
        self.cap.release()
        if self._h264_streamer is not None:
            self._h264_streamer.release()
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