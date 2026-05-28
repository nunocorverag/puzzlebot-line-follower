#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import cv2
import numpy as np
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── MJPEG server ──────────────────────────────────────────────
class _MJPEGHandler(BaseHTTPRequestHandler):
    _lock  = threading.Lock()
    _frame = None                       # bytes JPEG actuales

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            with _MJPEGHandler._lock:
                data = _MJPEGHandler._frame
            if data:
                try:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                        + data + b'\r\n'
                    )
                except BrokenPipeError:
                    break

    def log_message(self, *_):          # silencia el log del servidor
        pass

def _start_mjpeg_server(port=8080):
    srv = HTTPServer(('0.0.0.0', port), _MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv
# ──────────────────────────────────────────────────────────────

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
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        self.declare_parameter('use_undistort', True)
        self.declare_parameter('camera_params_path', '')
        self.declare_parameter('use_illumination_correction', True)
        self.declare_parameter('illumination_params_path', '')
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

        self.intersection_pending = False
        self.intersection_options = []
        self.intersection_decision = None
        self.intersection_frames = 0
        self.last_prompt_time = None
        self.commit_direction = None
        self.commit_until = None
        self.intersection_cooldown_until = None

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

        # Servidor MJPEG (acceder desde la PC: http://10.10.0.100:8080)
        _start_mjpeg_server(port=8080)
        self.get_logger().info("Autonomous Racer Started: Lines + Traffic Lights")
        self.get_logger().info("MJPEG stream disponible en http://10.10.0.100:8080")

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

        for params_path in candidate_paths:
            if params_path.exists():
                data = np.load(str(params_path))
                self.get_logger().info(f'Loaded camera calibration: {params_path}')
                return data['camera_matrix'], data['dist_coeffs']

        self.get_logger().warn('Camera calibration not found; running without undistort.')
        return None, None

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
        if self.intersection_pending and normalized not in self.intersection_options:
            self.get_logger().warn(
                f"Decision '{normalized}' not in current options: {', '.join(self.intersection_options)}"
            )
            return
        self.intersection_decision = normalized
        self.get_logger().info(f"Intersection decision received: {normalized}")

    def _black_mask(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    def _black_ratio(self, mask, x0, x1, y0, y1):
        h, w = mask.shape[:2]
        x0 = max(0, min(w, int(x0)))
        x1 = max(0, min(w, int(x1)))
        y0 = max(0, min(h, int(y0)))
        y1 = max(0, min(h, int(y1)))
        if x1 <= x0 or y1 <= y0:
            return 0.0
        roi = mask[y0:y1, x0:x1]
        return float(cv2.countNonZero(roi)) / float(roi.size)

    def _analyze_intersection(self, frame):
        h, w = frame.shape[:2]
        mask = self._black_mask(frame)
        roi_y0, roi_y1 = int(h * 0.42), int(h * 0.82)
        contours, _ = cv2.findContours(mask[roi_y0:roi_y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dashed = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 40 or area > 2400:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            y += roi_y0
            if bw < 5 or bh < 5:
                continue
            rectangularity = area / float(bw * bh)
            if rectangularity < 0.45:
                continue
            aspect = max(bw / float(bh), bh / float(bw))
            if aspect > 6.0:
                continue
            cx, cy = x + bw / 2.0, y + bh / 2.0
            dashed.append((cx, cy, bw, bh, area))

        center_x = w / 2.0
        option_x0, option_x1 = w * 0.08, w * 0.92
        option_y0, option_y1 = h * 0.35, h * 0.68
        option_dashed = [d for d in dashed if option_x0 <= d[0] <= option_x1 and option_y0 <= d[1] <= option_y1]
        left_dash = [d for d in option_dashed if d[0] < center_x - w * 0.12]
        center_dash = [d for d in option_dashed if abs(d[0] - center_x) <= w * 0.18]
        right_dash = [d for d in option_dashed if d[0] > center_x + w * 0.12]

        ahead_ratio = self._black_ratio(mask, w * 0.38, w * 0.62, h * 0.20, h * 0.54)
        left_ratio = self._black_ratio(mask, w * 0.05, w * 0.42, h * 0.36, h * 0.70)
        right_ratio = self._black_ratio(mask, w * 0.58, w * 0.95, h * 0.36, h * 0.70)

        dashed_detected = len(dashed) >= 5 or (len(center_dash) >= 2 and (len(left_dash) + len(right_dash)) >= 2)
        options = []
        if len(left_dash) >= 2:
            options.append('left')
        if len(center_dash) >= 2:
            options.append('straight')
        if len(right_dash) >= 2:
            options.append('right')

        debug = {
            'dashed_count': len(dashed),
            'left_dash': len(left_dash),
            'center_dash': len(center_dash),
            'right_dash': len(right_dash),
            'ahead_ratio': ahead_ratio,
            'left_ratio': left_ratio,
            'right_ratio': right_ratio,
        }
        return dashed_detected, options, debug

    def _publish_intersection_prompt(self, options, debug):
        option_text = ', '.join(options) if options else 'none'
        msg = String()
        msg.data = (
            f"Intersection detected. Options: {option_text}. "
            "Reply with: ros2 topic pub --once /intersection_decision "
            "std_msgs/msg/String \"{data: 'left'}\""
        )
        self.intersection_prompt_pub.publish(msg)
        self.get_logger().warn(
            f"[INTERSECTION] Waiting for decision. Options: {option_text}. Debug: {debug}"
        )

    def _draw_intersection_overlay(self, frame, options, debug):
        h, w = frame.shape[:2]
        cv2.putText(frame, 'INTERSECTION - WAITING', (20, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, f"options: {', '.join(options)}", (20, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        cv2.rectangle(frame, (int(w * 0.38), int(h * 0.20)), (int(w * 0.62), int(h * 0.54)), (255, 255, 0), 2)
        cv2.rectangle(frame, (int(w * 0.05), int(h * 0.36)), (int(w * 0.42), int(h * 0.70)), (255, 0, 255), 2)
        cv2.rectangle(frame, (int(w * 0.58), int(h * 0.36)), (int(w * 0.95), int(h * 0.70)), (255, 0, 255), 2)
        cv2.putText(frame, f"dash:{debug['dashed_count']} L:{debug['left_dash']} S:{debug['center_dash']} R:{debug['right_dash']}",
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
        # the candidates by X and assigning left→middle→right in order.
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
        dashed_detected, intersection_options, intersection_debug = self._analyze_intersection(frame)

        cooldown_active = (
            self.intersection_cooldown_until is not None
            and now < self.intersection_cooldown_until
        )
        if dashed_detected and not cooldown_active and self.commit_direction is None:
            self.intersection_frames += 1
        else:
            self.intersection_frames = 0

        if self.intersection_frames >= 4 and not self.intersection_pending:
            self.intersection_pending = True
            self.intersection_options = intersection_options
            self.intersection_decision = None
            self.last_prompt_time = None

        if self.intersection_pending:
            self._draw_intersection_overlay(frame, self.intersection_options, intersection_debug)
            should_prompt = self.last_prompt_time is None or (now - self.last_prompt_time).nanoseconds * 1e-9 > 1.0
            if should_prompt:
                self._publish_intersection_prompt(self.intersection_options, intersection_debug)
                self.last_prompt_time = now

            if self.intersection_decision is None:
                self.cmd_pub.publish(Twist())
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with _MJPEGHandler._lock:
                        _MJPEGHandler._frame = jpeg.tobytes()
                cv2.imshow("Frame", frame)
                cv2.waitKey(1)
                return

            self.commit_direction = self.intersection_decision
            self.commit_until = now + Duration(seconds=1.0)
            self.intersection_pending = False
            self.intersection_options = []
            self.intersection_decision = None
            self.intersection_frames = 0
            self.intersection_cooldown_until = now + Duration(seconds=3.0)

        # ---------------------------------------------------------
        # 3. LINE PERCEPTION
        # ---------------------------------------------------------
        bottom_y_start, bottom_y_end = int(h * 0.60), h
        bottom_x_start, bottom_x_end = int(w * 0.25), int(w * 0.75)

        top_y_start, top_y_end = int(h * 0.25), int(h * 0.50)
        top_x_start, top_x_end = int(w * 0.10), int(w * 0.90)

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

        # ---------------------------------------------------------
        # 5. SUPERVISOR OVERRIDE (Traffic Light Scale)
        # ---------------------------------------------------------
        cmd = Twist()

        if self.current_state == "RED":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("[ACTION] Stopped for RED light.")
        elif self.current_state == "YELLOW":
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

        self.cmd_pub.publish(cmd)

        self.get_logger().info("-" * 50)

        # Debug Visuals
        cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), h), (0, 255, 255), 2)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        # Publicar frame al stream MJPEG
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with _MJPEGHandler._lock:
                _MJPEGHandler._frame = jpeg.tobytes()

    def destroy_node(self):
        self.cap.release()
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