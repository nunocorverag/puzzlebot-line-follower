#!/usr/bin/env python3
"""Combined traffic sign (YOLO) + traffic light (HSV) detector.

Opens the CSI camera directly, runs both detectors on each frame,
draws overlays on a single preview window, and publishes to:
  /sign_detection       (String) — YOLO sign class name
  /traffic_light_state  (String) — RED / YELLOW / GREEN / NONE

Classes detected by YOLO:
  0: give-way
  1: stop
  2: straight
  3: trabajadores
  4: vuelta-derecha
  5: vuelta-izquierda

Usage:
  python3 tools/sign_detector.py
  python3 tools/sign_detector.py --confidence 0.4
"""
import sys
import time
from pathlib import Path
from collections import deque, Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))
from puzzlebot_ros.perception.camera import (  # noqa: E402
    load_camera_params,
    load_illumination_gain,
    open_csi_capture,
    preprocess_frame,
)
from puzzlebot_ros.perception.stream import Preview  # noqa: E402
MODEL_PATH = REPO_DIR / "config" / "best.pt"

# BGR colors for YOLO sign classes
CLASS_COLORS = {
    "give-way":         (0, 165, 255),
    "stop":             (0, 0, 255),
    "straight":         (0, 255, 0),
    "trabajadores":     (255, 165, 0),
    "vuelta-derecha":   (255, 0, 255),
    "vuelta-izquierda": (255, 255, 0),
}

# BGR colors for traffic light overlay indicator
TL_INDICATOR = {
    "RED":    (0, 0, 255),
    "YELLOW": (0, 220, 255),
    "GREEN":  (0, 200, 0),
    "NONE":   (60, 60, 60),
}


class SignDetectorNode(Node):

    def __init__(self, conf_threshold: float = 0.45):
        super().__init__("sign_detector")

        # --- YOLO model ---
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(MODEL_PATH))
            self.get_logger().info(f"Loaded YOLO model: {MODEL_PATH}")
        except ImportError:
            self.get_logger().error("ultralytics not installed — pip3 install ultralytics")
            raise

        self.conf = conf_threshold

        # --- Camera calibration ---
        self.camera_matrix, self.dist_coeffs = load_camera_params(
            REPO_DIR / "config" / "camera_params.npz"
        )
        self.illumination_gain = load_illumination_gain(
            REPO_DIR / "config" / "illumination_flatfield.npz"
        )

        # --- Traffic light parameters ---
        self.tl_min_detect_area = 30       # Minimum area just to see it exists (far away)
        self.tl_min_action_area = 350      # Minimum area to consider it "close enough" to trigger a stop
        self.tl_min_circularity = 0.60     # Slightly relaxed for distant blobs
        self.tl_kernel = np.ones((5, 5), np.uint8)
        
        # Temporal filtering to avoid flickering
        self.tl_history = deque(maxlen=5)  # Stores the state of the last 5 frames
        self.tl_last_published_state = "NONE"

        # --- Camera (direct CSI capture; self-contained, no separate node) ---
        self.cap = open_csi_capture(width=640, height=480, fps=30,
                                    log=self.get_logger().info)
        if self.cap is None:
            self.get_logger().error("Could not open CSI camera.")
            raise RuntimeError("camera unavailable")

        # --- Publishers ---
        self.sign_pub = self.create_publisher(String, "/sign_detection", 10)
        self.tl_pub   = self.create_publisher(String, "/traffic_light_state", 10)

        # --- Preview (h264 | local | none via $STREAM) ---
        self.preview = Preview.from_env("Detector", fps=10, log=self.get_logger().info)

        # --- Inference timer (10 Hz) ---
        self.create_timer(0.1, self._loop)

        self.frame_count = 0
        self.fps_time = time.time()
        self.get_logger().info(f"Detector ready — YOLO conf: {conf_threshold}")

    # ------------------------------------------------------------------
    # Camera grab
    # ------------------------------------------------------------------
    def _grab_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return preprocess_frame(
            frame,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            gain=self.illumination_gain,
            size=(640, 480),
        )

    # ------------------------------------------------------------------
    # Traffic light helpers
    # ------------------------------------------------------------------
    def _find_best_blob(self, mask: np.ndarray):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.tl_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.tl_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_area, best_center = 0, None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.tl_min_detect_area:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self.tl_min_circularity:
                continue
            m = cv2.moments(cnt)
            if m["m00"] == 0:
                continue
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            if area > best_area:
                best_area = area
                best_center = (cx, cy)

        return best_area, best_center

    def _detect_traffic_light(self, frame: np.ndarray):
        h = frame.shape[0]
        roi = frame[0:int(h * 0.75), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, (0,   100, 100), (10,  255, 255)),
            cv2.inRange(hsv, (160, 100, 100), (180, 255, 255)),
        )
        yellow_mask = cv2.inRange(hsv, (15,  70,  80), (45,  255, 255))
        green_mask  = cv2.inRange(hsv, (40,  80,  80), (90,  255, 255))

        candidates = [
            ("RED",    self._find_best_blob(red_mask)[0]),
            ("YELLOW", self._find_best_blob(yellow_mask)[0]),
            ("GREEN",  self._find_best_blob(green_mask)[0]),
        ]
        candidates.sort(key=lambda x: x[1], reverse=True)

        best_color, best_area = candidates[0]
        
        # Determine if it's visible, and if it's close enough to matter
        if best_area > self.tl_min_detect_area:
            is_actionable = best_area >= self.tl_min_action_area
            return best_color, best_area, is_actionable
        
        return "NONE", 0, False

    # ------------------------------------------------------------------
    # Main inference loop
    # ------------------------------------------------------------------
    def _loop(self):
        frame = self._grab_frame()
        if frame is None:
            return

        display = frame.copy()

        # --- YOLO sign detection ---
        results = self.model(frame, conf=self.conf, verbose=False)
        sign_detections = []

        for r in results:
            for box in r.boxes:
                cls_id   = int(box.cls[0])
                conf     = float(box.conf[0])
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                color = CLASS_COLORS.get(cls_name, (0, 255, 255))

                cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(display, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(display, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                sign_detections.append(cls_name)

        if sign_detections:
            msg = String(); msg.data = sign_detections[0]
            self.sign_pub.publish(msg)

        # --- Traffic light detection & smoothing ---
        raw_color, tl_area, is_actionable = self._detect_traffic_light(frame)
        
        # If it's not actionable (too far), the controller should treat it as NONE
        current_state = raw_color if is_actionable else "NONE"
        
        # Push to history buffer and calculate the most common state (majority vote)
        self.tl_history.append(current_state)
        smoothed_state = Counter(self.tl_history).most_common(1)[0][0]

        tl_msg = String(); tl_msg.data = smoothed_state
        self.tl_pub.publish(tl_msg)

        if smoothed_state != self.tl_last_published_state:
            self.get_logger().info(f"Traffic light changed to: {smoothed_state}")
            self.tl_last_published_state = smoothed_state

        # --- Traffic light indicator (top-right) ---
        # Visually distinct indicator: Show if it's FAR vs ACTIONABLE
        display_color = TL_INDICATOR[raw_color]
        cx, cy, r = 590, 38, 28
        
        cv2.circle(display, (cx, cy), r + 3, (30, 30, 30), -1)
        if raw_color != "NONE" and not is_actionable:
            # Draw an outlined circle if the light is visible but far away
            cv2.circle(display, (cx, cy), r, display_color, 4)
            tl_display_text = f"{raw_color} (FAR)"
        else:
            # Draw a solid circle if the light is close/actionable
            cv2.circle(display, (cx, cy), r, display_color, -1)
            tl_display_text = raw_color

        cv2.circle(display, (cx, cy), r, (200, 200, 200), 2)
        cv2.putText(display, tl_display_text, (cx - 45, cy + r + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, display_color, 2)

        # --- Status bar ---
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.fps_time
            self._fps = 30.0 / max(elapsed, 1e-6)
            self.fps_time = time.time()
        fps_val = getattr(self, "_fps", 0.0)

        sign_text = ",".join(sign_detections) if sign_detections else "none"
        status = f"sign: {sign_text}   light: {smoothed_state} (area: {tl_area})  fps: {fps_val:.1f}"
        
        cv2.putText(display, status, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(display, status, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        self.preview.show(display)

    def destroy_node(self):
        self.preview.close()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confidence", type=float, default=0.45)
    args = parser.parse_args()

    rclpy.init()
    node = SignDetectorNode(conf_threshold=args.confidence)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()