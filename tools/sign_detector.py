#!/usr/bin/env python3
"""Combined traffic sign (YOLO) + traffic light (HSV) detector.

Subscribes to /video_source/raw, runs both detectors on each frame,
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

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from line_vision_calibrator import load_camera_params, load_illumination_gain, apply_illumination_gain


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(msg.encoding, 3)
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame.copy()


REPO_DIR = Path(__file__).resolve().parents[1]
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

        # --- Traffic light parameters (tune via calibrator if needed) ---
        self.tl_min_area = 200
        self.tl_min_circularity = 0.65
        self.tl_kernel = np.ones((5, 5), np.uint8)
        self.tl_last_state = "NONE"

        # --- Camera subscription ---
        self.latest_frame = None
        self.create_subscription(Image, "/video_source/raw", self._image_cb, 10)
        self.get_logger().info("Subscribed to /video_source/raw")

        # --- Publishers ---
        self.sign_pub = self.create_publisher(String, "/sign_detection", 10)
        self.tl_pub   = self.create_publisher(String, "/traffic_light_state", 10)

        # --- Preview window ---
        cv2.namedWindow("Detector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detector", 640, 480)

        # --- Inference timer (10 Hz) ---
        self.create_timer(0.1, self._loop)

        self.frame_count = 0
        self.fps_time = time.time()
        self.get_logger().info(f"Detector ready — YOLO conf: {conf_threshold}")

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        frame = imgmsg_to_cv2(msg)
        # ros_deep_learning video_source hardcodes rotate-180 in its pipeline;
        # undo it so the orientation matches the calibrator.
        frame = cv2.flip(frame, -1)
        frame = cv2.resize(frame, (640, 480))
        if self.camera_matrix is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        self.latest_frame = apply_illumination_gain(frame, self.illumination_gain)

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
            if area < self.tl_min_area:
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

    def _detect_traffic_light(self, frame: np.ndarray) -> str:
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

        if candidates[0][1] > self.tl_min_area:
            return candidates[0][0]
        return "NONE"

    # ------------------------------------------------------------------
    # Main inference loop
    # ------------------------------------------------------------------
    def _loop(self):
        frame = self.latest_frame
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

        # --- Traffic light detection ---
        tl_state = self._detect_traffic_light(frame)

        tl_msg = String(); tl_msg.data = tl_state
        self.tl_pub.publish(tl_msg)

        if tl_state != self.tl_last_state:
            self.get_logger().info(f"Traffic light: {tl_state}")
            self.tl_last_state = tl_state

        # --- Traffic light indicator (top-right) ---
        ind_color = TL_INDICATOR[tl_state]
        cx, cy, r = 590, 38, 28
        cv2.circle(display, (cx, cy), r + 3, (30, 30, 30), -1)
        cv2.circle(display, (cx, cy), r, ind_color, -1)
        cv2.circle(display, (cx, cy), r, (200, 200, 200), 2)
        cv2.putText(display, tl_state, (cx - 28, cy + r + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, ind_color, 2)

        # --- Status bar ---
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.fps_time
            self._fps = 30.0 / max(elapsed, 1e-6)
            self.fps_time = time.time()
        fps_val = getattr(self, "_fps", 0.0)

        sign_text = ",".join(sign_detections) if sign_detections else "none"
        fps_val = getattr(self, "_fps", 0.0)
        status = f"sign: {sign_text}   light: {tl_state}   fps: {fps_val:.1f}"
        cv2.putText(display, status, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4)
        cv2.putText(display, status, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

        cv2.imshow("Detector", display)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
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
