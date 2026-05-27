#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import cv2
import numpy as np


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # =========================
        # Publishers
        # =========================
        self.state_pub = self.create_publisher(String, '/traffic_state', 10)
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)

        # =========================
        # Camera
        # =========================
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

        self.get_logger().info("Master Vision Node Started (Lights + Lines)")

        # Timer (30 Hz)
        self.timer = self.create_timer(0.033, self.loop)

        # =========================
        # State & Debugging Variables
        # =========================
        self.min_area = 500
        self.threshold_frames = 3

        self.current_state = "RED"
        self.allowed_state = "RED"
        
        self.last_detected_color = "UNKNOWN"
        self.last_state = "UNKNOWN"

        self.expected_next = {
            "RED": "GREEN",
            "GREEN": "YELLOW",
            "YELLOW": "RED"
        }

        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0
        
        # New variables for line debugging
        self.is_line_visible = False
        self.frame_count = 0

    # ==========================================================
    # Traffic Light Helper
    # ==========================================================
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

    # ==========================================================
    # Main Processing Loop
    # ==========================================================
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received")
            return
            
        self.frame_count += 1
        height, width, _ = frame.shape

        # ======================================================
        # 1. TRAFFIC LIGHT PROCESSING (Look at the whole frame)
        # ======================================================
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # HSV ranges
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([8, 255, 255])
        lower_red2 = np.array([172, 150, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_yellow = np.array([20, 150, 120])
        upper_yellow = np.array([32, 255, 255])
        lower_green = np.array([40, 120, 120])
        upper_green = np.array([85, 255, 255])

        # Masks & Detection
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        red_area, _, _ = self.detect_color(red_mask)
        yellow_area, _, _ = self.detect_color(yellow_mask)
        green_area, _, _ = self.detect_color(green_mask)

        detected_color = "UNKNOWN"
        if max(red_area, yellow_area, green_area) > self.min_area:
            if red_area > yellow_area and red_area > green_area:
                detected_color = "RED"
            elif yellow_area > red_area and yellow_area > green_area:
                detected_color = "YELLOW"
            elif green_area > red_area and green_area > yellow_area:
                detected_color = "GREEN"

        # ---> DEBUG: Traffic Light Raw Detection <---
        if detected_color != self.last_detected_color:
            self.get_logger().info(
                f"[VISION] Raw Color Detected: {detected_color} "
                f"(R:{red_area:.0f} Y:{yellow_area:.0f} G:{green_area:.0f})"
            )
            self.last_detected_color = detected_color

        # Temporal filtering
        if detected_color == "RED":
            self.red_count += 1
            self.yellow_count = 0
            self.green_count = 0
        elif detected_color == "YELLOW":
            self.yellow_count += 1
            self.red_count = 0
            self.green_count = 0
        elif detected_color == "GREEN":
            self.green_count += 1
            self.red_count = 0
            self.yellow_count = 0
        else:
            self.red_count = self.yellow_count = self.green_count = 0

        # FSM
        expected = self.expected_next[self.allowed_state]
        if detected_color == expected:
            if detected_color == "RED" and self.red_count >= self.threshold_frames:
                self.allowed_state = "RED"
            elif detected_color == "GREEN" and self.green_count >= self.threshold_frames:
                self.allowed_state = "GREEN"
            elif detected_color == "YELLOW" and self.yellow_count >= self.threshold_frames:
                self.allowed_state = "YELLOW"

        self.current_state = self.allowed_state

        # ---> DEBUG: Verified FSM State Change <---
        if self.current_state != self.last_state:
            self.get_logger().info(f"[STATE] Output changed to: {self.current_state}")
            self.last_state = self.current_state

        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)


        # ======================================================
        # 2. LINE DETECTION PROCESSING (Look at bottom 1/3)
        # ======================================================
        roi_top = int(height * 2/3)
        roi = frame[roi_top:height, 0:width]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Adaptive thresholding for robustness against lighting changes
        thresh = cv2.adaptiveThreshold(
            blurred, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        line_error = 0.0
        line_found_this_frame = False
        
        if len(contours) > 0:
            # Sort contours by area, grab the largest
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Filter out tiny specks of noise (adjust 100 if needed)
            if cv2.contourArea(largest_contour) > 100:
                line_found_this_frame = True
                
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    image_center_x = width // 2
                    line_error = float(image_center_x - cx)

        # ---> DEBUG: Line State Changes <---
        if line_found_this_frame and not self.is_line_visible:
            self.get_logger().info("[VISION] Line DETECTED in ROI.")
            self.is_line_visible = True
        elif not line_found_this_frame and self.is_line_visible:
            self.get_logger().warn("[VISION] Line LOST from ROI!")
            self.is_line_visible = False

        # ---> DEBUG: Periodic Line Error updates (Twice a second) <---
        if self.is_line_visible and self.frame_count % 15 == 0:
            self.get_logger().info(f"[VISION] Tracking Line | Current Error: {line_error:.1f}")

        # Publish Line Error
        error_msg = Float32()
        error_msg.data = line_error
        self.error_pub.publish(error_msg)


    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()