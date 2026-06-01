#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # =========================================================
        # Publisher
        # =========================================================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

        # =========================================================
        # Tracking Variables
        # =========================================================
        self.last_bottom_center = None
        self.last_top_center = None

        self.max_jump_distance = 80 

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

        # =========================================================
        # Line Loss Memory
        # =========================================================
        self.time_line_lost = None

        # =========================================================
        # Timer
        # =========================================================
        self.timer = self.create_timer(0.033, self.control_loop)

        self.get_logger().info("Dual ROI Line Follower Started (3-Line Explicit Tracking)")

    # =============================================================
    # ROI LINE DETECTOR 
    # =============================================================
    def detect_line_in_roi(
        self,
        frame,
        x_start,
        x_end,
        y_start,
        y_end,
        last_center,
        reference_x,
        draw_color=(0, 255, 0),
        force_middle_of_three=False  # <-- NEW: Explicit 3-line check
    ):

        roi = frame[y_start:y_end, x_start:x_end]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)

        _, mask = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 1. Gather all valid candidates first
        valid_candidates = []

        for c in contours:
            area = cv2.contourArea(c)
            if area < 250:
                continue

            x_box, y_box, w_box, h_box = cv2.boundingRect(c)
            if h_box < 15:
                continue

            # ASPECT RATIO FILTERING (Ignore horizontal seams)
            if w_box > h_box * 2.0:
                continue

            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue

            cx = int(moments["m10"] / moments["m00"]) + x_start
            cy = int(moments["m01"] / moments["m00"]) + y_start

            valid_candidates.append({
                'cx': cx,
                'cy': cy,
                'area': area
            })

        best_candidate = None

        # =========================================================
        # EXPLICIT 3-LINE CHECK (For Top ROI)
        # =========================================================
        if force_middle_of_three and len(valid_candidates) >= 3:
            # Sort by area (descending) to find the 3 largest/thickest lines
            valid_candidates.sort(key=lambda item: item['area'], reverse=True)
            top_3_lines = valid_candidates[:3]

            # Sort those 3 lines by their X coordinate (Left to Right)
            top_3_lines.sort(key=lambda item: item['cx'])

            # Explicitly select the middle line (Index 1)
            middle_line = top_3_lines[1]
            best_candidate = (middle_line['cx'], middle_line['cy'])

        # =========================================================
        # STANDARD ANCHORING FALLBACK (For Bottom ROI, or if < 3 lines found)
        # =========================================================
        else:
            best_score = float('inf')

            if last_center is None:
                last_center = (int(reference_x), y_end)
                allowed_jump = float('inf') 
            else:
                allowed_jump = self.max_jump_distance

            for cand in valid_candidates:
                cx = cand['cx']
                cy = cand['cy']
                area = cand['area']

                dist_to_ref = abs(cx - reference_x)
                dist_to_last = np.sqrt((cx - last_center[0])**2 + (cy - last_center[1])**2)

                if dist_to_last > allowed_jump:
                    continue

                score = (dist_to_ref * 0.4) + (dist_to_last * 0.6) - (area * 0.001)

                if score < best_score:
                    best_score = score
                    best_candidate = (cx, cy)

        # Debug Drawing
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
            self.get_logger().warn("No frame received")
            return

        h, w = frame.shape[:2]
        frame_center_x = w / 2.0
        now = self.get_clock().now()

        # =========================================================
        # ROI DEFINITIONS
        # =========================================================
        # Bottom ROI is intentionally narrow (25% to 75%) to isolate the center line.
        bottom_y_start = int(h * 0.75)
        bottom_y_end = h
        bottom_x_start = int(w * 0.25)
        bottom_x_end = int(w * 0.75)

        # Top ROI is incredibly wide (10% to 90%) to see all 3 tracks entering a curve.
        top_y_start = int(h * 0.25)
        top_y_end = int(h * 0.50)
        top_x_start = int(w * 0.10)
        top_x_end = int(w * 0.90)

        # =========================================================
        # DETECT LOWER ROI (Master - Standard Anchoring)
        # =========================================================
        bottom_reference_x = self.last_bottom_center[0] if self.last_bottom_center else frame_center_x

        bottom_candidate, bottom_mask = self.detect_line_in_roi(
            frame,
            bottom_x_start,
            bottom_x_end,
            bottom_y_start,
            bottom_y_end,
            self.last_bottom_center,
            reference_x=bottom_reference_x,
            draw_color=(0, 255, 0),
            force_middle_of_three=False # Bottom ROI relies on anchoring
        )

        # =========================================================
        # DETECT UPPER ROI (Slave - 3-Line Explicit Check)
        # =========================================================
        if bottom_candidate:
            top_reference_x = bottom_candidate[0]
        else:
            top_reference_x = self.last_top_center[0] if self.last_top_center else frame_center_x

        top_candidate, top_mask = self.detect_line_in_roi(
            frame,
            top_x_start,
            top_x_end,
            top_y_start,
            top_y_end,
            self.last_top_center,
            reference_x=top_reference_x,
            draw_color=(255, 0, 0),
            force_middle_of_three=True # Force Top ROI to find all 3 and pick the middle
        )

        # =========================================================
        # UPDATE MEMORY
        # =========================================================
        if bottom_candidate is not None:
            self.last_bottom_center = bottom_candidate

        if top_candidate is not None:
            self.last_top_center = top_candidate

        # =========================================================
        # CONTROL LOGIC
        # =========================================================
        cmd = Twist()
        steering_center_x = None

        # CASE 1: Normal operation -> trust bottom ROI
        if bottom_candidate is not None:
            self.time_line_lost = None  # Reset lost timer
            bottom_cx, bottom_cy = bottom_candidate
            steering_center_x = bottom_cx

            if top_candidate is not None:
                top_cx, top_cy = top_candidate
                curve_hint = top_cx - bottom_cx
                steering_center_x += curve_hint * 0.15
                cv2.line(frame, (bottom_cx, bottom_cy), (top_cx, top_cy), (255, 255, 0), 2)

        # CASE 2: Bottom lost but top visible
        elif top_candidate is not None:
            self.time_line_lost = None  # Reset lost timer
            top_cx, top_cy = top_candidate
            steering_center_x = top_cx
            cmd.linear.x = 0.04
            self.get_logger().warn("Using TOP ROI prediction")

        # CASE 3: Both lost (Intersection crossing logic)
        else:
            if self.time_line_lost is None:
                self.time_line_lost = now
                self.get_logger().warn("Line lost! Pushing forward for up to 5 seconds...")

            elapsed_time = (now - self.time_line_lost).nanoseconds * 1e-9

            if elapsed_time < 5.0:
                cmd.linear.x = 0.04  # Slow forward
                cmd.angular.z = 0.0  # Keep steering perfectly straight
            else:
                self.get_logger().error("5 seconds elapsed. Stopping!")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            self.cmd_pub.publish(cmd)
            
            # Show debug and exit early
            #cv2.imshow("Frame", frame)
            #cv2.imshow("Bottom Mask", bottom_mask)
            #cv2.imshow("Top Mask", top_mask)
            #cv2.waitKey(1)
            return

        # =========================================================
        # PD CONTROL
        # =========================================================
        line_error = frame_center_x - steering_center_x
        dt = (now - self.last_time).nanoseconds * 1e-9

        if dt > 0:
            raw_derivative = (line_error - self.last_error) / dt
            derivative = (0.7 * self.last_derivative) + (0.3 * raw_derivative)
            w_out = (self.kp * line_error) + (self.kd * derivative)

            curve_factor = max(0.4, 1.0 - (abs(line_error) / frame_center_x))

            if cmd.linear.x == 0.0:
                cmd.linear.x = self.max_v * curve_factor

            cmd.angular.z = max(-self.max_w, min(self.max_w, w_out))

            self.last_error = line_error
            self.last_derivative = derivative
            self.last_time = now

        # =========================================================
        # PUBLISH & VISUALIZE
        # =========================================================
        self.cmd_pub.publish(cmd)

        #cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), h), (0, 255, 255), 2)
        #cv2.imshow("Frame", frame)
        #cv2.imshow("Bottom Mask", bottom_mask)
        #cv2.imshow("Top Mask", top_mask)
        #cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()