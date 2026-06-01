#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Publishers
        self.ex_pub = self.create_publisher(Float32, '/ex', 10)
        self.area_pub = self.create_publisher(Float32, '/area', 10)
        self.detect_pub = self.create_publisher(Float32, '/object_detected', 10)

        # Camera (Jetson-friendly) - WITH LOW LATENCY FIX
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Camera not opened!")
        else:
            self.get_logger().info("Vision node started - camera OK")

        self.timer = self.create_timer(0.1, self.loop)

        # image center
        self.cx_center = 320
        self.cy_center = 240

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received")
            return

        h, w, _ = frame.shape
        self.cx_center = w // 2
        self.cy_center = h // 2

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # WIDENED RED MASK
        # Lower S and V minimums to catch shadows and glare
        # Widen H slightly (0-15 and 165-180)
        lower_red1 = np.array([0, 80, 40])
        upper_red1 = np.array([15, 255, 255])

        lower_red2 = np.array([165, 80, 40])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Clean noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ex_msg = Float32()
        area_msg = Float32()
        det_msg = Float32()

        # Draw a line down the center of the screen so you know where "0 error" is
        cv2.line(frame, (self.cx_center, 0), (self.cx_center, h), (255, 255, 255), 1)

        if len(contours) > 0:
            # largest contour
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 500:  # filter noise
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    ex = cx - self.cx_center

                    ex_msg.data = float(ex)
                    area_msg.data = float(area)
                    det_msg.data = 1.0

                    self.ex_pub.publish(ex_msg)
                    self.area_pub.publish(area_msg)
                    self.detect_pub.publish(det_msg)

                    # VISUALIZATION: Draw the contour and the centroid
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)  # Green outline
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)   # Blue dot on centroid
                    cv2.putText(frame, f"Area: {int(area)}", (cx - 20, cy - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Show the image window
                    #cv2.imshow("Robot View", frame)
                    #cv2.waitKey(1)
                    return

        # if nothing detected
        ex_msg.data = 0.0
        area_msg.data = 0.0
        det_msg.data = 0.0

        self.ex_pub.publish(ex_msg)
        self.area_pub.publish(area_msg)
        self.detect_pub.publish(det_msg)

        cv2.putText(frame, "SEARCHING...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Robot View", frame)
        cv2.waitKey(1)

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
