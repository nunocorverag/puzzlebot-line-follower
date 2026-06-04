#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import os

from puzzlebot_ros.perception.camera import open_csi_capture


class CalibrationCaptureNode(Node):

    def __init__(self):
        super().__init__('calibration_capture_node')

        # =========================
        # Directory setup
        # =========================
        self.save_dir = "datasets/checkerboard"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Created directory: ./{self.save_dir}/")

        # =========================
        # Camera initialization
        # =========================
        # Shared CSI capture (same resolution as the runtime: 640x480).
        self.cap = open_csi_capture(width=640, height=480, fps=30, downscale=True,
                                    log=self.get_logger().info)
        if self.cap is None:
            self.get_logger().error("Could not open camera.")
            return

        self.get_logger().info("Camera initialized successfully.")

        # =========================
        # Capture parameters
        # =========================
        self.image_count = 0
        self.target_count = 75  # Number of pictures to take

        # A timer that fires every timer_period seconds.
        self.timer_period = 0.15
        self.get_logger().info(f"Starting automatic capture: 1 photo every {self.timer_period} seconds.")
        self.timer = self.create_timer(self.timer_period, self.capture_loop)

    def capture_loop(self):
        # Exit once we reach the target number of photos.
        if self.image_count >= self.target_count:
            self.get_logger().info(f"Done! Saved {self.target_count} images in ./{self.save_dir}/")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Read a frame from the camera.
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received from the camera.")
            return

        # Save the image.
        filename = os.path.join(self.save_dir, f"calib_img_{self.image_count:03d}.png")
        cv2.imwrite(filename, frame)

        self.image_count += 1
        self.get_logger().info(f"Saved: {filename} ({self.image_count}/{self.target_count})")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Capture interrupted by the user.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
