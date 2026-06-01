#!/usr/bin/env python3

import rclpy
import math
import csv
import matplotlib.pyplot as plt

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ==========================================================
# Helper function to keep angles between -PI and PI
# ==========================================================
def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class TrafficMotionNode(Node):

    def __init__(self):

        super().__init__('traffic_motion_node')

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(
            String,
            '/traffic_state',
            self.state_cb,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        # =========================
        # Publisher
        # =========================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =========================
        # State Variables
        # =========================
        self.current_state = "RED"
        self.motion_state = "STOPPED"

        # Robot current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_received = False

        # Step initialization variables
        self.step_initialized = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # =========================
        # Waypoint Sequence Setup
        # =========================
        distance = 0.4

        self.sequence = [
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance),
            ('TURN', math.radians(-45)),
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance),
            ('TURN', math.radians(-45)),
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance)
        ]

        self.seq_idx = 0

        # =========================
        # Controller Parameters
        # =========================
        self.kp_linear = 0.8
        self.kp_angular = 1.5

        # Speed limits
        self.max_v = 0.15
        self.max_w = 0.40

        # Tolerances
        self.dist_tolerance = 0.01
        self.angle_tolerance = 0.03

        # Timer
        self.dt = 0.05

        self.timer = self.create_timer(
            self.dt,
            self.loop
        )

        # =========================
        # Logging Variables
        # =========================
        self.start_time = self.get_clock().now()

        self.time_data = []
        self.error_data = []
        self.linear_data = []
        self.angular_data = []

        self.csv_filename = "controller_data.csv"

        self.get_logger().info(
            "Traffic Motion Node Started"
        )

    # ==========================================================
    # Traffic Light Callback
    # ==========================================================
    def state_cb(self, msg):

        self.current_state = msg.data

    # ==========================================================
    # Odometry Callback
    # ==========================================================
    def odom_cb(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.current_yaw = 2.0 * math.atan2(qz, qw)

        self.odom_received = True

    # ==========================================================
    # Main Loop
    # ==========================================================
    def loop(self):

        cmd = Twist()

        error = 0.0

        # Wait for odometry
        if not self.odom_received:
            return

        detected = self.current_state.upper()

        # =========================
        # Decision Layer
        # =========================
        if detected == "RED":

            self.motion_state = "STOPPED"

        elif detected == "YELLOW":

            if self.motion_state != "STOPPED":
                self.motion_state = "SLOW"

        elif detected == "GREEN":

            self.motion_state = "GO"

        # =========================
        # End Sequence
        # =========================
        if self.seq_idx >= len(self.sequence):

            self.get_logger().info(
                "Trajectory completed"
            )

            self.cmd_pub.publish(cmd)

            return

        # =========================
        # Speed Scaling
        # =========================
        speed_scale = 0.0

        if self.motion_state == "GO":

            speed_scale = 1.0

        elif self.motion_state == "SLOW":

            speed_scale = 0.5

        # =========================
        # Controller
        # =========================
        if speed_scale > 0.0:

            action, target = self.sequence[self.seq_idx]

            # ----------------------
            # Initialize Step
            # ----------------------
            if not self.step_initialized:

                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_yaw = self.current_yaw

                self.step_initialized = True

                self.get_logger().info(
                    f"Starting {action}"
                )

            # ----------------------
            # MOVE Controller
            # ----------------------
            if action == 'MOVE':

                dist_traveled = math.sqrt(
                    (self.current_x - self.start_x)**2 +
                    (self.current_y - self.start_y)**2
                )

                error = target - dist_traveled

                desired_v = self.kp_linear * error

                current_limit = self.max_v * speed_scale

                cmd.linear.x = max(
                    -current_limit,
                    min(current_limit, desired_v)
                )

                cmd.angular.z = 0.0

                # Completion
                if abs(error) <= self.dist_tolerance:

                    self.get_logger().info(
                        "Finished MOVE"
                    )

                    self.seq_idx += 1

                    self.step_initialized = False

            # ----------------------
            # TURN Controller
            # ----------------------
            elif action == 'TURN':

                target_yaw_global = normalize_angle(
                    self.start_yaw + target
                )

                error = normalize_angle(
                    target_yaw_global - self.current_yaw
                )

                desired_w = self.kp_angular * error

                current_limit = self.max_w * speed_scale

                cmd.linear.x = 0.0

                cmd.angular.z = max(
                    -current_limit,
                    min(current_limit, desired_w)
                )

                # Completion
                if abs(error) <= self.angle_tolerance:

                    self.get_logger().info(
                        "Finished TURN"
                    )

                    self.seq_idx += 1

                    self.step_initialized = False

        else:

            # STOPPED
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # =========================
        # Data Logging
        # =========================
        current_time = (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9

        self.time_data.append(current_time)
        self.error_data.append(error)
        self.linear_data.append(cmd.linear.x)
        self.angular_data.append(cmd.angular.z)

        # =========================
        # Publish Command
        # =========================
        self.cmd_pub.publish(cmd)

    # ==========================================================
    # Save Results
    # ==========================================================
    def save_results(self):

        # --------------------------
        # Save CSV
        # --------------------------
        with open(
            self.csv_filename,
            mode='w',
            newline=''
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                "time",
                "error",
                "linear_velocity",
                "angular_velocity"
            ])

            for i in range(len(self.time_data)):

                writer.writerow([
                    self.time_data[i],
                    self.error_data[i],
                    self.linear_data[i],
                    self.angular_data[i]
                ])

        self.get_logger().info(
            f"CSV saved to {self.csv_filename}"
        )

        # --------------------------
        # Error Plot
        # --------------------------
        plt.figure()

        plt.plot(
            self.time_data,
            self.error_data
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Control Error")
        plt.title("Control Error vs Time")

        plt.grid()

        plt.savefig("error_plot.png")

        # --------------------------
        # Linear Velocity Plot
        # --------------------------
        plt.figure()

        plt.plot(
            self.time_data,
            self.linear_data
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Linear Velocity [m/s]")
        plt.title("Linear Velocity vs Time")

        plt.grid()

        plt.savefig("linear_velocity_plot.png")

        # --------------------------
        # Angular Velocity Plot
        # --------------------------
        plt.figure()

        plt.plot(
            self.time_data,
            self.angular_data
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Angular Velocity [rad/s]")
        plt.title("Angular Velocity vs Time")

        plt.grid()

        plt.savefig("angular_velocity_plot.png")

        self.get_logger().info(
            "Plots generated successfully"
        )


# ==========================================================
# Main
# ==========================================================
def main(args=None):

    rclpy.init(args=args)

    node = TrafficMotionNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.save_results()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()