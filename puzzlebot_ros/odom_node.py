#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy


class SimpleOdom(Node):

    def __init__(self):
        super().__init__('simple_odom')

        # PARAMS (ajústalos luego si hace falta)
        self.r = 0.05      # radio rueda (m)
        self.L = 0.19      # distancia entre ruedas (m)

        self.v_l = 0.0
        self.v_r = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_time = self.get_clock().now()

        # 🔥 QoS compatible con micro-ROS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Float32, '/VelocityEncL', self.left_cb, qos)
        self.create_subscription(Float32, '/VelocityEncR', self.right_cb, qos)

        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Simple Odom Node Started")

    def left_cb(self, msg):
        self.v_l = msg.data

    def right_cb(self, msg):
        self.v_r = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0:
            return

        # Modelo diferencial
        v = self.r * (self.v_r + self.v_l) / 2.0
        w = self.r * (self.v_r - self.v_l) / self.L

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt

        # Crear mensaje
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        qz = np.sin(self.theta / 2.0)
        qw = np.cos(self.theta / 2.0)

        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=qz,
            w=qw
        )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
