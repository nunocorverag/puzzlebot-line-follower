import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.pub = self.create_publisher(PoseStamped, '/waypoint', 10)

        self.declare_parameter('radius', 0.35)
        self.declare_parameter('center_x', 0.5)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('period', 3.0)

        self.radius = self.get_parameter('radius').value
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value

        self.waypoints = self.build_pentagon()
        self.i = 0
        self.done = False

        self.timer = self.create_timer(self.get_parameter('period').value, self.publish)

        self.get_logger().info("Trajectory Generator (Pentagon) ready")

    def build_pentagon(self):
        pts = []
        for i in range(5):
            a = 2 * math.pi * i / 5
            x = self.cx + self.radius * math.cos(a)
            y = self.cy + self.radius * math.sin(a)
            pts.append((x, y))

        pts.append(pts[0])  # cerrar figura
        return pts

    def publish(self):
        if self.done:
            return

        x, y = self.waypoints[self.i]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y

        self.pub.publish(msg)

        self.get_logger().info(f"Goal {self.i+1}/6 -> ({x:.2f}, {y:.2f})")

        self.i += 1
        if self.i >= len(self.waypoints):
            self.done = True
            self.get_logger().info("Pentagon generation finished")


def main():
    rclpy.init()
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
