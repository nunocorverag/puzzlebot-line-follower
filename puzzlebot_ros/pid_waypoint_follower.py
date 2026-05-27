import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


def clamp(x, m):
    return max(-m, min(m, x))


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        # ---------------- SUBS ----------------
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        self.sub_goal = self.create_subscription(
            PoseStamped, '/waypoint', self.goal_cb, 10)

        # ---------------- PUB ----------------
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.goal = None
        self.waypoints = []
        self.index = 0

        # 🔴 IMPORTANTE: inicia detenido SIEMPRE
        self.finished = True

        self.get_logger().info("Waypoint follower initialized in STOP mode")

        # control loop
        self.timer = self.create_timer(0.05, self.control_loop)

    # ---------------- ODOM ----------------
    def odom_cb(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    # ---------------- GOALS ----------------
    def goal_cb(self, msg):

        self.waypoints.append((
            msg.pose.position.x,
            msg.pose.position.y
        ))

        # si llega primer waypoint, activa ejecución
        if self.finished:
            self.finished = False
            self.index = 0
            self.get_logger().info("Trajectory received → starting execution")

    # ---------------- CONTROL ----------------
    def control_loop(self):

        # 🛑 STOP TOTAL SI NO HAY TRAJECTORY
        if self.finished or len(self.waypoints) == 0:
            self.pub_cmd.publish(Twist())
            return

        gx, gy = self.waypoints[self.index]

        dx = gx - self.x
        dy = gy - self.y

        dist = math.sqrt(dx*dx + dy*dy)

        # ---------------- SWITCH WAYPOINT ----------------
        if dist < 0.06:
            self.index += 1

            if self.index >= len(self.waypoints):
                self.finished = True
                self.pub_cmd.publish(Twist())

                self.get_logger().info("🏁 TRAJECTORY COMPLETED")

                self.get_logger().info(
                    f"Final pose: x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.1f}°"
                )

                self.waypoints = []
                return

            return

        # ---------------- CONTROL ----------------
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        cmd.linear.x = clamp(0.25 * dist, 0.25)
        cmd.angular.z = clamp(1.2 * angle_error, 1.2)

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 🔴 STOP de emergencia al salir SIEMPRE
    node.pub_cmd.publish(Twist())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
