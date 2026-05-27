#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorStopNode(Node):
    def __init__(self):
        super().__init__('motor_stop_node')
        
        # Publisher to the velocity topic
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to publish 10 times a second (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_zeros)
        
        self.get_logger().info("🛑 Motor Stop Node active. Forcing motors to 0.0...")
        self.get_logger().info("Press Ctrl+C to kill this node.")

    def publish_zeros(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotorStopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Motor Stop Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()