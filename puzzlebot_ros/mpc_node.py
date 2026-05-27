#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class VisualMPC(Node):
    def __init__(self):
        super().__init__('visual_mpc_node')

        # --- Subscribers (From Vision Node) ---
        self.create_subscription(Float32, '/ex', self.ex_cb, 10)
        self.create_subscription(Float32, '/area', self.area_cb, 10)
        self.create_subscription(Float32, '/object_detected', self.detect_cb, 10)

        # --- Publisher (To Motors) ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- State Variables ---
        self.e_x = 0.0
        self.area = 0.0
        self.is_detected = False

        # --- MPC & Control Parameters ---
        self.A_star = 15000.0  # Target area (Distance threshold - you will need to tune this)
        self.dt = 0.1          # Control loop time step (10 Hz)
        self.N = 3             # Prediction Horizon (Keep it small for compute speed)
        
        # Cost Function Weights [w1, w2, w3, w4]
        self.W_ex = 10.0       # Higher priority on centering
        self.W_area = 5.0      # Priority on distance
        self.W_v = 1.0         # Penalty for linear velocity
        self.W_w = 2.0         # Penalty for angular velocity

        # --- Control Loop Timer ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Visual MPC Node Started. Waiting for target...")

    # --- Callbacks ---
    def ex_cb(self, msg):
        self.e_x = msg.data

    def area_cb(self, msg):
        self.area = msg.data

    def detect_cb(self, msg):
        self.is_detected = bool(msg.data > 0.5)

    # --- Main Control Loop ---
    def control_loop(self):
        cmd = Twist()

        if not self.is_detected:
            # MODE 1: STANDBY (Target Lost)
            # Do not move at all. Wait for the dot to reappear.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("STANDBY: Target lost, waiting...", throttle_duration_sec=1.0)
            self.cmd_pub.publish(cmd)
            return

        # MODE 2: TRACKING (Target Visible -> Run MPC)
        best_v, best_w = self.run_mpc_rollout()
        
        self.get_logger().info(f"TRACKING | e_x: {self.e_x:.1f} | Area: {self.area:.1f} | Action: v={best_v}, w={best_w}")
        
        # Apply the best first action
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

    def run_mpc_rollout(self):
        # HARDWARE TRIM: If the robot drifts forward when turning, 
        # apply a tiny reverse speed to cancel it out.
        # Start at -0.02 and adjust until it spins perfectly in place.
        v_trim = -0.02 

        # 1. Candidate Actions (v, w)
        U_candidates = [
            (0.08, 0.0),       # Drive Forward
            (0.06, 0.3),       # Soft Curve Left
            (0.06, -0.3),      # Soft Curve Right
            (v_trim, 0.4),     # Spin Left in place (Compensated)
            (v_trim, -0.4),    # Spin Right in place (Compensated)
            (0.0, 0.0)         # Brake
        ]

        best_cost = float('inf')
        best_v = 0.0
        best_w = 0.0

        # Heuristic Gains
        K_pan = 200.0   
        K_zoom = 5000.0 

        # 2. Evaluate Each Candidate Sequence
        for (v, w) in U_candidates:
            cost = 0.0
            
            sim_ex = self.e_x
            sim_area = self.area

            for k in range(self.N):
                # Predict future state
                sim_ex = sim_ex + (K_pan * w * self.dt) 
                sim_area = sim_area + (K_zoom * v * self.dt)

                # --- CRITICAL FIX: NORMALIZED COST CALCULATION ---
                # Divide ex by 320 (half screen width) so it ranges roughly -1 to 1
                error_ex_norm = sim_ex / 320.0
                
                # Divide area error by target area so it ranges roughly -1 to 1
                error_area_norm = (sim_area - self.A_star) / self.A_star

                # Now the weights actually matter because the errors aren't massive numbers!
                stage_cost = (self.W_ex * (error_ex_norm ** 2) + 
                              self.W_area * (error_area_norm ** 2) + 
                              self.W_v * (v ** 2) + 
                              self.W_w * (w ** 2))
                
                cost += stage_cost

            # 3. Select the Minimum Cost Action
            if cost < best_cost:
                best_cost = cost
                best_v = v
                best_w = w

        return best_v, best_w

def main(args=None):
    rclpy.init(args=args)
    node = VisualMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
