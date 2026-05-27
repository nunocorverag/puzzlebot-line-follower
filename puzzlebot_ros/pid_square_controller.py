#!/usr/bin/env python3
"""
pid_square_controller.py
=========================
Closed-loop PID controller for navigating a 2m x 2m square trajectory.

Robustness strategies:
    1. Anti-windup: Integral terms are clamped to prevent saturation
    2. Derivative filtering: Uses filtered derivative to reduce noise sensitivity
    3. Dead-zone compensation: Ensures velocities stay in linear operating region
    4. Saturation guards: Clamps velocities to robot physical limits
    5. Position/orientation tolerances: Prevents oscillation near goal
    6. State machine: Manages transitions between waypoints cleanly

Control architecture:
    - Two independent PID loops: one for distance, one for heading
    - First rotates to face target, then drives straight
    - Uses odometry feedback for closed-loop control

FSM states:
    IDLE → ROTATE_TO_GOAL → DRIVE_TO_GOAL → (repeat for 4 corners) → STOP
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from typing import List, Tuple


class PIDController:
    """Generic PID controller with anti-windup and derivative filtering."""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 integral_limit: float = 1.0, alpha: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.alpha = alpha
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        
    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
            
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        
        raw_derivative = (error - self.prev_error) / dt
        filtered_derivative = self.alpha * raw_derivative + (1 - self.alpha) * self.prev_derivative
        
        output = self.kp * error + self.ki * self.integral + self.kd * filtered_derivative
        
        self.prev_error = error
        self.prev_derivative = filtered_derivative
        
        return output


class SquarePIDController(Node):
    
    IDLE = 0
    ROTATE_TO_GOAL = 1
    DRIVE_TO_GOAL = 2
    FINAL_ROTATE = 3
    STOP = 4
    
    def __init__(self):
        super().__init__('pid_square_controller')
        
        self.declare_parameter('side_length', 0.5)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.02)
        self.declare_parameter('kd_linear', 0.15)
        
        self.declare_parameter('kp_angular', 2)
        self.declare_parameter('ki_angular', 0.03)
        self.declare_parameter('kd_angular', 0.2)
        
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('min_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('min_angular_vel', 0.1)
        
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('orientation_tolerance', 0.087)
        
        self.declare_parameter('control_rate', 50.0)
        
        self.declare_parameter('integral_limit_linear', 1.0)
        self.declare_parameter('integral_limit_angular', 1.0)
        
        side = self.get_parameter('side_length').value
        
        self._waypoints = self._generate_square_waypoints(side)
        self._current_waypoint = 0
        
        kp_lin = self.get_parameter('kp_linear').value
        ki_lin = self.get_parameter('ki_linear').value
        kd_lin = self.get_parameter('kd_linear').value
        int_lim_lin = self.get_parameter('integral_limit_linear').value
        
        kp_ang = self.get_parameter('kp_angular').value
        ki_ang = self.get_parameter('ki_angular').value
        kd_ang = self.get_parameter('kd_angular').value
        int_lim_ang = self.get_parameter('integral_limit_angular').value
        
        self._pid_linear = PIDController(kp_lin, ki_lin, kd_lin, int_lim_lin)
        self._pid_angular = PIDController(kp_ang, ki_ang, kd_ang, int_lim_ang)
        
        self._max_v = self.get_parameter('max_linear_vel').value
        self._min_v = self.get_parameter('min_linear_vel').value
        self._max_w = self.get_parameter('max_angular_vel').value
        self._min_w = self.get_parameter('min_angular_vel').value
        
        self._pos_tol = self.get_parameter('position_tolerance').value
        self._ori_tol = self.get_parameter('orientation_tolerance').value
        
        self._current_pose = [0.0, 0.0, 0.0]
        self._pose_received = False
        
        self._state = self.IDLE
        self._prev_time = None
        
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        
        rate = self.get_parameter('control_rate').value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)
        
        self.get_logger().info(f'PID Square Controller initialized')
        self.get_logger().info(f'Square side length: {side}m')
        self.get_logger().info(f'Waypoints: {len(self._waypoints)}')
        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(f'  WP{i}: x={wp[0]:.2f}m, y={wp[1]:.2f}m, theta={np.degrees(wp[2]):.1f}deg')
        self.get_logger().info(f'PID Linear: Kp={kp_lin}, Ki={ki_lin}, Kd={kd_lin}')
        self.get_logger().info(f'PID Angular: Kp={kp_ang}, Ki={ki_ang}, Kd={kd_ang}')
        
    def _generate_square_waypoints(self, side: float) -> List[Tuple[float, float, float]]:
        """Generate waypoints for a square starting at origin."""
        return [
            (side, 0.0, 0.0),
            (side, side, np.pi/2),
            (0.0, side, np.pi),
            (0.0, 0.0, -np.pi/2)
        ]
    
    def _odom_callback(self, msg: Odometry):
        """Extract pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        theta = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
        
        self._current_pose = [x, y, theta]
        
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(f'Odometry received. Starting position: x={x:.3f}, y={y:.3f}, theta={np.degrees(theta):.1f}deg')
            self._transition(self.ROTATE_TO_GOAL)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def _get_distance_to_goal(self) -> float:
        """Calculate Euclidean distance to current waypoint."""
        goal = self._waypoints[self._current_waypoint]
        dx = goal[0] - self._current_pose[0]
        dy = goal[1] - self._current_pose[1]
        return np.hypot(dx, dy)
    
    def _get_heading_to_goal(self) -> float:
        """Calculate required heading to face current waypoint."""
        goal = self._waypoints[self._current_waypoint]
        dx = goal[0] - self._current_pose[0]
        dy = goal[1] - self._current_pose[1]
        return np.arctan2(dy, dx)
    
    def _get_heading_error(self) -> float:
        """Calculate heading error (normalized to [-pi, pi])."""
        target_heading = self._get_heading_to_goal()
        error = self._normalize_angle(target_heading - self._current_pose[2])
        return error
    
    def _get_orientation_error(self) -> float:
        """Calculate orientation error to final goal orientation."""
        goal = self._waypoints[self._current_waypoint]
        error = self._normalize_angle(goal[2] - self._current_pose[2])
        return error
    
    def _transition(self, new_state: int):
        """Transition to new state and reset PIDs."""
        self._state = new_state
        self._pid_linear.reset()
        self._pid_angular.reset()
        
        state_names = {
            0: 'IDLE',
            1: 'ROTATE_TO_GOAL',
            2: 'DRIVE_TO_GOAL',
            3: 'FINAL_ROTATE',
            4: 'STOP'
        }
        
        self.get_logger().info(
            f'[FSM] -> {state_names.get(new_state, "UNKNOWN")} '
            f'(WP {self._current_waypoint + 1}/{len(self._waypoints)})'
        )
    
    def _publish_velocity(self, v: float = 0.0, w: float = 0.0):
        """Publish velocity command with saturation and dead-zone handling."""
        if abs(v) > 0 and abs(v) < self._min_v:
            v = np.sign(v) * self._min_v
        v = np.clip(v, -self._max_v, self._max_v)
        
        if abs(w) > 0 and abs(w) < self._min_w:
            w = np.sign(w) * self._min_w
        w = np.clip(w, -self._max_w, self._max_w)
        
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)
    
    def _control_loop(self):
        """Main control loop executing PID control."""
        if not self._pose_received:
            return
        
        now = self.get_clock().now()
        
        if self._prev_time is None:
            self._prev_time = now
            return
        
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        
        if dt <= 0:
            return
        
        if self._state == self.IDLE:
            self._publish_velocity()
            
        elif self._state == self.ROTATE_TO_GOAL:
            heading_error = self._get_heading_error()
            
            if abs(heading_error) < self._ori_tol:
                self._transition(self.DRIVE_TO_GOAL)
            else:
                w = self._pid_angular.compute(heading_error, dt)
                self._publish_velocity(w=w)
        
        elif self._state == self.DRIVE_TO_GOAL:
            distance = self._get_distance_to_goal()
            heading_error = self._get_heading_error()
            
            if distance < self._pos_tol:
                self._transition(self.FINAL_ROTATE)
            else:
                v = self._pid_linear.compute(distance, dt)
                w = self._pid_angular.compute(heading_error, dt) * 0.5
                self._publish_velocity(v=v, w=w)
        
        elif self._state == self.FINAL_ROTATE:
            orientation_error = self._get_orientation_error()
            
            if abs(orientation_error) < self._ori_tol:
                self._current_waypoint += 1
                
                if self._current_waypoint >= len(self._waypoints):
                    self._transition(self.STOP)
                else:
                    self._transition(self.ROTATE_TO_GOAL)
            else:
                w = self._pid_angular.compute(orientation_error, dt)
                self._publish_velocity(w=w)
        
        elif self._state == self.STOP:
            self._publish_velocity()
            self._timer.cancel()
            self.get_logger().info('Square trajectory complete!')
            self.get_logger().info(f'Final pose: x={self._current_pose[0]:.3f}m, y={self._current_pose[1]:.3f}m, theta={np.degrees(self._current_pose[2]):.1f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = SquarePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_velocity()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
