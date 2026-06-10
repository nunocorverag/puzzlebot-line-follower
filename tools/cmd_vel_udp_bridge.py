#!/usr/bin/env python3
"""UDP -> /cmd_vel bridge (Jetson side of the laptop WASD teleop).

Receives "v w" datagrams (linear m/s, angular rad/s) from the laptop teleop
(tools/teleop_wasd_gui.py) and republishes the latest on /cmd_vel at 50 Hz.

A watchdog zeroes the command if no datagram arrives within WATCHDOG_S
(default 0.3 s), so a lost WiFi link or a killed sender stops the robot.

Env: UDP_PORT (5005), WATCHDOG_S (0.3).
"""

from __future__ import annotations

import os
import signal
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_udp_bridge")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.port = int(os.environ.get("UDP_PORT", "5005"))
        self.watchdog = float(os.environ.get("WATCHDOG_S", "0.3"))
        self.lin = 0.0
        self.ang = 0.0
        self.last_rx = 0.0
        self.lock = threading.Lock()
        # Yield to the line follower when it is driving: when /drive_enable is True
        # we stop publishing so the follower owns /cmd_vel; when it is False (drive
        # off) we publish teleop. So the panel's 'd' toggle arbitrates who drives.
        # Default active (no follower / standalone teleop -> teleop works).
        self._yield = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        threading.Thread(target=self._rx_loop, daemon=True).start()
        self.create_timer(1.0 / 50.0, self._publish)
        self.create_subscription(Bool, "/drive_enable", self._on_drive_enable, 10)
        self.get_logger().info(
            f"cmd_vel_udp_bridge listening on :{self.port} (watchdog {self.watchdog}s)"
        )

    def _on_drive_enable(self, msg: Bool) -> None:
        self._yield = bool(msg.data)

    def _rx_loop(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except OSError:
                break
            try:
                parts = data.split()
                v, w = float(parts[0]), float(parts[1])
            except (ValueError, IndexError):
                continue
            with self.lock:
                self.lin, self.ang, self.last_rx = v, w, time.monotonic()

    def _publish(self) -> None:
        if self._yield:           # follower is driving -> don't fight it
            return
        with self.lock:
            stale = (time.monotonic() - self.last_rx) > self.watchdog
            v = 0.0 if stale else self.lin
            w = 0.0 if stale else self.ang
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def zero(self) -> None:
        z = Twist()
        for _ in range(10):
            self.pub.publish(z)


def main() -> int:
    rclpy.init()
    node = CmdVelUdpBridge()

    def _shutdown(*_args) -> None:
        node.zero()
        rclpy.try_shutdown()
        os._exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zero()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
