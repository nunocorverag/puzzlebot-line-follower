#!/usr/bin/env python3
"""Robust real-time WASD teleop for /cmd_vel.

Hold-to-go: a held key (its OS auto-repeat) keeps the robot moving; release it
and it stops after HOLD_TIMEOUT (~0.4 s). One tap gives a real velocity at once
(no key-mashing). Set HOLD_TIMEOUT=0 for sticky mode (stays until you press
space). Holding "w" then nudging "a" gives a forward-left CURVE, not a one-wheel
pivot.

Why two turn modes? Differential drive mixes wheel speed = v +/- w*(L/2). With a
motor deadband ~0.09 m/s, a big w while creeping forward drops the inner wheel
below the deadband -> it pivots on one wheel. So:
  - a / d  = gentle STEER (sized so the inner wheel stays above the deadband
             while moving -> real curve)
  - q / e  = strong PIVOT in place (w large, v = 0)

Keys:
  w / s    forward / back        (linear = +/- V_MAX)
  a / d    steer left / right    (curve while moving)
  q / e    pivot left / right    (rotate in place)
  z        straighten (w = 0)    x   stop linear (keep steer)
  space    full STOP             - / =   slower / faster (scale)
  Ctrl-C / ESC   quit (sends zero)

Env (m/s, rad/s):
  V_MAX (0.15)  STEER_W (0.5)  PIVOT_W (1.2)
"""

from __future__ import annotations

import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class WASDTeleop(Node):
    def __init__(self) -> None:
        super().__init__("teleop_wasd")
        # Depth-1 keep-last so only the LATEST command is queued: no backlog of
        # stale setpoints (a real source of lag). Reliable to match the firmware
        # subscription QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.v_max = float(os.environ.get("V_MAX", "0.15"))
        self.steer_w = float(os.environ.get("STEER_W", "0.5"))
        self.pivot_w = float(os.environ.get("PIVOT_W", "1.2"))
        self.scale = 1.0
        self.lin = 0.0
        self.ang = 0.0
        # Hold-to-go: a terminal cannot see key release, only the OS auto-repeat
        # while a key is held. So we stop once no key has arrived for
        # HOLD_TIMEOUT seconds. Must be a bit longer than the OS initial-repeat
        # delay (~0.25-0.3 s) to avoid a stutter right after the first press.
        # Set HOLD_TIMEOUT=0 to restore the old sticky behaviour.
        self.hold_timeout = float(os.environ.get("HOLD_TIMEOUT", "0.4"))
        self.last_input = time.monotonic()
        # Republish at 50 Hz so the bridge never starves and a changed setpoint
        # is on the wire within ~20 ms.
        self.create_timer(1.0 / 50.0, self._publish)

    def handle_key(self, key: str) -> None:
        # Any "go" key (incl. its auto-repeat while held) keeps motion alive.
        if key in ("w", "W", "s", "S", "a", "A", "d", "D", "q", "Q", "e", "E"):
            self.last_input = time.monotonic()
        if key in ("w", "W"):
            self.lin = self.v_max
        elif key in ("s", "S"):
            self.lin = -self.v_max
        elif key in ("a", "A"):
            self.ang = self.steer_w            # gentle steer (curves with w)
        elif key in ("d", "D"):
            self.ang = -self.steer_w
        elif key in ("q", "Q"):
            self.lin = 0.0; self.ang = self.pivot_w     # pivot in place (left)
        elif key in ("e", "E"):
            self.lin = 0.0; self.ang = -self.pivot_w    # pivot in place (right)
        elif key == "z":
            self.ang = 0.0
        elif key == "x":
            self.lin = 0.0
        elif key in (" ", "k"):
            self.lin = 0.0; self.ang = 0.0
        elif key in ("-", "_"):
            self.scale = max(0.2, round(self.scale - 0.1, 2))
        elif key in ("=", "+"):
            self.scale = min(1.5, round(self.scale + 0.1, 2))
        else:
            return
        self._print_status()

    def _print_status(self) -> None:
        sys.stdout.write(
            f"\r[wasd] v={self.lin * self.scale:+.3f} m/s  "
            f"w={self.ang * self.scale:+.2f} rad/s  scale={self.scale:.1f}   "
            f"(space=stop  q/e=pivot  Ctrl-C=quit)     "
        )
        sys.stdout.flush()

    def _publish(self) -> None:
        # Hold-to-go watchdog: stop if no key (or its auto-repeat) arrived
        # within the timeout, i.e. the key was released.
        if self.hold_timeout > 0 and (self.lin or self.ang) and \
                time.monotonic() - self.last_input > self.hold_timeout:
            self.lin = 0.0
            self.ang = 0.0
            self._print_status()
        msg = Twist()
        msg.linear.x = self.lin * self.scale
        msg.angular.z = self.ang * self.scale
        self.pub.publish(msg)

    def stop(self) -> None:
        self.lin = 0.0
        self.ang = 0.0
        zero = Twist()
        for _ in range(10):
            self.pub.publish(zero)


def main() -> int:
    rclpy.init()
    node = WASDTeleop()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)

    def restore_and_stop(*_args) -> None:
        try:
            node.stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)

    # Safety: zero the velocity if the SSH session hangs up (SIGHUP) or the
    # process is killed (SIGTERM), so the robot never keeps the last command.
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, lambda *_a: (restore_and_stop(), os._exit(0)))

    def drain_keys() -> str:
        """Read every byte already buffered (handles key auto-repeat / fast
        bursts), so a quick "wa" applies both without lag."""
        data = sys.stdin.read(1)
        while select.select([sys.stdin], [], [], 0)[0]:
            data += sys.stdin.read(1)
        return data

    try:
        tty.setcbreak(fd)
        print(__doc__)
        node._print_status()
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            data = drain_keys()
            # A lone ESC quits; ESC followed by more bytes is an arrow/escape
            # sequence -> ignore it (do not let arrow keys quit or steer).
            if data == "\x1b":
                break
            for ch in data:
                if ch == "\x1b":
                    break  # start of an escape sequence in the burst; skip rest
                node.handle_key(ch)
    except KeyboardInterrupt:
        pass
    finally:
        restore_and_stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\n[wasd] stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
