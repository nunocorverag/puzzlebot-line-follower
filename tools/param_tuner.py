#!/usr/bin/env python3
"""Interactive PD + warp tuner for the line follower (WASD-style, in one screen).

Runs as a tiny ROS2 node next to the follower (node name ``autonomous_racer``)
and tunes its parameters LIVE via the parameter services -- no rebuild, no
``set_*.sh`` per value. Arrow/j-k pick a field, left/right (or -/=) nudge it, and
the bottom line shows the live lane metrics from ``/lane_status``. ``s`` saves the
current values to ``config/lane_params.json`` + ``config/control_params.json``
(so they survive a restart).

Uses the raw ``set_parameters`` / ``get_parameters`` services (works on every
rclpy version), not AsyncParameterClient.

Run on the Jetson (same machine as the follower) over SSH:

    scripts/run_param_tuner_jetson.sh

Keys: j/k select  -/= nudge  s save  d drive on/off  1/2/3 = left/straight/right
      0 reset intersection  q/ESC quit. Drive + intersection commands publish from
      this already-running node, so they're instant (no per-call DDS discovery).
"""

from __future__ import annotations

import curses

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from std_msgs.msg import Bool, Float32MultiArray, String

TARGET_NODE = "autonomous_racer"

# (param name, kind 'f'=float/'i'=int, step). Curated to the knobs that matter.
FIELDS = [
    ("kp",                       "f", 0.0005),
    ("kd",                       "f", 0.001),
    ("ff_gain",                  "f", 0.1),
    ("max_v",                    "f", 0.01),
    ("max_w",                    "f", 0.05),
    ("curve_slow_gain",          "f", 0.05),
    ("curve_min_scale",          "f", 0.05),
    ("snapshot_interval",        "f", 0.5),   # REC rate (s); 0 = off
    # --- intersection (tune on the robot) ---
    ("k_align",                  "f", 0.1),   # APPROACH heading-align strength
    ("intersection_slow_speed",  "f", 0.01),  # slow-zone speed near a cross
    ("commit_turn_w",            "f", 0.05),  # turn rate of the L/R maneuver
    ("commit_duration",          "f", 0.25),  # how long the L/R turn runs (~90 deg)
    ("lane.eval_y_pct",          "i", 1),
    ("lane.lookahead_y_pct",     "i", 1),
    ("lane.src_top_y_pct",       "i", 1),
    ("lane.src_top_half_w_pct",  "i", 1),
    ("lane.src_bot_y_pct",       "i", 1),
    ("lane.src_bot_half_w_pct",  "i", 1),
    ("lane.base_search_half_w_pct", "i", 1),
]
DEFAULTS = {
    "kp": 0.0018, "kd": 0.01, "ff_gain": 1.0, "max_v": 0.08, "max_w": 0.6,
    "curve_slow_gain": 0.6, "curve_min_scale": 0.4, "snapshot_interval": 2.0,
    "k_align": 0.6, "intersection_slow_speed": 0.08,
    "commit_turn_w": 0.6, "commit_duration": 2.0,
    "lane.eval_y_pct": 72, "lane.lookahead_y_pct": 45,
    "lane.src_top_y_pct": 55, "lane.src_top_half_w_pct": 14,
    "lane.src_bot_y_pct": 95, "lane.src_bot_half_w_pct": 42,
    "lane.base_search_half_w_pct": 26,
}


def _pv_to_py(pv):
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if pv.type == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    return None


class Tuner(Node):
    def __init__(self):
        super().__init__("param_tuner")
        self.values = dict(DEFAULTS)
        self.metrics = [0.0, 0.0, 0.0, 0.0, 0.0]   # off, conf, curv, v, w
        self.drive_on = False
        self.recording = False
        self.msg = "connecting..."
        self.create_subscription(Float32MultiArray, "/lane_status",
                                 self._status_cb, 10)
        self.save_pub = self.create_publisher(Bool, "/save_params", 10)
        # Persistent publishers = instant commands (no per-call DDS discovery).
        self.drive_pub = self.create_publisher(Bool, "/drive_enable", 10)
        self.decision_pub = self.create_publisher(String, "/intersection_decision", 10)
        self.reset_pub = self.create_publisher(Bool, "/intersection_reset", 10)
        self.recorder_pub = self.create_publisher(Bool, "/recorder_enable", 10)
        self.set_cli = self.create_client(SetParameters, f"/{TARGET_NODE}/set_parameters")
        self.get_cli = self.create_client(GetParameters, f"/{TARGET_NODE}/get_parameters")
        if self.set_cli.wait_for_service(timeout_sec=5.0):
            self._read_current()
            self.msg = f"connected to /{TARGET_NODE}"
        else:
            self.msg = f"WARN: /{TARGET_NODE} not found -- is the follower running?"

    def _status_cb(self, msg):
        if len(msg.data) >= 5:
            self.metrics = list(msg.data[:5])

    def _read_current(self):
        req = GetParameters.Request()
        req.names = [f[0] for f in FIELDS]
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=4.0)
        res = fut.result()
        if res is None:
            return
        for name, pv in zip(req.names, res.values):
            val = _pv_to_py(pv)
            if val is not None:
                self.values[name] = val

    def set_value(self, name, kind, value):
        value = round(value, 4) if kind == "f" else int(value)
        self.values[name] = value
        if not self.set_cli.service_is_ready():
            self.msg = "param service not ready (follower running?)"
            return
        pv = ParameterValue()
        if kind == "f":
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        else:
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        req = SetParameters.Request()
        req.parameters = [ParameterMsg(name=name, value=pv)]
        self.set_cli.call_async(req)   # fire-and-forget; node on_set is fast
        self.msg = f"set {name} = {value}"

    def save(self):
        self.save_pub.publish(Bool(data=True))
        self.msg = "saved -> lane_params.json + control_params.json"

    def toggle_drive(self):
        self.drive_on = not self.drive_on
        self.drive_pub.publish(Bool(data=self.drive_on))
        self.msg = f"DRIVE {'ON' if self.drive_on else 'off'}"

    def decide(self, direction):
        self.decision_pub.publish(String(data=direction))
        self.msg = f"intersection -> {direction}"

    def reset_intersection(self):
        self.reset_pub.publish(Bool(data=True))
        self.msg = "intersection RESET -> FOLLOW"

    def toggle_recording(self):
        self.recording = not self.recording
        self.recorder_pub.publish(Bool(data=self.recording))
        self.msg = f"REC {'ON' if self.recording else 'off'}"


def _safe_addstr(stdscr, y, x, text, attr=0):
    """addstr that never throws on small terminals: skip off-screen rows and
    truncate text to the window width (curses errors if you write past the edge)."""
    max_y, max_x = stdscr.getmaxyx()
    if y < 0 or y >= max_y or x >= max_x:
        return
    text = text[: max(0, max_x - x - 1)]
    try:
        stdscr.addstr(y, x, text, attr)
    except curses.error:
        pass


def _draw(stdscr, tuner, sel):
    stdscr.erase()
    _safe_addstr(stdscr, 0, 2, f"PD / WARP TUNER        node: /{TARGET_NODE}", curses.A_BOLD)
    _safe_addstr(stdscr, 1, 2, "-" * 52)
    row = 2
    for i, (name, kind, step) in enumerate(FIELDS):
        val = tuner.values.get(name, 0.0)
        valstr = f"{val:.4f}" if kind == "f" else f"{int(val)}"
        marker = ">" if i == sel else " "
        attr = curses.A_REVERSE if i == sel else curses.A_NORMAL
        _safe_addstr(stdscr, row, 2, f"{marker} {name:<26} {valstr:>10}   (step {step})", attr)
        row += 1
    _safe_addstr(stdscr, row + 1, 2, "-" * 52)
    off, conf, curv, v, w = tuner.metrics
    drive = "ON " if tuner.drive_on else "off"
    drive_attr = curses.A_BOLD if tuner.drive_on else curses.A_NORMAL
    _safe_addstr(stdscr, row + 2, 2, "drive: ")
    _safe_addstr(stdscr, row + 2, 9, drive, drive_attr)
    _safe_addstr(stdscr, row + 2, 14,
                 f"  off {off:+.2f}  conf {conf:.2f}  curv {curv:+.2f}  "
                 f"v {v:.3f}  w {w:+.2f}")
    if tuner.recording:
        _safe_addstr(stdscr, row + 2, 56, "REC", curses.A_REVERSE)
    _safe_addstr(stdscr, row + 4, 2, "[j/k] select  [-/=] nudge  [s] save  [q] quit")
    _safe_addstr(stdscr, row + 5, 2, "[d] drive  [1/2/3] L/S/R  [0] reset cross  [r] record")
    _safe_addstr(stdscr, row + 6, 2, tuner.msg[:60])
    stdscr.refresh()


def _loop(stdscr, tuner):
    curses.curs_set(0)
    stdscr.nodelay(True)
    sel = 0
    while rclpy.ok():
        rclpy.spin_once(tuner, timeout_sec=0.05)
        _draw(stdscr, tuner, sel)
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1
        if key == -1:
            continue
        name, kind, step = FIELDS[sel]
        if key in (ord("q"), 27):
            break
        elif key in (curses.KEY_DOWN, ord("j")):
            sel = (sel + 1) % len(FIELDS)
        elif key in (curses.KEY_UP, ord("k")):
            sel = (sel - 1) % len(FIELDS)
        elif key in (curses.KEY_RIGHT, ord("="), ord("+"), ord(".")):
            tuner.set_value(name, kind, tuner.values[name] + step)
        elif key in (curses.KEY_LEFT, ord("-"), ord("_"), ord(",")):
            tuner.set_value(name, kind, tuner.values[name] - step)
        elif key == ord("s"):
            tuner.save()
        elif key == ord("d"):
            tuner.toggle_drive()
        elif key == ord("1"):
            tuner.decide("left")
        elif key == ord("2"):
            tuner.decide("straight")
        elif key == ord("3"):
            tuner.decide("right")
        elif key == ord("0"):
            tuner.reset_intersection()
        elif key == ord("r"):
            tuner.toggle_recording()


def main():
    rclpy.init()
    tuner = Tuner()
    try:
        curses.wrapper(_loop, tuner)
    except KeyboardInterrupt:
        pass  # Ctrl-C is a normal way to quit; don't dump a traceback
    finally:
        try:
            tuner.destroy_node()
        except Exception:
            pass
        if rclpy.ok():            # avoid "rcl_shutdown already called"
            rclpy.shutdown()


if __name__ == "__main__":
    main()
