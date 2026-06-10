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

# Page-oriented control surface. Keep this curated: these are the knobs worth
# touching during live robot tests.
GROUPS = [
    ("Drive", [
        ("max_v",                    "f", 0.01,   "normal forward speed"),
        ("max_w",                    "f", 0.05,   "max steering angular speed"),
        ("kp",                       "f", 0.0005, "PD proportional gain"),
        ("kd",                       "f", 0.001,  "PD derivative gain"),
        ("ff_gain",                  "f", 0.1,    "curve/lookahead feed-forward"),
        ("curve_slow_gain",          "f", 0.05,   "slow down on curvature"),
        ("curve_min_scale",          "f", 0.05,   "minimum curve speed scale"),
        ("curve_memory_s",           "f", 0.05,   "hold curve slowdown after curve"),
        ("curve_min_v",              "f", 0.005,  "minimum speed in tight curves"),
    ]),
    ("Curve", [
        ("curve_arc_enabled",        "b", 1,      "arc mode for curves (vs PD)"),
        ("curve_arc_pre_s",          "f", 0.1,    "go STRAIGHT this long before turning"),
        ("curve_arc_post_s",         "f", 0.1,    "advance straight after the turn"),
        ("curve_arc_recenter_s",     "f", 0.1,    "2nd short turn to re-center"),
        ("curve_arc_w",              "f", 0.02,   "left turn rate in the arc"),
        ("curve_arc_v",              "f", 0.01,   "forward speed in the arc"),
        ("curve_arc_enter",          "f", 0.05,   "|curv| to START the arc"),
        ("curve_arc_exit",           "f", 0.05,   "|curv| under this (centered)=exit"),
        ("curve_arc_exit_frames",    "i", 1,      "stable frames before leaving arc"),
        ("curve_arc_min_s",          "f", 0.1,    "arc at least this long"),
        ("curve_arc_max_s",          "f", 0.5,    "arc safety cap"),
        ("curve_heading_gain",       "f", 0.05,   "(legacy) heading steer; 0=off"),
        ("curve_heading_deadband",   "f", 0.05,   "(legacy) heading deadband"),
    ]),
    ("Intersection", [
        ("intersection_slow_speed",  "f", 0.01,   "speed cap near zebra"),
        ("approach_speed",           "f", 0.01,   "creep speed toward zebra"),
        ("detect_distance_cm",       "f", 1.0,    "distance that triggers ADVANCE"),
        ("read_distance_cm",         "f", 1.0,    "dist to entry to start crossing"),
        ("read_advance_extra_cm",    "f", 1.0,    "odom nudge onto cross (0=off)"),
        ("read_after_entry_max_cm",  "f", 1.0,    "max cm after entry row"),
        ("read_cross_jump_cm",       "f", 1.0,    "z_dist jump = crossed first row"),
        ("advance_center_gain",      "f", 0.01,   "ADVANCE centring gain (0=straight)"),
        ("advance_lane_keep_gain",   "f", 0.1,    "ADVANCE lane steering gain"),
        ("advance_lane_keep_max_w",  "f", 0.01,   "ADVANCE lane steering cap"),
        ("commit_turn_w",            "f", 0.05,   "left/right commit turn rate"),
        ("commit_turn_pre_advance_cm", "f", 1.0,  "straight cm before L/R turn"),
        ("commit_duration",          "f", 0.25,   "L/R commit safety timeout"),
        ("commit_min_s",             "f", 0.1,    "L/R min commit before reacquire"),
        ("commit_duration_straight", "f", 0.25,   "STRAIGHT cross safety timeout"),
        ("commit_straight_min_s",    "f", 0.25,   "STRAIGHT min cross before reacq"),
        ("align_in_place",          "b", 1,      "rotate in READ to square up"),
        ("k_align",                  "f", 0.1,    "square-up-in-place gain (sign!)"),
        ("align_max_w",              "f", 0.05,   "max READ square-up turn rate"),
        ("align_prior_min_frames",   "i", 1,      "lane frames to trust square-up"),
        ("align_prior_heading_deg",  "f", 1.0,    "min prior lane heading"),
        ("align_prior_curv",         "f", 0.05,   "min prior lane curve"),
        ("align_tol_deg",            "f", 1.0,    "square-up tolerance (deg)"),
    ]),
    ("Zebra", [
        ("zebra.stop_distance_cm",   "f", 1.0,    "stop distance to row"),
        ("zebra.slow_distance_cm",   "f", 1.0,    "start approach distance"),
        ("zebra.min_dashes",         "i", 1,      "row dashes required"),
        ("zebra.min_span_cm",        "f", 1.0,    "row lateral span"),
        ("zebra.trigger_min_dashes", "i", 1,      "FSM trigger dash count"),
        ("zebra.option_align_deg",   "f", 1.0,    "max skew to trust options"),
        ("zebra.opt_min_dashes",     "i", 1,      "exit dashes required"),
        ("zebra.opt_min_span_cm",    "f", 1.0,    "exit lateral span"),
        ("zebra.opt_side_back_cm",   "f", 1.0,    "side dash window before row"),
        ("zebra.opt_side_line_tol_cm", "f", 0.5,    "side dash line tolerance"),
        ("zebra.opt_side_relaxed_min_y_cm", "f", 1.0, "row Y to relax side read"),
        ("zebra.opt_side_relaxed_min_dashes", "i", 1, "relaxed side dashes"),
        ("zebra.straight_by_line",  "b", 1,      "straight uses continuous line"),
        ("zebra.straight_center_on_row", "b", 1,   "center straight ROI on dash row"),
        ("zebra.trigger_max_angle_deg", "f", 1.0,  "max skew to trigger ADVANCE"),
        ("zebra.trigger_max_center_cm", "f", 1.0,  "max row center to trigger"),
        ("zebra.straight_min_len_cm", "f", 1.0,    "min continuous straight line"),
        ("zebra.straight_corridor_cm", "f", 1.0,   "central line corridor width"),
        ("zebra.straight_black_thresh", "i", 5,     "dark threshold for straight line"),
        ("zebra.straight_max_width_cm", "f", 0.5,    "max continuous line width"),
        ("zebra.straight_aspect_min", "f", 0.05,     "min height/width ratio"),
        ("zebra.straight_lookahead_cm", "f", 1.0,  "forward line search depth"),
    ]),
    ("Lane", [
        ("lane.dual_line",           "i", 1,      "follow center between borders"),
        ("lane.lane_half_px",        "i", 5,      "one-border fallback half width"),
        ("lane.min_line_gap_pct",    "i", 1,      "min dual-line gap"),
        ("lane.eval_y_pct",          "i", 1,      "near steering read"),
        ("lane.lookahead_y_pct",     "i", 1,      "far steering read"),
        ("lane.continuity",          "i", 1,      "stay on same line"),
        ("lane.base_hist_h_pct",     "i", 1,      "bottom base slice height"),
        ("lane.base_search_half_w_pct", "i", 1,   "initial base search corridor"),
        ("lane.continuity_search_half_w_pct", "i", 1, "prev-base search corridor"),
        ("lane.window_half_w_pct",   "i", 1,      "sliding window half width"),
        ("lane.min_pix",             "i", 10,     "pixels to recenter window"),
        ("lane.line_open_px",        "i", 2,      "remove thin puzzle outlines"),
        ("lane.line_core_px",        "i", 1,      "keep thick painted line core"),
        ("lane.fit_max_rmse_px",     "i", 2,      "reject scattered lane fits"),
    ]),
    ("AntiZebra", [
        ("lane.zebra_row_reject",    "i", 1,      "anti-zebra filter on/off"),
        ("lane.zebra_row_fill_pct",  "i", 1,      "row fill %% = transversal bar"),
        ("lane.zebra_row_close_px",  "i", 1,      "close px to bridge dash gaps"),
        ("lane_hold_conf",           "f", 0.05,   "conf to refresh held heading"),
        ("lane_hold_s",              "f", 0.1,    "max s to hold heading at cross"),
        ("lane_hold_curve_s",        "f", 0.05,   "hold BEV target in curves"),
        ("lane_curve_dropout_s",     "f", 0.05,   "bridge BEV loss in curves"),
        ("lane_curve_refresh_conf",  "f", 0.05,   "weak curve hold refresh conf"),
        ("lane_hold_curve_min_curv", "f", 0.05,   "min curve for BEV hold"),
        ("lane_base_max_jump_pct",   "i", 1,      "max base jump %% (branch guard)"),
        ("lane_base_edge_margin_pct", "i", 1,     "reject BEV bases near warp edge"),
        ("lane_base_hold_s",         "f", 0.1,    "sticky base hold s at cross"),
        ("lane_curve_max_jump_pct",  "i", 1,      "max base jump %% in curves"),
        ("lane_curve_guard_conf",    "f", 0.05,   "conf below this guards curve jumps"),
        ("lane_curve_guard_max_offset", "f", 0.05, "max weak-fit offset in curves"),
        ("lane_curve_min_turn_w",    "f", 0.005,  "min W while in strong curve"),
        ("lane_curve_hold_assist_conf", "f", 0.05, "use held curve lookahead below conf"),
    ]),
    ("Warp", [
        ("lane.src_top_y_pct",       "i", 1,      "warp top y"),
        ("lane.src_top_half_w_pct",  "i", 1,      "warp top half width"),
        ("lane.src_bot_y_pct",       "i", 1,      "warp bottom y"),
        ("lane.src_bot_half_w_pct",  "i", 1,      "warp bottom half width"),
        ("zebra.widen_kx",           "f", 0.1,    "wide zebra warp scale"),
    ]),
    ("Signs", [
        ("sign_turn_act_area_pct", "f", 0.1, "arrow area pct to latch"),
        ("sign_act_area_pct", "f", 0.5,      "stop/yield area pct"),
        ("workers_min_speed", "f", 0.01,     "min speed while workers slow"),
        ("sign_cooldown_s", "f", 0.5,       "same sign cooldown"),
        ("sign_forget_s", "f", 0.5,         "turn forget while FOLLOW"),
        ("sign_lane_mask_margin_px", "i", 2, "mask sign boxes from lane BEV"),
        ("sign_lane_mask_extend_down_px", "i", 5, "extend sign mask downward"),
    ]),
    ("Traffic", [
        ("traffic_light_roi_y_pct", "i", 1,      "top image pct to search"),
        ("traffic_light_min_area", "f", 10.0,   "min circular blob area"),
        ("traffic_light_max_area", "f", 100.0,  "max circular blob area"),
        ("traffic_light_min_circularity", "f", 0.05, "min circle score"),
        ("traffic_light_aspect_tol", "f", 0.05, "bbox square tolerance"),
        ("traffic_light_min_fill", "f", 0.05,   "min circle fill"),
        ("traffic_light_max_fill", "f", 0.05,   "max circle fill"),
        ("traffic_light_action_min_radius_px", "f", 1.0, "min radius to obey"),
        ("traffic_light_action_max_radius_px", "f", 1.0, "max radius to obey"),
        ("traffic_light_action_min_distance_cm", "f", 1.0, "min dist to obey"),
        ("traffic_light_action_max_distance_cm", "f", 1.0, "max dist to obey"),
        ("traffic_light_distance_k_cm_px", "f", 10.0, "distance K/radius"),
    ]),
    ("Debug", [
        ("snapshot_interval",        "f", 0.1,    "REC snapshot interval; 0 off"),
    ]),
]
FIELDS = [(name, kind, step) for _, items in GROUPS for name, kind, step, _ in items]
FIELD_META = {name: (kind, step, help_text) for _, items in GROUPS for name, kind, step, help_text in items}
DEFAULTS = {
    "kp": 0.0018, "kd": 0.0, "ff_gain": 1.0, "max_v": 0.08, "max_w": 0.6,
    "curve_slow_gain": 0.6, "curve_min_scale": 0.7, "curve_memory_s": 1.20,
    "curve_min_v": 0.045,
    "curve_arc_enabled": True, "curve_arc_pre_s": 0.6, "curve_arc_w": 0.30,
    "curve_arc_v": 0.08, "curve_arc_enter": 0.60, "curve_arc_exit": 0.30,
    "curve_arc_exit_frames": 3,
    "curve_arc_min_s": 0.6, "curve_arc_max_s": 4.0,
    "curve_arc_pre_s": 0.6, "curve_arc_post_s": 0.4, "curve_arc_recenter_s": 0.3,
    "curve_heading_gain": 0.0, "curve_heading_deadband": 0.40,
    "snapshot_interval": 0.5,
    "k_align": 0.6, "approach_align_slope": 0.15, "intersection_slow_speed": 0.08,
    "approach_speed": 0.06, "commit_turn_w": 0.6, "commit_turn_pre_advance_cm": 4.0,
    "commit_duration": 2.0, "commit_min_s": 0.8,
    "commit_duration_straight": 5.0, "commit_straight_min_s": 3.0,
    "align_in_place": False, "align_max_w": 0.20,
    "align_prior_min_frames": 5, "align_prior_heading_deg": 8.0,
    "align_prior_curv": 0.30, "align_tol_deg": 12.0,
    "detect_distance_cm": 22.0, "read_distance_cm": 6.0, "advance_center_gain": 0.0,
    "advance_lane_keep_gain": 1.0, "advance_lane_keep_max_w": 0.12,
    "read_advance_extra_cm": 0.0, "read_after_entry_max_cm": 6.0, "read_cross_jump_cm": 8.0,
    "zebra.stop_distance_cm": 10.0, "zebra.slow_distance_cm": 30.0,
    "zebra.min_dashes": 3, "zebra.min_span_cm": 7.0,
    "zebra.trigger_min_dashes": 6,
    "zebra.option_align_deg": 18.0, "zebra.opt_min_dashes": 2,
    "zebra.opt_min_span_cm": 4.0, "zebra.opt_side_back_cm": 12.0,
    "zebra.opt_side_line_tol_cm": 2.5, "zebra.opt_side_relaxed_min_y_cm": 18.0,
    "zebra.opt_side_relaxed_min_dashes": 3, "zebra.straight_by_line": True,
    "zebra.straight_center_on_row": False,
    "zebra.trigger_max_angle_deg": 18.0, "zebra.trigger_max_center_cm": 18.0,
    "zebra.straight_min_len_cm": 8.0, "zebra.straight_corridor_cm": 11.8,
    "zebra.straight_black_thresh": 90, "zebra.straight_max_width_cm": 7.0,
    "zebra.straight_aspect_min": 1.35, "zebra.straight_lookahead_cm": 34.0,
    "zebra.widen_kx": 2.4,
    "traffic_light_roi_y_pct": 55, "traffic_light_min_area": 80.0,
    "traffic_light_max_area": 5000.0, "traffic_light_min_circularity": 0.65,
    "traffic_light_aspect_tol": 0.35, "traffic_light_min_fill": 0.45,
    "traffic_light_max_fill": 1.15,
    "sign_turn_act_area_pct": 1.4, "sign_act_area_pct": 6.0,
    "workers_min_speed": 0.04,
    "sign_cooldown_s": 6.0, "sign_forget_s": 15.0,
    "sign_lane_mask_margin_px": 32,
    "sign_lane_mask_extend_down_px": 90,
    "traffic_light_action_min_radius_px": 10.0,
    "traffic_light_action_max_radius_px": 80.0,
    "traffic_light_action_min_distance_cm": 12.0,
    "traffic_light_action_max_distance_cm": 45.0,
    "traffic_light_distance_k_cm_px": 360.0,
    "lane.dual_line": 0, "lane.lane_half_px": 90, "lane.min_line_gap_pct": 14,
    "lane.eval_y_pct": 72, "lane.lookahead_y_pct": 45,
    "lane.continuity": 1, "lane.base_hist_h_pct": 18,
    "lane.src_top_y_pct": 55, "lane.src_top_half_w_pct": 14,
    "lane.src_bot_y_pct": 95, "lane.src_bot_half_w_pct": 42,
    "lane.base_search_half_w_pct": 26, "lane.continuity_search_half_w_pct": 12,
    "lane.window_half_w_pct": 12, "lane.min_pix": 60,
    "lane.line_open_px": 5, "lane.line_core_px": 7, "lane.fit_max_rmse_px": 28,
    "lane.zebra_row_reject": 1, "lane.zebra_row_fill_pct": 40,
    "lane.zebra_row_close_px": 9, "lane_hold_conf": 0.5, "lane_hold_s": 1.5,
    "lane_hold_curve_s": 1.20, "lane_curve_dropout_s": 2.0,
    "lane_curve_refresh_conf": 0.35,
    "lane_hold_curve_min_curv": 0.55,
    "lane_base_max_jump_pct": 15, "lane_base_edge_margin_pct": 12,
    "lane_base_hold_s": 1.0,
    "lane_curve_max_jump_pct": 10, "lane_curve_guard_conf": 0.80,
    "lane_curve_guard_max_offset": 0.35, "lane_curve_min_turn_w": 0.075,
    "lane_curve_hold_assist_conf": 0.80,
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
        self.step_mult = 1.0          # nudge multiplier ([ / ] to change)
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
        if kind == "b":
            value = bool(value)
        else:
            value = round(value, 4) if kind == "f" else int(value)
        self.values[name] = value
        if not self.set_cli.service_is_ready():
            self.msg = "param service not ready (follower running?)"
            return
        pv = ParameterValue()
        if kind == "f":
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        elif kind == "b":
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = bool(value)
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


STEP_MULTS = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]


def _next_mult(cur, direction):
    try:
        i = STEP_MULTS.index(cur)
    except ValueError:
        i = STEP_MULTS.index(1.0)
    return STEP_MULTS[max(0, min(len(STEP_MULTS) - 1, i + direction))]


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


def _page_fields(page_idx):
    return GROUPS[page_idx][1]


def _draw(stdscr, tuner, page_idx, sel):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()
    if max_y < 16 or max_x < 70:
        _safe_addstr(stdscr, 0, 2, "PUZZLEBOT CONTROL PANEL", curses.A_BOLD)
        _safe_addstr(stdscr, 2, 2, "Terminal/pane too small for the panel.")
        _safe_addstr(stdscr, 3, 2, "Use scripts/run_demo_tmux.sh and switch to the full control window,")
        _safe_addstr(stdscr, 4, 2, "or enlarge this pane/window.")
        _safe_addstr(stdscr, 6, 2, tuner.msg[: max(0, max_x - 4)])
        stdscr.refresh()
        return

    _safe_addstr(stdscr, 0, 2, f"PUZZLEBOT CONTROL PANEL        node: /{TARGET_NODE}", curses.A_BOLD)

    tabs = []
    for i, (title, _) in enumerate(GROUPS):
        label = f" {title} "
        tabs.append((curses.A_REVERSE if i == page_idx else curses.A_NORMAL, label))
    x = 2
    for attr, label in tabs:
        _safe_addstr(stdscr, 1, x, label, attr)
        x += len(label) + 1

    off, conf, curv, v, w = tuner.metrics
    drive = "ON" if tuner.drive_on else "off"
    rec = "ON" if tuner.recording else "off"
    status = (f"drive:{drive}  rec:{rec}  off:{off:+.2f} conf:{conf:.2f} "
              f"curv:{curv:+.2f} v:{v:.3f} w:{w:+.2f}")
    _safe_addstr(stdscr, 3, 2, status, curses.A_BOLD if tuner.drive_on else curses.A_NORMAL)
    _safe_addstr(stdscr, 4, 2, "Cmd: [d] drive [r] REC [1/2/3] L/S/R [0] reset [s] save [q] quit+pull")
    _safe_addstr(stdscr, 5, 2, f"Nav: [Tab h/l] page  [j/k] select  [-/=] nudge  [[/]] step x{tuner.step_mult:g}")
    _safe_addstr(stdscr, 6, 2, "-" * max(20, min(90, max_x - 4)))

    fields = _page_fields(page_idx)
    if sel >= len(fields):
        sel = max(0, len(fields) - 1)
    row = 7
    for i, (name, kind, step, help_text) in enumerate(fields):
        val = tuner.values.get(name, 0.0)
        if kind == "b":
            valstr = "ON" if bool(val) else "off"
        else:
            valstr = f"{float(val):.4f}" if kind == "f" else f"{int(val)}"
        marker = ">" if i == sel else " "
        attr = curses.A_REVERSE if i == sel else curses.A_NORMAL
        eff = step * tuner.step_mult if kind == "f" else max(1, int(round(step * tuner.step_mult)))
        eff_str = "toggle" if kind == "b" else (f"{eff:g}" if kind == "f" else f"{eff}")
        _safe_addstr(stdscr, row, 2,
                     f"{marker} {name:<26} {valstr:>8} st {eff_str:<5} {help_text}", attr)
        row += 1
        if row >= max_y - 4:
            break

    help_text = fields[sel][3] if fields else ""
    _safe_addstr(stdscr, max_y - 3, 2, f"Hint: {help_text}")
    _safe_addstr(stdscr, max_y - 2, 2, tuner.msg[: max(0, max_x - 4)])
    stdscr.refresh()


def _loop(stdscr, tuner):
    curses.curs_set(0)
    stdscr.nodelay(True)
    page_idx = 0
    sel = 0
    while rclpy.ok():
        rclpy.spin_once(tuner, timeout_sec=0.05)
        fields = _page_fields(page_idx)
        sel = min(sel, max(0, len(fields) - 1))
        _draw(stdscr, tuner, page_idx, sel)
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1
        if key == -1:
            continue
        name, kind, step, _ = fields[sel]
        if key in (ord("q"), 27):
            break
        elif key in (9, ord("l")):
            page_idx = (page_idx + 1) % len(GROUPS)
            sel = 0
        elif key in (curses.KEY_BTAB, ord("h")):
            page_idx = (page_idx - 1) % len(GROUPS)
            sel = 0
        elif key in (curses.KEY_DOWN, ord("j")):
            sel = (sel + 1) % len(fields)
        elif key in (curses.KEY_UP, ord("k")):
            sel = (sel - 1) % len(fields)
        elif key in (curses.KEY_RIGHT, ord("="), ord("+"), ord(".")):
            if kind == "b":
                tuner.set_value(name, kind, True)
            else:
                d = step * tuner.step_mult if kind == "f" else max(1, int(round(step * tuner.step_mult)))
                tuner.set_value(name, kind, tuner.values[name] + d)
        elif key in (curses.KEY_LEFT, ord("-"), ord("_"), ord(",")):
            if kind == "b":
                tuner.set_value(name, kind, False)
            else:
                d = step * tuner.step_mult if kind == "f" else max(1, int(round(step * tuner.step_mult)))
                tuner.set_value(name, kind, tuner.values[name] - d)
        elif key in (ord("]"), ord("}")):
            tuner.step_mult = _next_mult(tuner.step_mult, +1)
            tuner.msg = f"step x{tuner.step_mult:g}"
        elif key in (ord("["), ord("{")):
            tuner.step_mult = _next_mult(tuner.step_mult, -1)
            tuner.msg = f"step x{tuner.step_mult:g}"
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
