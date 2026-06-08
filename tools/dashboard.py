#!/usr/bin/env python3
"""Live telemetry dashboard for the line follower (runs on the LAPTOP).

The follower broadcasts a compact JSON status over UDP (no ROS needed on the
laptop, same pattern as the H264 stream). This renders, in real time:
  - the state machine (FOLLOW -> APPROACH -> WAIT -> COMMIT) with the active stage
  - the zebra detection (distance in cm, skew angle, options/exits)
  - lane metrics and the motion command
  - the key live parameters to keep in mind
  - a rolling log of state transitions / decisions

Usage:  python3 tools/dashboard.py [--port 5005]
Bind on the laptop; the follower must run with telemetry_host=<laptop IP>
(run_line_follower_jetson.sh already passes the laptop IP as h264_host, which the
telemetry reuses by default).
"""
import argparse
import json
import socket
import time
from collections import deque

ESC = "\033["
RESET = ESC + "0m"


def c(code, s):
    return f"{ESC}{code}m{s}{RESET}"


GREEN, YELLOW, RED, CYAN, ORANGE, GREY, BOLD = "32", "33", "31", "36", "38;5;208", "90", "1"
STAGES = ["FOLLOW", "ADVANCE", "READ", "COMMIT"]


def stage_of(d):
    if d.get("commit"):
        return "COMMIT"
    ph = d.get("phase")
    if ph == "wait":
        return "READ"
    if ph == "approach":
        return "ADVANCE"
    return "FOLLOW"


def bar(label, value, vmax, width=20, color=CYAN):
    n = 0 if vmax <= 0 else max(0, min(width, int(round(width * value / vmax))))
    return f"{label:>10} [" + c(color, "#" * n) + " " * (width - n) + f"] {value}"


def render(d, last_seen, log):
    out = []
    age = time.time() - last_seen
    stale = age > 1.0
    title = "PUZZLEBOT — LINE FOLLOWER TELEMETRY"
    out.append(c(BOLD, title) + ("   " + c(RED, "● NO DATA") if stale else
                                 "   " + c(GREY, f"t={d.get('t','?')}s")))
    out.append("=" * 78)

    # --- state machine -----------------------------------------------------
    cur = stage_of(d)
    cells = []
    for s in STAGES:
        if s == cur:
            col = {"FOLLOW": GREEN, "ADVANCE": YELLOW, "READ": RED, "COMMIT": ORANGE}[s]
            cells.append(c(BOLD, c(col, f"[{s}]")))
        else:
            cells.append(c(GREY, f" {s} "))
    out.append("  STATE:  " + c(GREY, " -> ").join(cells))
    if cur == "ADVANCE" and d.get("advance") is not None:
        out.append(f"          advancing {d.get('advance')}/{d.get('advance_target')} cm to read window")
    raw = d.get("state", "?")
    drive = c(GREEN, "ON") if d.get("drive") else c(RED, "off")
    light = d.get("light", "?")
    lcol = {"RED": RED, "GREEN": GREEN, "YELLOW": YELLOW, "IGN": GREY}.get(light, GREY)
    out.append(f"          {c(BOLD, raw):<28}  drive:{drive}   light:{c(lcol, light)}")
    out.append("")

    # --- zebra detection ---------------------------------------------------
    z = d.get("zebra")
    out.append(c(BOLD, "  ZEBRA / INTERSECTION"))
    if z is None:
        out.append("    " + c(GREY, "(detector off or no frame)"))
    elif not z.get("seen"):
        out.append("    " + c(GREY, "not seen") + f"   (dashes={z.get('ndash',0)})")
    else:
        dist = z.get("dist")
        dcol = RED if (dist is not None and dist <= d.get("params", {}).get("stop_cm", 10)) else YELLOW
        out.append("    " + c(dcol, c(BOLD, f"SEEN  dist={dist} cm")) +
                   f"   skew={z.get('angle')}°   dashes={z.get('ndash')}   span={z.get('span')}cm")
    opts = (z or {}).get("options") or d.get("options") or []
    exits = " ".join(c(GREEN, o.upper()) for o in opts) if opts else c(GREY, "—")
    out.append("    exits/options: " + exits)
    zdbg = (z or {}).get("debug") or {}
    if zdbg:
        parts = [f"{k}:{v}" for k, v in zdbg.items()]
        out.append("    why: " + c(GREY, " | ".join(parts)[:130]))
    sign = d.get("sign")
    pend = d.get("pending_turn")
    if sign or pend:
        s = c(YELLOW, sign.upper()) if sign else c(GREY, "—")
        p = c(GREEN, f"auto->{pend}") if pend else ""
        out.append(f"    SIGN: {s}  {p}")
    out.append("")

    # --- lane + command ----------------------------------------------------
    ln = d.get("lane", {})
    cmd = d.get("cmd", {})
    out.append(c(BOLD, "  LANE & COMMAND"))
    out.append(f"    src:{ln.get('src','?'):<6} off:{ln.get('off')}  conf:{ln.get('conf')}  "
               f"curv:{ln.get('curv')}")
    out.append(f"    v:{c(CYAN, cmd.get('v'))} m/s   w:{c(CYAN, cmd.get('w'))} rad/s")
    out.append("")

    # --- params to keep in mind -------------------------------------------
    p = d.get("params", {})
    out.append(c(BOLD, "  PARAMS (live)"))
    out.append(f"    zebra_bev:{p.get('use_zebra_bev')}  stop_cm:{p.get('stop_cm')}  "
               f"slow_cm:{p.get('slow_cm')}  approach_v:{p.get('approach_v')}")
    out.append(f"    kp:{p.get('kp')}  kd:{p.get('kd')}  ff:{p.get('ff_gain')}  "
               f"max_v:{p.get('max_v')}  max_w:{p.get('max_w')}")
    out.append("")

    # --- rolling log -------------------------------------------------------
    out.append(c(BOLD, "  LOG (state transitions / decisions)"))
    for line in log:
        out.append("    " + line)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5055)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.3)
    print(f"Dashboard listening on UDP :{args.port} ... (Ctrl-C to quit)")

    last = {}
    last_seen = 0.0
    log = deque(maxlen=12)
    prev_stage = None
    prev_opts = None
    try:
        while True:
            try:
                payload, _ = sock.recvfrom(65535)
                d = json.loads(payload.decode())
                last = d
                last_seen = time.time()
                st = stage_of(d)
                if st != prev_stage:
                    log.appendleft(f"t={d.get('t','?')}s  {prev_stage or '—'} -> "
                                   f"{c(BOLD, st)}")
                    prev_stage = st
                opts = tuple((d.get("zebra") or {}).get("options")
                             or d.get("options") or [])
                if opts != prev_opts and opts:
                    log.appendleft(f"t={d.get('t','?')}s  options: "
                                   + ",".join(opts))
                    prev_opts = opts
            except socket.timeout:
                pass
            except (ValueError, KeyError):
                continue
            # redraw
            print(ESC + "2J" + ESC + "H", end="")
            print(render(last, last_seen, log) if last else
                  "Waiting for telemetry... is the follower running with telemetry_host set?")
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
