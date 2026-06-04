# Line Follower — Run & Tune Runbook

Quick, copy-paste guide to drive and tune the Puzzlebot line follower.
All commands run from the repo root on the **laptop**; each script SSHes to the
Jetson (`puzzlebot@10.10.0.100`) by itself.

> Golden rule: **one follower at a time.** `run_line_follower_jetson.sh` now kills
> any previous instance before launching, and `Ctrl-C` stops the remote node
> (it uses `ssh -t`). If anything still misbehaves, `scripts/stop_demo.sh` nukes
> everything (see [Cleanup](#cleanup)).

---

## 0. Build (only after changing node code)

```bash
scripts/sync_to_jetson.sh && scripts/build_on_jetson.sh
```

The interactive tuner (`tools/param_tuner.py`) is a standalone script — it does
**not** need a build, only a sync (its run script syncs automatically).

---

## 1. Startup (4 terminals)

| Terminal | Command | Stays open? |
|---|---|---|
| 1 — motors | `scripts/run_motor_agent_jetson.sh` | yes |
| 2 — follower + video | `IGNORE_TRAFFIC_LIGHT=1 scripts/run_line_follower_jetson.sh` | yes (opens the video) |
| 3 — tuner | `scripts/run_param_tuner_jetson.sh` | yes |
| 4 — one-shot commands | `scripts/set_drive_jetson.sh on` … | no (free terminal) |

Order: **build → T1 → T2 → (wheels up) T4 `set_drive on` → T3 tune while watching T2.**

**Sanity checks**
- T2 video must show the **HUD bar** on top (`FOLLOW … drive:off … kp:0.0030 …`).
  No bar = old build is running → rebuild (step 0).
- T3 bottom line must say **`connected to /autonomous_racer`**.
  If `not found` = the follower (T2) isn't running yet.

---

## 2. Drive control (Terminal 4)

```bash
scripts/set_drive_jetson.sh on      # allow motion (wheels up first!)
scripts/set_drive_jetson.sh off     # stop / hold
```

At an intersection (HUD shows `WAIT`):

```bash
scripts/set_intersection_jetson.sh left      # left | right | straight
scripts/set_intersection_jetson.sh reset     # bail out -> back to FOLLOW
```

---

## 3. Tuning (Terminal 3 tuner)

Keys: `j`/`k` select · `-`/`=` (or ←/→) nudge · `s` save to JSON · `q` quit.
Also (instant, published from the running tuner node — no `set_*.sh` lag):
`d` drive on/off · `1`/`2`/`3` left/straight/right at a cross · `0` reset cross ·
`r` toggle snapshot recording.
Prefer these over the Terminal-4 scripts while tuning — the scripts pay a ~1–2 s
DDS-discovery cost on every call; the tuner is already connected.

**Snapshots for review:** `r` records the annotated frame every ~2 s into
`debug_dataset/follower_session/` on the Jetson (HUD shows `REC n`). Pull them to
the laptop with `scripts/pull_follower_snapshots.sh` → `datasets/follower_session/`.

**Quiet console:** the follower runs with `verbose:=false` by default (no per-frame
log spam; the video HUD is the live state display). Add `-p verbose:=true` to debug.

Tip: set **`max_v` to your real run speed first** — PD gains depend on speed.
Change **one thing at a time**, small steps, watch the video.

| Robot behavior | Change | Direction |
|---|---|---|
| Zigzags / oscillates on a straight | `lane.eval_y_pct` (first) | **lower** 88→78→70 (more lookahead) |
| …still oscillates | `kp` ↓ and/or `kd` ↑ | less P, more damping |
| Reacts late / cuts or runs wide in a curve | `lane.eval_y_pct` ↓ or `kp` ↑ | more anticipation / more push |
| Too slow | `max_v` | raise |
| Too fast / overshoots | `max_v` | lower |
| Turns jerky ("yanks the wheel") | `max_w` | lower |
| Can't make a tight curve | `max_w` | raise |
| Doesn't slow in curves (enters fast, runs off) | `curve_slow_gain` | raise |
| Brakes too much / stalls in a curve | `curve_slow_gain` ↓ or `curve_min_scale` ↑ | |
| Locks onto a wrong / side line | `lane.base_search_half_w_pct` | lower (search nearer center) |

**Reading the `live` HUD line:**
- `off` — line offset (`-` left, `+` right). On a straight, well tuned → stays near **0** without wobbling.
- `conf` — detection confidence (want ~1.00).
- `curv` — detected curvature (≈0 on a straight).
- `w` — commanded turn rate; if `w` flips `+/-` fast on a straight → `kp` too high.

**Typical recipe:** wheels down, `drive on`, on a **straight** → tune until `off`≈0
and `w` is steady → then test a **curve**, adjust `max_w` / `curve_slow_gain` →
press `s` to save. Saved values auto-load next run.

---

## Cleanup

```bash
scripts/stop_demo.sh     # kills all Jetson nodes + local viewers, zeroes /cmd_vel
```

Use this if the robot misbehaves, if you launched the follower twice, or to shut
down at the end. To check what's running on the Jetson:

```bash
ssh puzzlebot@10.10.0.100 "pgrep -af 'line_follower|micro_ros_agent|param_tuner'"
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| No HUD bar in the video | Old build running → `sync_to_jetson.sh && build_on_jetson.sh`. |
| Tuner says `rclpy too old` / `not found` | Re-sync the tuner (relaunch `run_param_tuner_jetson.sh`); make sure the follower is running. |
| Wheels don't move with `drive on` | Motor agent (T1) not running, or robot/board power off. |
| Robot stuck in `WAIT` on a straight | False intersection (puzzle-floor seams look like zebra). `set_intersection_jetson.sh reset`. |
| Weird/contradictory motion | Multiple followers running → `stop_demo.sh`, then start ONE. |

---

## File map

- Tilt setpoint: `config/camera_pose.json` (re-level with `RELEVEL=1 scripts/run_tilt_assistant_jetson.sh`).
- Warp + lane tuning: `config/lane_params.json`.
- PD gains: `config/control_params.json`.
- Full design notes: [`docs/LANE_FOLLOWING.md`](LANE_FOLLOWING.md). Script reference: [`docs/SCRIPTS.md`](SCRIPTS.md).
