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

**Snapshots for review:** `r` records the annotated frame (now with the
**bird's-eye debug panel glued on the side**: mask + sliding windows + fit) every
~2 s on the Jetson (HUD shows `REC n`). When you **Ctrl-C the follower** they are
pulled to the laptop automatically into `datasets/follower_session/<session>/` and
wiped on the Jetson — no manual pull needed. (`scripts/pull_follower_snapshots.sh`
still works for a mid-session pull.)

**Quiet console:** the follower runs with `verbose:=false` by default (no per-frame
log spam; the video HUD is the live state display). Add `-p verbose:=true` to debug.

Tip: set **`max_v` to your real run speed first** — PD gains depend on speed.
Change **one thing at a time**, small steps, watch the video.

| Robot behavior | Change | Direction |
|---|---|---|
| Zigzags / oscillates on a straight | `lane.eval_y_pct` (first) | **lower** 88→78→70 (more lookahead) |
| …still oscillates | `kp` ↓ and/or `kd` ↑ | less P, more damping |
| **Runs wide / cuts the curve (rides the outer edge)** | **`ff_gain`** (curve feedforward) | **raise** 1.0→1.5→2.0 (anticipate the bend) |
| …feedforward over-steers / wobbles | `ff_gain` ↓ or `lane.lookahead_y_pct` ↑ | less / read the bend nearer |
| Reacts late in a curve | `lane.eval_y_pct` ↓ or `kp` ↑ | more anticipation / more push |
| Too slow | `max_v` | raise |
| Too fast / overshoots | `max_v` | lower |
| Turns jerky ("yanks the wheel") | `max_w` | lower |
| Can't make a tight curve (see physics below) | `max_w` ↑ **or** `max_v` ↓ | enough turn rate for the radius |
| Doesn't slow in curves (enters fast, runs off) | `curve_slow_gain` | raise |
| Brakes too much / stalls in a curve | `curve_slow_gain` ↓ or `curve_min_scale` ↑ | |
| Locks onto a wrong / side line | `lane.base_search_half_w_pct` | lower (search nearer center) |

**Reading the `live` HUD line:**
- `off` — line offset (`-` left, `+` right). On a straight, well tuned → stays near **0** without wobbling.
- `conf` — detection confidence (want ~1.00).
- `curv` — detected curvature (≈0 on a straight).
- `ff` — curve feedforward gain (gains row). 0 = pure feedback (old behavior).
- `w` — commanded turn rate; if `w` flips `+/-` fast on a straight → `kp` too high.
  **If `w` pins at `±max_w` (e.g. `±0.60`) through a curve → it is turn-rate
  SATURATED (see physics below), not a vision problem.**

**Typical recipe:** wheels down, `drive on`, on a **straight** → tune until `off`≈0
and `w` is steady → then test a **curve**, raise `ff_gain` until it tracks the bend,
adjust `max_w` / `curve_slow_gain` → press `s` to save. Saved values auto-load next run.

### Curves: feedforward + the physics of "it runs off the curve"

Two independent things make a robot leave a curve:

1. **Reacts late (vision/control).** Pure-P steers to where the line is *now*, not
   where it is *going*, so it rides the outer edge. Fix: the **`ff_gain`**
   feedforward (steers ahead by the bend = far offset − near offset) and more
   lookahead (`lane.eval_y_pct` ↓). The bird's-eye warp itself needs **no** special
   "curve" tuning — it flattens the ground and the polynomial fit handles a curved
   line directly.

2. **Can't physically turn fast enough (geometry).** The tightest radius the robot
   can follow is `R_min = v / max_w`. With `v = 0.08`, `max_w = 0.60` →
   **R_min ≈ 13.3 cm.** If a curve is tighter than that, it *cannot* make it at that
   speed no matter how good the vision — it runs wide. Tell: `w` saturates at
   `±max_w`. Fix: **slow down** (lower `max_v`, or raise `curve_slow_gain` so it
   brakes into bends) and/or **raise `max_w`**.

   | v (m/s) | R_min @ max_w=0.6 |  | to make R=10 cm |
   |---|---|---|---|
   | 0.08 | 13.3 cm |  | need `max_w ≥ 0.80` or `v ≤ 0.06` |
   | 0.06 | 10.0 cm |  | |
   | 0.05 | 8.3 cm |  | |

   Measure the **radius of your tightest curve** (center of the turn → middle of
   the black line, perpendicular to it) and pick `v`/`max_w` so `R_min` ≤ that radius.

---

## 4. Recording datasets (e.g. illumination)

Every recorder **auto-pulls its session to the laptop on exit and wipes the
Jetson** — no manual pull. Each run lands in its own timestamped folder.

| What | Command | Lands in (on Ctrl-C) |
|---|---|---|
| Clean frames + see the camera + start/pause with **Enter** | `CATEGORY=illumination scripts/run_recorder_jetson.sh` | `datasets/illumination/<ts>/` |
| Follower snapshots (with BEV panel), tuner `r` | `…run_line_follower_jetson.sh` | `datasets/follower_session/<ts>/` |
| Tilt calibration snapshots | `scripts/run_tilt_assistant_jetson.sh` | `datasets/calibration/tilt_session/<ts>/` |

To **drive while recording illumination**: run `scripts/run_recorder_jetson.sh`
(watch the camera, Enter = record) in one terminal and
`scripts/run_teleop_wasd_combo.sh` in another (snappy WASD). The recorder owns the
camera; the WASD bridge only touches `/cmd_vel`, so they coexist. Saved frames are
**clean** (no overlay); the preview just shows RECORDING/PAUSED.

---

## 5. Intersections (robust: curve→cross, straight→cross, doubles)

Current machine (vision-anchored): **FOLLOW → DETECT (zebra ≤ `detect_distance_cm`)
→ ADVANCE (go straight to the cross, stop on the FIRST row by vision) → READ
(square-up if skewed, then wait for the decision) → COMMIT (cross by time) →
re-acquire lane → travel guard**. Key points:

- **DETECT → ADVANCE** fires when a zebra is seen within `detect_distance_cm`.
  ADVANCE goes **straight** (`advance_center_gain` 0, no lane steering, so it
  doesn't veer into the side dashes) at `approach_speed`.
- ADVANCE **stops on the first cross row by VISION** — a jump in the zebra
  distance (`read_cross_jump_cm`) or `dist ≤ read_distance_cm` — **not** by odom
  (the command overestimates real distance; see the handoff). No square-up if it
  arrived straight (`align_skip_when_straight`).
- The **turn is open-loop** (`commit_turn_w` / `commit_duration`, straight uses
  `commit_duration_straight`), with a minimum (`commit_min_s` /
  `commit_straight_min_s`) so it fully **crosses the intersection and re-acquires
  the continuing black line** (does not follow the dashes). A distance guard
  (`intersection_min_travel_m`) stops a double cross from re-firing.

Send the decision (or use the tuner / control panel keys `1/2/3` = L/S/R, `0` = reset):
```bash
scripts/set_intersection_jetson.sh left      # left | right | straight | reset
```

**Params to tune on the robot** (live, no rebuild — defaults in parens):

| Phase | Param | What |
|---|---|---|
| DETECT | `detect_distance_cm` (22) | zebra distance that triggers ADVANCE |
| ADVANCE | `approach_speed` (0.06) | straight crawl to the cross (keep > ~0.08 cmd deadband in mind) |
| ADVANCE | `advance_center_gain` (0) | lane steering during advance — keep 0 (go straight) |
| ADVANCE→READ | `read_distance_cm` (6) | stop when the cross row is this close |
| ADVANCE→READ | `read_cross_jump_cm` (8) | stop on a distance jump (crossing the first row) |
| READ | `align_in_place` (off) / `align_skip_when_straight` (on) | square-up the heading, but skip it if it arrived straight |
| COMMIT | `commit_turn_w` (0.6) + `commit_duration` (3.5 s) | the ~90° L/R turn |
| COMMIT | `commit_duration_straight` (6 s) | go-straight maneuver length |
| COMMIT | `commit_min_s` (2) / `commit_straight_min_s` (4.5) | minimum cross time so it clears the cross before re-acquiring |
| guard | `intersection_min_travel_m` (0.25) | gap before the next cross can fire (doubles) |

> Tune `commit_turn_w`/`commit_duration` first on a single cross until L/R land on
> the exit lane, then raise `commit_*_min_s` until it always clears the dashes and
> re-locks the continuing line, then test a double. Curve→cross arrives skewed —
> `align_in_place` helps but is still imperfect (see the handoff open issues).

Future (not yet implemented): a **topological map** of the track (graph of crosses +
route) for known sequences — see `docs/LANE_FOLLOWING.md`. Full behaviour & open
issues: [`docs/HANDOFF_2026-06-08.md`](HANDOFF_2026-06-08.md).

---

## 6. Traffic signs (YOLO) & traffic light

Both are **opt-in and non-blocking** — they never break line following.

**Signs** (off by default) — enable with `USE_SIGNS=1`:

```bash
USE_SIGNS=1 scripts/run_demo_tmux.sh        # or USE_SIGNS=1 scripts/run_line_follower_jetson.sh
```

- `trabajadores` → slow (`workers_speed_factor`) for `workers_slow_s`.
- `stop`/`give-way` → halt `stop_seconds`/`giveway_seconds`, **only when close**
  (`area_pct ≥ sign_act_area_pct`).
- arrows / `straight` → latch the next cross decision (manual `1/2/3` overrides).
- `USE_SIGNS=1` preloads torch's libgomp and keeps user-site so ultralytics
  imports on the Jetson; the log line `[signs] model loaded` confirms it.
- `best.pt` classes are Spanish; the detector maps them by substring.

**Traffic light** — optional by default (`traffic_light_optional`): the robot
drives without waiting for green and only obeys a RED/YELLOW it actually sees,
validated by circular shape on the gray screen/plate. To require a GREEN before
moving: `TRAFFIC_LIGHT_OPTIONAL=0`. To ignore the light entirely (testing):
`IGNORE_TRAFFIC_LIGHT=1`.

> Known issue: a red reflection near the screen can be marked as a false RED
> (debounce protects behaviour). Robust fix (classify by disc position on the
> screen) is pending — see the handoff.

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
| Signs not detected / no `[signs] model loaded` | Launch with `USE_SIGNS=1`; check `config/best.pt` exists and ultralytics imports (`python3 -c "import ultralytics"` on the Jetson). |
| Robot stops for no light | False RED reflection near the screen; raise `traffic_light_plate_max_sat` strictness or run with `IGNORE_TRAFFIC_LIGHT=1` to confirm. |

---

## File map

- Tilt setpoint: `config/camera_pose.json` (re-level with `RELEVEL=1 scripts/run_tilt_assistant_jetson.sh`).
- Warp + lane tuning: `config/lane_params.json` (incl. `eval_y_pct`, `lookahead_y_pct`).
- PD + curve gains: `config/control_params.json` (`kp`, `kd`, `ff_gain`, `max_v`, `max_w`, `curve_slow_gain`, `curve_min_scale`).
- Full design notes: [`docs/LANE_FOLLOWING.md`](LANE_FOLLOWING.md). Script reference: [`docs/SCRIPTS.md`](SCRIPTS.md).
