# Lane following (bird's-eye) + track reference

How the robust line follower works, what we use vs. what we don't, the physical
track measurements, and how to mount/calibrate the camera. Companion to
[PERCEPTION_TUNING.md](PERCEPTION_TUNING.md) (intersection tuning) and
[SCRIPTS.md](SCRIPTS.md) (every script).

---

## 1. Track measurements (real, cm)

Measured on the Manchester puzzle-mat track. Used both for the bird's-eye warp
scale and to tune the intersection detector in real units instead of by eye.

| Measure | cm | Axis / meaning | Used for |
| --- | --- | --- | --- |
| Center & side line width | 2.2 | line stroke | warp X scale, line-width sanity |
| Lane width (edge of side line ↔ edge of center) | 11.8 | lateral (X) | **warp X scale (primary ruler)** |
| Dash, along travel | 2.2 | longitudinal (Y) | **warp Y scale** (measure a dash in warped px = 2.2 cm) |
| Dash, perpendicular (long side) | 3.15 | lateral (X) | dash area/aspect (≈6.9 cm², aspect ≈1.43) |
| Gap between dashes | 0.8 | — | zebra pattern |
| Intersection ↔ straight line | 4.85 | longitudinal | entry band position (`roi_y*`) |
| Intersection depth (entry↔exit dashed lines, same box) | 26.1 | longitudinal | option (left/straight/right) zone size |
| Between two separate intersections | 9.8 | longitudinal | avoid re-triggering on the next one |

**For the warp you do NOT need camera/wheel distances.** It is calibrated
visually (straight line → vertical) plus these two scales: 11.8 cm lateral and
2.2 cm dash longitudinal. The calibration frame must show a straight section
with a visible dashed line **and** the lane width.

---

## 2. What we have — and what we use vs. don't

### Line following

| Component | Status |
| --- | --- |
| `puzzlebot_ros/perception/lane.py` — bird's-eye warp + CLAHE/threshold mask + center-restricted sliding window + polynomial fit | **PRIMARY.** Used in normal following. |
| Legacy two-ROI detector (`detect_line_in_roi`, `force_middle_of_three`) in `line_follower.py` | **FALLBACK only.** Runs when `use_birdseye=false`, when the bird's-eye view is not confident (warp not tuned yet), and during an intersection approach. Same crude tracker as `archive/nodes/line_detector.py`. |
| Line-lost recovery (turn toward last-seen side) | Active in the fallback path (`recover_*` params). |

> Why the change: the legacy wide top ROI (10–90%) plus 3-lane tracker grabs
> parallel floor seams / off-lane lines and hijacks steering. The bird's-eye
> path rectifies the ground and restricts the line search to a band around the
> center, so distractor lines can't take over. It also yields a real curvature
> estimate to slow down smoothly on bends.

### Intersections

| Component | Status |
| --- | --- |
| `puzzlebot_ros/perception/intersection.py` — zone-based dash detection (entry band + left/straight/right ROIs), robust entry-line fit, centering gate | **USED.** Shared by the runtime and the line calibrator. |
| `config/intersection_params.json` (tuned via `save_calib`) | Loaded at startup; **falls back to built-in defaults if missing.** Verify it exists on the Jetson — if not, intersections run on defaults. |
| `debug_dataset/` (472 labeled frames: normal_straight/curve/side_lane_visible/true_intersection/finish_dead_end) | For **intersection** tuning + offline validation (run the calibrator with `--image`). Not used for line following. |

---

## 3. Bird's-eye pipeline & params (`LaneParams`)

1. **Warp** the ground to top-down via a homography from a symmetric trapezoid
   (`src_top_y_pct`, `src_top_half_w_pct`, `src_bot_y_pct`, `src_bot_half_w_pct`)
   to a `warp_w`×`warp_h` rectangle.
2. **Mask:** CLAHE (`use_clahe`, `clahe_clip_x10`, `clahe_grid`) then threshold —
   `mask_method` 0 = global Otsu (adapts to overall brightness), 1 = local
   adaptive (`adaptive_block`, `adaptive_c`, best under shadows/gradients).
3. **Sliding window:** histogram base restricted to ±`base_search_half_w_pct`
   around the center (the guard that ignores side lines), then `nwindows`
   windows of half-width `window_half_w_pct`, `min_pix` to recenter.
4. **Fit** `x = a·y² + b·y + c` → offset at `eval_y_pct` (steering) and
   curvature (speed). `min_windows_conf_pct` gates confidence; below it the
   follower falls back to the legacy path.

Runtime knobs in `line_follower.py`: `use_birdseye` (default true),
`curve_slow_gain`, `curve_min_scale`, `lane_params_path`. Optional metric output
via `px_per_cm_x10`.

Params persist to `config/lane_params.json` (same pattern as intersections).

---

## 4. Calibration workflow

The camera changed height, so redo the warp and flat-field. **Intrinsics
(`camera_params.npz`) stay valid** — height/tilt changes don't affect them, so
no checkerboard redo.

```bash
# 1. Focus (mount may have shifted it)
scripts/run_focus_assist_jetson.sh
# 2. New flat-field at the new pose
scripts/run_illumination_calibrator_jetson.sh
# 3. Tune the warp live until a straight line is vertical
scripts/run_warp_calibrator_jetson.sh
scripts/set_warp_param.sh src_top_y_pct 55       # adjust the 4 points...
scripts/set_warp_param.sh src_top_half_w_pct 14
scripts/set_warp_param.sh mask_method 1          # try adaptive under shadows
scripts/set_warp_param.sh save_lane 1            # -> config/lane_params.json
# 4. Build + run (node code changed)
scripts/sync_to_jetson.sh && scripts/build_on_jetson.sh
```

**Illumination robustness (profes change the lights):** capture a multi-lighting
dataset (normal / dim / side-lamp shadow) over straight + curve + intersection
with `CATEGORY=illumination scripts/run_recorder_jetson.sh` (watch the camera,
Enter = start/pause) while driving via `scripts/run_teleop_wasd_combo.sh` in
another terminal. The session lands clean in `datasets/illumination/<ts>/`. Then
validate the mask offline on each frame with `tools/warp_calibrator.py --image
<frame.jpg>`, comparing Otsu vs. adaptive until the line is clean under every light.

### Current calibration state (2026-06-04)

| File | Status |
|---|---|
| `config/camera_params.npz` | ✅ intrinsics (tilt-independent, no redo needed) |
| `config/camera_pose.json` | ✅ tilt setpoint **+11.1°** (re-leveling only; see §5) |
| `config/illumination_flatfield.npz` | ⚠️ captured 2026-06-01 — recapture at the final tilt if the mount moved |
| `config/lane_params.json` | 🟡 **starting** warp calibration — see below |
| `config/best.pt` | ✅ YOLO signs model |

**`config/lane_params.json` is a starting calibration, not final.** The trapezoid
**Y range (`src_top_y_pct=55`, `src_bot_y_pct=95`) is already aligned** to the
GROUND band of the +11.1° tilt, but the **half-widths (`src_top_half_w_pct=14`,
`src_bot_half_w_pct=42`) are design estimates** and must be verified live: run
`scripts/run_warp_calibrator_jetson.sh` and tweak them until a straight track
line looks **vertical** in the bird's-eye, then `set_warp_param.sh save_lane 1`.
Until then the runtime loads these values (no more "calibration not found"
fallback), but the warp geometry is not yet confirmed against the real mount.

### Dataset layout

All datasets live under `datasets/` (gitignored — keep originals in Drive/Roboflow):

```
datasets/
  signs/                     YOLO signs training set
  traffic_light/             traffic-light training set
  calibration/               calibration captures pulled from the Jetson
    tilt_session/            tilt sweep + tilt_log.jsonl (chosen +11.1°)
```

`scripts/pull_calibration_dataset.sh` rsyncs the Jetson's `debug_dataset/` into
`datasets/calibration/` (override with `LOCAL_DATASET=`). On the Jetson the tools
still write to `debug_dataset/`; that path only exists locally if a pull recreates
it, and stays gitignored.

### Live tuning & diagnostics (oscillation)

The follower streams a **status HUD** (top banner): current state
(`FOLLOW` / `APPROACH` / `WAIT` / `COMMIT` / `RECOVER` / `HOLD`), `drive` gate,
traffic light, lane `off`/`conf`/`curv`, the live commanded `v`/`w`, and the
active PD gains. Use it to see *why* the robot is doing what it does.

**Tune everything live — one interactive screen (recommended):**

```bash
scripts/run_param_tuner_jetson.sh    # curses TUI next to the running follower
```
Pick a field (`j`/`k`), nudge it (`-`/`=`), watch the live `/lane_status` metrics,
and press `s` to persist to `config/lane_params.json` + `config/control_params.json`
(so the values survive a restart). It tunes **both** the PD gains
(`kp`/`kd`/`max_v`/`max_w`/`curve_*`) and the warp params (`lane.*`, e.g.
`lane.eval_y_pct`). No ROS or X needed on the laptop. Saved gains auto-load next run.

Alternatives: `scripts/run_param_gui_jetson.sh` (official `rqt_reconfigure`
sliders via `ssh -X`); `scripts/set_gain_jetson.sh kp 0.0025` (one-shot, scriptable);
or plot `/lane_status` (`[off, conf, curv, v, w]`) live in PlotJuggler / Foxglove.

**If it follows then oscillates**, two levers (in order) — both live in the tuner:
1. **More lookahead** — lower `lane.eval_y_pct` (e.g. 88 → 70): reads the line
   further ahead, so corrections are gentler. (Applies instantly; `s` to keep it.)
2. **Soften the PD** — drop `kp` and/or raise `kd` until the wobble damps out on
   a straight.

Record a tuning trace with `CONTROLLER_LOG=1 scripts/run_line_follower_jetson.sh`
→ `puzzlebot_ros/controller_data.csv` (`t,state,off,conf,curv,error,deriv,v,w,kp,kd`).

**At an intersection** the robot stops in `WAIT`. Answer it (or bail out):

```bash
scripts/set_intersection_jetson.sh left      # left | right | straight
scripts/set_intersection_jetson.sh reset     # stuck? -> back to FOLLOW
```

### Robust intersection approach (geometry-agnostic)

The state machine is decoupled so that **curve→cross, straight→cross and double
crosses use the same path** (the Duckietown / pure-pursuit pattern):
`FOLLOW → slow-zone → APPROACH (center+align) → WAIT → COMMIT (open-loop turn) →
re-acquire → travel-guard`.

- Detection trigger (`entry_seen` in `intersection.py`) is **independent of
  centering**, so a skewed curve-exit still enters APPROACH; APPROACH then
  actively straightens (`k_align`·entry-slope) instead of *requiring* a centered
  arrival. The feedforward is relaxed + speed capped (`intersection_slow_speed`)
  once a zebra is seen, killing the overshoot into the cuadrito.
- The turn is **open-loop and tunable** (`commit_speed/turn_w/duration`); the
  bird's-eye follower re-acquires the branch afterwards. A distance proxy
  (`intersection_min_travel_m`, integrated commanded speed — no encoder needed)
  prevents a double intersection from re-firing the one just left.
- Full param list + on-robot tuning order: see [`RUNBOOK.md`](RUNBOOK.md) §5.

**Next step (not implemented):** a **topological map** of the track — a small
graph of intersections and their connections + a planned route — would let the
robot pick L/S/R automatically for *known* sequences instead of being told each
time. It plugs in at the WAIT→decision step.

---

## 5. Camera mounting & tilt

**The bird's-eye warp is valid for exactly ONE fixed camera pose (tilt + height).**
This drives every recommendation:

- **Mount rigidly at a single tilt.** No servo, no adjustable/loose bracket.
  The tall acrylic mount must not wobble — vibration drifts the warp.
- **You do NOT need to train/capture at −10° / +10° / etc.** The warp calibrator
  absorbs whatever the real mounted tilt is (you set the trapezoid to match it).
  Different tilts would each need their own homography, which is pointless with a
  fixed mount. Capture the illumination dataset **at the final fixed tilt only.**

**One camera does two jobs in two image regions** — this, not "point it down",
is what sets the tilt. The bird's-eye warp only uses the lower part of the frame
(the trapezoid lives at ~55–95% height), so the upper half is free for the
far field:

```
upper ~50%  -> traffic light, signs (stop), far curve preview   (image-space:
                                                                  HSV light + YOLO signs)
lower ~50%  -> ground: line + intersection dashes               (bird's-eye warp)
```

- **Recommended tilt: MODERATE — close to the current near-horizontal pose,
  maybe only slightly down.** Set it so the **ground/line fills the lower ~40–50%**
  (where the warp trapezoid sits) while **signs and the traffic light stay
  visible in the upper ~50%**. Do **not** point it steeply down: that maximizes
  IPM accuracy but blinds the robot to lights/signs and far curves, which it must
  react to.
- **The wide-angle lens is an asset here** — its large vertical FoV captures near
  ground *and* the far field in the same frame, so a single camera is enough.
- **Anticipate curves** from the bird's-eye curvature (within its lookahead); to
  see further, raise the warp reach (`src_top_y_pct` a bit higher) *without*
  eating into the sign region above it.
- **Keep undistort ON** (wide lens) — the warp calibrator already applies it.
- **Make the tilt repeatable:** mark the bracket / use a fixed screw position so a
  remount returns to the same pose without re-calibrating the warp.

### Tilt assistant (`tools/tilt_assistant.py`)

To *measure* the tilt while you physically adjust the camera, run the tilt
assistant. It estimates the **pitch in degrees (0 = horizontal)** from the
vanishing point of the two parallel track lines and the intrinsics
(`pitch = atan((cy - v_y)/fy)`), and overlays the horizon plus the ground/far
bands so you can confirm signs land in the upper band.

```bash
scripts/run_tilt_assistant_jetson.sh                       # live pitch + bands + score
CAMERA_HEIGHT_CM=13 scripts/run_tilt_assistant_jetson.sh   # also shows ~cm distances
scripts/set_tilt_param.sh save 1                           # -> config/camera_pose.json
```

Stand the robot on a **straight section** for a reading.

**Session capture + ranking.** Capture is **off until you arm it**, and it only
snaps when the camera is **held still at a new angle** (never mid-motion). So the
workflow is:

```bash
scripts/set_tilt_param.sh start 1     # arm capture when you're ready
# start near HORIZONTAL (reads ~0deg), then tilt DOWN in small steps, HOLDING
# ~1-2 s at each angle (the overlay shows "HELD" when it grabs one). Sweep
# ~0 -> 25-30 deg, pausing every ~3-5 deg. Each held pose = one clean snapshot.
scripts/set_tilt_param.sh mark 1      # or force a snapshot of the current pose
scripts/set_tilt_param.sh stop 1      # pause capturing
scripts/set_tilt_param.sh q 1         # quit -> prints the ranking
```

Snapshots (overlay + raw + `tilt_log.jsonl`) land in
`debug_dataset/tilt_session/`, each scored for the dual purpose (line in the
ground band, horizon above it for signs, moderate down-pitch). On quit it prints
a **ranking** and writes `tilt_summary.json`. Pull with
`scripts/pull_calibration_dataset.sh` to review the actual frames and confirm the
best, then `save` that pitch. (Defaults: holds for ~0.7 s within 0.8 deg count as
"still"; poses closer than 1.5 deg to the last are skipped.)

**Re-leveling (if the camera gets bumped).** Run with `RELEVEL=1`; it loads the
saved setpoint and shows a big **tilt UP / tilt DOWN** prompt with the live delta
until you are back within tolerance (border turns green = LEVEL OK):

```bash
RELEVEL=1 scripts/run_tilt_assistant_jetson.sh
```

`warp_calibrator.py` remains the final check: a straight line vertical in the
bird's-eye view means tilt + warp are right.

**Chosen setpoint (this robot).** Sweep on 2026-06-04 (`debug_dataset/jetson/tilt_session/`,
35 held poses, `tilt_log.jsonl`) settled on **pitch ≈ +11.1°** as the dual-purpose
sweet spot: signs + traffic light comfortably in the upper band, clean long lane
lines in the lower band, and forward preview retained. The 11–14° band is
equivalent in practice (lower = more preview, higher = bigger/more readable signs).
Saved in `config/camera_pose.json`; re-level to it with `RELEVEL=1`. Note: nothing
in the runtime reads `camera_pose.json` — it is only the re-leveling setpoint; the
actual lane geometry lives in the warp calibration.

---

## 6. Roadmap (NOT implemented yet — design notes)

Captured here so the vision pipeline above leaves room for them:

- **Traffic signs** (YOLO `best.pt`, `tools/sign_detector.py`): classes to handle
  — `stop`, `turn_right`, `turn_left`, `go_straight`, `workers`, `give_way`.
  Detected in the **upper (far) band**; trigger an action as they grow/approach.
- **Parking**: enter a parking box, stop, and later exit back onto the lane.
- **Dead-end detection**: recognize a `finish_dead_end` pattern (already a
  dataset label) and stop / turn around.
- These behaviors will sit on top of the existing state machine
  (`FOLLOW_LINE` → intersection `APPROACH`/`WAIT`/`COMMIT`); each adds a state.
