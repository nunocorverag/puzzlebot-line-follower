# Puzzlebot Line Follower - Handoff Context

Last updated: 2026-05-28

This document is the working memory for the Puzzlebot line follower project. It
captures what was built, what worked, what failed, the current calibration logic,
and what should happen next. Read this before changing the robot behavior.

## High-Level Goal

Make the Puzzlebot robust on the Manchester puzzle-track mat:

- Follow solid black lane lines.
- Detect zebra/dashed intersection markers.
- Stop at intersections.
- Infer available options: left, straight, right.
- Ask for a decision first; later execute decisions automatically or semi-automatically.
- Avoid false positives from neighboring track segments, wall/floor background,
  chair legs, people, shadows, glare, socks, and puzzle seams.
- Keep a laptop/Jetson hybrid workflow so code can be edited locally, synced to
  the Jetson, and tested quickly.

## Repo And Runtime Context

Local repo:

```text
/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower
```

Jetson target:

```text
puzzlebot@10.10.0.100
/home/puzzlebot/ros2_ws/src/puzzlebot_ros
```

GitHub repo:

```text
https://github.com/nunocorverag/puzzlebot-line-follower
```

Main branch:

```text
main
```

Important scripts:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_line_calibrator_jetson.sh
scripts/run_illumination_calibrator_jetson.sh
scripts/set_calibrator_param.sh
scripts/jog_forward_jetson.sh
scripts/run_line_follower_jetson.sh
scripts/stop_demo.sh
```

## Current Git State Warning

At the time this handoff was written there are intentional uncommitted changes:

```text
M  README.md
M  tools/line_vision_calibrator.py
?? docs/HANDOFF_CONTEXT.md
?? scripts/jog_forward_jetson.sh
```

Do not discard these. They contain the newest calibrator improvements:

- dynamic dash area threshold by visual distance,
- cleaner overlay,
- separate state panel,
- split option ROIs,
- straight jog command for real robot motion tests.

Before continuing, run:

```bash
git status --short
git diff -- tools/line_vision_calibrator.py
```

If the user confirms the behavior works, commit these changes.

## What Was Implemented

### 1. Laptop/Jetson Workflow

The project was brought from the Jetson into a local repo and set up for a
repeatable workflow:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_line_follower_jetson.sh
```

The scripts default to:

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws
```

This lets development happen locally while runtime happens on the Jetson.

### 2. Line Vision Calibrator

Created/expanded:

```text
tools/line_vision_calibrator.py
scripts/run_line_calibrator_jetson.sh
scripts/set_calibrator_param.sh
```

Purpose:

- Run camera vision without ROS control.
- Never publish `/cmd_vel`.
- Show processed image, mask, controls, and now state panel.
- Tune ROI and dash detection parameters live.
- Save debug samples with raw/processed/mask/overlay/metadata.
- Accept parameter commands from terminal or command file.

Run it:

```bash
scripts/run_line_calibrator_jetson.sh
```

Change live parameters from laptop:

```bash
scripts/set_calibrator_param.sh roi_y0_pct 72
scripts/set_calibrator_param.sh roi_y1_pct 88
scripts/set_calibrator_param.sh dash_min_area 40
scripts/set_calibrator_param.sh near_dash_min_area 700
scripts/set_calibrator_param.sh label true_intersection
```

Keys in the calibrator:

```text
s  save sample
u  toggle undistort
p  pause/resume
h  toggle state panel
q  quit
```

### 3. Dataset Capture Workflow

The calibrator can save labeled samples to `debug_dataset/` on the Jetson. We
used labels such as:

```text
normal_straight
true_intersection
false_intersection_background
finish_dead_end
```

There was a mistake where several samples were saved under the wrong label. We
manually corrected/deleted labels during the session. The lesson is: always set
`label` before pressing `s`, and verify the overlay shows the correct label.

Pull dataset from Jetson:

```bash
scripts/pull_calibration_dataset.sh
```

### 4. Camera Calibration

Camera calibration file:

```text
config/camera_params.npz
```

Source calibration images were replaced with the teacher/activity set from:

```text
/home/gnuno/Documents/Academic/Tec/sem_6/TE3002B_Implementation_of_Intelligent_Robotics/mod2_computer_vision/Activities/activity_2_07/calibration_images
```

They were copied into:

```text
puzzlebot_ros/calibration_images/calib_img_000.jpg ... calib_img_081.jpg
```

The calibration metadata observed on Jetson:

```text
successful_images=82
skipped_images=0
image_size=[640 480]
rms=2.2325
```

What worked:

- Using the better `camera_params.npz` improved corner/edge geometry.
- The track looked more stable after undistortion.

Caveat:

- If camera position or focus changes significantly, recalibrate or validate
  again. Undistortion depends on camera intrinsics, not lighting.

### 5. Illumination Flat-Field Calibration

Created/used:

```text
tools/illumination_calibrator.py
scripts/run_illumination_calibrator_jetson.sh
config/illumination_flatfield.npz
```

Purpose:

- Reduce the red/pink cast and uneven camera response.
- Improve Otsu/mask behavior under uneven lighting.

How to capture:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Point camera at the same white lona / uniform track-like material in the mounted
robot pose. Press `c` after exposure settles. Press `q` to exit.

What we learned:

- A wall is uniform, but the lona is more representative because it has the same
  texture/reflection as the actual track.
- The target should fill the frame as much as possible.
- It does not need to be perfectly geometrically straight, but it should be
  uniformly lit and not include strong shadows.
- Overhead light creates strong robot/cable shadows. Diffuse light or moving the
  robot/camera so the shadow is out of frame helps.

### 6. Intersection Detection Concept

Core idea:

- Zebra/dashed markers are the reliable cue that an intersection is approaching.
- When dashed markers are stable for several frames, the robot should stop and
  ask for a decision.
- For now, the system is still calibration-first and decision-assist, not fully
  autonomous route planning.

Initial issue:

- A single broad ROI caused false positives because it saw neighboring track,
  chair legs, walls, people, and incoming-lane zebras.

Improved approach:

- Use a low red entry/detection band to detect the incoming zebra.
- Once the entry zebra is found, compute an option ROI above it dynamically.
- Split the option ROI into left/straight/right regions.
- Only show/use the option ROIs when an intersection is stable.
- Keep the red entry band and option ROIs from overlapping.

### 7. Dynamic Dash Area Threshold

Problem observed:

- Distant zebra blocks are small and need low `dash_min_area`.
- Near zebra blocks are large; with a low area threshold, noise and tiny objects
  are accepted.
- With a high global threshold such as `700`, near zebras looked good, but far
  zebras disappeared.

Implemented in `tools/line_vision_calibrator.py`:

```text
dash_min_area        # far / normal minimum area
near_dash_min_area   # close zebra minimum area
dynamic_dash_area    # 0/1 enable
near_dash_y0_pct     # y-position where the threshold starts increasing
```

Current useful values:

```text
dash_min_area=40
near_dash_min_area=700
dynamic_dash_area=1
near_dash_y0_pct=72
```

Overlay state line shows:

```text
area:40->700
```

Meaning:

- Above `near_dash_y0_pct`, use low threshold.
- Below/near the robot, interpolate toward `near_dash_min_area`.

This worked well for the near-entry zebra.

### 8. Current ROI Defaults In The Calibrator

The latest calibrated defaults are intended to start close to what worked live:

```text
roi_y0_pct=72
roi_y1_pct=88
rectangularity_pct=25
option_x0_pct=4
option_x1_pct=96
entry_y0_pct=58
entry_margin_pct=10
dynamic_option_height_pct=22
split_option_rois=1
option_gap_pct=4
straight_option_width_pct=24
```

Interpretation:

- Red ROI: low entry/dash detection band.
- Orange line: detected entry zebra y-position.
- Option ROI: computed above the entry line.
- Split option ROIs: left / straight / right.
- State panel: shows counts, valid options, state, current ROI, area thresholds.

### 9. State Panel / Cleaner Overlay

Problem:

- The main overlay was saturated with text and ROI boxes.

Implemented:

- Main image shows only compact state text, centerline, active red entry ROI,
  detected boxes, and option ROIs when relevant.
- New `State` window shows details:

```text
STATE
label
paused
undistort
stable
dash
options
counts
valid
entry_y
entry_roi
option_roi
area
keys
jog command
```

This makes debugging easier without hiding the camera view.

### 10. Jog Forward Script

The calibrator intentionally does not publish `/cmd_vel`. To test ROI behavior
while the robot physically approaches the zebra, a separate jog script was added:

```text
scripts/jog_forward_jetson.sh
```

Usage:

```bash
scripts/jog_forward_jetson.sh 0.04 1.5
```

Meaning:

- Publish `/cmd_vel` with `linear.x=0.04` for `1.5` seconds.
- Then publish zero velocity.

Safer slow example:

```bash
scripts/jog_forward_jetson.sh 0.03 1.0
```

This was tested with speed `0.0` to verify ROS/SSH/quoting without moving the
robot. The user also ran:

```bash
scripts/jog_forward_jetson.sh 0.04 1.5
```

and it completed:

```text
Jogging /cmd_vel straight: speed=0.04 m/s duration=1.5s
Stopped /cmd_vel
```

Important safety:

- Keep `scripts/stop_demo.sh` ready.
- Use low speeds and short durations.
- Do not run the autonomous line follower and jog script at the same time unless
  you intentionally want competing `/cmd_vel` publishers.

## What Worked

### Camera/Illumination

- Loading `config/camera_params.npz` improved geometry.
- Loading `config/illumination_flatfield.npz` helped with uneven red/pink camera
  cast.
- Using the lona as illumination target is more representative than a wall.

### Detection

- Dashes/zebras are visible in the binary mask even when the overlay initially
  missed some boxes.
- Dynamic area threshold is the right direction: low threshold far, high
  threshold close.
- Low red ROI around `72-88%` is better than a large mid-frame red ROI.
- Entry-based dynamic option ROI is better than a fixed option ROI because the
  robot sees the zebra at different y positions as it approaches.
- Separating red entry detection from option detection reduced conceptual
  confusion.

### Workflow

- Live parameter updates through `set_calibrator_param.sh` work.
- Saving labeled samples works, but labels must be set carefully.
- Sync-to-Jetson workflow works.
- Jog script can move the robot forward for controlled visual testing.

## What Failed Or Was Confusing

### False Intersections From Background

Observed cases:

- A person/chair/object in the background triggered intersection-like masks.
- White/black background changes affected Otsu thresholding.
- Chair legs and track borders can appear as valid dark regions.

Current mitigation:

- Keep detection in a low road-surface ROI.
- Use dynamic area threshold.
- Do not rely only on black pixel ratios.

Still needed:

- Better road-surface mask / trapezoid mask.
- Better geometric validation of zebra patterns.

### Global Thresholds Were Not Enough

`dash_min_area=700` was good when the zebra was close but bad for far dashes.

Fix:

- Dynamic area threshold by y-position.

### ROI Overlap

The blue option ROI used to overlap the red entry ROI. That made the system
count incoming zebra blocks as options.

Fix direction:

- Red band detects the entry zebra.
- Option ROIs sit above the orange entry line with a margin.
- `entry_margin_pct` and `dynamic_option_height_pct` tune this.

### Labels In Dataset

Some samples were saved under the wrong label, especially when label changes
were forgotten.

Lesson:

- Always verify `label:` in overlay/state before pressing `s`.
- If labels are wrong, fix/delete before training/tuning from them.

### Bird's-Eye View Discussion

Bird's-eye transform was discussed. It is useful but was deferred because:

- The current raw/undistorted camera view still exposes important failure modes.
- Bird's-eye needs stable camera pose and homography points.
- A bad homography can hide bugs or introduce new geometry errors.

Professional path:

- First stabilize mask, entry ROI, and option validation.
- Then add optional bird's-eye for metric geometry.

## Key Concepts Learned

1. Intersections should be detected by pattern geometry, not just total dark
   pixels.
2. The same physical zebra changes apparent size as the robot approaches, so at
   least some thresholds should depend on y-position or inferred distance.
3. Separate perception states make debugging clearer:

```text
FOLLOW_LINE -> APPROACH_ENTRY -> READ_OPTIONS -> WAIT_DECISION -> COMMIT_DECISION -> FOLLOW_LINE
```

4. The incoming zebra is not the same thing as the available options. Counting
   it as an option creates false `straight`/wrong options.
5. A low entry ROI works well for knowing when the robot has reached the zebra.
6. Options should be read in a different ROI above the entry zebra.
7. For left/right options, diagonal alignment of zebra boxes matters.
8. Real track modules are puzzle pieces. The seams and printed connector shapes
   can produce black contours, so area/rectangularity alone is insufficient.
9. Lighting matters a lot. Shadows and red camera tint can change the mask more
   than the actual code change.
10. Keep movement tests separate from perception calibration for safety.

## Current Professional Design Direction

The robust architecture should become:

### State 1: Normal Line Follow

- Track solid lane line.
- Ignore option ROIs.
- Keep red entry ROI low.
- Monitor for stable zebra-like dashes.

### State 2: Approach Entry Zebra

- Triggered when red ROI sees stable zebra blocks.
- Slow down or stop.
- Estimate entry y-position from lower dash boxes.
- Move option ROIs above entry line.

### State 3: Read Options

- Show/use three option ROIs only now.
- Validate left/right by diagonal pattern.
- Validate straight by central repeated blocks or forward-lane evidence.
- Do not count incoming/lower zebra blocks as options.

### State 4: Wait Decision

- Publish zero `/cmd_vel`.
- Ask user for left/straight/right.
- Ignore impossible options unless user overrides.

### State 5: Commit Decision

- Move forward enough to clear the entry zebra.
- Turn if left/right.
- Use cooldown to avoid retriggering immediately.

### State 6: Reacquire Line

- Temporarily ignore very-low zebra remnants after a turn.
- Reacquire line with bottom/top tracking.
- Return to normal line follow.

## Current Gaps

### Calibrator Gaps

- The split option ROI logic is newly added and needs live validation.
- Left/right slope signs may need correction after real testing.
- Straight option validation is still simple count-based.
- The state panel is calibrator-only; the runtime node does not yet have the
  same polished state debug.
- The three ROI boxes are rectangular. A future version should follow expected
  diagonal bands more tightly.

### Runtime Node Gaps

`puzzlebot_ros/line_follower.py` still needs to be updated to match the newest
calibrator logic. Do not assume runtime behavior equals calibrator behavior.

Needed runtime port:

- Dynamic dash area threshold.
- Low entry ROI defaults.
- Entry-y-based option ROI.
- Split left/straight/right option ROIs.
- Geometric option validation.
- Cleaner debug overlay / state publication.
- Possibly ROS parameters/YAML for all tunables.

### Decision/Movement Gaps

- Jog script only moves straight for testing; it is not an autonomous state
  transition.
- Commit left/right maneuvers should be tested slowly after option detection is
  reliable.
- Need avoid duplicate `/cmd_vel` publishers during tests.

### Mask/Lighting Gaps

- Otsu thresholding still reacts to background and shadows.
- Need stronger road-surface mask or perspective/trapezoid mask.
- Could use HLS/Lab/color normalization after illumination correction.
- Could ignore objects above the road horizon more aggressively.

## Recommended Next Steps

### Immediate Next Test

1. Restart calibrator:

```bash
scripts/run_line_calibrator_jetson.sh
```

2. Place robot before an intersection, aligned with the lane.
3. Jog forward slowly:

```bash
scripts/jog_forward_jetson.sh 0.03 1.0
```

4. Watch:

- Red band stays low and catches entry zebra.
- Orange entry line tracks the lower zebra.
- Three option ROIs appear only after stable detection.
- State panel transitions from `FOLLOW_LINE` to `APPROACH_ENTRY` to
  `READ_OPTIONS`.
- Options are not shown from incoming/lower zebra blocks.

5. If option ROIs overlap red band, adjust:

```bash
scripts/set_calibrator_param.sh entry_margin_pct 12
scripts/set_calibrator_param.sh dynamic_option_height_pct 18
```

6. If near noise is accepted, adjust:

```bash
scripts/set_calibrator_param.sh near_dash_min_area 800
```

7. If far dashes are missed, adjust:

```bash
scripts/set_calibrator_param.sh dash_min_area 30
```

### Validate Split Option ROIs

Test these scenarios:

- True left + straight + right intersection.
- Left + straight only.
- Left + right only.
- Finish/dead-end area.
- Straight line with neighboring track visible.
- Person/chair/background visible.
- After left turn with zebra very low in the camera.

Save samples with correct labels for each.

### After Calibrator Works

Port the validated logic into:

```text
puzzlebot_ros/line_follower.py
```

Then test with wheels lifted first, then slow physical runs.

## Useful Commands

Run calibrator:

```bash
scripts/run_line_calibrator_jetson.sh
```

Set params live:

```bash
scripts/set_calibrator_param.sh roi_y0_pct 72
scripts/set_calibrator_param.sh roi_y1_pct 88
scripts/set_calibrator_param.sh dash_min_area 40
scripts/set_calibrator_param.sh near_dash_min_area 700
scripts/set_calibrator_param.sh near_dash_y0_pct 72
scripts/set_calibrator_param.sh dynamic_dash_area 1
scripts/set_calibrator_param.sh entry_margin_pct 10
scripts/set_calibrator_param.sh dynamic_option_height_pct 22
scripts/set_calibrator_param.sh option_gap_pct 4
scripts/set_calibrator_param.sh straight_option_width_pct 24
```

Move robot forward slowly for visual testing:

```bash
scripts/jog_forward_jetson.sh 0.03 1.0
scripts/jog_forward_jetson.sh 0.04 1.5
```

Stop robot:

```bash
scripts/stop_demo.sh
```

Sync local repo to Jetson:

```bash
scripts/sync_to_jetson.sh
```

Build on Jetson:

```bash
scripts/build_on_jetson.sh
```

Run autonomous line follower:

```bash
scripts/run_line_follower_jetson.sh
```

Pull debug samples:

```bash
scripts/pull_calibration_dataset.sh
```

Run illumination calibration:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

## Current Parameter Cheat Sheet

Good current starting point:

```text
roi_y0_pct=72
roi_y1_pct=88
dash_min_area=40
near_dash_min_area=700
dynamic_dash_area=1
near_dash_y0_pct=72
dash_max_area=2400
rect_pct=25
max_aspect_x10=60
min_dash_count=5
stable_frames=6
option_dash_count=2
option_x0_pct=4
option_x1_pct=96
dynamic_option_roi=1
entry_y0_pct=58
entry_margin_pct=10
dynamic_option_height_pct=22
split_option_rois=1
option_gap_pct=4
straight_option_width_pct=24
ratio_fallback=0
```

Do not treat these as final. They are the best live-tested starting point.

## Notes For The Next AI

- The user prefers practical, iterative testing over abstract planning.
- Speak Spanish.
- Do not reset or discard local changes.
- The user is actively testing on a real Jetson robot.
- Safety matters: keep `/cmd_vel` movement separate and short.
- The calibrator is the active playground; runtime node should only be changed
  after calibrator behavior is validated.
- The user wants professional robustness, not a brittle threshold hack.
- If asked to implement, inspect current repo state first because changes may be
  uncommitted.
- If adding behavior that moves the robot, make it opt-in and easy to stop.
- Avoid flooding the camera overlay with text; use separate debug/state windows.

## Terminology Used In The Session

- Zebra: dashed black rectangular crossing markers near intersections.
- Red ROI: low entry/dash detection band.
- Blue/option ROI: region used to infer left/straight/right choices.
- Orange line: estimated y-position of the incoming entry zebra.
- Split option ROIs: separate left, straight, and right regions above the entry
  zebra.
- Near dash area: larger minimum contour area for close zebra blocks.
- Far dash area: smaller minimum contour area for distant zebra blocks.

## Do Not Forget

The newest calibrator changes are not yet committed unless someone commits them
after this document. Before continuing work, confirm with:

```bash
git status --short
```

If behavior is good, commit:

```bash
git add README.md tools/line_vision_calibrator.py docs/HANDOFF_CONTEXT.md scripts/jog_forward_jetson.sh
git commit -m "Document line follower calibration handoff"
git push origin main
```
