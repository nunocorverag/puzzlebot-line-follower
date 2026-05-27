# Puzzlebot Line Follower Calibration Plan

This document is the working calibration plan for the current line follower in
`puzzlebot_ros/line_follower.py`. The goal is to make the robot robust on the
Manchester-style track with curves, intersections, dashed markings, start/finish
areas, and visible neighboring track segments.

## Current System

The current node does these things:

- Reads the Jetson CSI camera at 640x480 through `nvarguscamerasrc`.
- Optionally applies camera undistortion with `config/camera_params.npz`.
- Detects black line components through grayscale + Otsu thresholding.
- Uses two ROIs:
  - Bottom ROI: main tracking anchor.
  - Top ROI: lookahead / lane prediction.
- Tracks up to three top anchors: left, middle, right.
- Controls with PD using lateral pixel error.
- Detects dashed-line/intersection cues.
- Stops at an intersection and waits for a user decision on
  `/intersection_decision`.

Current important constants in `line_follower.py`:

| Parameter | Current | Meaning |
| --- | ---: | --- |
| `use_undistort` | `True` | Applies camera calibration before detection. |
| `max_jump_distance` | `80 px` | Max per-frame bottom candidate jump. |
| `anchor_max_jump` | `100 px` | Max per-frame top anchor jump. |
| `kp` | `0.003` | Proportional steering gain. |
| `kd` | `0.008` | Derivative steering gain. |
| `max_v` | `0.08 m/s` | Max forward velocity. |
| `max_w` | `0.6 rad/s` | Max angular velocity. |
| `intersection_frames` threshold | `4 frames` | Stable dashed detection before pausing. |
| `commit` duration | `1.0 s` | Short move after choosing left/straight/right. |
| `intersection_cooldown` | `3.0 s` | Avoids immediately retriggering. |

The code still has hardcoded values. A later step should expose the tuning
values as ROS parameters or YAML so calibration does not require editing code.

## Why The Latest False Positive Happened

In the shown frame the robot is on a normal straight lane, but the overlay says:

```text
INTERSECTION - WAITING
options: left, straight, right
dash:4 L:1 S:3 R:0
```

This is a false positive. The likely causes are:

1. The dashed detector accepts any small dark rectangular components in the
   upper/mid image, not only real dashed road markings.
2. The side option logic also uses black pixel ratios in large left/right boxes.
   Long solid borders and neighboring lanes can make those ratios pass.
3. The detector currently does not require dashed boxes to be arranged with
   regular spacing, consistent orientation, or expected distance from the robot.
4. The detector analyzes too high in the image, where the camera sees walls,
   chairs, neighboring lanes, and perspective distortion.

Professional fix: an intersection must be confirmed by a geometric pattern, not
just by total dark pixels. Real dashed markings should look like multiple similar
rectangles aligned in a lane-crossing band, with roughly consistent spacing and
inside the track surface area.

## Professional Calibration Philosophy

Do not tune everything at once. Freeze all but one subsystem per test. Use logs
and saved frames. A good calibration sequence is:

1. Camera image correctness.
2. Binary mask quality.
3. Line candidate quality.
4. Tracking continuity.
5. Controller behavior.
6. Intersection detection.
7. Decision execution.
8. Full route tests.

Each stage has a pass/fail metric. If a stage fails, do not tune later stages.

## Stage 0: Safety And Test Setup

Goal: make every calibration run repeatable and safe.

Required setup:

- Wheels lifted for first tests.
- Battery/motor power reachable.
- `scripts/stop_demo.sh` ready in a terminal.
- MJPEG open at `http://10.10.0.100:8080`.
- Prompt monitor open:

```bash
ssh puzzlebot@10.10.0.100
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/puzzlebot_ros/env_jetson.sh
ros2 topic echo /intersection_prompt
```

Decision terminal:

```bash
ssh puzzlebot@10.10.0.100
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/puzzlebot_ros/env_jetson.sh
ros2 topic pub --once /intersection_decision std_msgs/msg/String "{data: 'straight'}"
```

Pass criteria:

- `stop_demo.sh` reliably stops `/cmd_vel`.
- Prompt and decision topics work.
- MJPEG overlay is visible.

## Stage 1: Camera Calibration And Undistortion

Goal: remove enough lens distortion that pixel geometry is stable.

Current calibration source:

```text
config/camera_params.npz
```

It was copied from:

```text
mod2_computer_vision/Activities/activity_2_07/camera_params.npz
```

What to check:

- Straight track lines should look straighter after undistort.
- Undistortion should not crop the bottom ROI too aggressively.
- The center line in the image should remain close to the camera optical center.

Recommended tests:

1. Run with undistort enabled.
2. Run once with undistort disabled:

```bash
ros2 run puzzlebot_ros line_follower --ros-args -p use_undistort:=false
```

3. Compare saved screenshots of the same straight track segment.

Pass criteria:

- Undistorted image makes lane geometry more stable.
- No large black borders or unusable image areas appear.
- If undistort makes detection worse, disable it until a camera-specific
  calibration is recaptured from the same mounted camera pose.

Future improvement:

- Add bird's-eye perspective transform after undistortion. This should be done
  only after Stage 2 and Stage 3 are understood in the raw/undistorted view.

## Stage 2: Binary Mask Calibration

Goal: black track markings are detected cleanly, while beige track texture,
white floor, glare, socks, chair legs, and shadows are rejected.

Current method:

```text
grayscale -> GaussianBlur -> Otsu inverse threshold -> morphology open
```

What to inspect:

- Are black solid lane lines continuous?
- Are dashed marks separated as individual boxes?
- Are puzzle-piece seams detected as black lines?
- Is glare creating holes in black markings?

Current useful thresholds inside code:

| Filter | Current |
| --- | ---: |
| ROI candidate min area | `100 px^2` |
| dashed min area | `70 px^2` |
| dashed max area | `1800 px^2` |
| dashed rectangularity min | `0.42` |
| dashed max aspect | `6.5` |

Recommended starting values for your current false-positive case:

| Parameter | Current | Suggested next test |
| --- | ---: | ---: |
| dashed min area | `70` | `120` |
| dashed rectangularity min | `0.42` | `0.55` |
| dashed max aspect | `6.5` | `4.0` |
| dashed detection stable frames | `4` | `6-8` |

Pass criteria:

- On a normal straight segment, `dash` should stay below trigger level.
- On a real dashed/intersection zone, dashed boxes should appear clearly.
- The mask should not count long solid lane borders as dashed boxes.

## Stage 3: ROI Geometry Calibration

Goal: the robot should only consider image regions that are useful for the next
1-2 seconds of motion.

Current ROIs:

| ROI | Current x range | Current y range |
| --- | --- | --- |
| Bottom tracking | `25%-75%` | `60%-100%` |
| Top lookahead | `10%-90%` | `25%-50%` |
| Intersection ahead | `38%-62%` | `20%-54%` |
| Intersection left | `5%-42%` | `36%-70%` |
| Intersection right | `58%-95%` | `36%-70%` |

Problem:

The intersection boxes are too large and too high for the current camera view.
They can see neighboring lanes and background clutter. This explains the false
positive screenshot.

Recommended next test:

| ROI | Suggested x range | Suggested y range | Reason |
| --- | --- | --- | --- |
| Intersection ahead | `35%-65%` | `38%-68%` | Look closer to the robot. |
| Intersection left | `10%-38%` | `45%-72%` | Avoid far background and edge borders. |
| Intersection right | `62%-90%` | `45%-72%` | Avoid far background and edge borders. |

Better professional rule:

- Intersection detection should be evaluated in a band near the projected road
  plane, not the whole image.
- Side options should only be accepted if they connect to, or are near, the
  current bottom/middle lane estimate.

Pass criteria:

- Normal straight segments do not trigger intersection.
- Real dashed zones trigger within 0.2-0.5 s at slow speed.
- Side options are not reported unless a side branch is actually visible on the
  track surface.

## Stage 4: Line Candidate Scoring

Goal: select the correct line when neighboring lanes are visible.

Current behavior:

- Bottom ROI chooses a candidate by distance to reference and last known point.
- Top ROI tracks left/middle/right anchors.
- Steering uses bottom point plus 15% of top-bottom offset.

Weakness:

The robot still uses points, not a full local path model. It can jump to a
neighboring lane if that lane becomes the nearest valid blob.

Professional scoring should include:

| Feature | Desired behavior |
| --- | --- |
| Distance to last point | Penalize sudden jumps. |
| Distance to predicted point | Prefer continuity of motion. |
| Component orientation | Prefer line segments aligned with current heading. |
| Component area | Reject tiny noise and huge transverse blobs. |
| Connection to bottom ROI | Prefer candidates connected to near-field path. |
| Side-lane penalty | Penalize candidates far from the current path unless in intersection mode. |

Recommended next parameters:

| Parameter | Current | Suggested range |
| --- | ---: | ---: |
| `max_jump_distance` | `80 px` | `45-70 px` |
| `anchor_max_jump` | `100 px` | `60-90 px` |
| Top influence | `0.15` | `0.10-0.25` |

Pass criteria:

- On straight segments with neighboring visible tracks, the chosen bottom point
  remains on the current lane.
- On curves, the top point moves smoothly, not by sudden jumps.
- Candidate identity survives glare and small occlusions.

## Stage 5: Control Gain Calibration

Goal: make the robot follow smoothly without oscillation or late turns.

Current controller:

```text
omega = kp * lateral_error + kd * filtered_derivative
v = max_v * curve_factor
```

Current values:

| Parameter | Current |
| --- | ---: |
| `kp` | `0.003` |
| `kd` | `0.008` |
| `max_v` | `0.08` |
| `max_w` | `0.6` |

Calibration order:

1. Set `kd = 0.0` temporarily.
2. Tune `kp` until it follows straight lines and gentle curves without weaving.
3. Add `kd` only to reduce overshoot.
4. Increase `max_v` only after steering is stable.

Recommended starting tests:

| Scenario | Suggested values |
| --- | --- |
| First safe test | `max_v=0.04`, `max_w=0.35`, `kp=0.0025`, `kd=0.003` |
| Current normal | `max_v=0.08`, `max_w=0.6`, `kp=0.003`, `kd=0.008` |
| If oscillating | Lower `kp` or `kd`; test `kp=0.002`, `kd=0.003` |
| If turns too late | Raise top influence or slightly raise `kp` |
| If it overreacts to one bad frame | Lower `max_w` and increase candidate stability |

Pass criteria:

- On straight line, angular command settles near zero.
- On curves, robot turns before crossing the lane border.
- No repeated left-right oscillation.
- Robot never moves fast when confidence is low.

## Stage 6: Intersection Detection Calibration

Goal: detect real dashed/intersection zones and avoid false positives on normal
track segments.

Current trigger:

```text
dashed_detected = len(dashed) >= 4
                 or (center_dash >= 2 and left_dash + right_dash >= 2)

pause after 4 consecutive detected frames
```

Current option detection:

```text
left     if left_dash >= 2 or left_ratio > 0.035
straight if center_dash >= 2 or ahead_ratio > 0.030
right    if right_dash >= 2 or right_ratio > 0.035
```

Main issue:

The ratio fallback is too permissive. It can report left/right because solid
track borders are black inside the side boxes.

Recommended professional trigger rule:

A real intersection should require all of these:

1. At least `N` dash-like rectangles.
2. Rectangles must be inside a road-plane band, not the far background.
3. Rectangles must have similar size.
4. Rectangles must be aligned mostly horizontally or vertically depending on the
   expected marking orientation.
5. Rectangles must have regular spacing.
6. Detection must persist for `6-8` frames.

Recommended immediate parameter changes for next code pass:

| Parameter | Current | Next test |
| --- | ---: | ---: |
| Consecutive frames | `4` | `6` |
| `left_ratio` option threshold | `0.035` | Remove ratio fallback or raise to `0.08` |
| `right_ratio` option threshold | `0.035` | Remove ratio fallback or raise to `0.08` |
| `ahead_ratio` option threshold | `0.030` | Keep only as secondary evidence |
| Minimum dash count | `4` | `5-6` |
| Dash y band | `20%-68%` | `38%-72%` |

Preferred next implementation:

- Keep ratios only for debug.
- Make options depend primarily on dash clusters and path continuity.
- Add `dash_cluster_score`:

```text
score = count_score + alignment_score + spacing_score + size_consistency_score
```

Trigger only if `score >= threshold`.

Pass criteria:

- The screenshot false positive must not trigger.
- A true intersection with dashed markings must trigger consistently.
- Options should match reality: no `left` unless left branch/dashes are visible;
  no `right` unless right branch/dashes are visible.

## Stage 7: Human Decision And Commit Maneuver

Goal: after choosing a direction, the robot should enter that branch without
retriggering the same intersection immediately.

Current behavior:

| Direction | Commit command |
| --- | --- |
| `left` | `v=0.04`, `w=+0.25` for 1.0 s |
| `straight` | `v=0.04`, `w=0.0` for 1.0 s |
| `right` | `v=0.04`, `w=-0.25` for 1.0 s |
| Cooldown | `3.0 s` |

Recommended tuning:

| Scenario | Change |
| --- | --- |
| Does not enter branch enough | Increase commit duration to `1.3-1.6 s`. |
| Turns too sharply | Lower commit angular to `0.18-0.22`. |
| Retriggers same intersection | Increase cooldown to `4-5 s`. |
| Misses line after commit | Lower speed and rely on line reacquire. |

Pass criteria:

- After choosing `left`, it is visibly on the left branch before normal tracking resumes.
- After choosing `right`, it is visibly on the right branch before normal tracking resumes.
- After choosing `straight`, it clears the dashed zone and does not stop again.

## Stage 8: Start, Finish, And Dead-End Handling

Goal: distinguish a real end of track from a temporary lost line.

Current behavior:

- If line is lost, the robot can keep moving briefly.
- It does not yet classify finish/dead-end.

Needed behavior:

- If bottom and top line disappear after a known finish marker or dead-end zone,
  stop permanently.
- If line disappears briefly on glare or seam, continue slowly using last heading.
- If a side line is visible but not connected, do not jump to it.

Recommended future signals:

- `line_confidence`
- `lost_duration`
- `last_heading`
- `finish_candidate`
- `dead_end_candidate`

Pass criteria:

- Robot stops at final track end.
- Robot does not jump to a neighboring visible lane after finish.

## Stage 9: Data Collection And Offline Tuning

Goal: tune thresholds using saved frames instead of moving the robot every time.

Needed script:

- Save raw frame.
- Save undistorted frame.
- Save binary mask.
- Save overlay.
- Save JSON metadata:
  - bottom candidate
  - top candidate
  - dashed count
  - left/straight/right options
  - command output

Recommended dataset folders:

```text
debug_dataset/
  normal_straight/
  curve_left/
  curve_right/
  true_intersection/
  false_intersection/
  finish/
  side_lane_visible/
```

Pass criteria:

- At least 20 frames per class.
- False positives can be reproduced offline.
- Parameter changes can be tested without robot motion.

## Parameter Priority

Tune in this order:

1. `use_undistort` and camera params validity.
2. ROI y/x ranges.
3. Dash geometry filters.
4. Intersection trigger consecutive frames.
5. Candidate jump distances.
6. PD gains.
7. Commit maneuver.
8. Speed.

Do not start by increasing speed. Speed is the last parameter.

## Immediate Next Code Changes Recommended

The next code pass should do these, in order:

1. Expose all tuning constants as ROS parameters.
2. Add debug publishing or logging for:
   - `dashed_count`
   - `left_dash`, `center_dash`, `right_dash`
   - `left_ratio`, `ahead_ratio`, `right_ratio`
   - `intersection_frames`
   - selected options
3. Tighten intersection detection:
   - Analyze lower road-plane band only.
   - Remove side ratio fallback for option detection.
   - Require dash cluster alignment and spacing.
4. Add a frame capture key/topic/service for offline dataset collection.
5. Add a simple `calibration_mode` that never publishes nonzero `/cmd_vel`.

## Test Checklist

For each test segment, record:

```text
Date/time:
Undistort on/off:
Track segment:
Lighting:
Robot speed:
False intersection? yes/no
Missed intersection? yes/no
Options reported:
Actual correct options:
Line tracking stable? yes/no
Notes:
```

Minimum tests before increasing speed:

- Straight with no nearby side lane.
- Straight with nearby side lane visible.
- Left curve.
- Right curve.
- True intersection with left/straight/right.
- True intersection with only straight/right.
- Final/dead-end section.
- Start zone.

## Current Recommendation For The Screenshot

For the exact false positive shown, the next tuning change should not be PD or
camera calibration. It should be intersection detection:

1. Move intersection detection lower in the image.
2. Require at least 6 stable frames instead of 4.
3. Remove or heavily reduce `left_ratio` / `right_ratio` fallback.
4. Require at least 2 dash-like rectangles on a side before reporting that side.
5. Add a debug dataset entry under `false_intersection/`.

This should stop the robot from asking for an intersection while it is just on a
straight lane with other black lines visible in the scene.

## Calibration Tool Added

A standalone tool exists at:

```text
tools/line_vision_calibrator.py
```

Run it on the Jetson camera:

```bash
scripts/run_line_calibrator_jetson.sh
```

This is now the recommended first step before changing the live robot node. Use
it to tune intersection and mask parameters without publishing `/cmd_vel`.

Recommended workflow for the current false-positive issue:

1. Place the robot on a normal straight segment where it falsely reports an
   intersection.
2. Run `LABEL=false_intersection scripts/run_line_calibrator_jetson.sh`.
3. Adjust sliders until `status:normal` and dash evidence stays below trigger.
4. Press `s` to save raw/processed/mask/overlay/metadata.
5. Move to a real dashed intersection.
6. Run `LABEL=true_intersection scripts/run_line_calibrator_jetson.sh`.
7. Adjust only if true intersections still trigger reliably.
8. Copy the final slider values into `line_follower.py` or, preferably, into
   YAML/ROS parameters in the next implementation step.

The tool starts with stricter values than the current live node:

| Parameter | Calibrator start |
| --- | ---: |
| `roi_y0_pct` | `38` |
| `roi_y1_pct` | `72` |
| `dash_min_area` | `120` |
| `rectangularity_pct` | `55` |
| `max_aspect_x10` | `40` |
| `min_dash_count` | `5` |
| `stable_frames_needed` | `6` |
| `enable_ratio_fallback` | `0` |

These are intentionally conservative to reduce false positives.
