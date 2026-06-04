# Perception and calibration - current flow

This guide summarizes the stack used to calibrate, test and tune vision on the
Puzzlebot. The general idea is: direct CSI camera over GStreamer, fast H264
preview, and a single source of truth for geometry, illumination and
intersection detection.

> For the bird's-eye **lane follower** (the robust primary line-following path),
> its warp calibration, the track measurements and the camera tilt
> recommendation, see **[LANE_FOLLOWING.md](LANE_FOLLOWING.md)**.

---

## Recommended order

1. **Lens focus**

```bash
scripts/run_focus_assist_jetson.sh
```

Point at a textured target at the working distance and turn the lens until the
focus value is maximized. Do this before calibrating and do not touch the focus
again afterwards.

2. **Intrinsics with the checkerboard**

```bash
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
```

Result validated in this session:

```text
30/30 images detected
RMS reprojection error: 0.3449 px
Verdict: EXCELLENT
pattern: 5x7
image_size: 640x480
```

Generated, version-controllable file:

```text
config/camera_params.npz
```

3. **Illumination flat-field**

```bash
scripts/run_illumination_calibrator_jetson.sh
```

It now waits for you to press `Enter`: position the white banner watching the
H264, press `Enter`, there is a pause to remove hands/shadows, and then it
collects the good frames automatically.

Result validated in this session:

```text
std BGR before: 3.4 4.4 14.5
std BGR after : 0.6 0.6 0.8
```

Generated, version-controllable file:

```text
config/illumination_flatfield.npz
```

4. **Test Otsu, mask and ROIs**

```bash
scripts/run_line_calibrator_jetson.sh
```

By default it opens a fast H264 dashboard with:

- processed view with overlay
- Otsu mask
- compact debug panel

To go back to the old mode with OpenCV trackbars:

```bash
STREAM=local scripts/run_line_calibrator_jetson.sh
```

On WSL/X11, keep H264 but choose the X11 sink explicitly:

```bash
VIDEO_SINK=ximagesink scripts/run_line_calibrator_jetson.sh
```

Native Ubuntu can keep the default `autovideosink`.

---

## How to test the camera alone

Raw camera with no calibrations:

```bash
scripts/run_camera_h264_jetson.sh
```

Preview with calibrations applied, without moving the robot:

```bash
scripts/run_recorder_jetson.sh
```

If you do not press `Enter`, it stays as a preview. This path loads
`camera_params.npz` and `illumination_flatfield.npz`.

---

## Otsu and black mask

The line/intersection mask lives in:

```text
puzzlebot_ros/perception/intersection.py
```

The key function is `black_mask(frame)`:

```text
BGR -> grayscale -> Gaussian blur -> Otsu inverse threshold -> morphology open
```

In the H264 dashboard, the right half shows that mask. What to expect:

- the track's black line and dashes show up white in the mask
- the light background shows up black
- shadows, a chair, cables and the lab background should not dominate the mask

If there is a lot of noise, adjust the detector filters before touching the Otsu
algorithm.

---

## H264 dashboard of the line calibrator

Main command:

```bash
scripts/run_line_calibrator_jetson.sh
```

In H264 there are no real sliders because H264 only transmits video. Changes are
made from the terminal or from another shell with `set_calibrator_param.sh`.

Commands inside the calibrator terminal:

```text
min_dash_count=6
roi_y0_pct=72
roi_skew=8
s=1
p=1
u=1
q=1
```

Commands from another terminal:

```bash
scripts/set_calibrator_param.sh min_dash_count 6
scripts/set_calibrator_param.sh roi_y0_pct 72
scripts/set_calibrator_param.sh roi_skew 8
scripts/set_calibrator_param.sh label sample
scripts/set_calibrator_param.sh s 1
scripts/set_calibrator_param.sh u 1
scripts/set_calibrator_param.sh p 1
scripts/set_calibrator_param.sh q 1
```

Special commands:

```text
s=1  save raw/processed/mask/overlay + JSON
p=1  pause/resume
u=1  toggle undistort
q=1  quit
```

---

## Intersection ROIs

Detection has two levels:

1. **Intersection entry**: low red band. Only this zone triggers the
   `READ_OPTIONS` state.
2. **Options left/straight/right**: translucent upper polygons. These only
   classify which way you can go.

The option ROIs are no longer fixed rectangles for counting; they are now
polygons with adjustable skew to better follow the perspective.

Useful parameters:

```text
roi_y0_pct / roi_y1_pct              red entry band
dynamic_option_roi                   1 = places options based on the detected entry
entry_margin_pct                     vertical gap between entry and options
dynamic_option_height_pct            height of the options zone
option_gap_pct                       gap between left/straight/right
straight_option_width_pct            width of the center ROI
option_roi_skew_pct                  diagonal of the polygons
roi_skew                             alias of option_roi_skew_pct
rect_pct                             minimum dash rectangularity
max_aspect_x10                       maximum allowed dash shape
min_dash_count                       how many real dashes trigger an intersection
stable_frames                        how many consecutive frames are required
```

Starting values we used while exploring:

```bash
scripts/set_calibrator_param.sh roi_skew 8
scripts/set_calibrator_param.sh entry_margin_pct 15
scripts/set_calibrator_param.sh dynamic_option_height_pct 32
scripts/set_calibrator_param.sh option_gap_pct 6
scripts/set_calibrator_param.sh straight_option_width_pct 20
scripts/set_calibrator_param.sh rect_pct 35
scripts/set_calibrator_param.sh max_aspect_x10 45
```

What to look for visually:

- `dash` should be 6 if the real intersection entry has 6 dashes.
- Detected dashes are marked with translucent color.
- The side ROIs should cover the left/right diagonals without grabbing
  background.
- The center ROI should cover the straight-ahead branch.
- If a puzzle-piece track tab turns into a false dash, raise `rect_pct` or lower
  `max_aspect_x10` before changing the ROI.

---

## Parking at an intersection for diagonal ROI tuning

The calibrator never drives the robot. To tune the diagonal option ROIs, first let
the follower drive to the intersection and center the bottom ROI on the detected
entry center:

```bash
# Terminal 1: motor bridge
scripts/run_motor_agent_jetson.sh

# Terminal 2: follower without traffic-light gating
IGNORE_TRAFFIC_LIGHT=1 scripts/run_line_follower_jetson.sh

# Terminal 3: allow motion, then halt when it is centered/waiting
scripts/set_drive_jetson.sh on
scripts/set_drive_jetson.sh off
```

Then free the camera and open the calibrator from that parked pose:

```bash
scripts/stop_demo.sh
LABEL=roi_diagonal_debug scripts/run_line_calibrator_jetson.sh
scripts/set_calibrator_param.sh s 1
scripts/pull_calibration_dataset.sh
```

Use `VIDEO_SINK=ximagesink` with the calibrator command on WSL. The follower
uses `entry_center_x` during `APPROACH_CENTER`; if it is drifting toward a side
branch, tune the entry band and dash filters before touching the controller.

## Files that DO get committed

After calibrating:

```bash
git add config/camera_params.npz config/illumination_flatfield.npz
git commit -m "Recalibrate camera and illumination"
```

It is also worth committing the code/doc changes for the H264 flow and tuning.

Raw captures and previews are NOT committed:

```text
calibration_images/
config/undistorted_preview.jpg
config/illumination_preview.jpg
```

---

## Quick diagnostic commands

Raw H264 camera:

```bash
scripts/run_camera_h264_jetson.sh
```

Focus:

```bash
scripts/run_focus_assist_jetson.sh
```

Checkerboard:

```bash
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
```

Illumination:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Mask/Otsu/ROIs:

```bash
scripts/run_line_calibrator_jetson.sh
```

Mode with real sliders:

```bash
STREAM=local scripts/run_line_calibrator_jetson.sh
```
