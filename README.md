# Puzzlebot Line Follower

ROS2 Humble package copied from the working Jetson workspace and set up for the
laptop/Jetson workflow used in the Manchester projects.


## Handoff / Working Memory

For the full current context of what has been implemented, what worked, what failed, current calibration values, and next steps, read:

```text
docs/HANDOFF_CONTEXT.md
```

## Layout

- `puzzlebot_ros/line_follower.py`: current autonomous racer node. It reads the
  Jetson CSI camera, follows the line, handles traffic-light state, publishes
  `/cmd_vel`, and exposes MJPEG on `http://10.10.0.100:8080`.
- `puzzlebot_ros/line_detector.py`: simpler dual-ROI line follower.
- `launch/`: original Puzzlebot launch files from the Jetson package.
- `scripts/`: sync, build, run, tmux demo, and stop helpers.

## Jetson Defaults

The scripts default to:

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws
```

Override any of them inline if needed:

```bash
JETSON_HOST=10.10.0.101 scripts/run_demo_tmux.sh
```

## Common Flow

From this repo on the laptop:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_line_follower_jetson.sh
```

For the full tmux workflow:

```bash
scripts/run_demo_tmux.sh
```

That opens panes for sync/build, micro-ROS agent, line follower, and topic
monitoring. The camera preview stream is available at:

```text
http://10.10.0.100:8080
```

Stop safely with:

```bash
scripts/stop_demo.sh
```

## Intersection Decision Mode

The line follower now detects stable dashed-line patterns as an intersection cue.
When that happens it publishes zero `/cmd_vel`, overlays the available options in
MJPEG, and publishes a text prompt on `/intersection_prompt`.

Watch prompts:

```bash
ros2 topic echo /intersection_prompt
```

Answer from another terminal on the Jetson:

```bash
ros2 topic pub --once /intersection_decision std_msgs/msg/String "{data: 'left'}"
ros2 topic pub --once /intersection_decision std_msgs/msg/String "{data: 'straight'}"
ros2 topic pub --once /intersection_decision std_msgs/msg/String "{data: 'right'}"
```

Spanish aliases also work: `izquierda`, `recto`, `adelante`, `derecha`.
After receiving a valid decision, the robot performs a short slow commit
maneuver and then resumes normal line following.

## Camera Undistortion

`config/camera_params.npz` comes from `mod2_computer_vision/Activities/activity_2_07`.
Undistortion is enabled by default. Override it with ROS parameters if needed:

```bash
ros2 run puzzlebot_ros line_follower --ros-args -p use_undistort:=false
ros2 run puzzlebot_ros line_follower --ros-args -p camera_params_path:=/path/to/camera_params.npz
```

## Vision Calibration Tool

Use this before tuning the robot controller. It never publishes `/cmd_vel`.

Run live on the Jetson camera:

```bash
scripts/run_line_calibrator_jetson.sh
```

Optional label for saved samples:

```bash
LABEL=false_intersection scripts/run_line_calibrator_jetson.sh
```

Inside the OpenCV window:

- `s`: save raw/processed/mask/overlay + JSON metadata.
- `u`: toggle undistortion.
- `p`: pause live camera.
- `q`: quit.

The most important sliders for the current false positive are:

- `roi_y0_pct`, `roi_y1_pct`: vertical band used for dashed detection.
- `dash_min_area`, `rect_pct`, `max_aspect_x10`: dash-shape filters.
- `min_dash_count`, `stable_frames`: how much evidence is needed.
- `option_x0_pct`, `option_x1_pct`, `option_y0_pct`, `option_y1_pct`: option box used only to infer left/straight/right exits; this excludes edge noise and lower incoming-lane dashes.
- `ratio_fallback`: keep at `0` while tuning; ratios are debug only.

Pull saved samples from the Jetson:

```bash
scripts/pull_calibration_dataset.sh
```

Run on saved images locally:

```bash
python3 tools/line_vision_calibrator.py --image path/to/frame.jpg --label false_intersection
```

You can also change calibrator sliders and the active save label without touching the OpenCV controls.
In the calibrator terminal, type commands like:

```text
min_dash_count=5
roi_y0_pct 42
set dash_min_area 40
label=true_intersection
```

Or from another laptop terminal while the Jetson calibrator is running:

```bash
scripts/set_calibrator_param.sh min_dash_count 5
scripts/set_calibrator_param.sh roi_y0_pct 42
scripts/set_calibrator_param.sh dash_min_area 40
scripts/set_calibrator_param.sh label true_intersection
```

The `Controls` sliders update when parameter commands are applied, and the overlay shows the current `label:` used by the next `s` save.

### Illumination Flat-Field Calibration

Capture a white-lona flat-field reference on the Jetson:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Point the camera at clean white lona/cardstock in the robot camera pose, wait for exposure to settle, press `c` to save `config/illumination_flatfield.npz`, then press `q`. The line calibrator and `puzzlebot_ros/line_follower.py` load that file automatically when it exists.

### Dynamic Entry-Based Option ROI

The line calibrator can place the option ROI above the detected entry zebra instead of using a fixed blue box. This keeps the option ROI from overlapping the lower entry zebra.

Relevant live parameters:

```text
dynamic_option_roi = 1
entry_y0_pct = 58
entry_margin_pct = 4
dynamic_option_height_pct = 28
option_x0_pct = 8
option_x1_pct = 92
```

The orange line in the overlay is the detected entry zebra y-position. The blue box is the effective option ROI used for left/straight/right. Set `dynamic_option_roi=0` to return to the fixed option box.
