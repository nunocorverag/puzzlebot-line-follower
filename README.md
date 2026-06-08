# Puzzlebot Line Follower

ROS2 Humble package copied from the working Jetson workspace and set up for the
laptop/Jetson workflow used in the Manchester projects.


## Handoff / Working Memory

For the full current context of what has been implemented, what worked, what failed, current calibration values, and next steps, read the latest handoff:

```text
docs/HANDOFF_2026-06-08.md
```

(Older handoffs `docs/HANDOFF_2026-06-04.md` / `docs/HANDOFF_CONTEXT.md` are kept
for history but are superseded.)

## Layout

- `puzzlebot_ros/line_follower.py`: current autonomous racer node. It reads the
  Jetson CSI camera, follows the line, runs the cross/intersection state machine,
  the optional traffic light and optional YOLO signs, publishes `/cmd_vel`, and
  exposes MJPEG on `http://10.10.0.100:8080`.
- `puzzlebot_ros/perception/`: ROS-free perception modules shared by the node and
  the offline tools — `lane.py` (line follower), `zebra.py` (cross detector),
  `signs.py` (YOLO sign detector).
- `puzzlebot_ros/traffic_light.py`: standalone HSV traffic-light node.
- `puzzlebot_ros/pictures.py`: chessboard capture for camera intrinsics.
- `puzzlebot_ros/stopnoise.py`: emergency zero-`/cmd_vel` helper node.
- `tools/`: non-ROS perception tools (line/illumination calibrators, recorders,
  YOLO sign detector).
- `launch/`: active launch files (`camera_jetson`, `micro_ros_agent`).
- `scripts/`: sync, build, run, calibrate, tmux demo, and stop helpers.
  **See `docs/SCRIPTS.md` for a full catalog of every script and tool.**
- `archive/`: nodes/launch from earlier course modules, kept out of the build.
  See `archive/README.md`.

## First-Time Setup

New laptop? See **[docs/SETUP.md](docs/SETUP.md)**. Setup splits into two
independent tracks — run whichever applies:

```bash
scripts/setup_laptop.sh        # A. libraries (every laptop)
scripts/setup_robonet.sh       # B. network — only the laptop that hosts RoboNet
scripts/check_setup.sh         # verify both (read-only checklist)
```

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

`stop_demo.sh` is the clean stop for the full test stack: it stops local H264
receivers, Jetson camera/perception tools, the follower, micro-ROS, and sends a
zero `/cmd_vel` burst.

## Intersection Decision Mode

The line follower detects stable dashed-line patterns as an intersection cue.
In test mode it can ignore the traffic light, approach the intersection, and
center itself using the detected `entry_center_x`; the bottom line-following ROI
is recentered around that entry during the approach. When the target entry band
is reached and centered, it publishes zero `/cmd_vel`, overlays the available
options, and publishes a text prompt on `/intersection_prompt`.

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

## Traffic Signs (YOLO) & Traffic Light

Both are **opt-in / non-blocking** — they never break line following.

YOLO signs (off by default) are enabled with `USE_SIGNS=1`:

```bash
USE_SIGNS=1 scripts/run_demo_tmux.sh
```

`best.pt` (Spanish classes) maps to driving actions: `trabajadores` slows down,
`stop`/`give-way` halt for a few seconds (only when close), and the arrows /
`straight` latch the next cross decision (the manual `1/2/3` still overrides).
`USE_SIGNS=1` makes the run script preload torch's libgomp and keep user-site so
ultralytics imports on the Jetson.

The **traffic light is optional** (`traffic_light_optional`, default on): the
robot drives by default and only obeys a RED/YELLOW that is actually seen — it
does not wait for green. The light is validated by circular shape on the gray
screen/plate so stray colored objects (and a STOP sign's red) are rejected.

See **[docs/HANDOFF_2026-06-08.md](docs/HANDOFF_2026-06-08.md)** for the full
behaviour, params, and current open issues.

## Camera Undistortion

`config/camera_params.npz` holds the camera intrinsics. To recalibrate this CSI
camera from scratch (auto-guided checkerboard capture + compute, all over H264),
see **[docs/CALIBRATION_CHECKERBOARD.md](docs/CALIBRATION_CHECKERBOARD.md)**.
Undistortion is enabled by default. Override it with ROS parameters if needed:

```bash
ros2 run puzzlebot_ros line_follower --ros-args -p use_undistort:=false
ros2 run puzzlebot_ros line_follower --ros-args -p camera_params_path:=/path/to/camera_params.npz
```

## Vision Calibration Tool

Use this before tuning the robot controller. It never publishes `/cmd_vel`.

Run live on the Jetson camera. The script sends `drive_enable=false` before
opening the CSI camera, so the calibrator stays perception-only. Default is the
fast H264 dashboard; use `STREAM=local` for the old OpenCV trackbars:

```bash
scripts/run_line_calibrator_jetson.sh
STREAM=local scripts/run_line_calibrator_jetson.sh
VIDEO_SINK=ximagesink scripts/run_line_calibrator_jetson.sh   # WSL/X11
```

For laptop-specific video setup, configure it once:

```bash
# Native Ubuntu
scripts/set_local_video_sink.sh autovideosink

# WSL/Windows with X11
scripts/set_local_video_sink.sh ximagesink
```

That writes `scripts/local.env`, which is ignored by git and loaded by the H264
receiver. After that, WSL teammates can run the calibrator normally:

```bash
scripts/run_line_calibrator_jetson.sh
```

If their script does not support `scripts/local.env` or `VIDEO_SINK`, they are
on an older checkout and should pull the latest branch before testing.

Full current workflow: **[docs/PERCEPTION_TUNING.md](docs/PERCEPTION_TUNING.md)**.
Bird's-eye lane following, track measurements and camera tilt:
**[docs/LANE_FOLLOWING.md](docs/LANE_FOLLOWING.md)**.

Optional label for saved samples:

```bash
LABEL=false_intersection scripts/run_line_calibrator_jetson.sh
```

In H264 mode, type commands in the calibrator terminal: `s=1`, `u=1`, `p=1`,
`q=1`, or set parameters such as `roi_y0_pct=72`. The same controls work from
another laptop terminal with `scripts/set_calibrator_param.sh s 1`,
`scripts/set_calibrator_param.sh u 1`, etc. In `STREAM=local` mode, the OpenCV
window keys still work: `s`, `u`, `p`, `q`.

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

Removes the reddish color cast and vignetting. Auto-guided over H264 — point the
camera at the white banner filling the frame and it averages good frames by itself:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

It writes `config/illumination_flatfield.npz` (loaded automatically by the line
calibrator and `line_follower.py`). Run it **after** the camera calibration. Full
guide with sampling tips: **[docs/CALIBRATION_ILLUMINATION.md](docs/CALIBRATION_ILLUMINATION.md)**.

### Dynamic Entry-Based Option ROI

The line calibrator can place the option ROI above the detected entry zebra instead of using a fixed blue box. Option ROIs are now drawn and counted as perspective-friendly polygons with adjustable `roi_skew`.

Relevant live parameters:

```text
dynamic_option_roi = 1
entry_y0_pct = 58
entry_margin_pct = 4
dynamic_option_height_pct = 28
option_x0_pct = 8
option_x1_pct = 92
option_roi_skew_pct = 8
```

The orange line in the overlay is the detected entry zebra y-position. The colored polygons are the effective option ROIs used for left/straight/right. At runtime, the follower also uses the detected `entry_center_x` to recenter its lower line-following ROI during `APPROACH_CENTER`, which is what parks the robot squarely at the intersection before tuning or decision handling. Set `dynamic_option_roi=0` to return to the fixed option box.
