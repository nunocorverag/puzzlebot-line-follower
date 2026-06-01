# Scripts & Tools Reference

Catalog of every helper script and standalone tool, what it does, and when to
run it. The laptop is the dev machine; the Jetson (`puzzlebot@10.10.0.100`) is
the runtime target.

## Shared conventions

All `scripts/*.sh` honor these environment overrides (with defaults):

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws        # remote workspace
# remote package = ${REMOTE_WS}/src/puzzlebot_ros
```

Override inline, e.g. `JETSON_HOST=10.10.0.101 scripts/run_line_follower_jetson.sh`.

> **Known inconsistency (cleanup pending):** sourcing, SSH flags (`-X`/`-t`),
> and variable names (`REMOTE_PACKAGE` vs `REMOTE_PKG`) differ between scripts.
> The plan is to factor a `scripts/lib/common.sh`. Until then, prefer the
> scripts marked "(canonical)" below as the reference examples.

---

## Daily line-follower workflow

| Script | What it does | When to run |
| --- | --- | --- |
| `sync_to_jetson.sh` (canonical) | `rsync` the repo to the Jetson package dir (excludes build/datasets), then `chmod +x` the scripts. | Before every build/run; most run-scripts call it for you. |
| `build_on_jetson.sh` (canonical) | `colcon build --packages-select puzzlebot_ros` on the Jetson. | After syncing changed Python that ROS needs installed (entry points/launch). |
| `run_line_follower_jetson.sh` | Runs `ros2 run puzzlebot_ros line_follower` (the autonomous racer). MJPEG at `http://10.10.0.100:8080`. Override node with `NODE=`. | Real autonomous run. Wheels-up first. |
| `run_demo_tmux.sh` | tmux session: sync/build + micro-ROS agent + line follower + topic monitor. | Full demo orchestration. |
| `stop_demo.sh` (canonical) | Kills tmux, kills `line_follower`/`autonomous_racer`, publishes a zero `/cmd_vel` burst, stops micro-ROS agent. | **Emergency stop / clean shutdown.** Keep it in a ready terminal. |
| `kill_all_jetson.sh` | `pkill` of recorder/camera/agent processes on the Jetson. | Quick cleanup of stray processes. |
| `start_all_jetson.sh` | `nohup` launch of camera + recorder in background with logs in `/tmp`. | Headless data-collection sessions. |

## Calibration & perception tuning

| Script / Tool | What it does | When to run |
| --- | --- | --- |
| `run_line_calibrator_jetson.sh` | Syncs, then runs `tools/line_vision_calibrator.py --gstreamer` (no `/cmd_vel`). Set `LABEL=` for saved samples. | Tune intersection/mask params live. **Primary perception playground.** |
| `tools/line_vision_calibrator.py` | The calibrator itself. Live trackbars, mask/overlay/state windows, saves labeled samples. Also runs offline on saved images: `--image path.jpg`. | Live on Jetson or offline tuning on the laptop. |
| `set_calibrator_param.sh` | Writes `PARAM=VALUE` (or `label X`) into the calibrator command file the tool watches. | Adjust a calibrator slider from a second terminal. |
| `run_illumination_calibrator_jetson.sh` | Runs `tools/illumination_calibrator.py` to capture `config/illumination_flatfield.npz` (press `c`). | Flat-field capture against a uniform white surface. |
| `tools/illumination_calibrator.py` | Builds a per-channel gain map from a uniform reference. | See illumination notes in `HANDOFF_CONTEXT.md`. |
| `pull_calibration_dataset.sh` | `rsync` the Jetson `debug_dataset/` down to `debug_dataset/jetson/`. | After a labeling session, to tune offline. |
| `puzzlebot_ros/pictures.py` (`ros2 run puzzlebot_ros pictures`) | Captures chessboard images into `calibration_images/` for **camera intrinsics**. | When recalibrating the real CSI camera (see below). |

## Robot motion (use with care)

| Script | What it does | When to run |
| --- | --- | --- |
| `jog_forward_jetson.sh SPEED DURATION` | Publishes `/cmd_vel linear.x=SPEED` for `DURATION` s, then zero. e.g. `0.04 1.5`. | Move the robot forward for visual ROI tests **without** the autonomous node. Never run alongside `line_follower`. |
| `teleop_jetson.sh` | `teleop_twist_keyboard` over SSH. | Manual driving. |

## Camera & data capture

| Script / Tool | What it does | When to run |
| --- | --- | --- |
| `run_camera_jetson.sh` | `ros2 launch puzzlebot_ros camera_jetson.launch.py`. | Bring up the CSI camera topic. |
| `run_recorder_jetson.sh` + `tools/recorder.py` | Periodically saves frames to `dataset/`. | Collect a raw image dataset. |
| `run_teleop_recorder_jetson.sh` + `tools/teleop_recorder.py` | Drive + record simultaneously. | Build a driving dataset. |
| `run_sign_detector_jetson.sh` + `tools/sign_detector.py` | YOLO (`config/best.pt`) traffic-sign / light detection with live preview. Auto-installs `ultralytics`. | Validate the trained YOLO model on the Jetson. |

## Camera preview / streaming

The follower can stream the annotated frame two ways:

There are two different things you can view; pick by intent:

| What you see | Script | Notes |
| --- | --- | --- |
| **Raw camera, no overlays** | `scripts/run_camera_h264_jetson.sh` | Pure GStreamer (no ROS, no line follower). Just the camera. Lowest latency. |
| **Line follower's annotated view** (ROI boxes, anchors, steering line) | `scripts/run_line_follower_h264.sh` | Runs the autonomous racer and streams the frame it draws on. For debugging perception. |

> The CSI camera allows only ONE process at a time. Run either the follower or
> the raw-camera preview, never both. Stop a running follower with
> `scripts/stop_demo.sh` before switching.

Transport options for the follower stream:

| Mode | How | Notes |
| --- | --- | --- |
| MJPEG (default) | Open `http://10.10.0.100:8080` in a browser. | Per-frame JPEG. Tune with `stream_fps`, `stream_quality`, `stream_max_width` ROS params. Event-driven (each frame sent once). |
| H264/RTP over UDP | Hardware-encoded (`nvv4l2h264enc`), much lower bandwidth. | Opt-in. Inter-frame compression; best for laggy WiFi. |

H264 workflow (one command, IP auto-detected):

```bash
# Starts follower in H264 mode AND opens the receiver window. Ctrl+C stops both.
scripts/run_line_follower_h264.sh
```

The laptop IP is auto-detected from the route to the Jetson. On the `RoboNet`
connection (ipv4.method=shared) the laptop is always `10.10.0.1`, so you never
set it. Manual/split equivalent:

```bash
STREAM_MODE=h264 scripts/run_line_follower_jetson.sh   # H264_HOST auto-detected
scripts/view_h264_stream.sh                            # on the laptop
```

Tuning ROS params (also work via the run-script env or `--ros-args`):
`stream_fps` (15), `stream_quality` (60), `stream_max_width` (0=full),
`h264_bitrate` (2000000 bits/s), `h264_port` (5000). If the H264 writer fails to
open it falls back to MJPEG automatically.

## Camera intrinsics recalibration (recommended)

The current `config/camera_params.npz` was calibrated from a **different
camera's** image set (`activity_2_07`, rms≈2.23). To get geometrically correct
undistortion for this CSI camera in its mounted pose:

```bash
# 1. Capture chessboard views with the real camera
ros2 run puzzlebot_ros pictures           # saves to calibration_images/
# 2. Recompute intrinsics from those images, overwrite config/camera_params.npz
# 3. Re-verify straight lines look straight in the calibrator (key 'u' toggles undistort)
```
